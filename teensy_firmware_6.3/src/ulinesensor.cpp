/***************************************************************************
 *   Copyright (C) 2014-2022 by DTU
 *   jca@elektro.dtu.dk            
 * 
 * 
 * The MIT License (MIT)  https://mit-license.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. */

#include "main.h"
// #include "ucontrol.h"
#include "ulinesensor.h"
#include "ueeconfig.h"
#include "ulog.h"
#include "uad.h"
#include "uencoder.h"
#include "uimu2.h"
#include "ustate.h"

ULineSensor ls;
//////////////////////////////////////////////


void ULineSensor::setup()
{
  highPowerPin = PIN_LINE_LED_HIGH; // can be changed by HW config load (eePromLoadLinesensor)
  pinModeLed = OUTPUT; // switch to input for half power
  pinMode ( highPowerPin, OUTPUT ); // line sensor LED full power
  pinMode ( PIN_LINE_LED_LOW, OUTPUT ); // LED line sensor - half power (HW3)
  lineSensorOn = false;
  digitalWriteFast ( highPowerPin, lineSensorOn );
  //
  addPublistItem("liv", "Get line-sensor raw AD value 'liv ls1 ls2 ls3 ls4 ls5 ls6 ls7 ls8'");
  if (true or not state.robobot)
  { // we are in Regbot mode
    addPublistItem("liw", "Get line-sensor white (AD) level 'liw w1 w2 w3 w4 w5 w6 w7 w8'");
    addPublistItem("lib", "Get line-sensor black (AD) level 'lib b1 b2 b3 b4 b5 b6 b7 b8'");
    addPublistItem("lig", "Get line-sensor channel gain 'lig g1 g2 g3 g4 g5 g6 g7 g8'");
    addPublistItem("livn", "Get line-sensor normalized value 'livn ls1 ls2 ls3 ls4 ls5 ls6 ls7 ls8'");
    addPublistItem("lip", "Get line-sensor position 'lip on wh le leCnt re reCnt ... some more'");
  }
  usb.addSubscriptionService(this);
}

bool ULineSensor::decode(const char* buf)
{
  bool used = true;
  // is for the line sensor
  if (strncmp(buf, "lip ", 4) == 0)
  { // assumed white line
    char * p1 = (char *)&buf[4];
    lineSensorOn = strtol(p1, &p1, 10);
    lsIsWhite = strtol(p1, &p1, 10);
    lsPowerHigh = strtol(p1, &p1, 10);
    lsTiltCompensate = strtol(p1, &p1, 10);
    crossingDetect = strtof(p1, &p1);
    wideSensor = strtol(p1, &p1, 10);
    if (strlen(p1) > 0)
      swapLeftRight = strtol(p1, &p1, 10);
    //usb.send("# got a lip\n");
  }
  else if (strncmp(buf, "licw", 4) == 0)
  { // calibrate white
    for (int i = 0; i < 8; i++)
    {
      int16_t v = ad.adcLSH[i] - ad.adcLSL[i];
      if (blackLevel[i] == v)
          v++;
      whiteLevel[i] = v;
      lsGain[i] = 1.0/(whiteLevel[i] - blackLevel[i]);
    }
  } 
  else if (strncmp(buf, "licb", 4) == 0)
  { // calibrate black
    for (int i = 0; i < 8; i++)
    {
      int16_t v = ad.adcLSH[i] - ad.adcLSL[i];
      if (v < 0)
        v = 0;
      if (whiteLevel[i] == v)
        v--;
      blackLevel[i] = v;
      lsGain[i] = 1.0/(whiteLevel[i] - blackLevel[i]);
    }
  }
  else
    used = false;
  return used;
}

void ULineSensor::sendHelp()
{
//   const int MRL = 150;
//   char reply[MRL];
  usb.send("# Line sensor -------\r\n");
  usb.send("# -- \tlip p w h t xth wi s \tSet sensor basics p=on, w=white, h=high power, t=tilt comp, xth=cross_th, wi=wide, s=swap\r\n");
  if (true or not state.robobot)
  {
    usb.send("# -- \tlicw \tUse current value as full white\r\n");
    usb.send("# -- \tlicb \tUse current value as black limit\r\n");
  }
}

void ULineSensor::tick()
{ //
  bool toOn = lineSensorOn and not lineSensorIsOn;
  if (ls.lsPowerHigh and (pinModeLed == INPUT or toOn))
  { // high power mode - use both pin 18/6 and pin 32
    pinMode(highPowerPin, OUTPUT); // Line sensor power control
    pinModeLed = OUTPUT;
    lineSensorIsOn = lineSensorOn;
  }
  else if (not ls.lsPowerHigh and (pinModeLed == OUTPUT or toOn))
  { // low power mode - use pin 32 only (or pin 25 when power board is installed)
    pinMode(highPowerPin, INPUT); // Line sensor power control
    pinModeLed = INPUT;
    lineSensorIsOn = lineSensorOn;
  }
  // average sensor (sum) values and increase sample number
  // for robobot and better display when low data rate
  for (int i = 0; i < 8; i++)
  {
    adcLSDA[i] += ad.adcLSH[i] - ad.adcLSL[i];;
  }
  adcLSDACnt++;
  //
  if (true or not state.robobot)
    estimteLineEdgePosition();
}

void ULineSensor::sendData(int item)
{
  switch (item)
  {
    case 0: // liv
      sendStatusLineSensor(false);
      break;
    case 1: // u9 -> liw
      sendStatusLineSensorLimitsWhite();
      break;
    case 2: // u10 -> lib
      sendStatusLineSensorLimitsBlack();
      break;
    case 3: // u11 -> lig
      sendLineSensorGain();
      break;
    case 4: // livn
      sendStatusLineSensor(true);
      break;
    case 5: // u13 -> lip
      sendLineSensorPosition();
//       usb.send("# line sensor last\n");
      break;
    default:
      usb.send("# line sensor error\n");
      break;
  }
}

void ULineSensor::sendLineSensorPosition()
{
  const int MRL = 150;
  char reply[MRL];
  snprintf(reply, MRL, "lip %d %d %.4f %d %.4f %d %d %d %d %d %d %d %d %g %d %d\r\n" ,
           lineSensorOn, 
           lsIsWhite,
           lsLeftSide, lsEdgeValidCnt,
           lsRightSide, lsEdgeValidCnt, 
           4, //control.mission_line_LeftEdge,  // not visible in client
           44, 44, crossingLineCnt, crossingLineCnt,
           lsPowerHigh, lsTiltCompensate, crossingDetect,
           wideSensor, swapLeftRight
  );
  usb.send(reply);
}

//////////////////////////////////////////////

void ULineSensor::sendStatusLineSensorLimitsWhite()
{
  const int MRL = 70;
  char reply[MRL];
  snprintf(reply, MRL, "liw %d %d %d %d %d %d %d %d\r\n" ,
           whiteLevel[0],
           whiteLevel[1],
           whiteLevel[2],
           whiteLevel[3],
           whiteLevel[4],
           whiteLevel[5],
           whiteLevel[6],
           whiteLevel[7]
  );
  usb.send(reply);
}

void ULineSensor::sendStatusLineSensorLimitsBlack()
{
  const int MRL = 70;
  char reply[MRL];
  snprintf(reply, MRL, "lib %d %d %d %d %d %d %d %d\r\n" ,
           blackLevel[0],
           blackLevel[1],
           blackLevel[2],
           blackLevel[3],
           blackLevel[4],
           blackLevel[5],
           blackLevel[6],
           blackLevel[7]
  );
  usb.send(reply);
}

//////////////////////////////////////////////

void ULineSensor::sendStatusLineSensor(bool normalized)
{
  const int MRL = 170;
  char reply[MRL];
//   sendLineSensorPosition();
  if (normalized)
  { // compensated for calibration and tilt,
    // value in range 0..1
    snprintf(reply, MRL, "livn %d %d %d %d %d %d %d %d\r\n" ,
             int(lineSensorValue[0] * 1000),
             int(lineSensorValue[1] * 1000),
             int(lineSensorValue[2] * 1000),
             int(lineSensorValue[3] * 1000),
             int(lineSensorValue[4] * 1000),
             int(lineSensorValue[5] * 1000),
             int(lineSensorValue[6] * 1000),
             int(lineSensorValue[7] * 1000)
    );
  }
  else
  { // raw value from AD converter, but averaged since last sample
    int div;
    if (adcLSDACnt < 1)
      div = 1;
    else
      div = adcLSDACnt;
    snprintf(reply, MRL, "liv %ld %ld %ld %ld %ld %ld %ld %ld\r\n" ,
             adcLSDA[0]/div,
             adcLSDA[1]/div,
             adcLSDA[2]/div,
             adcLSDA[3]/div,
             adcLSDA[4]/div,
             adcLSDA[5]/div,
             adcLSDA[6]/div,
             adcLSDA[7]/div
    );
    adcLSDACnt = 0;
    for (int i = 0; i < 8; i++)
      adcLSDA[i] = 0;
  }
  usb.send(reply);
    // send also position
}


void ULineSensor::sendLineSensorGain()
{
  const int MRL = 120;
  char reply[MRL];
  //   sendLineSensorPosition();
  //if (useLineSensor)
  snprintf(reply, MRL, "#lig %g %g %g %g %g %g %g %g\r\n" ,
           lsGain[0],
           lsGain[1],
           lsGain[2],
           lsGain[3],
           lsGain[4],
           lsGain[5],
           lsGain[6],
           lsGain[7]
  );
  usb.send(reply);
}

//////////////////////////////////////////////



void ULineSensor::estimteLineEdgePosition()
{
  if (lineSensorOn)
  { // find edge position ..
    normalize();
    // findEdgeV3();
    findEdgeV4();
    // and estimate if we met a crossing line
    findCrossingLine();
    if (swapLeftRight)
    {
      float a = lsLeftSide;
      lsLeftSide = lsRightSide;
      lsRightSide = a;
    }
  }
  else
  { // nothing is valid if sensor is off
    lsEdgeValid = false;
    crossingLineCnt = 0;
    findCrossingLineVal = 0.0;
    lsLeftSide = 0;
    lsRightSide = 0;
  }
}

/////////////////////////////////////////////

void ULineSensor::normalize(void)
{
  float lineValMin = 1.0;
  float lineValMax = 0.0;
  float lsv;
  for (int i = 0; i < 8; i++)
  { // get difference between illuminated and not.
    int16_t v = ad.adcLSH[i] - ad.adcLSL[i];
    // average value a bit (2ms)
    adcLSD[i] = (v + adcLSD[i])/2;
    // normalize
    v = adcLSD[i] - blackLevel[i];
    lsv = v * lsGain[i];
    // if in balance, then compensate for distance change when robot is tilting
    if (lsTiltCompensate)
    { // assumed to be in balance within +/- 18 degrees
      // compensate intensity as the robot tilts.
      // assumes the intensity is calibrated at tilt angle 0
      // leaning forward (pose[3] positive) then decrease value
      if (encoder.pose[3] >= 0)
      { // leaning forward - tilt is positive
        if (i == 0 or i == 7)
          lsv /= (1.0 + 2.5 * encoder.pose[3]);
        else
          lsv /= (1.0 + 1.5 * encoder.pose[3]);
      }
      else //if (pose[3] > -0.3)
      {  // leaning away from line - tilt is negative
        if (i == 0 or i == 7)
          lsv /= (1.0 + 2.0 * encoder.pose[3]);
        else if (i == 1 or i == 6)
          lsv /= (1.0 + 1.8 * encoder.pose[3]);
        else
          lsv /= (1.0 + 1.6 * encoder.pose[3]);
      }
    }
    // save normalized value
    lineSensorValue[i] = lsv;
    // get max and min values for all sensors
    if (lineSensorValue[i] > lineValMax)
      lineValMax = lineSensorValue[i];
    if (lineSensorValue[i] < lineValMin)
      lineValMin = lineSensorValue[i];
  }
}


void ULineSensor::lsPostProcess()
{
  lsControlStoppedAt = hbTimerCnt;
  lsEdgeValid = false;
}


///////////////////////////////////////////////////


void ULineSensor::findCrossingLine()
{ 
  float crossingTreshold = crossingDetect;
  float vl, vr;
  float dist = encoder.distance - oldPos2mm;
  //
  if (((hb10us - oldDistTime) > 50000) or (oldPos2mm > encoder.distance))
  { // do not trust crossing line until control is settled,
    // or if line sensor has been inactive for some time (not moving)
    // or oldPos2mm is from an old mission line
    invalidCnt = 10; // in sample time
    oldDistTime = hb10us;
    oldPos2mm = encoder.distance;
//     crossingLineCnt = 0;
    edgePos2mm[0] = lsLeftSide;
    edgePos2mm[1] = lsRightSide;
//     usb.send("# No movement \n");
  }
  // debug log
  if (false and logLineSensorExtra)
  {
    logger.dataloggerExtra[0] = lsLeftSide;
    logger.dataloggerExtra[1] = lsRightSide;
    logger.dataloggerExtra[2] = invalidCnt;
    logger.dataloggerExtra[3] = encoder.distance;
    logger.dataloggerExtra[4] = oldPos2mm;
    logger.dataloggerExtra[5] = dist;
  }
  // debug log end
  if (lsEdgeValid)
  { // edges are valid
    if (invalidCnt == 0)
    { // add a bit rate of change per mm with a gain of kd
      vl = lsLeftSide + (lsLeftSide - edgePos2mm[0]) * kd;
      if (vl < -3.5)
        vl = -3.5;
      else if (vl > 3.5)
        vl = 3.5;
      vr = lsRightSide + (lsRightSide - edgePos2mm[1]) * kd;
      if (vr < -3.5)
        vr = -3.5;
      else if (vr > 3.5)
        vr = 3.5;
    }
    else
    { // history (rate of change) is not valid
      vl = lsLeftSide;
      vr = lsRightSide;
    }
    // save rate of change (per mm)
    if (encoder.distance - oldPos2mm > 0.002)
    { // distance is 1mm (or more), so we save a value
      // (to avoid turning at a crossing line)
      oldPos2mm = encoder.distance;
      oldDistTime = hb10us;
      edgePos2mm[0] = lsLeftSide;
      edgePos2mm[1] = lsRightSide;
    }
    const float kg = 0.0005;
    // on some robobot z-axis is gyro x, on regbot it is z, so add the two
    findCrossingLineVal = vr - vl - fabsf(imu2.gyro[2] + imu2.gyro[0]) * kg;
    // debug log
    if (logLineSensorExtra)
    {
      logger.dataloggerExtra[6] = findCrossingLineVal;
      logger.dataloggerExtra[7] = vl;
      logger.dataloggerExtra[8] = vr;
    }
    // debug log end
    // simple threshold to detect
    crossing =  findCrossingLineVal > crossingTreshold;
    if (invalidCnt > 0)
      invalidCnt--;
  }
  else
  { // no valid input
    if (invalidCnt < 4)
      invalidCnt++;
    findCrossingLineVal = 0.0;
    crossing = false;
    edgeAngle = 0.0;
  }
  // make a quality count to be used as mission parameter
  if (crossing)
  { // increase up to maximum (20)
    if (crossingLineCnt < crossingCntLimit)
      crossingLineCnt++;
  }
  else
  { // decrease until 0
    if (crossingLineCnt > 0)
      crossingLineCnt--;
  }
}




void ULineSensor::findEdgeV4()
{ // destination for extra logging data
  float * nv = logger.dataloggerExtra; // normalized value - for debug logging only;
  if (logLineSensorExtra)
    // zero extra logging data
    memset(logger.dataloggerExtra, 0, sizeof(logger.dataloggerExtra));
  const float minOnWhiteLine = 0.8; // hard limit - part of span from white to black
  const float maxOnBlackLine = 0.3; // hard limit - part of span from white to black
  lsEdgeValid = false;
  for (int i = 0; i < 8; i++)
  {
    if (lsIsWhite)
    { // looking for white line
      detect[i] = lineSensorValue[i] > minOnWhiteLine;
      if (detect[i])
        lsEdgeValid = true;
    }
    else
    { // looking for a black line
      detect[i] = lineSensorValue[i] < maxOnBlackLine;
      if (detect[i])
        lsEdgeValid = true;
    }
  }
  //   if (not lsIsWhite)
  //     usb.send("# NB! code not valid for black line yet\n");
  //
  if (lsEdgeValid)
  {
    int dimax = 0, dimin = 0;
    float dvmin, dvmax;
    // find gratest gradient (dv) - positive and negative
    dvmin = lineSensorValue[0];
    dvmax = dvmin;
    // debug log
    if (logLineSensorExtra)
      nv[0] = lineSensorValue[0];
    // debug log end
    // find midt line
    for (int i = 1; i < 8; i++)
    {
      // debug log
      if (logLineSensorExtra)
        nv[i] = lineSensorValue[i];
      // debug log end
      if (lineSensorValue[i] > dvmax)
      {
        dvmax = lineSensorValue[i];
        dimax = i;
      }
      else if (lineSensorValue[i] < dvmin)
      { // for black line
        dvmin = lineSensorValue[i];
        dimin = i;
      }
    }
    // debug log
    if (logLineSensorExtra)
    {
      logger.dataloggerExtra[17] = dimax; // white
      logger.dataloggerExtra[18] = dimin; // black
    }
    // debug log end
    int leftIdx = 0, rightIdx = 7;
    for (int i = dimax - 1; i > 0; i--)
    {
      if (lineSensorValue[i] < minOnWhiteLine)
      {
        leftIdx = i;
        break;
      }
    }
    for (int i = dimax + 1; i < 7; i++)
    {
      if (lineSensorValue[i] < minOnWhiteLine)
      {
        rightIdx = i;
        break;
      }
    }
    // debug log
    if (logLineSensorExtra)
    {
      logger.dataloggerExtra[19] = leftIdx * 100 + rightIdx;
    }
    // debug log end
    // left side (positive gradient if line is white)
    float mleft, mright; // gradient values left and right of maximum
    float edgePos;
    // left edge of line
    mleft = lineSensorValue[leftIdx];
    mright = lineSensorValue[leftIdx + 1];
    // find edge position in LED index
    float s = mright-mleft;
    if (s > 0.001)
      edgePos = (minOnWhiteLine - mleft)/s + leftIdx;
    else
      // left edge increasing - no good
      edgePos = leftIdx - 0.5;
    // debug log
    if (logLineSensorExtra)
    {
      logger.dataloggerExtra[9] = mleft;
      logger.dataloggerExtra[11] = mright;
      logger.dataloggerExtra[10] = s;
      logger.dataloggerExtra[12] = edgePos;
    }
    // debug log end
    // convert to cm.
    lsLeftSide = (edgePos - lsMidtIndex) * lsLEDdistance;
    if (lsLeftSide < -3.0)
      lsLeftSide = -3.0;
    //
    mleft = lineSensorValue[rightIdx - 1];
    mright = lineSensorValue[rightIdx];
    // find edge position in LED index
    s = mright-mleft;
    if (s < -0.001)
      edgePos = (minOnWhiteLine - mleft)/s + rightIdx - 1;
    else
      // right edge is increasing - no good
      edgePos = rightIdx + 0.5;
    // debug log
    if (logLineSensorExtra)
    {
      logger.dataloggerExtra[13] = mleft;
      logger.dataloggerExtra[15] = mright;
      logger.dataloggerExtra[14] = s;
      logger.dataloggerExtra[16] = edgePos;
    }
    // debug log end
    // convert to cm.
    lsRightSide = (edgePos - lsMidtIndex) * lsLEDdistance;
    if (lsRightSide > 3.0)
      lsRightSide = 3.0;
    //
    if (lsEdgeValidCnt < edgeDetectCntLimit)
      lsEdgeValidCnt++;
  }
  else
  {
    lsRightSide = 0.0;
    lsLeftSide = 0.0;
    if (lsEdgeValidCnt > 0)
      lsEdgeValidCnt--;
  }  
}


/////////////////////////////////////////////////////

void ULineSensor::eePromSaveLinesensor()
{
  char v = 0x00;
  if (lineSensorOn)
    v |= 0x01;
  if (lsIsWhite)
    v |= 0x02;
  if (lsPowerHigh)
    v |= 0x04;
  if (lsTiltCompensate)
    v |= 0x08;
  if (wideSensor)
    v |= 0x10;
  if (swapLeftRight)
    v |= 0x20;
  eeConfig.pushByte(v);
  eeConfig.pushByte(int(crossingDetect*10.0));
  for (int i = 0; i < 8; i++)
  {
    eeConfig.pushWord(blackLevel[i]);
    eeConfig.pushWord(whiteLevel[i]);
  }
}

/////////////////////////////////////////////////////

void ULineSensor::eePromLoadLinesensor()
{
  char v = eeConfig.readByte();
//  char v = eeprom_read_byte((uint8_t*)eePushAdr++);
  lineSensorOn = (v & 0x01) == 0x01;
  lsIsWhite = (v & 0x02) == 0x02;
  lsPowerHigh = (v & 0x04) == 0x04;
  lsTiltCompensate = (v & 0x08) == 0x08;
  wideSensor = (v & 0x10) == 0x10;
  swapLeftRight = (v & 0x20) == 0x20;
  // limit 4 crossing detect
  crossingDetect = float(eeConfig.readByte()) / 10.0;
  // number of bytes to skip if not robot-specific configuration
  int skipCount = 8*(2 + 2);
  if (not eeConfig.isStringConfig())
  { // load from flash
    for (int i = 0; i < 8; i++)
    {
      blackLevel[i] = eeConfig.readWord();
      whiteLevel[i] = eeConfig.readWord();
    }
    for (int i = 0; i < 8; i++)
    { // set gains from new values
      lsGain[i] = 1.0/(whiteLevel[i] - blackLevel[i]);
    }
  }
  else
    // load from hard-coded mission
    eeConfig.skipAddr(skipCount);
  #ifdef REGBOT_HW41
  if (state.robotHWversion == 9)
    highPowerPin = PIN_LINE_LED_HIGH;
  #endif
}
