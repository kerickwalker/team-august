/***************************************************************************
 *   Copyright (C) 2014-2024 by DTU
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

#include <string>
#include "main.h"
// #include "ucontrol.h"
#include "ulinesensor.h"
#include "ueeconfig.h"
#include "ulog.h"
#include "uad.h"
#include "uencoder.h"
#include "uimu2.h"
#include "urobot.h"

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
  addPublistItem("liw", "Get line-sensor white (AD) level 'liw w1 w2 w3 w4 w5 w6 w7 w8'");
  addPublistItem("lib", "Get line-sensor black (AD) level 'lib b1 b2 b3 b4 b5 b6 b7 b8'");
  addPublistItem("lig", "Get line-sensor channel gain 'lig g1 g2 g3 g4 g5 g6 g7 g8'");
  addPublistItem("livn", "Get line-sensor normalized value 'livn ls1 ls2 ls3 ls4 ls5 ls6 ls7 ls8' values x 1000");
  addPublistItem("lis", "Get line-sensor settings 'lis on white high tilt crossTh wide swap'");
  addPublistItem("lip", "Get line-sensor position 'lip left right valid validCnt crossing crossingCnt'");
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
    if (strlen(p1) > 2)
      lsIsWhite = strtol(p1, &p1, 10);
    if (strlen(p1) > 2)
      lsPowerHigh = strtol(p1, &p1, 10);
    if (strlen(p1) > 2)
      lsTiltCompensate = strtol(p1, &p1, 10);
    if (strlen(p1) > 2)
      wideSensor = strtol(p1, &p1, 10);
    if (strlen(p1) > 2)
      swapLeftRight = strtol(p1, &p1, 10);
    if (strlen(p1) > 2)
      lineValidThreshold = strtof(p1, &p1);
    if (strlen(p1) > 2)
      crossingThreshold = strtof(p1, &p1);
    //usb.send("# got a lip\n");
  }
  else if (strncmp(buf, "litw ", 5) == 0 and strlen(buf) > 19)
  { // calibrate white from provided values
    const char * p1 = &buf[5];
    for (int i = 0; i < 8; i++)
    {
      int16_t v = strtol(p1, (char **)&p1, 10);
      if (blackLevel[i] >= v)
        v = blackLevel[i] + 1;
      whiteLevel[i] = v;
      lsGain[i] = 1.0/(whiteLevel[i] - blackLevel[i]);
    }
    usb.send("# ULineSensor:: white level set from values\n");
  }
  else if (strncmp(buf, "licw", 4) == 0)
  { // calibrate white from current values
    const char * p1 = &buf[4];
    int n = strtol(p1, nullptr, 10);
    if (n == 0)
      n = 10;
    // for (int i = 0; i < 8; i++)
    // {
    //   int16_t v = ad.adcLSH[i] - ad.adcLSL[i];
      calibrateWhite = n;
    //   if (false)
    //   {
    //     if (blackLevel[i] == v)
    //         v++;
    //     whiteLevel[i] = v;
    //     lsGain[i] = 1.0/(whiteLevel[i] - blackLevel[i]);
    //   }
    // }
  } 
  else if (strncmp(buf, "litb ", 5) == 0 and strlen(buf) > 19)
  { // calibrate black from provided values
    const char * p1 = &buf[5];
    for (int i = 0; i < 8; i++)
    {
      int16_t v = strtol(p1, (char **)&p1, 10);
      if (v < 0)
        v = 0;
      if (whiteLevel[i] == v)
        v--;
      blackLevel[i] = v;
      lsGain[i] = 1.0/(whiteLevel[i] - blackLevel[i]);
    }
    usb.send("# ULineSensor:: black level set from values\n");
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
  usb.send("# -- \tlip p [w h t wi s lth xth] \tSettings: p=on, w=white, h=high power, t=tilt comp, wi=wide, s=swap, lth=line thresh (0..1), xth=cross_th (0..1), all but first parameter are optional.\r\n");
  usb.send("# -- \tlicw N\tUse current value as white, average over N samples\r\n");
  usb.send("# -- \tlicb \tUse current value as black (should be zero)\r\n");
  usb.send("# -- \tlitw w w w w w w w w \tUse these values as white\r\n");
  usb.send("# -- \tlitb b b b b b b b b \tUse these values as black (should be zero)\r\n");
}

void ULineSensor::tick()
{ //
  tickCnt++;
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
  if (adcLSDACnt > 1000)
  { // too long time (1 second or more)
    for (int i = 0; i < 8; i++)
    {
      adcLSDA[i] = ad.adcLSH[i] - ad.adcLSL[i];;
    }
    adcLSDACnt = 1;
  }
  else
  {
    for (int i = 0; i < 8; i++)
    {
      adcLSDA[i] += ad.adcLSH[i] - ad.adcLSL[i];;
    }
    adcLSDACnt++;
  }
  //
  if (lineSensorOn)
  { // calibrate, if requested
    calibrateWhiteNow();
    // normalize based on white/black level
    normalize();
    // detect line position and crossing line
    lineDetect();
  }
  else if (calibrateWhite > 0)
  { // Wait a couple of ticks
    // after turn on, to get stable values
    lineSensorOn = true;
    lineSensorOnCnt = 7;
  }
}

void ULineSensor::lineDetect()
{
  float sum = 0;
  float posSum = 0;
  float low = 2.0;
  float high = 0.0;
  // # find levels (and average)
  // # using normalised readings (0.0 (no reflection) to 1.0 (calibrated white)))
  for (int i = 0; i < 8; i++)
  {
    sum += lineSensorValue[i]; // for average
    if (lineSensorValue[i] > high)
      high = lineSensorValue[i]; // # most bright value (like line)
  }
  reflectAverage = sum / 8.0;
  // use average for crossing detect
  crossing = reflectAverage >= crossingThreshold;
  // use high for line valid
  lineValid = high >= lineValidThreshold;
  // # find line position
  // # using COG method from lowest value
  sum = 0;
  // discard anything below this value
  low = lineValidThreshold - 0.1;
  for (int i = 0; i < 8; i++)
  {
    float v = lineSensorValue[i] - low;
    if (v > 0)
    { // probably a line
      sum += v;
      posSum += (i+1) * v;
    }
  }
  if (sum > 0 and lineValidCnt > 0)
  { // get COG for line part.
    linePosition = posSum/sum - 4.5;
  }
  else
    linePosition = 0;
  //
  // make compatible with logging and old code
  lsLeftSide = linePosition;
  lsRightSide = linePosition;
  if (lineValid and lineValidCnt < 20)
    lineValidCnt++;
  else if (not lineValid)
  {
    if (lineValidCnt > 0)
      lineValidCnt --;
    else
      lineValidCnt = 0;
  }
  if (crossing and crossingLineCnt < 20)
    crossingLineCnt++;
  else if (not crossing)
  {
    crossingLineCnt --;
    if (crossingLineCnt < 0)
      crossingLineCnt = 0;
  }
}


void ULineSensor::calibrateWhiteNow()
{
  if (calibrateWhite <= 0 or lineSensorOnCnt > 0)
  { // no calibration or just turned on
    // wait a bit after turn on
    lineSensorOnCnt--;
    return;
  }
  if (calibrateWhiteSumCnt < calibrateWhite)
  {
    for (int i = 0; i < 8; i++)
    { // sum value from this measurement
      calibrateWhiteSum[i] += ad.adcLSH[i] - ad.adcLSL[i];;
    }
    calibrateWhiteSumCnt++;
  }
  else if (calibrateWhiteSumCnt > 0)
  { // done summing - implement result
    for (int i = 0; i < 8; i++)
    {
      whiteLevel[i] = calibrateWhiteSum[i]/calibrateWhiteSumCnt;
      calibrateWhiteSum[i] = 0; // reset
      if (whiteLevel[i] == blackLevel[i])
        whiteLevel[i]++; // avoid divide by zero
      lsGain[i] = 1.0/(whiteLevel[i] - blackLevel[i]);
    }
    const int MSL = 230;
    char s[MSL];
    snprintf(s, MSL, "# ULineSensor::calibrateWhiteNow: set white from %d measurements\n", calibrateWhiteSumCnt);
    usb.send(s);
    calibrateWhiteSumCnt = 0;
    calibrateWhite = 0;
  }
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
    case 5: // u13 -> lis
      sendLineSensorStatus();
      break;
    case 6: // u13 -> lip
      sendLineSensorPosition();
      break;
    default:
      usb.send("# line sensor error\n");
      break;
  }
}

void ULineSensor::sendLineSensorStatus()
{
  const int MRL = 150;
  char reply[MRL];
  snprintf(reply, MRL, "lis %d %d %d %d %d %d %.2f %.2f\r\n",
           lineSensorOn, 
           lsIsWhite,
           lsPowerHigh, lsTiltCompensate,
           wideSensor, swapLeftRight, lineValidThreshold, crossingThreshold
  );
  usb.send(reply);
}

void ULineSensor::sendLineSensorPosition()
{
  const int MRL = 150;
  char reply[MRL];
  snprintf(reply, MRL, "lip %.2f %.2f %d %d %d %d\r\n",
           lsLeftSide, lsRightSide,
           lineValid, lineValidCnt,
           crossing, crossingLineCnt
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
    int n = lineSensorValueSumCnt;
    if (n < 1)
      n = 1;
    snprintf(reply, MRL, "livn %d %d %d %d %d %d %d %d %d\r\n" ,
             int(lineSensorValueSum[0]/n * 1000),
             int(lineSensorValueSum[1]/n * 1000),
             int(lineSensorValueSum[2]/n * 1000),
             int(lineSensorValueSum[3]/n * 1000),
             int(lineSensorValueSum[4]/n * 1000),
             int(lineSensorValueSum[5]/n * 1000),
             int(lineSensorValueSum[6]/n * 1000),
             int(lineSensorValueSum[7]/n * 1000), n
    );
    for (int i = 0; i < 8; i++)
      lineSensorValueSum[i] = 0;
    lineSensorValueSumCnt = 0;
  }
  else
  { // raw value from AD converter, but averaged since last sample
    int div;
    if (adcLSDACnt < 1)
      div = 1;
    else
      div = adcLSDACnt;
    snprintf(reply, MRL, "liv %ld %ld %ld %ld %ld %ld %ld %ld %d\r\n" ,
             adcLSDA[0]/div,
             adcLSDA[1]/div,
             adcLSDA[2]/div,
             adcLSDA[3]/div,
             adcLSDA[4]/div,
             adcLSDA[5]/div,
             adcLSDA[6]/div,
             adcLSDA[7]/div, div
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


void ULineSensor::normalize(void)
{
  float lsv;
  if (lineSensorValueSumCnt > 300)
  { // unuseful long time - restart
    lineSensorValueSumCnt = 0;
    for (int i = 0; i < 8 ; i++)
      lineSensorValueSum[i] = 0;
  }
  for (int i = 0; i < 8; i++)
  { // get difference between illuminated and not.
    int16_t v = ad.adcLSH[i] - ad.adcLSL[i];
    // average value a bit (2 samples)
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
    // also average for subscriptions
    lineSensorValueSum[i] += lsv;
  }
  lineSensorValueSumCnt += 1;
}

///////////////////////////////////////////////////

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
  // limit value space (error values)
  if (crossingThreshold >= 1 or crossingThreshold < 0.3)
    crossingThreshold = 0.8;
  if (lineValidThreshold >= 1 or lineValidThreshold < 0.3)
    lineValidThreshold = 0.85;
  eeConfig.pushByte(int(crossingThreshold*200.0));
  eeConfig.pushByte(int(lineValidThreshold*200.0));
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
  crossingThreshold = float(eeConfig.readByte()) / 200.0;
  lineValidThreshold = float(eeConfig.readByte()) / 200.0;
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
  if (robot.robotHWversion == 9)
    highPowerPin = PIN_LINE_LED_HIGH;
  #endif
}
