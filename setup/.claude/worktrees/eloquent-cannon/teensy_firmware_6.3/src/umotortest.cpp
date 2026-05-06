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

#include <stdlib.h>
#include <math.h>
#include "main.h"
#include "ulog.h"
#include "umotor.h"
#include "umotortest.h"
#include "ueeconfig.h"
#include "ustate.h"
#include "uusb.h"
//#include "ucontrol.h"
#include "usubss.h"
#include "ucurrent.h"
#include "uencoder.h"
#include "uad.h"

UMotorTest motortest;



void UMotorTest::setup()
{ //
  if (setupCnt == 0)
  { // only once
    addPublistItem("motest", "Get estimated parameters 'motest mot[0,1] cv=0/ccv=1 Km R L B S I");
    addPublistItem("motpar", "Get test parameters 'motpars lowVolt highVolt state-time'");
    usb.addSubscriptionService(this);
  }
  setupCnt++;
}


void UMotorTest::sendHelp()
{
  const int MRL = 300;
  char reply[MRL];
  usb.send("# Motor test -------\r\n");
  snprintf(reply, MRL, "# -- \tmottest C\tStart motor test C=0:CV, C=1:CCV, C=-1:stop (NB! Allow wheels to rotate freely)\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tmotset Vlow Vhigh\tMotortest voltage Vlow 3-6 Volt, Vhigh > VLow, samples per state (ms).\r\n");
  usb.send(reply);
}


bool UMotorTest::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "motset ", 7) == 0)
  {
    const char * p1 = &buf[7];
    voltageLow = strtof(p1, (char**)&p1);
    voltageHigh = strtof(p1, (char**)&p1);
    if (voltageLow < 0.5)
      voltageLow = 3;
    if (voltageHigh < 0.6)
      voltageHigh = 6;

    int state_ms = strtol(p1, (char**)&p1, 10);
    int max = LOG_BUFFER_MAX / sizeof(UMotorTestMeasurementData) - 1;
    if (state_ms * 7 > max)
      state_ms = max/7 -1;
    stateLength = state_ms;
  }
  else if (strncmp(buf, "mottest", 7) == 0)
  { // get parameter value
    // default is 0 (CV test)
    if (voltageLow < 0.5)
      voltageLow = 3;
    if (voltageHigh < 0.6)
      voltageHigh = 6;
    const char * p1 = &buf[7];
    switch (strtol(p1, nullptr, 10))
    {
      case 0:
//         motorTestType = MT_CV;
        voltageCCV = 1;
        mLogIndex = 0;
        motorTestRunning = true;
        break;
      case 1:
//         motorTestType = MT_CCV;
        voltageCCV = -1;
        mLogIndex = 0;
        motorTestRunning = true;
        break;
      default:
//         motorTestType = MT_STOP;
        motorTestEnd = true;
        break;
    }
    usb.send("# ----- starting motor test (in 1 second) ------\n");
  }
  else
    used = false;
  return used;
}

void UMotorTest::sendData(int item)
{
  if (item == 0)
    sendMotorParameters();
  else if (item == 1)
    sendMotortestSetup();
}

void UMotorTest::sendMotorParameters()
{
  const int MSL = 150;
  char s[MSL];
  snprintf(s, MSL, "motest %d %f %g %g %g %g %g %g %d\r\n",
           0, voltageCCV, mKonstant[0], mResistance[0], mInductance[0], mFricDyn[0], mFricStat[0], mInertia[0], testValidLeft);
  usb.send(s);
  snprintf(s, MSL, "motest %d %f %g %g %g %g %g %g %d\r\n",
           1, voltageCCV, mKonstant[1], mResistance[1], mInductance[1], mFricDyn[1], mFricStat[1], mInertia[1], testValidRight);
  usb.send(s);
}

void UMotorTest::sendMotortestSetup()
{
  const int MSL = 150;
  char s[MSL];
  snprintf(s, MSL, "motpar %f %f %d\r\n",
           voltageLow * voltageCCV, voltageHigh * voltageCCV, stateLength);
  usb.send(s);
}

void UMotorTest::tick()
{ //
  if (mLogIndex == 0 and motorTestRunning)
  { // motortest uses the logger buffer
    logger.stopLogging();
    logger.logRowCnt = 0;
    mLog = (UMotorTestMeasurementData *)logger.logBuffer;
    clear(&mLog[0]);
    mLogIndexMax = LOG_BUFFER_MAX / sizeof(UMotorTestMeasurementData) - 1;
    motorTestEnd = false;
    tickCnt = 0;
    testState = MS_OFFSET;
    motor.motorVoltage[0] = 0;
    motor.motorVoltage[1] = 0;
    motor.motorSetEnable(1,1);
    encoderStart[0] = encoder.encoder[0];
    encoderStart[1] = encoder.encoder[1];
  }
  if (motorTestRunning)
  {
    if (testState == MS_OFFSET)
    { // current offset should not be needed
      // skipped for now
      testState = MS_OFF;
    }
    if (mLogIndex < mLogIndexMax and mLog != nullptr)
    {
      mLog[mLogIndex].mTime10us = hb10us;
      for (int i = 0; i < 2; i++)
      {
        mLog[mLogIndex].mCurrent[i] = current.getMotorCurrentM(i);
        mLog[mLogIndex].mVoltage[i] = motor.motorVoltage[i];
        mLog[mLogIndex].mEncoder[i] = encoder.encoder[i]; // - encoderStart[i];
        mLog[mLogIndex].velocity[i] = encoder.wheelVelocityEst[i] / encoder.odoWheelRadius[i] * encoder.gear;
      }
      mLog[mLogIndex].sysCurrent = current.getSupplyGurrent();
      mLog[mLogIndex].batVolt = state.batteryVoltage;
      mLogIndex++;
      clear(&mLog[mLogIndex]);
    }
    else
    {
      motorTestEnd = true;
      usb.send("# motor test log is full (or no buffer)\n");
    }
    // state model
    if (tickCnt > stateLength or (testState == MS_OFF and tickCnt > 20))
    { // test state finished
      tickCnt = 0;
      switch (testState)
      { // switch to next state and note log index
        case MS_OFF:
          // start rolling
          testState = MS_LLOW;
          leftTestIndex[0] = mLogIndex;
          leftTestIndex[1] = mLogIndex + steadyStateTime;
          motor.motorVoltage[0] = voltageLow * voltageCCV;
          motor.motorVoltage[1] = 0;
          break;
        case MS_LLOW:
          // switch to high voltage
          testState = MS_LHIGH;
          leftTestIndex[2] = mLogIndex - 1; // end of low
          leftTestIndex[3] = mLogIndex; // start of high
          leftTestIndex[4] = mLogIndex + steadyStateTime; // start of SS
          motor.motorVoltage[0] = voltageHigh * voltageCCV;
          motor.motorVoltage[1] = 0;
          break;
        case MS_LHIGH:
          // end of low
          testState = MS_LEND;
          leftTestIndex[5] = mLogIndex - 1; // end of low
          motor.motorVoltage[0] = 0;
          motor.motorVoltage[1] = 0;
          break;
        case MS_LEND:
          // start right motor
          testState = MS_RLOW;
          rightTestIndex[0] = mLogIndex; // start of low
          rightTestIndex[1] = mLogIndex + steadyStateTime; // start of SS
          motor.motorVoltage[0] = 0;
          motor.motorVoltage[1] = -voltageLow * voltageCCV;
          break;
        case MS_RLOW:
          // switch to high voltage
          testState = MS_RHIGH;
          rightTestIndex[2] = mLogIndex -1 ; // end of low
          rightTestIndex[3] = mLogIndex; // start of high
          rightTestIndex[4] = mLogIndex + steadyStateTime; // start of SS
          motor.motorVoltage[0] = 0;
          motor.motorVoltage[1] = -voltageHigh * voltageCCV;
          break;
        case MS_RHIGH:
          // switch to roll off
          testState = MS_ROLL_OFF;
          rightTestIndex[5] = mLogIndex - 1; // end of high
          motor.motorVoltage[0] = 0;
          motor.motorVoltage[1] = 0;
          break;
        default:
          motorTestEnd = true;
          testState = MS_END;
          break;
      }
    }
    // terminate
    if (motorTestEnd)
    { // stop
      motorTestRunning = false;
      // stop motors
      motor.motorVoltage[0] = 0;
      motor.motorVoltage[1] = 0;
      motor.motorSetEnable(0,0);
      // calculate
      estimateMotorParams();
    }
  }
  else if (motorTestGetLog)
  {
//     usb.send("# motor test log get\n");
//     if (motorTestGetIndex == 0)
//     { // prepare filter filtering backwards
//       for (int m = 0; m < 2; m++)
//       {
//         const int n = 10;
//         const int d = 1;
//         int32_t dt_10us = mLog[mLogIndex - 1].mTime10us - mLog[mLogIndex - 1 - d].mTime10us;
//         float v = int32_t(mLog[mLogIndex - 1].mEncoder[m] - mLog[mLogIndex - 2 - d].mEncoder[m])/(float(dt_10us)*0.00001)*n;
//         for (int i = mLogIndex - 2 - d; i >= 0; i--)
//         { // time span for d=3 measurements
//           int dt_10us = mLog[i + d].mTime10us - mLog[i].mTime10us;
//           // velocity over these d measurements
//           int32_t de = int32_t(mLog[i + d].mEncoder[m] - mLog[i].mEncoder[m]);
//           float b = float(de)/(float(dt_10us)*0.00001);
//           // new estimate in encoder ticks per second (*n)
//           v = v*(n-1)/n + b;
//           mLog[i].filtVel[m] = v / n / float(encoder.pulsPerRev) * M_PI * 2.0f;
//         }
//       }
//     }
    if (motorTestGetIndex >= (mLogIndex - 35))
    {
      motorTestGetLog = false;
      usb.send("logend\n");
    }
    else
    {
      sendTestLog();
      motorTestGetIndex++;
    }
  }
  tickCnt++;
}

///////////////////////////////////////////////////////

void UMotorTest::eePromSave()
{
  // save status
  uint16_t flags = 0;
  flags |= testValidLeft << 0;
  flags |= testValidRight << 1;
  flags |= encoderReversed << 4;
  eeConfig.pushWord(flags);
  // save in kHz
  stateLength = eeConfig.readWord();
  eeConfig.pushFloat(voltageLow);
  eeConfig.pushFloat(voltageHigh);
  for (int i = 0; i < 2; i++)
  {
    eeConfig.pushFloat(mResistance[i]);
    eeConfig.pushFloat(mInductance[i]);
    eeConfig.pushFloat(mKonstant[i]);
    eeConfig.pushFloat(mFricDyn[i]);
    eeConfig.pushFloat(mFricStat[i]);
    eeConfig.pushFloat(mInertia[i]);
  }
}

void UMotorTest::eePromLoad()
{
  if (not eeConfig.isStringConfig())
  {
    uint16_t flags = eeConfig.readWord();
    testValidLeft = (flags & 0x01) > 0;
    testValidRight = (flags & 0x02) > 0;
    encoderReversed = (flags & 0x10) > 0;
    // stored value is in kHz
    stateLength = eeConfig.readWord();
    voltageLow = eeConfig.readFloat();
    voltageHigh = eeConfig.readFloat();
    if (stateLength < 300 or stateLength > 2000)
      stateLength = 1000;
    if (voltageLow < 0.5 or isnanf(voltageLow))
      voltageLow = 3;
    if (voltageHigh < 0.6 or isnanf(voltageHigh))
      voltageHigh = 6;
    for (int i = 0; i < 2; i++)
    {
      mResistance[i] = eeConfig.readFloat();
      mInductance[i] = eeConfig.readFloat();
      mKonstant[i] = eeConfig.readFloat();
      mFricDyn[i] = eeConfig.readFloat();
      mFricStat[i] = eeConfig.readFloat();
      mInertia[i] = eeConfig.readFloat();
    }
    setup();
  }
  else
  { // skip robot specific items
    // one word + 3+12 floats
    int skip = 2 + 3*4 + 2*6*4;
    eeConfig.skipAddr(skip);
  }
}

bool UMotorTest::estimateKandR(int motor, int idx[], float & estK, float & estR)
{
  int lowN = idx[2] - idx[1] - 4;
  int highN = idx[5] - idx[4] - 4;
  float dtLow = float(mLog[idx[2]].mTime10us - mLog[idx[1]].mTime10us)/100000.0;
  float dtHigh = float(mLog[idx[5]].mTime10us - mLog[idx[4]].mTime10us)/100000.0;
  // debug
  const int MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "# calc K and R init: lowIdx=%d, %d, %d; highIdx=%d, %d, %d\n", idx[0], idx[1], idx[2], idx[3], idx[4], idx[5]);
  usb.send(s);
  snprintf(s, MSL, "# calc K and R init: Nlow=%d, Nhigh=%d, dtLow=%.3f, dtHigh=%.3f\n", lowN, highN, dtLow, dtHigh);
  usb.send(s);
  // debug end
  // test sanity for motor voltage 0
  //
  float sumCurLow = 0;
  float sumCurHigh = 0;
  UMotorTestMeasurementData * pd = &mLog[idx[1]] + 2;
  for (int i = 0; i < lowN; i++)
    sumCurLow += pd++->mCurrent[motor];
  meanCurLow[motor] = sumCurLow / lowN;
  pd = &mLog[idx[4]] + 2;
  for (int i = 0; i < highN; i++)
    sumCurHigh += pd++->mCurrent[motor];
  meanCurHigh[motor] = sumCurHigh / highN;
  // debug
  snprintf(s, MSL, "# calc K and R vel: sumCurLow=%.3f, sumCurHigh=%.3f, curLow=%.3fA, curHigh=%.3f A\n",
           sumCurLow, sumCurHigh, meanCurLow[motor], meanCurHigh[motor]);
//   usb.send(s);
  // debug end
  // velocity in rad/s
  uint32_t encStart = encoderStart[motor];
  float app = M_PI * 2.0f / float(encoder.pulsPerRev); // angle per encoder pulse
  if (motor == 0)
    app *= -1;
  float enc1 = (int32_t(mLog[idx[1]].mEncoder[motor] - encStart)) * app;
  float enc2 = (int32_t(mLog[idx[2]].mEncoder[motor] - encStart)) * app;
  float enc4 = (int32_t(mLog[idx[4]].mEncoder[motor] - encStart)) * app;
  float enc5 = (int32_t(mLog[idx[5]].mEncoder[motor] - encStart)) * app;
  float dradLow  = enc2 - enc1;
  float dradHigh = enc5 - enc4;
  velLow[motor] = dradLow / dtLow;
  velHigh[motor] = dradHigh / dtHigh;
  // debug
  snprintf(s, MSL, "# calc K and R vel: dRadLow=%.1f, dRadHigh=%.1f, velLow=%.3f r/s, velHigh=%.3f r/s\n",
           dradLow, dradHigh, velLow[motor], velHigh[motor]);
  usb.send(s);
  // debug end
  float voltLow = mLog[idx[1]].mVoltage[motor];
  float voltHigh = mLog[idx[4]].mVoltage[motor];
  if (voltHigh * velHigh[motor] < 0.0 and motor == 0)
  { // these should have same sign
    // otherwise encoder is reversed (encoder A and B swapped)
    encoderReversed = not encoderReversed;
    usb.send("# encoder reversed is fixed, run test again\n");
    return false;
  }
  /**
   % % calculation                              *
   lcurRel = lHighCur/llowCur
   lhvdif = lHighvel - llowvel * lcurRel
   lK = (lHighVolt - lLowVolt * lcurRel)/lhvdif
   rcurRel = lHighCur/llowCur
   rhvdif = rHighvel - rlowvel * rcurRel
   rK = (rHighVolt - rLowVolt * rcurRel)/rhvdif   * */
  float curRel = meanCurHigh[motor] / meanCurLow[motor];
  float velRelDif = velHigh[motor] - velLow[motor] * curRel;
  estK = (voltHigh - voltLow * curRel)/velRelDif;
  // debug
  snprintf(s, MSL, "# calc K and R vel: vL=%f, vH=%f, curRel=%.3f, velRelDif=%.3f, estK=%.4f\n",
           voltLow, voltHigh, curRel, velRelDif, estK);
  usb.send(s);
  // debug end
  estR = fabsf(voltHigh - velHigh[motor] * estK) / meanCurHigh[motor];
  // debug
  snprintf(s, MSL, "# calc K and R vel: R=%.3f, OK=%d\n", estR, estR > 0 and estK > 0);
  usb.send(s);
  // debug end
  return estR > 0 and estK > 0;
}


bool UMotorTest::estimateRandL(int motor, int idx[3], float& estK, float& estR)
{ // Use the initial acceleration with high current
  // to estimate L and re-estimate R
  //
  // hmmm ... need to think about this
  //
  return true;
}


bool UMotorTest::estimateBandS(int motor, int idx[], float & estB, float & estS)
{ // dunamic friction
  estB = (meanCurHigh[motor] - meanCurLow[motor]) * mKonstant[motor] / fabsf(velHigh[motor] - velLow[motor]);
  // static friction (Nm)
  estS = fabsf(meanCurHigh[motor] * mKonstant[motor] - fabsf(velHigh[motor]) * estB);
  //
//   // debug
//   const int MSL = 200;
//   char s[MSL];
//   snprintf(s, MSL, "# calc B and S init: cur*K=%g Nm, dyn friction = %g Nm\n", meanCurHigh[motor] * mKonstant[motor], velHigh[motor] * estB);
//   usb.send(s);
//   snprintf(s, MSL, "# calc B and S init: dynamic friction=%g Nm/(rad/sec), Static friction limit = %g Nm\n", estB, estS);
//   usb.send(s);
//   // debug end
  return estB > 0 and estS > 0;
}

bool UMotorTest::estimateJ(int motor, int idx[], float & estJ)
{
  int nLow = idx[1] - idx[0];

  float a1, a2, a3, da; // rad
  float dt; // sec
  uint32_t t1, t2, t3; // 10us
  float w; // rad/s
  float j = 0; // kg m^2
  float cur; // Amps
  float tw; // torque for acceleration
  float k = mKonstant[motor];
  float bf = mFricDyn[motor];
  float sf = mFricStat[motor];
  uint32_t encStart = encoderStart[motor];
  float app = M_PI * 2.0f / float(encoder.pulsPerRev); // angle per encoder pulse
  if (motor == 0)
    app *= -1;
  // set start sample pointer
  UMotorTestMeasurementData * pd = &mLog[idx[0] - 1];
  a1 = (int32_t(pd->mEncoder[motor] - encStart)) * app;
  t1 = pd->mTime10us - logStartTime;
  pd++;
  a2 = (int32_t(pd->mEncoder[motor] - encStart)) * app;
  t2 = pd->mTime10us - logStartTime;
  // debug
//   float t0 = float(t2)/100000.0; // start time (sec)
//   const int MSL = 200;
//   char s[MSL];
//   snprintf(s, MSL, "# estimate J: t0=%f, t1=%f, t2=%f (sec)\n", t0, t1/100000.0f, t2/100000.0f);
//   usb.send(s);
//   snprintf(s, MSL, "# estimate J: t1=%lu, t2=%lu, t3=%lu (10us)\n", pd[-1].mTime10us, pd[0].mTime10us, pd[1].mTime10us);
//   usb.send(s);
  // debug end
  // integrate energy into inertia
  for (int i = 0; i < nLow; i++)
  { // current in this sample
    cur = pd->mCurrent[motor];
    // use next and previous sample to calculate
    // velocity for this sample.
    pd++;
    a3 = (int32_t(pd->mEncoder[motor] - encStart)) * app;
    da = a3 - a1;
    t3 = pd->mTime10us - logStartTime;
    dt = float(t3 - t1) / 100000.0;
    w = da/dt;
    // integrate this sample:
    // torque after static friction
    if (cur > 0.0)
    {
      tw = cur * k - sf;
      if (tw > 0)
        // more than static friction
        j += tw - bf * w;
    }
    else
    {
      tw = cur * k + sf;
      if (tw < 0)
        // more than static friction
        j += tw - bf * w;
    }
    // debug
//     if (i % 30 == 0)
//     {
//       snprintf(s, MSL, "# estimate J: i=%d, da=%.3f, dt=%.3f, w=%f, cur=%.3f, tw=%g, j=%g\n", i, da, dt, w, cur, tw, j);
//       usb.send(s);
//     }
    // debug end
    // iterate
    t1 = t2;
    t2 = t3;
    a1 = a2;
    a2 = a3;
  }
  estJ = j / velLow[motor];
  bool isOK = estJ > 0;
  // debug
//   snprintf(s, MSL, "# estimate J: j=%g, estJ=%g, velLow=%g\n", j, estJ, velLow[motor]);
//   usb.send(s);
  // debug end
  return isOK;
}


void UMotorTest::estimateMotorParams()
{
  if (mLog == nullptr or mLogIndex == 0)
    return;
  const int MSL = 300;
  char s[MSL];
  // reset values
  for (int i = 0; i < 2; i++)
  { // set dummy values, if test fails
    mKonstant[i] = 1.0;
    mResistance[i] = 0.1;
    mInductance[i] = 0.001;
    mFricDyn[i] = 1.0;
    mFricStat[i] = 1.0;
    mInertia[i] = 1.0;
  }
  // save start time for easy debug
  logStartTime = mLog[0].mTime10us;
  // steady state
//   bool isOK = estimateKandR(0, leftTestIndex, mKonstant[0], mResistance[0]);
//   if (isOK)
//     isOK = estimateRandL(0, leftTestIndex, mKonstant[0], mResistance[0]);
//   if (isOK)
//     isOK = estimateBandS(0, leftTestIndex, mFricDyn[0], mFricStat[0]);
//   if (isOK)
//     isOK = estimateJ(0, leftTestIndex, mInertia[0]);
  //
  testValidLeft = testModel(0); // left motor
  //
//   if (not isOK)
//     usb.send("# estimate failed for motor 0\n");
  // save result
//   testValidLeft = true;
  //
//   if (true)
//     isOK = estimateKandR(1, rightTestIndex, mKonstant[1], mResistance[1]);
//   if (isOK)
//     isOK = estimateRandL(1, rightTestIndex, mKonstant[1], mResistance[1]);
//   if (isOK)
//     isOK = estimateBandS(1, rightTestIndex, mFricDyn[1], mFricStat[1]);
//   if (isOK)
//     isOK = estimateJ(1, rightTestIndex, mInertia[1]);
  //
  testValidRight = testModel(1); // right motor
  //
//   if (not isOK)
//     usb.send("# estimate failed for motor 1\n");
//   // save result
//   testValidRight = isOK;
  //
  snprintf(s, MSL, "# UMotorTest:: encoder reversed=%d, Low voltage=%g, High voltage=%g\n",
           encoderReversed, voltageLow * voltageCCV, voltageHigh * voltageCCV);
  usb.send(s);
  snprintf(s, MSL, "# UMotorTest:: R=%.4f Ohm, K=%.5f V/(rad/s) or Nm/A, L=%.5f Henry - left motor)\n",
           mResistance[0], mKonstant[0], mInductance[0]);
  usb.send(s);
  snprintf(s, MSL, "# UMotorTest:: R=%.4f Ohm, K=%.5f V/(rad/s) or Nm/A, L=%.5f Henry - right motor)\n",
           mResistance[1], mKonstant[1], mInductance[1]);
  usb.send(s);
  snprintf(s, MSL, "# UMotorTest:: friction: static=%g Nm, dynamic=%g Nm/(rad/sec) - left\n",
           mFricStat[0], mFricDyn[0]);
  usb.send(s);
  snprintf(s, MSL, "# UMotorTest:: friction: static=%g Nm, dynamic=%g Nm/(rad/sec) - right\n",
           mFricStat[1], mFricDyn[1]);
  usb.send(s);
  snprintf(s, MSL, "# UMotorTest:: inertia=%g kg m^2 - left\n",
           mInertia[0]);
  usb.send(s);
  snprintf(s, MSL, "# UMotorTest:: inertia=%g kg m^2 - right\n",
           mInertia[1]);
  usb.send(s);
}

float UMotorTest::testCurrent(int m, float mK, float mR, float mInduc, float & kerr)
{
  UMotorTestMeasurementData * dd = mLog;
  float dt;
  int32_t t0 = dd[0].mTime10us;
  float dv = dd[0].mVoltage[m];
  float Vm = dv;
  float w = 0.1;
  float dwi = 0.0;
  float wsum = 0.0;
  float diffSum = 0.0;
  float Km = mK;
  float Ra = mR;
  float iL = 0;
  float L = mInduc;
  int kCnt = 0;
  float Vdif = 0;
  float kerrSum = 0;
  //
  bool toLog = true;
  //
//   clearEsimate(&dd[0]);
  const int MSL = 150;
  char s[MSL];
//   snprintf(s, MSL, "# testCurrent: - has %d samples (%d bytes) mK=%g, mR=%g, mL=%g\n",
//            mLogIndex, mLogIndex * sizeof(UMotorTestMeasurementData),
//            mK, mR, mInduc);
//   usb.send(s);
  for (int j = 1; j < mLogIndex; j++)
  {
    //dd[j].clearEsimate(); // remove all previous estimates
    dt = float(int32_t(dd->mTime10us) - t0) / 100000.0;
    if (false and dd[0].mVoltage[m] != dd[1].mVoltage[m])
    {
      snprintf(s, MSL, "# testCurrent: %d/%d/%d new motor voltage %.1f -> %.1f\n", j, mLogIndex, mLogIndexMax, dd[0].mVoltage[m], dd[1].mVoltage[m]);
      usb.send(s);
    }
    if (dt > 0 and fabsf(dd->mVoltage[m]) > 0.5)
    { // next sample will have a motor voltage.
      // using odometry estimated velocity (a bit filtered)
      float vel0 = fabsf(dd->velocity[m]);
      Vm = fabsf(dd[-1].mVoltage[m]); // actual motor voltage (from previous sample)
      Vdif = Vm - vel0 * Km; // anker coil voltage
      float Vl = Vdif - iL * Ra; // voltage over inductor
      iL += Vl * dt / L; // motor current
      if (fabsf(Vm) < 1.0)
        w = 0.0;
      else
      {
        if (fabsf(dv - Vm) > 0.3)
        {
          dwi = 0.1;
          w = 0.2;
        }
        dv = Vm;
        w += dwi;
        if (w > 1.0)
        {
          w = 1.0;
          dwi = -0.005;
        }
        else if (w < 0.01 and fabsf(dwi) > 0.00001)
        {
          w = 0.03;
          dwi = 0.0;
        }
        if (false and fabsf(dwi) > 0.00001 and (((j % 20) == 13) or (isnormal(iL) != 0)))
        {
          snprintf(s, MSL, "# j=%d, w=%g, dwi=%g, wsum=%g, difsum=%g, iM=%g, iL=%g, dt=%g, Vl=%g, Vdif=%g\n",
                  j, w, dwi, wsum, diffSum, dd->mCurrent[m], iL, dt, Vl, Vdif);
          usb.send(s);
        }
      }
      wsum += w;
      float di = iL - dd->mCurrent[m]; // current is always positive
      diffSum += w * di * di;
      if (fabsf(Vm) > 0.5)
      {
        kerrSum += di * di;
        kCnt++;
      }
    }
    if (toLog)
    {
      dd->weightCurrent = w;
      dd->e_dt = dt;
      dd->e_current = iL;
      dd->e_dif = Vdif;
    }
    t0 = dd->mTime10us;
    dd++;
  }
  kerr = kerrSum/kCnt;
  return diffSum/wsum;
}


float UMotorTest::estimateFriction(int m, float b, float s, float j, float & jerr)
{
  UMotorTestMeasurementData * dd = mLog;
  //   int32_t en0 = dd[0].mEncoder[motor];
  float estVel = 0;
  bool toLog = true;
  float sumErr = 0.0;
  float sumJweight = 0;
  float sumJerr = 0;
  int sumCnt = 0;
  float w = 0;
  float dwj = 0;
  float Vm = 0;
  float tau0, tau1, tau2;
  int ddd = 0;
  // initial
  dd->e_current = 0;
  dd->e_tau0 = 0;
  dd->e_tau1 = 0;
  dd->e_tau2 = 0;
  dd->e_vel = 0;
  dd->weightInertia = 0;
  bool step = false;
  for (int i = 1; i < mLogIndex; i++)
  {
    float vel = fabsf(dd->velocity[m]); // actual velocity
    float acc = 0;
    float Vm1 = fabsf(dd[-1].mVoltage[m]);
    if (not step)
      step = (Vm1 - Vm) > 0.1;
    Vm = Vm1;
    if (Vm1 > 0.5 and vel > 1.0)
    { // we are driving the motor and the motor is running
      // positive or negative, but
      // change sign is not allowed as measured current always is positive.
      float dt = dd->e_dt; // time since last sample
      tau0 = dd->mCurrent[m] * mKonstant[m]; // generated torque
      // static friction
      if (tau0 > 0 and tau0 > s)
        tau1 = tau0 - s;
      else if (tau0 < -s)
        tau1 = tau0 + s;
      else
        tau1 = 0;
      // dynamic friction
      tau2 = tau1 - b * vel;
      // acceleration
      float acc = tau2 / j;
      estVel += acc*dt; // estimated velocity
      // weight for Inertia estimate
      if (step)
      { // make a new weight wave
        dwj = 0.15;
        w = 0.1;
        step = false;
      }
      w += dwj;
      if (w > 1.0)
      {
        w = 1.0;
        dwj = -0.0075;
      }
      else if (w < 0.01 and fabsf(dwj) > 0.00001)
      {
        w = 0.01;
        dwj = 0.0;
      }
      //
      float verr = vel - estVel;
      sumErr += verr * verr;
      sumCnt++;
      sumJerr += w * verr * verr;
      sumJweight += w;
    }
    else
    {
      w = 0;
      tau0 = 0.0077;
      tau1 = 0.0088;
      tau2 = 0;
    }
    if (false and ddd < 60 and i > 20)
    {
      const int MSL = 300;
      char s[MSL];
      snprintf(s, MSL, "# m %d, i=%d, current %.2fA, est %.2fA, tau0=%g, tau1=%g, tau2=%g, acc=%g, estVel=%g, vel=%g, "
      "err=%g, cnt=%d, res=%g, Jerr=%g, Jweight=%g, jres=%g, w=%.4f, Vm=%.1f, Vmot=%.1f, step=%d\n",
      m, i, dd->mCurrent[m], dd->e_current, tau0, tau1, tau2, acc, estVel, vel,
      sumErr, sumCnt, sumErr/(sumCnt+1), sumJerr, sumJweight, sumJerr/sumJweight, w, Vm, dd[-1].mVoltage[m], step);
      usb.send(s);
      ddd++;
    }
    if (toLog)
    {
      // save to log
      dd->e_tau0 = tau0;
      dd->e_tau1 = tau1;
      dd->e_tau2 = tau2;
      dd->e_vel = estVel;
      dd->weightInertia = w;
    }
    //
    dd++;
  }  // go to next value.
  jerr = sumJerr / sumJweight;
  return sumErr / sumCnt;
}


bool UMotorTest::testModel(int m)
{
  // debug
//   if (m == 1)
//     return false;
  // debug end
  const int MSL = 200;
  char s[MSL];
  float r1 = 1.1; // Ohm
  float km1 = 0.0188; // Nm/A
  float l1 = 0.003; // Henry
  float e2r, e2k, e2l;
  float dR = 0.01;
  float dK = 0.0001;
  float dL = 0.0001;
  float k1e, k2e, dummy;
  bool dRs = false;
  bool dKs = false;
  bool dLs = false;
  bool isOK = false;
  float e1;
  //
  for (int i = 0; i < 30; i++)
  {
    e1 = testCurrent(m, km1, r1, l1, k1e);
    float r2 = r1 + dR;
    e2r = testCurrent(m, km1, r2, l1, dummy);
    float km2 = km1 + dK; // Nm/A
    e2k = testCurrent(m, km2, r1, l1, k2e);
    float l2 = l1 + dL; // Henry
    e2l = testCurrent(m, km1, r1, l2, dummy);
    if (true)
    {
      snprintf(s, MSL, "# motor %d current %d r1=%.4f Ohm, K1=%.5f Nm/A, L=%.5f Henry, e1=%g, e2r=%g, e2k=%g, e2L=%g, k2e=%g\n",
                m, i, r2, km2, l2, e1, e2r, e2k, e2l, k2e);
      usb.send(s);
    }
    bool sign = (e2r < e1);
    if (sign)
      r1 += dR;
    else
      r1 -= dR;
    if (sign != dRs)
    {
      if (i > 3)
        dR *= 0.8;
      dRs = sign;
    }
    sign = (k2e < k1e);
    if (sign)
      km1 += dK;
    else
      km1 -= dK;
    if (sign != dKs)
    {
      if (i > 3)
        dK *= 0.8;
      dKs = sign;
    }
    sign = e2l < e1;
    if (sign)
      l1 += dL;
    else
      l1 -= dL;
    if (sign != dLs)
    {
      if (i > 3)
        dL *=0.8;
      dLs = sign;
    }
  }
  // save result
  if (km1 > 0.001 and r1 > 0.2 and l1 > 0.0005)
  { // OK sanity
    mKonstant[m] = km1;
    mInductance[m] = l1;
    mResistance[m] = r1;
    isOK = e1 < 0.02;;
    if (false)
      usb.send("# saved K, R and L estimates for motor 0; use save to flash to remember values\n");
  }
  //
  // now for the remaining constants
  // friction and moment of inertia
  float b1 = 7.0e-6; // (Nm s/rad)
  float s1 = 0.0037; // (Nm)
  float j1 = 0.0000037; // (Kg m^2)
  float db = b1 / 10.0;
  float ds = s1 / 20.0;
  float dj = j1 / 20.0;
  bool dbs = false;
  bool dss = false;
  bool djs = false;
  float e1j, e2j;
  usb.send("#-------\n");
  snprintf(s, MSL, "# motor %d friction test using: b1=%g, s1=%g, j1=%g\n",
           m, b1, s1, j1);
  usb.send(s);
  for (int i = 0; i < 100; i++)
  {
    e1 = estimateFriction(m, b1, s1, j1, e1j);
    float e2b = estimateFriction(m, b1 + db, s1, j1, dummy);
    float e2s = estimateFriction(m, b1, s1 + ds, j1, dummy);
    dummy = estimateFriction(m, b1, s1, j1 + dj, e2j);
    bool sign = e2b < e1;
    if (sign)
      b1 += db;
    else
      b1 -= db;
    if (sign != dbs)
    {
      if (i > 3)
        db *= 0.95;
      dbs = sign;
    }
    if (true)
    {
      sign = e2s < e1;
      if (sign)
        s1 += ds;
      else
        s1 -= ds;
      if (sign != dss)
      {
        if (i > 3)
          ds *= 0.95;
        dss = sign;
      }
    }
    sign = e2j < e1j;
    if (sign)
       j1 += dj;
     else
       j1 -= dj;
    if (sign != djs)
    {
      if (i > 3)
        dj *= 0.95;
      djs = sign;
    }
    snprintf(s, MSL, "# motor %d fric %d b1=%11.4g, dbs=%d, s1=%.5fg, j1=%11.4g,\t e1=%.0f, e2b=%.0f, e2s=%.0f, e2j=%.0f, e1j=%.0f, djs=%d\n",
              m, i, b1, dbs, s1, j1, e1, e2b, e2s, e2j, e1j, djs);
    usb.send(s);
  }
  if (b1 > 0 and s1 > 0 and j1 > 0 and e1 < 300)
  {
    mFricStat[m] = s1;
    mFricDyn[m] = b1;
    mInertia[m] = j1;
  }
  return isOK and e1 < 300;
}


void UMotorTest::getMotorTestLog()
{
  const int MSL = 200;
  char s[MSL];
  usb.send("% Motor test log\n");
  usb.send("% 1 time stamp (sec)\n");
  usb.send("% 2 sample number\n");
  usb.send("% 3 Left motor voltage (V)\n");
  usb.send("% 4 Left motor current (Amps)\n");
  usb.send("% 5 Left encoder position (ticks)\n");
  usb.send("% 6 Left velocity (rad/s)\n");
  usb.send("% 7 Right motor voltage (V)\n");
  usb.send("% 8 Right motor current (Amps)\n");
  usb.send("% 9 Right encoder position (ticks)\n");
  usb.send("% 10 Right velocity (rad/s)\n");
  usb.send("% 11 Battery voltage (Volt)\n");
  usb.send("% 12 System current (Amps)\n");
  usb.send("% 13 sample time (ms)\n");
  usb.send("% 14 Vm - Vemf\n");
  usb.send("% 15 estmated current\n");
  usb.send("% 16 Weight for R and L estimate\n");
  usb.send("% 17 estimated generated torque (Nm)\n");
  usb.send("% 18 estimated generated torque (Nm) after static friction\n");
  usb.send("% 19 estimated generated torque (Nm) after friction\n");
  usb.send("% 14 weight for J estimate\n");
  usb.send("% 21 estimated velocity\n");
  snprintf(s, MSL, "%% M=0,  Km = %g, Ra = %g, B = %g, S = %g, J = %g, L = %g\n",
           mKonstant[0], mResistance[0], mFricDyn[0], mFricStat[0], mInertia[0], mInductance[0]);
  usb.send(s);
  snprintf(s, MSL, "%% M=1,  Km = %g, Ra = %g, B = %g, S = %g, J = %g,L = %g\n",
           mKonstant[1], mResistance[1], mFricDyn[1], mFricStat[1], mInertia[1], mInductance[1]);
  usb.send(s);
  snprintf(s, MSL, "%% M=0, enc0 = %lu, encEnd = %lu\n", mLog[0].mEncoder[0], mLog[mLogIndex-1].mEncoder[1]);
  usb.send(s);
  snprintf(s, MSL, "%% M=1, enc0 = %lu, encEnd = %lu\n", mLog[0].mEncoder[1], mLog[mLogIndex-1].mEncoder[1]);
  usb.send(s);
  snprintf(s, MSL, "%% Time = %lu ms, Time End = %lu ms\n", mLog[0].mTime10us / 100, mLog[mLogIndex-1].mTime10us / 100);
  usb.send(s);
  snprintf(s, MSL, "%% Pulsees per revolution=%d\n", encoder.pulsPerRev);
  usb.send(s);
  if (mLogIndex == 0 or mLog == nullptr)
  {
    usb.send("%\n% Log empty\n");
  }
  else
  {
    motorTestGetIndex = 0;
    motorTestGetLog = true;
    logStartTime = mLog[0].mTime10us;
    valL = 0;
    valR = 0;
  }
}

void UMotorTest::sendTestLog()
{
  if (true)
    send(&mLog[motorTestGetIndex], motorTestGetIndex, logStartTime);
//   else
//   {
//     UMotorTestMeasurementData * ml = &mLog[motorTestGetIndex];
//     const int MSL = 400;
//     char s[MSL];
//     const int n = 10;
//     const int d = 1;
//     // time span for d=3 measurements
//     int dt_10us = ml[d].mTime10us - ml->mTime10us;
//     // velocity over these d measurements
//     int32_t de0 = ml[d].mEncoder[0] - ml->mEncoder[0];
//     int32_t de1 = ml[d].mEncoder[1] - ml->mEncoder[1];
//     float bL = float(de0)/(float(dt_10us)*0.00001);
//     float bR = float(de1)/(float(dt_10us)*0.00001);
//     // new estimate (*n)
//     valL = valL*(n-1)/n + bL;
//     valR = valR*(n-1)/n + bR;
//     //
//     float encLeft  = int32_t(ml->mEncoder[0]) - int32_t(encoderStart[0]) / float(encoder.pulsPerRev) * M_PI * 2.0f;
//     float encRight = int32_t(ml->mEncoder[1]) - int32_t(encoderStart[1]) / float(encoder.pulsPerRev) * M_PI * 2.0f;
//     snprintf(s, MSL, "%.4f %d %.0f %.2f %.3f %ld %g  %.2f %.3f %ld %g %.3f %.2f %.4f %.4f  %.4g %lu %.4g %.4g %.4g %.4g %.4g %.4g %.3f\n",
//             float(ml->mTime10us - logStartTime) / 100000.0, motorTestGetIndex, voltageCCV,
//             ml->mVoltage[0], ml->mCurrent[0], int32_t(ml->mEncoder[0]), encLeft ,
//             ml->mVoltage[1], ml->mCurrent[1], int32_t(ml->mEncoder[1]), encRight,
//             ml->sysCurrent, ml->batVolt,
//             valL/n / float(encoder.pulsPerRev) * M_PI * 2.0f, valR/n / float(encoder.pulsPerRev) * M_PI * 2.0f,
//             ml->e_dt, ml->e_denc,
//             ml->vel0, ml->e_dif, ml->e_current, ml->e_tau0, ml->e_tau1, ml->e_tau2, ml->e_vel
//             );
//     usb.send(s);
//   }
}

  void UMotorTest::send(UMotorTestMeasurementData * d, int j, uint32_t logStart)
  {
    const int MSL = 400;
    char s[MSL];
    snprintf(s, MSL, "%.5f %d  %.1f %.3f %ld %.1f  %.1f %.3f %ld %.1f \t%.2f %.3f \t%g %g %g %.3f \t %g %g %g %.3f \t%g\n",
             float(d->mTime10us - logStart) / 100000.0, j,
             d->mVoltage[0], d->mCurrent[0], int32_t(d->mEncoder[0]), d->velocity[0],
             d->mVoltage[1], d->mCurrent[1], int32_t(d->mEncoder[1]), d->velocity[1],
             d->batVolt, d->sysCurrent,
             d->e_dt*1000, d->e_dif, d->e_current, d->weightCurrent, d->e_tau0, d->e_tau1, d->e_tau2, d->weightInertia, d->e_vel);
    usb.send(s);
  }

    void UMotorTest::clearEsimate(UMotorTestMeasurementData * d)
  {
    d->e_dt = 0;
    d->e_denc = 0;
    d->e_dif = 0; // Voltage - emf
    d->e_current = 0; // estimated current
    d->e_tau0 = 0; // generated
    d->e_tau1 = 0; // after static friction
    d->e_tau2 = 0; // after dyn friction
    d->e_vel = 0;  // estimate velocity
  }

  void UMotorTest::clear(UMotorTestMeasurementData * d)
  {
    d->mTime10us = 0;
    for (int i = 0; i < 2; i++)
    {
      d->mVoltage[i] = 0;
      d->mCurrent[i] = 0;
      d->mEncoder[i] = 0;
      d->velocity[i] = 0;
    }
    d->batVolt = 0;
    d->sysCurrent = 0;
    clearEsimate(d);
  }
