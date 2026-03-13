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
#include "main.h"
#include "uencoder.h"
#include "ueeconfig.h"
#include "urobot.h"
#include "uusb.h"
// #include "ucontrol.h"
#include "umotortest.h"
#include "umotor.h"
#include "uservice.h"

UEncoder encoder;


void UEncoder::setup()
{ // input should be default, but pin PIN_RIGHT_ENCODER_B on HW41 fails
  pinMode(M1ENC_A, INPUT_PULLUP);
  pinMode(M1ENC_B, INPUT_PULLUP);
  pinMode(M2ENC_A, INPUT_PULLUP);
  pinMode(M2ENC_B, INPUT_PULLUP);
  // use hysteresis on input levels (to avoid extra trigger on slow transitions)
  *digital_pin_to_info_PGM[M1ENC_A].pad |= IOMUXC_PAD_HYS;
  *digital_pin_to_info_PGM[M1ENC_B].pad |= IOMUXC_PAD_HYS;
  *digital_pin_to_info_PGM[M2ENC_A].pad |= IOMUXC_PAD_HYS;
  *digital_pin_to_info_PGM[M2ENC_B].pad |= IOMUXC_PAD_HYS;

  // debug of spurious interrupts, AS5147 SPI port used as debug
  pinMode(7, OUTPUT); // CS
  pinMode(11, OUTPUT);//MOSI
  pinMode(12, OUTPUT);//MISO
  #define USE_SPI_PINS
  // debug end

  attachInterrupt ( M1ENC_A, m1EncoderA, CHANGE );
  attachInterrupt ( M2ENC_A, m2EncoderA, CHANGE );
  attachInterrupt ( M1ENC_B, m1EncoderB, CHANGE );
  attachInterrupt ( M2ENC_B, m2EncoderB, CHANGE );
  // data subscription service topics
  addPublistItem("enc", "Get encoder value 'enc M1 M2' (int32)");
  addPublistItem("pose", "Get current pose 'pose t x y h tilt' (sec,m,m,rad, rad)");
  addPublistItem("vel", "Get velocity 'left right' (m/s)");
  addPublistItem("conf", "Get robot conf (radius, radius, gear, pulsPerRev, wheelbase, sample-time, reversed)");
  addPublistItem("vem", "Get motor and wheel velocity 'left right left right' (rad/s)");
  addPublistItem("ene", "Get encoder error 'enc NANcount reversed M1err M2err' (int32)");
  usb.addSubscriptionService(this);
}

/*  snprintf(s, MSL, "conf %.4f %.4f %.3f %u %4f %4f\r\n", odoWheelRadius[0], odoWheelRadius[1], 
    gear, pulsPerRev, odoWheelBase, SAMPLETIME
*/

void UEncoder::sendHelp()
{
  const int MRL = 300;
  char reply[MRL];
  usb.send("# Encoder settings -------\r\n");
  snprintf(reply, MRL, "# -- \tenc0 \tReset pose to (0,0,0)\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tconfw rl rr g t wb \tSet configuration (radius gear encTick wheelbase)\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tencrev R \tSet motortest reversed encoder (R=0 normal, R=1 reversed)\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "# -- \t         \tNormal: motv 3 3 => left enc decrease, right enc increase.\r\n");
  usb.send(reply);
}


bool UEncoder::decode(const char* buf)
{
  const int MSL = 200;
  char s[MSL];
  bool used = true;
  if (strncmp(buf, "enc0", 4) == 0)
  { // reset pose
    clearPose();
  }
  else if (strncmp(buf, "encd", 4) == 0)
  { // debug encoder print
    int n = eportCnt;
    snprintf(s, MSL, "encp %d\n", n);
    usb.send(s);
    for (int i = 0; i < n; i++)    
    {
      snprintf(s, MSL, "enct %d %d %d %d %d %f\n", i, eport[i][3], eport[i][0], eport[i][1], eport[i][2], edt[i]);
      usb.send(s);
    }
    eportCnt = 0;
  }
  else if (strncmp(buf, "confw ", 5) == 0)
  { // robot configuration
    const char * p1 = &buf[5];
    odoWheelRadius[0] = strtof(p1, (char**) &p1);
    odoWheelRadius[1] = strtof(p1, (char**) &p1);
    gear = strtof(p1, (char**) &p1);
    pulsPerRev = strtol(p1, (char**) &p1, 10);
    odoWheelBase = strtof(p1, (char**) &p1);
    // debug
    snprintf(s, MSL, "# got confw: r1=%g, r2=%g, G=%g, PPR=%d, WB=%g\n", odoWheelRadius[0],
             odoWheelRadius[1], gear, pulsPerRev, odoWheelBase);
    usb.send(s);
    // debug end
    if (pulsPerRev < 1)
      pulsPerRev = 1;
    if (gear < 1)
      gear = 1;
    if (odoWheelBase < 0.02)
      odoWheelBase = 0.02;
    anglePerPuls = 2.0 * M_PI / (pulsPerRev);
  }
  else if (strncmp(buf, "encrev ", 6) == 0)
  { // robot configuration
    const char * p1 = &buf[6];
    motortest.encoderReversed = strtol(p1, (char**) &p1, 10);
  }
  else
    used = false;
  return used;
}

void UEncoder::sendData(int item)
{
  if (item == 0)
    sendEncStatus();
  else if (item == 1)
    sendPose();
  else if (item == 2)
    sendVelocity();
  else if (item == 3)
    sendRobotConfig();
  else if (item == 4)
    sendMotorVelocity();
  else if (item == 5)
    sendEncoderErrors();
}


void UEncoder::sendEncStatus()
{ // return esc status
  const int MSL = 100;
  char s[MSL];
  // changed to svs rather than svo, the bridge do not handle same name 
  // both to and from robot - gets relayed back to robot (create overhead)
  snprintf(s, MSL, "enc %lu %lu\r\n", encoder[0], encoder[1]);
  usb.send(s);
}

void UEncoder::sendEncoderErrors()
{ // return esc status
  const int MSL = 100;
  char s[MSL];
  // changed to svs rather than svo, the bridge do not handle same name
  // both to and from robot - gets relayed back to robot (create overhead)
  snprintf(s, MSL, "ene %d %d %d %d\r\n",
           nanCnt,
           motortest.encoderReversed,
           errCntA[0][0] + errCntA[0][1] + errCntB[0][0] + errCntB[0][1],
           errCntA[1][0] + errCntA[1][1] + errCntB[1][0] + errCntB[1][1]);
  usb.send(s);
}


void UEncoder::sendRobotConfig()
{ // return esc status
  const int MSL = 100;
  char s[MSL];
  // changed to svs rather than svo, the bridge do not handle same name 
  // both to and from robot - gets relayed back to robot (create overhead)
  snprintf(s, MSL, "conf %.4f %.4f %.3f %u %.4f %.4f %d\r\n", odoWheelRadius[0], odoWheelRadius[1], 
    gear, pulsPerRev, odoWheelBase, float(service.sampleTime_us)/1e6, motor.motorReversed
  );
  usb.send(s);
}

void UEncoder::sendPose()
{
  const int MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "pose %.4f %.3f %.3f %.4f %.4f\n",
           service.time_sec(),
           pose[0], pose[1], pose[2], pose[3]);
  usb.send(s);
}

void UEncoder::sendVelocity()
{
  const int MSL = 130;
  char s[MSL];
  if (velSubscribeCnt > 0)
  { // use as average since last report
    snprintf(s, MSL, "vel %.4f %.3f %.3f %.4f %.3f %d\n",
             service.time_sec(),
             wheelVelocityEstSum[0]/velSubscribeCnt,
             wheelVelocityEstSum[1]/velSubscribeCnt,
             robotTurnrateSum/velSubscribeCnt,
             robotVelocitySum/velSubscribeCnt,
             velSubscribeCnt);
    usb.send(s);
    wheelVelocityEstSum[0] = 0;
    wheelVelocityEstSum[1] = 0;
    robotTurnrateSum = 0.0;
    robotVelocitySum = 0.0;
    velSubscribeCnt = 0;
  }
}

void UEncoder::sendMotorVelocity()
{
  const int MSL = 130;
  char s[MSL];
  snprintf(s, MSL, "vem %.4f %.1f %.1f %.2f %.2f %d %d %d %d %d %d %d %d\n",
           service.time_sec(),
           motorVelocity[0], motorVelocity[1], wheelVelocity[0], wheelVelocity[1],
           errCntA[0][0], errCntA[0][1], errCntA[1][0], errCntA[1][1],
           errCntB[0][0], errCntB[0][1], errCntB[1][0], errCntB[1][1]);
  usb.send(s);
}

void UEncoder::tick()
{ // Update pose estimates
  tickCnt++;
  // estimate sample time - average over about 50 samples
  uint32_t t_CPU = ARM_DWT_CYCCNT;
  uint32_t dt = t_CPU - lastSample_CPU;
  uint32_t dt_us = dt * CPU_us;
  lastSample_CPU = t_CPU;
  // filter over 50 samples (float)
  sampleTime_us = (sampleTime_us * 49 + dt_us)/50.0;
  //
  // update motor velocity (in rad/sec before gear)
  updateVelocityEstimate();
  // should not happen
  if (isnan(pose[0]) or isnan(pose[1]) or isnan(pose[2]))
  {
    clearPose();
    nanCnt++;
  }
  // update odometry pose
  updatePose(tickCnt);
  // update trip time
  tripAtime += float(dt_us) * 1e-6;
  tripBtime += float(dt_us) * 1e-6;
}

///////////////////////////////////////////////////////

void UEncoder::eePromSave()
{
  // save desired PWM FRQ
  //eeConfig.pushWord(PWMfrq);
  eeConfig.pushFloat(odoWheelRadius[0]);
  eeConfig.pushFloat(odoWheelRadius[1]);
  eeConfig.pushFloat(gear);
  eeConfig.pushWord(pulsPerRev);
  eeConfig.pushFloat(odoWheelBase);
  { // debug
//     const int MSL = 150;
//     char s[MSL];
//     snprintf(s, MSL, "# eeLoad %f %f %f %d %f\n", odoWheelRadius[0], odoWheelRadius[1],
//              gear, pulsPerRev, odoWheelBase);
//     usb.send(s);
  }
}

void UEncoder::eePromLoad()
{
  if (not eeConfig.isStringConfig())
  {
    //  PWMfrq = eeConfig.readWord();
    odoWheelRadius[0] =  eeConfig.readFloat();
    odoWheelRadius[1] =  eeConfig.readFloat();
    gear = eeConfig.readFloat();
    pulsPerRev = eeConfig.readWord();
    odoWheelBase = eeConfig.readFloat();
    { // debug
  //     const int MSL = 150;
  //     char s[MSL];
  //     snprintf(s, MSL, "# eeLoad %f %f %f %d %f\n", odoWheelRadius[0], odoWheelRadius[1],
  //              gear, pulsPerRev, odoWheelBase);
  //     usb.send(s);
    }
    if (pulsPerRev < 1)
      pulsPerRev = 1;
    if (gear < 1)
      gear = 1;
    if (odoWheelRadius[0] < 0.001)
      odoWheelRadius[0] = 0.001;
    if (odoWheelRadius[1] < 0.001)
      odoWheelRadius[1] = 0.001;
    if (odoWheelBase < 0.01)
      odoWheelBase = 0.01;
    anglePerPuls = 2.0 * M_PI / (pulsPerRev);
  }
  else
  { // skip robot specific items
    int skip = 3*4 + 2 + 4;
    eeConfig.skipAddr(skip);
  }
}

void UEncoder::clearPose()
{
  pose[0] = 0;
  pose[1] = 0;
  pose[2] = 0;
  // pose[3] = 0; NB! tilt should NOT be reset
  encoder[0] = 0;
  encoder[1] = 0;
  encoderLast[0] = 0;
  encoderLast[1] = 0;
  distance = 0.0;
  tripAreset();
  tripBreset();
}



void UEncoder::updatePose(uint32_t loop)
{
  // wheel velocity on radians per second
  wheelVelocity[0] = -motorVelocity[0] /gear;
  wheelVelocity[1] =  motorVelocity[1] /gear;
  // wheel velocity in m/s
  wheelVelocityEst[0] = wheelVelocity[0] * odoWheelRadius[0];
  wheelVelocityEst[1] = wheelVelocity[1] * odoWheelRadius[1];
  // Turnrate in rad/sec
  robotTurnrate = (wheelVelocityEst[1] - wheelVelocityEst[0])/odoWheelBase ;
  // Velocity in m/s
  robotVelocity = (wheelVelocityEst[0] + wheelVelocityEst[0])/2.0;
  //
  // sum for subscriber
  wheelVelocityEstSum[0] += wheelVelocityEst[0];
  wheelVelocityEstSum[1] += wheelVelocityEst[1];
  robotTurnrateSum += robotTurnrate;
  robotVelocitySum += robotVelocity;
  velSubscribeCnt++;
  // calculate movement and pose based on encoder count
  // encoder count is better than velocity based on time.
  // encoder position now
  uint32_t p1 = encoder[0];
  uint32_t p2 = encoder[1];
  // position change in encoder tics since last update
  int dp1 = (int32_t)p1 - (int32_t)encoderLast[0];
  int dp2 = (int32_t)p2 - (int32_t)encoderLast[1];
  // save current tick position to next time
  encoderLast[0] = p1;
  encoderLast[1] = p2;
  // position movement with forward as positive
  float d1 =  -dp1 * anglePerPuls / gear * odoWheelRadius[0];
  float d2 =   dp2 * anglePerPuls / gear * odoWheelRadius[1];
  // heading change in radians
  float dh = (d2 - d1) / odoWheelBase;
  // distance change in meters
  float ds = (d1 + d2) / 2.0;
  distance += ds;
  tripA += ds;
  tripAh += dh;
  tripB += ds;
  tripBh += dh;

  // add half the angle
  pose[2] += dh/2.0;
  // update pose position
  pose[0] += cosf(pose[2]) * ds;
  pose[1] += sinf(pose[2]) * ds;
  // add other half angle
  pose[2] += dh/2.0;
  // fold angle
  if (pose[2] > M_PI)
    pose[2] -= M_PI * 2;
  else if (pose[2] < -M_PI)
    pose[2] += M_PI * 2;

}

void UEncoder::updateVelocityEstimate()
{
  const float    one_sec_in_cpu  = F_CPU;
  const uint32_t half_sec_in_cpu = F_CPU/2;
  // motor 1 velocity
  int j = active;
  active = (j + 1) % 2;
  float velSum[MOTOR_CNT] = {0};
  int velSumCnt[MOTOR_CNT] = {0};
  float velSlowSum[MOTOR_CNT] = {0};
  int velSlowSumCnt[MOTOR_CNT] = {0};
  // angle for full period of encoder
  const float app = anglePerPuls * 4;
  //
  for (int m = 0; m < MOTOR_CNT; m++)
  { // save increment counter - debug
    for (int ab4 = 0; ab4 < 4; ab4++)
    { // just for debug
      dEncoder[m][ab4] = incrEncoder[m][j][ab4];
      // debug end
      if (incrEncoder[m][j][ab4] == 0)
      { // no increment since last
        // calculate velocity based on current time
        uint32_t dt_cpu = ARM_DWT_CYCCNT - lastTransitionTime_cpu[m][ab4];
        float v = 0;
        if (dt_cpu > 0 and dt_cpu < half_sec_in_cpu)
        { // no overload, and valid timing
          // calculate velocity based on this
          v = one_sec_in_cpu/dt_cpu;
          if (fabsf(v) * app < fabsf(motorVelocity[m]))
          { // use only if slower than last time
            // keep sign from last estimate
            if (velocityPart[m][ab4] > 0)
              velocityPart[m][ab4] = v * app;
            else
              velocityPart[m][ab4] = -v * app;
          }
        }
        else
          velocityPart[m][ab4] = 0;
        // edges used in this sample
        incrEnc[m][ab4] = 0;
        velSlowSum[m] +=  velocityPart[m][ab4];
        velSlowSumCnt[m]++;
      }
      else
      { // use time since saved transition
        uint32_t dt_cpu = transitionTime_cpu[m][j][ab4] - lastTransitionTime_cpu[m][ab4];
        float v = 0;
        if (dt_cpu > 0 and dt_cpu < half_sec_in_cpu)
        { // no overload, and valid timing
          v = one_sec_in_cpu / dt_cpu * incrEncoder[m][j][ab4] * app;
          velocityPart[m][ab4] = v;
        }
        // save new transition time as last
        lastTransitionTime_cpu[m][ab4] = transitionTime_cpu[m][j][ab4];
        // edges used in this sample
        incrEnc[m][ab4] = incrEncoder[m][j][ab4];
        // prepare for next period
        incrEncoder[m][j][ab4] = 0;
        // use this velocity
        velSum[m] += v;
        velSumCnt[m]++;
      }
    }
    if (velSumCnt[m] > 0)
      motorVelocity[m] = velSum[m] / velSumCnt[m];
    else
      motorVelocity[m] = velSlowSum[m] / velSlowSumCnt[m];
    // debug
    // const int MSL = 100;
    // char s[MSL];
    // snprintf(s, MSL, "# UpdateVel:: %g %g\r\n", motorVelocity[0], motorVelocity[1]);
    // usb.send(s);
    // debug end
  }
}

#ifdef USE_SPI_PINS
// debug use of SPI pins
int p11 = 0;
int p12a = 0;
int p12b = 0;
#endif

void UEncoder::encoderInterrupt(int m, bool encA)
{ // get interrupt timing
  uint32_t edge_cpu = ARM_DWT_CYCCNT;
  uint8_t pA, pB;
  // get encoder values for this motor
  pA = digitalReadFast(encApin[m]);
  pB = digitalReadFast(encBpin[m]);
  bool ccv;
  bool err = false;
  // edge index: A-up = 0, A-down = 1, B-up = 2, B-down = 3
  int ab4;
#ifdef USE_SPI_PINS
  if (m == 0)
    digitalWriteFast(11, p11++ % 2);
  else
  {
    if (encA)
      digitalWriteFast(12, p12a++ % 2);
    else
      digitalWriteFast(7, p12b++ % 2);
  }
#endif
  if (encA)
  { // encode pin A interrupt
    ccv = pA == pB;
    err = pA == lastA[m];
    lastA[m] = pA;
    if (err)
      errCntA[m][pA]++;
    if (pA)
      ab4 = 0;
    else
      ab4 = 1;
  }
  else
  { // encode pin B interrupt
    ccv = pA != pB;
    err = pB == lastB[m];
    lastB[m] = pB;
    if (err)
      errCntB[m][pB]++;
    if (pA)
      ab4 = 2;
    else
      ab4 = 3;
  }
  if (err)
  { // this was a spurious interrupt, ignore
    // encoder value didn't change
    return;
  }
  // use this set of data to save values
  int j = active;
  if (ccv)
  {
    encoder[m]--;
    // and within sample period
    incrEncoder[m][j][ab4]--;
  }
  else
  {
    encoder[m]++;
    // and within sample period
    incrEncoder[m][j][ab4]++;
  }
  transitionTime_cpu[m][j][ab4] = edge_cpu;
}



//////////////////////////////////////////////////////////////

void m1EncoderA()
{ // motor 1 encoder A change
    encoder.encoderInterrupt(0,true);
    encoder.intCnt++;
//   // get timestamp now
}

void m2EncoderA()
{ // motor 2 encoder A
    encoder.encoderInterrupt(1, true);
    encoder.intCnt++;
}

void m1EncoderB()
{ // motor 1 encoder pin B
    encoder.encoderInterrupt(0, false);
    encoder.intCnt++;
}

void m2EncoderB()
{ // motor 2 encoder pin B
    encoder.encoderInterrupt(1, false);
    encoder.intCnt++;
}
