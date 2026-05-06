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
#include "umotor.h"
#include "ueeconfig.h"
#include "urobot.h"
#include "uusb.h"
// #include "ucontrol.h"
#include "usubss.h"
#include "ulog.h"
#include "uencoder.h"

UMotor motor;

void UMotor::setup()
{
  analogWriteResolution(12); /// resolution (12 bit)
  // find offset for motor current
  motorPreEnabled = true;
  //
//   motorSetEnable(0,0);
  pinMode(PIN_LEFT_IN2,OUTPUT); // motor 1 IN1 ()
  pinMode(PIN_RIGHT_IN2,OUTPUT); // motor 2 IN1 ()
  pinMode(PIN_LEFT_IN1,OUTPUT); // motor 1 IN2 ()
  pinMode(PIN_RIGHT_IN1,OUTPUT); //motor 2 IN2 ()
  motorSetPWM(0,0);
  switch (robot.robotHWversion)
  {
    case 8: // no used anymore
      pinMode(PIN_MOTORS_ENABLE,OUTPUT); // hardware 8 has common enable
      digitalWriteFast(PIN_MOTORS_ENABLE, LOW); // set to sleep
      break;
    default:
      break;
  }
  setPWMfrq(PWMfrq);
  motorSetPWM(0,0);
  //
  if (setupCnt == 0)
  { // only once
    addPublistItem("mot", "Get motor voltage 'mot m1(V) m2(V) vel_ref1(m/s) vel_ref2(m/s) reversed'");
    addPublistItem("motpwm", "Get motor direction and PWM 'motpwm dir1 pwm1 dir2 pwm2'");
    usb.addSubscriptionService(this);
  }
  setupCnt++;
}


/**
 * e2 used on hardware < 3 only */
void UMotor::motorSetEnable(bool e1, bool e2)
{
  if (motorPreEnabled and (e1 or e2) and not (motorEnable[0] or motorEnable[1]))
  { // switch off current zero offset calculation
    motorPreEnabled = false;
    motorPreEnabledRestart = true;
  }
  // reset overload
  if (e1 and not motorEnable[0])
    overloadCount = 0;
  // enable motors (or disable)
  motorEnable[0] = e1;
  motorEnable[1] = e2;
  //
  if (not motorEnable[0])
  {// disable motor - set to sleep
    digitalWriteFast(PIN_LEFT_IN2, LOW);
    digitalWriteFast(PIN_LEFT_IN1, LOW);
    motorSleeping[0] = true;
//     usb.send("# motor 1 sleep\n");    
  }
  if (not motorEnable[1])
  { // disable motor - set to sleep
    digitalWriteFast(PIN_RIGHT_IN2, LOW);
    digitalWriteFast(PIN_RIGHT_IN1, LOW);
    motorSleeping[1] = true;
//     usb.send("# motor 2 sleep\n");    
  }
  digitalWriteFast(PIN_MOTORS_ENABLE, e1 or e2); // enable is at least one is used
}


void UMotor::motorSetAnchorVoltage()
{
  float batteryNominalVoltage = robot.batteryVoltage;
  if (batteryNominalVoltage < 5.0)
    // not on battery - just for test and avoid divide by zero
    batteryNominalVoltage = 11.1;
  const float voltageLoss = 1.0; // from battery to motor driver
  float scaleFactor = MAX_PWM / (batteryNominalVoltage - voltageLoss);
  // overload check
  if (overloadCount > 500 and motorEnable[0])
  { // disable motor (after 0.5 second)
    motorSetEnable(false, false);
    //
    usb.send("# UMotor::motorSetAnchorVoltage: overload, disabled motors\n");
    //
  }
  // compensate for minimum pulsewidth
  float v1, v2;
  const float minV = 0.4; // else no PWM out of motor driver
  if (motorVoltage[0] > 0.001)
    v1 = motorVoltage[0] + minV;
  else if (motorVoltage[0] < -0.001)
    v1 = motorVoltage[0] - minV;
  else
    v1 = 0;
  if (motorVoltage[1] > 0.001)
    v2 = motorVoltage[1] + minV;
  else if (motorVoltage[1] < -0.001)
    v2 = motorVoltage[1] - minV;
  else
    v2 = 0;
  // convert to PWM values
  int w1, w2;
  if (motorReversed)
    // wrongly wired big 12V motors
    scaleFactor *= -1.0;
  //   normal motors
  w1 = int16_t(v1 * scaleFactor);
  // the right motor must run the other way
  w2 = int16_t(-v2 * scaleFactor);
  // implement
  motorSetPWM(w1, w2);
}

/**
 * allowed input is +/- 4096, where 4096 is full battery voltage
 * Translated to above and below 1024 for DRV8874 chip
 * */
void UMotor::motorSetPWM(int m1PWM, int m2PWM)
{ // PWM is 12 bit
  const int max_pwm = 4096;
  // for logging
  motorAnkerPWM[0] = max_pwm/2 + m1PWM/2;
  motorAnkerPWM[1] = max_pwm/2 + m2PWM/2;
  motorAnkerDir[0] = m1PWM >= 0; // just for debug/log
  motorAnkerDir[1] = m2PWM >= 0;
  //
  // implement
  digitalWriteFast(PIN_LEFT_IN1, motorEnable[0]);
  analogWrite(PIN_LEFT_IN2, motorAnkerPWM[0]);
  //
  digitalWriteFast(PIN_RIGHT_IN1, motorEnable[1]);
  analogWrite(PIN_RIGHT_IN2, motorAnkerPWM[1]);
}


void UMotor::sendHelp()
{
  const int MRL = 300;
  char reply[MRL];
  usb.send("# Motor -------\r\n");
  snprintf(reply, MRL, "# -- \tmotr V \tSet motor reversed; V=0 for small motors, V=1 for some big motors\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tmotv m1 m2 \tSet motor voltage -24.0..24.0 - and enable motors\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tmotfrq \tSet motor PWM frequency [100..50000], is %d\r\n", PWMfrq);
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tdeadband L R\tSet PWM deadband ([0..100%%], is left=%.1f%%, right=%.1f%%\r\n", float(pwmDeadband[0])/MAX_PWM*100.0, float(pwmDeadband[1])/MAX_PWM*100.0);
  usb.send(reply);
}


bool UMotor::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "motr", 4) == 0)
  {
    const char * p1 = &buf[4];
    // get two values - if no value, then 0 is returned
    bool rev = strtol(p1, (char**)&p1, 10);
    if (p1 == &buf[4] or rev)
      motorReversed = true;
    else
      motorReversed = false;
  }
  else if (strncmp(buf, "motv", 4) == 0)
  {
    float m1, m2;
    const char * p1 = &buf[4];
    // get two values - if no value, then 0 is returned
    m1 = strtof(p1, (char**)&p1);
    m2 = strtof(p1, (char**)&p1);
    motorVoltage[0] = m1;
    motorVoltage[1] = m2;
    if ((fabs(m1) < 0.01) and (fabs(m2) < 0.01))
      motorSetEnable(0, 0);
    else
      motorSetEnable(1,1);
    // debug
    // should be in tick() only
    motorSetAnchorVoltage();
//     usb.send("# setting motor voltage\n");
    // debug end 
  }
  else if (strncmp(buf, "motfrq ", 7) == 0)
  {
    const char * p1 = &buf[7];
    int frq = strtol(p1, nullptr, 10);
    if (frq < 100)
      frq = 100;
//     if (frq > 50000)
//       frq = 50000;
    setPWMfrq(frq);
  }
  else if (strncmp(buf, "deadband ", 9) == 0)
  {
    const char * p1 = &buf[9];
    char * p2, *p3;
    float deadbandLeft = strtof(p1, &p2);
    float deadbandRight = strtof(p2, &p3);
    if (p2 != p1)
      pwmDeadband[0] = int(deadbandLeft / 100.0 * MAX_PWM);
    if (p2 != p3)
      pwmDeadband[1] = int(deadbandRight / 100.0 * MAX_PWM);
    const int MSL = 200;
    char s[MSL];
    snprintf(s, MSL, "# PWM dead-band set to left=%.1f%%, right=%.1f%%\r\n", float(pwmDeadband[0])/MAX_PWM*100.0, float(pwmDeadband[1])/MAX_PWM*100.0);
    usb.send(s);
    usb.send("# NB! dead-band setting is not saved to flash (lost after reboot)\n");
  }
  else
    used = false;
  return used;
}

void UMotor::sendData(int item)
{
  if (item == 0)
    sendMotorValues();
  if (item == 1)
    sendMotorPWM();
}

void UMotor::sendMotorValues()
{
  const int MSL = 150;
  char s[MSL];
  snprintf(s, MSL, "mot %.2g %.2g %.3g %.4g %d\r\n", 
           motorVoltage[0], motorVoltage[1], 0.1, 0.1, motorReversed);
  usb.send(s);
}

void UMotor::sendMotorPWM()
{
  const int MSL = 150;
  char s[MSL];
  snprintf(s, MSL, "motpwm %d %d %d %d %d %d %d\r\n", 
           motorAnkerDir[0], motorAnkerPWM[0], motorAnkerDir[1], motorAnkerPWM[1], PWMfrq, m1ok, m2ok);
  usb.send(s);
}

void UMotor::setPWMfrq(const char* line)
{
  // pwmfrq 400\n
  const char * p1 = line;
  int frq = strtol(p1, (char**)&p1, 10);
  if (p1 != line)
    // valid number
    setPWMfrq(frq);
}

void UMotor::setPWMfrq(int frq)
{
  // pwmfrq 400\n
  if (frq > 150000) // limit'ish of motor driver
    PWMfrq=
    65000; // default
  else if (frq < 100)
    PWMfrq = 100;
  else
    PWMfrq = frq;
  analogWriteFrequency(PIN_LEFT_IN2, PWMfrq); /// frequency (Hz)
  analogWriteFrequency(PIN_RIGHT_IN2, PWMfrq); /// frequency (Hz)
  // debug
  if (false)
  {
    const int MSL = 130;
    char s[MSL];
    snprintf(s, MSL, "# setting motor PWM frq to %d Hz\n", PWMfrq);
    usb.send(s);
  }
}

void UMotor::tick()
{ //
  tickCnt++;
  if (fabsf(motorVoltage[0]) <= 0.01 and
      fabsf(motorVoltage[1]) <= 0.01 and
      fabsf(encoder.motorVelocity[0]) < 0.1 and
      fabsf(encoder.motorVelocity[1]) < 0.1)
  { // saves a bit of current
    motorSetEnable(false, false);
  }
  else
    motorSetAnchorVoltage();
  // set flags for o-led display
  m1ok = motorEnable[0];
  m2ok = motorEnable[1];
}

///////////////////////////////////////////////////////

void UMotor::eePromSave()
{
  // save desired PWM FRQ
  uint16_t flags = 0;
  flags |= motorReversed << 0;
  eeConfig.pushWord(flags);
  // save in kHz
  eeConfig.pushWord(PWMfrq/1000);
}

void UMotor::eePromLoad()
{
  if (not eeConfig.isStringConfig())
  {
    uint16_t flags = eeConfig.readWord();
    motorReversed = (flags & 0x01) > 0; 
    // stored value is in kHz
    PWMfrq = eeConfig.readWord() * 1000;
    // run setup to implement
    setup();
  }
  else
  { // skip robot specific items
    int skip = 2 + 2;
    eeConfig.skipAddr(skip);
  }
}

void UMotor::setMotorVoltage(float left, float right)
{ // left
  if (left > maxMotorVoltage)
    motorVoltage[0] = maxMotorVoltage;
  else if (left < -maxMotorVoltage)
    motorVoltage[0] = -maxMotorVoltage;
  else
    motorVoltage[0] = left;
  // right
  if (right > maxMotorVoltage)
    motorVoltage[1] = maxMotorVoltage;
  else if (right < -maxMotorVoltage)
    motorVoltage[1] = -maxMotorVoltage;
  else
    motorVoltage[1] = right;
  // enable ?
  if (fabsf(left) > 0.01 and fabsf(right) > 0.01)
    motorSetEnable(true, true);
}


void UMotor::stopAllMotors()
{ // 
  motorVoltage[0] = 0;
  motorVoltage[1] = 0;
  motorSetEnable(false, false);
}


