 /***************************************************************************
 * 
 *   Copyright (C) 2022 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
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
 
#include <stdio.h>
// #include "ucontrol.h"
#include "ustate.h"
#include "ucommand.h"
#include "ueeconfig.h"
#include "ulog.h"
// #include "umotor.h"
#include "uusb.h"
// #include "uled.h"
#include "usubss.h"
#include "uad.h"
// #include "umission.h"
#include "uservo.h"
#include "umotor.h"
#include "umotortest.h"
// #include "umission.h"
#include "uimu2.h"
#include "udisplay.h"
#include "uledband.h"

// UState::UState()
// {
// }

UState state;

void UState::setup()
{ // hold power on
  pinMode ( PIN_POWER_ROBOT, OUTPUT );
  powerOn();
  pinMode ( PIN_START_BUTTON, INPUT_PULLUP ); // start switch - version 2B
  pinMode ( PIN_LED_DEBUG, OUTPUT );
  //
  // info messages
  addPublistItem("hbt", "Get time and state 'hbt time idx revision batVolt state hw'");
  addPublistItem("id",  "Get device type and name 'name type name'");
  addPublistItem("time",  "sample timing [us] 'time CPU_clk ad_us sensor_us control_us done_us T_us load_o/o'");
  addPublistItem("pin",  "Get pin value (pin is set by 'pind') 'pin pin value'");
  usb.addSubscriptionService(this);
  //
  setBatVoltageScale();  //
  // init CPU cycle counter
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  //
  // Teensy 3.2 at 92MHz is a bit too slow for 1kHz sample frequency
  // so reduce (by default) to 800Hz
  if (F_CPU < 100000000)
  {
    CONTROL_PERIOD_10us = 125;
    SAMPLETIME = (0.00001 * CONTROL_PERIOD_10us);
  }
}

void UState::setBatVoltageScale()
{
#if defined(REGBOT_HW41) || defined(REGBOT_HW63_35)
//   if (robotHWversion == 8)
//     // different resistors
//     batVoltIntToFloat = 3.3 / lpFilteredMaxADC * (15.0 + 1.2)/1.2;
//   else
    batVoltIntToFloat = 3.3 / lpFilteredMaxADC * (47.0 + 4.7)/4.7;
  // other versions use a constant definition
#endif
}

void UState::tick()
{ // safety things
  tickCnt++;
  //
  batteryVoltage = getBatteryVoltage(ad.batVoltRawAD);
  // check battery (and turn off if needed)
  batteryMonitoring();
  // read start button
  // start button is LOW when pressed
  bool b = not digitalRead(PIN_START_BUTTON);
  // prell prohibit
  if (b)
  { // butten pressed
    if (buttonCnt == 0)
    { // register press time
      pressTime = missionTime;
      buttonCnt = 50;
      // debug
//       const int MSL = 300;
//       char s[MSL];
//       snprintf(s, MSL, "# UState:: press mission %gs, pressed at %gs, press time %gs\n", missionTime, pressTime, missionTime - pressTime);
//       usb.send(s);
//       usb.send("# UState:: button pressed\n");
      // debug end
    }
    else if (buttonCnt < 50)
    {
      usb.send("# prell\n");
      buttonCnt++;
    }
  }
  else
  { // button not pressed
    if (buttonCnt == 1)
    { // Button was pressed and is now not pressed in 50 samples (50ms)
      if ((missionTime - pressTime) > 5.0)
      { // has been pressed more than 5 seconds
        // time to power off
        if (usb.usbIsUp)
          // tell Raspberry power will be off in 20 seconds
          powerOff(20.0);
        else
          // Just Teensy - do it now
          powerOff(0.3);
      }
      else
      {
        usb.send("start\n");
        usb.send("# sent start (button release)\n");
        if (batteryOff)
        {
          powerOn();
        }
      }
      button = false;
      // debug
      const int MSL = 300;
      char s[MSL];
      snprintf(s, MSL, "# UState:: mission %gs, pressed %gs, press time %gs\n", missionTime, pressTime, missionTime - pressTime);
      usb.send(s);
      // debug end
    }
    if (buttonCnt > 0)
    {
      buttonCnt--;
    }
  }

  // check for emergency stop
//   bool em = digitalReadFast(STOP_ON_LOW_BROKEN);
//   if (em == LOW)
//   { // emergency pressed
//     if (not emergencyStop)
//     {
//       oldState = control.state;
//       emergencyStop = true;
//     }
//     stop();
//   }
//   else if (emergencyStop)
//   {
//     emergencyStop = false;
//     control.state = oldState;
//   }
  if (poweringOff)
  {
    powerOffCntDown--;
    if (powerOffCntDown <= 0)
    { 
      powerOff();
      poweringOff = false;
    }
  }
  // alive
  int a = (hbTimerCnt & 0x3ff) >> 2;
  if (logger.loggerLogging())
  {
    setStatusLed ( a < 127 );
  }
  else
  {
    setStatusLed ( a < 30 );
  }
}


void UState::sendState()
{
  const int MRL = 100;
  char reply[MRL];
  /** format
   * hbt 1 : time in seconds, updated every sample time
   *     2 : device ID (probably 1)
   *     3 : software revision number - from SVN * 10 + REV_MINOR
   *     4 : Battery voltage
   *     5 : state (0 = teensy control not active)
   *     6 : hw type
   *     7 : load %
   *     8,9 : motor enabled (left,right)
   *     10 : idle loops between samples
   * */
  snprintf(reply, MRL,  "hbt %lu.%04lu %d %d %.2f %d %d %.1f 0 0 %d\n",
           tsec, tusec/100,
           deviceID,
           command.getRevisionNumber()/10, 
           state.batteryVoltage, //sensor.batteryVoltagef, 
           4, //control.controlState,
           robotHWversion,
           load * 100,
           // motor.motorEnable[0],
           // motor.motorEnable[1],
           state.idleLoops
          );
  usb.send(reply);
}

void UState::sendId()
{
  const int MRL = 100;
  char reply[MRL];
  snprintf(reply, MRL, "dname %s %s\r\n", deviceName, getRobotName());
  usb.send(reply);
}


void UState::sendHelp()
{
  const int MRL = 150;
  char reply[MRL];
  usb.send("# State settings -------\r\n");
  snprintf(reply, MRL, "# -- \tsetidx N \tSet ID to N (sets robot name) (id=%d, part of hbt).\r\n", deviceID);
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tsetid string \tSet device type to string (< 32 chars, is=%s).\r\n", deviceName);
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tsethw N \tSet hardware version (is = %d, part of hbt).\r\n", robotHWversion);
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tstop \tStop and set manual mode\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tstart \tStart (activate something)\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "# -- \toff T\tTurn off power (cuts power after T seconds)\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "# -- \tstime us\tSet sample time typically around 1000 (> 20)\r\n");
  usb.send(reply);
  usb.send(            "# -- \tpind pin v [p]\tSet pin direction v=1 output, p=1 pull up, p=-1 pull down\r\n");
  usb.send(            "# -- \tpinv pin v\tSet pin to v [0..1]\r\n");
}

bool UState::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "setidx ", 7) == 0)
  {
    const char * p1 = &buf[7];
    deviceID = strtol(p1, NULL, 10);
    if (not robotIDvalid() and deviceID != 0)
      deviceID = 0;
  }
  else if (strncmp(buf, "sethw ", 6) == 0)
  {
    const char * p1 = &buf[6];
    robotHWversion = strtol(p1, NULL, 10);
    setBatVoltageScale();
    const int MSL = 100;
    char s[MSL];
    snprintf(s, MSL, "# UState:decode  hw=%d from sethw '%s'\n", robotHWversion, p1);
    usb.send(s);
  }
  else if (strncmp(buf, "setid ", 6) == 0)
  { // set drone name
    const char * p1 = &buf[6];
    while (isSpace(*p1))
      p1++;
    if (*p1 < ' ')
    {
            deviceName[0] = '_';
            deviceName[1] = '\0';
      usb.send("# set name to nothing ('_')\n");
    }
    else
    {
      usb.send("# got new name (get with 'id')\n");
      for (int i = 0; i < MAX_NAME_LENGTH-1; i++)
      {
        if (*p1 <= ' ')
        {
                    deviceName[i] = '\0';
          break;
        }
        if (isalnum(*p1))
                    deviceName[i] = *p1;
        else
                    deviceName[i] = '_';
        p1++;
      }
            deviceName[MAX_NAME_LENGTH-1] = '\0';
    }
    if (strncasecmp(deviceName, "robobot", 7) == 0)
    { // for Robobot most functions are disabled
      // sensordata and actuators remain
      robobot = true;
    }
    else
      robobot = false;

  }
  else if (strncmp(buf, "pind ", 5) == 0)
  {
    const char * p1 = &buf[5];
    debugPin = strtol(p1, (char**)&p1, 10);
    int dir = strtol(p1, (char**)&p1, 10);
    int pud = strtol(p1, (char**)&p1, 10);
    switch (pud)
    {
      case 1:  pinMode(debugPin, INPUT_PULLUP); break;
      case -1: pinMode(debugPin, INPUT_PULLDOWN); break;
      default: pinMode(debugPin, dir); break;
    }
    pinMode(debugPin, dir);
  }
  else if (strncmp(buf, "pinv ", 5) == 0)
  {
    const char * p1 = &buf[5];
    int pin = strtol(p1, (char**)&p1, 10);
    int val = strtol(p1, (char**)&p1, 10);
    digitalWriteFast(pin, val);
  }
  else if (strncmp(buf, "halt", 4) == 0)
  { // stop all 12V power (or turn on if 12 V power is off (and switch is on))
    if (buf[4] >= ' ')
      batteryHalt = strtol(&buf[5], NULL, 10);
    else
      batteryHalt = true;
  }
  //   else if (strncmp(buf, "arm", 3) == 0)
//   {
//     motor.stopAllMotors();
//     control.state = control.STATE_ARMED;
//   }
  else if (strncmp(buf, "stop", 4) == 0)
  { // stop motors now
    missionStart = false;
    stop();
  }
  else if (strncmp(buf, "start", 5) == 0)
  { // stop motors now
//     userMission.missionStop = false;
    missionStart = true;
//     usb.send("start\n");
    usb.send("# starting\n");
  }
  else if (strncmp(buf, "off", 3) == 0)
  { // power off request
    const char * p1 = &buf[4];
    float ts = strtof(p1, nullptr);
    powerOff(ts);
  }
  else if (strncmp(buf, "stime ", 6) == 0)
  { // power off request
    const char * p1 = &buf[6];
    int32_t ts = strtol(p1, nullptr, 10);
    if (ts >= 20 and ts <= 500000)
      setSampleTime(ts);
    else
      usb.send("# sample time T out of bounds (19<=T<=500000 (us))\r\n");
  }
  else if (subscribeDecode(buf)) {}
  else
    used = false;
  return used;
}

void UState::sendData(int item)
{
  if (item == 0)
    sendState();
  else if (item == 1)
    sendId();
  else if (item == 2)
    sendTiming();
  else if (item == 3)
    sendPinValue();
}

void UState::sendPinValue()
{
  int v = digitalReadFast(debugPin);
  const int MSL = 30;
  char s[MSL];
  snprintf(s, MSL, "pin %d %d\r\n", debugPin, v);
  usb.send(s);
}

void UState::sendTiming()
{
  const int MSL = 100;
  char s[MSL];
//   const int32_t  us = F_CPU / 1000000;
  if (true)
    snprintf(s, MSL, "time %ld %ld %ld %ld %ld %ld %.1f\r\n",
           cycleTime2[0], 
           cycleTime2[1],
           cycleTime2[2],
           cycleTime2[3],
           cycleTime2[4],
           cycleTime2[5], 
           float(cycleTime2[6]) / 10.0
            );
  usb.send(s);
}

void UState::saveCycleTime()
{
  const uint32_t us = F_CPU / 1000000;
  uint32_t t0 = cycleTime[0];
  cycleTimeInterval = t0 - cycleTime2[0];
  cycleTime2[0] = t0;
  cycleTime2[1] = (cycleTime[1] - t0)/us;
  cycleTime2[2] = (cycleTime[2] - t0)/us;
  cycleTime2[3] = (cycleTime[3] - t0)/us;
  cycleTime2[4] = (cycleTime[4] - t0)/us;
  if (cycleTimeInterval > 0)
    load = float(cycleTime[4] - t0) / float(cycleTimeInterval);
  cycleTime2[5] = cycleTimeInterval / us;
  cycleTime2[6] = int(load * 1000.0);
}

void UState::setSampleTime(int32_t sampleTimeus)
{
  if (sampleTimeus < 20 or sampleTimeus > 500000)
    usb.send("# sample time T out of bounds (19<=T<=500000 (us))\r\n");
  else
  {
    CONTROL_PERIOD_10us = sampleTimeus/10;
    SAMPLETIME = (0.00001 * CONTROL_PERIOD_10us);
//     control.initControl();
    imu2.imuAvailable = 10;
  }
}


float UState::getBatteryVoltage(uint adValue)
{ /// conversion from battery voltage integer to floating point in Volts
  return adValue * batVoltIntToFloat;
}

void UState::batteryMonitoring()
{ // keep an eye on battery voltage 
  // increased to 10.2V to allow some misbalance of cells.
  const uint16_t batteryIdleVoltageInt = int(10.2 / batVoltIntToFloat);
  // - if on USB, then battery is between 0 and 3 Volts - no error
  if (batteryOff)
  { // battery may be back on
//     if (batLowCnt < 0)
//     { // wait until capacitor is discharged
//       batLowCnt++;
//       if (batLowCnt == 0)
//         // power back seen at least XX times
//         batLowCnt = 5;
//     }
//     else if (ad.batVoltRawAD >= batteryIdleVoltageInt) // and not batteryHalt)
//     { // battery is high or switch on command
//       if (batLowCnt == 0)
//       { // stop processor to save a bit more current
//         usb.send("# Power back on\r\n");
//         // turn power on if new power board is installed
//         powerOn();
// //         digitalWriteFast(PIN_POWER_ROBOT, true);
// //         batteryOff = false;
//         batteryHalt = false;
//       }
//       else
//         batLowCnt--;
//     }
  }
  else if ((ad.batVoltRawAD < batteryIdleVoltageInt and ad.batVoltRawAD > int(5.3 / batVoltIntToFloat)) or batteryHalt)
  {
    batLowCnt++;
    if (batLowCnt % 1000 == 100 and batLowCnt < 10000 )
    { // send warning first 10 seconds and stop mission
      const int MSL = 100;
      char s[MSL];
      snprintf(s, MSL, "# Battery low - going POWER OFF in %d second!\r\n", (10000 - batLowCnt) / 1000);
      usb.send(s);
//       userMission.missionStop = true;
      if (batLowCnt >= 5000)
      {
        if (servo.servoEnabled[0] or servo.servoEnabled[1] or servo.servoEnabled[2])
        { // to prohibit servo power drain while shutting down to USB power
          usb.send("# Battery low - disabling servo!\r\n");
          servo.setServoPWM(0, 0, false, 0);
          servo.setServoPWM(1, 0, false, 0);
          servo.setServoPWM(2, 0, false, 0);
#if defined(REGBOT_HW41) || defined(REGBOT_HW63_35)
          servo.setServoPWM(3, 0, false, 0);
          servo.setServoPWM(4, 0, false, 0);
#endif
        }
      }
    }
    if (batLowCnt > 10000 or batteryHalt)
    { // stop processor to save a bit more current
      if (servo.servoEnabled[0] or servo.servoEnabled[1] or servo.servoEnabled[2])
      { // to prohibit servo power drain while shutting down to USB power
        // this part effective if issuing a HALT command
        usb.send("# disabling servo!\r\n");
        servo.setServoPWM(0, 0, false, 0);
        servo.setServoPWM(1, 0, false, 0);
        servo.setServoPWM(2, 0, false, 0);
#if defined(REGBOT_HW41) || defined(REGBOT_HW63_35)
        servo.setServoPWM(3, 0, false, 0);
        servo.setServoPWM(4, 0, false, 0);
#endif
      }
      if (not batteryHalt)
        usb.send("# Battery low! (shut down all (but USB) power!)\r\n");
      // turn power off if new power board is installed
//       digitalWriteFast(PIN_POWER_ROBOT, false);
      powerOff();
      // delay for power to drop
      batLowCnt = -800;
      // stop processor - goes down to about 30mA@12V (from about 60mA@12V) with buck-boost converter
      // stopTeensy();
    }
  }
  else
    batLowCnt = 0;
}

void UState::powerOn()
{ // no warning, just off
  digitalWriteFast(PIN_POWER_ROBOT, HIGH);
  batteryOff = false;
  display.setLine(deviceName);
  usb.send("# UState:: power on\n");
}

void UState::powerOff()
{ // no warning, just off
  digitalWriteFast(PIN_POWER_ROBOT, LOW);
  batteryOff = true;
  display.setLine("Power off");
}

void UState::powerOff(float after)
{ // no warning, just off
  powerOffCntDown = 10/SAMPLETIME;
  if (after < -0.5)
    // stop count down
    poweringOff = false;
  else 
  {
    if (after > 0.001)
      powerOffCntDown = int(after/SAMPLETIME);
    else
      // default is 15 seconds
      powerOffCntDown = int(15.0/SAMPLETIME);
    poweringOff = true;
  }
  // alarm the PC, that we are powering off
  const int MSL = 50;
  char s[MSL];
  snprintf(s, MSL, "power off %g\n", after);
  usb.send(s);
  if (poweringOff)
    display.setLine(s);
  else
  { // canceled
    powerOn();
  }
}

void UState::stop()
{ // command motors to stop
  // and set state to manual
//   const int MSL = 100;
//   char s[MSL];
//   snprintf(s, MSL,"# stopping %d\n",tickCnt);
  usb.send("# stopping\n");
  //
//   userMission.stopMission();
  motor.stopAllMotors();
  motortest.motorTestRunning = false;
//   control.resetControl();
}


void UState::eePromLoad()
{
  if (not eeConfig.isStringConfig())
  {
    int ts = eeConfig.readWord(); // in 10 us unit
    setSampleTime(ts*10); // param in us
    deviceID = eeConfig.readWord();
    if (not robotIDvalid() and deviceID != 0)
      deviceID = 0;
    robotHWversion = eeConfig.readByte();
    setBatVoltageScale();
    // name
    eeConfig.readBlock(deviceName, MAX_NAME_LENGTH);
    // ensure it is zero terminated - to avoid garbage
    deviceName[MAX_NAME_LENGTH-1] = '\0';
    if (deviceName[0] == '\0')
      strncpy(deviceName, "unknown", MAX_NAME_LENGTH);
    else if (strncasecmp(deviceName, "robobot", 7) == 0)
    { // for Robobot most functions are disabled
      // sensordata and actuators remain
      robobot = true;
    }
    else
    { // make sure there is valid characters only in the name
      robobot = false;
      for (int i = 1; i < MAX_NAME_LENGTH; i++)
      {
        if (deviceName[i] != '\0' and not isalnum(deviceName[i]))
                deviceName[i] = '_';
      }
    }
    display.setLine(deviceName);
  }
  else
  { // hard coded mission should not change name and ID
    int skip = 2 + 2 + 1 + MAX_NAME_LENGTH;
    eeConfig.skipAddr(skip);
  }
}

void UState::eePromSave()
{
  eeConfig.pushWord(CONTROL_PERIOD_10us);
  eeConfig.pushWord(deviceID);
  eeConfig.pushByte(robotHWversion);
  eeConfig.pushBlock(deviceName, MAX_NAME_LENGTH);
}

