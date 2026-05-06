 /***************************************************************************
 * 
 *   Copyright (C) 2024 by DTU                             *
 *   jcan@dtu.dk                                                    *
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
#include "urobot.h"
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
#include "uservice.h"
#include "ucurrent.h"

// URobot::URobot()
// {
// }

URobot robot;

void URobot::setup()
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
  addPublistItem("auto",  "Get value of auto mission  start flag 'start value' value=[0,1].");
  usb.addSubscriptionService(this);
  //
  setBatVoltageScale();  //
  // init CPU cycle counter
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}

void URobot::setBatVoltageScale()
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

bool decrease(int  v[3])
{
  bool counting = true;
  if (v[0] > 0)
    v[0]--;
  else if (v[1] > 0)
    v[1]--;
  else if (v[2] > 0)
    v[2]--;
  else
    counting = false;
  return counting;
}


void URobot::tick()
{ // safety things and timing
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
      pressTime = service.time_sec();
      buttonCnt = 50;
    }
    else if (buttonCnt < 20)
    {
      // usb.send("# start button prell\n");
      buttonCnt++;
    }
    if ((service.time_sec() - pressTime) > 5.0)
    { // has been pressed more than 5 seconds
      // time to power off
      if (usb.usbIsUp)
        // tell Raspberry power will be off in 20 seconds
        powerOff(40.0);
      else
        // Just Teensy - do it now
        powerOff(0.00003);
    }
  }
  else
  { // button not pressed
    if (buttonCnt == 1 and not poweringOff)
    { // Button was pressed and is now not pressed in 50 samples (50ms)
      if (usb.use_CRC)
      {
        usb.send("start\r\n");
        usb.send("# sent start (button release)\r\n");
      }
      if (batteryOff)
      {
        powerOn();
      }
      // debug
      if (false)
      {
        const int MSL = 300;
        char s[MSL];
        snprintf(s, MSL, "# URobot:: start %gs, pressed %gs, press time %gs\r\n", service.time_sec(), pressTime, service.time_sec() - pressTime);
        usb.send(s);
      }
      // debug end
    }
    if (buttonCnt > 0)
    {
      buttonCnt--;
    }
  }
  if (poweringOff)
  {
    int tickPerSec = 1e6 / service.sampleTime_us;
    int n = powerOffCntMax / 900;
    if (powerOffCntDown == powerOffCntMax)
    {
      for (int i = 0; i < 3; i++)
      {
        r1[i] = 100;
        r2[i] = 100;
        r3[i] = 100;
      }
    }
    else if (powerOffCntDown % n == 0)
    {
      if (decrease(r3))
        {}
      else if (decrease(r2))
        {}
      else
        decrease(r1);
      if (true)
      {
        const int MSL = 150;
        char s[MSL];
        snprintf(s, MSL, "# Shut down %d %d %d (count=%d)\n", r1[0], r2[0], r3[0], powerOffCntDown);
        usb.send(s);
      }
      ledband.setPixel(14, r1[0], r1[1] , r1[2]);
      ledband.setPixel(15, r2[0], r2[1] , r2[2]);
      ledband.setPixel(16, r3[0], r3[1] , r3[2]);
    }
    powerOffCntDown--;
    if (powerOffCntDown <= 0)
    { // cut power (may continue on USB power)
      powerOff();
      poweringOff = false;
    }
    else if (powerOffCntDown % tickPerSec == 1)
    { // every 1 sec
      const int MSL = 300;
      char s[MSL];
      snprintf(s, MSL, "power off %d sec\n", powerOffCntDown / tickPerSec);
      display.setLine(s);
    }

  }
  // alive
  int a = millis() % 1000;
  if (logger.loggerLogging())
  {
    setStatusLed ( a < 500 );
  }
  else
  {
    setStatusLed ( a < 100 );
  }
  if (missionAutoStart and tickCnt == 300)
  { // autostart mission after first 300 ticks (0.3 second if sample time is 1ms)
    missionStart = true;
  }
}


void URobot::sendState()
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
   *     7 : load 
   *     8,9 : motor enabled (left,right)
   * */
  snprintf(reply, MRL,  "hbt %.4f %d %d %.2f %d %d %.1f %.2f %d\r\n",
           service.time_sec(),
           deviceID,
           command.getRevisionNumber()/10, 
           robot.batteryVoltage, //sensor.batteryVoltagef,
           4, //control.controlState,
           robotHWversion,
           load * 100,
           current.getSupplyCurrent(),
           batLowCnt
          );
  usb.send(reply);
}

void URobot::sendId()
{
  const int MRL = 100;
  char reply[MRL];
  snprintf(reply, MRL, "dname %s %s\r\n", deviceName, getRobotName());
  usb.send(reply);
}


void URobot::sendHelp()
{
  const int MRL = 150;
  char reply[MRL];
  usb.send("# Robot settings -------\r\n");
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
  snprintf(reply, MRL, "# -- \tstime us\tSet sample time typically around 1000 (> 20) is %ld\r\n", service.sampleTime_us);
  usb.send(reply);
  usb.send(            "# -- \tpind pin v [p]\tSet pin direction v=1 output, p=1 pull up, p=-1 pull down\r\n");
  usb.send(            "# -- \tpinv pin v\tSet pin to v [0..1]\r\n");
  usb.send(            "# -- \tFor all subscriptions ('sub' commands) below, 'N' is the interval in ms\r\n");
}

bool URobot::decode(const char* buf)
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
    snprintf(s, MSL, "# URobot:decode  hw=%d from sethw '%s'\r\n", robotHWversion, p1);
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
      usb.send("# set name to nothing ('_')\r\n");
    }
    else
    {
      usb.send("# got new name (get with 'id')\r\n");
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
    usb.send("# starting\r\n");
  }
  else if (strncmp(buf, "auto", 4) == 0)
  { // set mission start flag
    // NB! requires a 'save to flash'
    const char * p1 = &buf[4];
    char * p2;
    int a = strtol(p1, &p2, 10);
    if (p2 == p1)
    {
      sendAutoFlag();
    }
    else
      missionAutoStart = a != 0;
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
      service.setSampleTime(ts);
    else
      usb.send("# sample time T out of bounds (19<=T<=500000 (us))\r\n");
  }
  else if (subscribeDecode(buf)) {}
  else
    used = false;
  return used;
}

void URobot::sendData(int item)
{
  if (item == 0)
    sendState();
  else if (item == 1)
    sendId();
  else if (item == 2)
    sendTiming();
  else if (item == 3)
    sendPinValue();
  else if (item == 4)
    sendAutoFlag();
}

void URobot::sendPinValue()
{
  int v = digitalReadFast(debugPin);
  const int MSL = 30;
  char s[MSL];
  snprintf(s, MSL, "pin %d %d\r\n", debugPin, v);
  usb.send(s);
}

void URobot::sendAutoFlag()
{
  const int MSL = 30;
  char s[MSL];
  snprintf(s, MSL, "start %d\r\n", missionAutoStart);
  usb.send(s);
}

void URobot::sendTiming()
{
  const int MSL = 100;
  char s[MSL];
//   const int32_t  us = F_CPU / 1000000;
  if (true)
    snprintf(s, MSL, "time %ld %ld %ld %ld %ld %ld %ld %.1f\r\n",
           cycleTime2[0], 
           cycleTime2[1],
           cycleTime2[2],
           cycleTime2[3],
           cycleTime2[4],
           cycleTime2[7],
           cycleTime2[5],
           float(cycleTime2[6]) / 10.0
            );
  usb.send(s);
}

void URobot::saveCycleTime()
{
  const uint32_t us = 1; //F_CPU / 1000000;
  uint32_t t0 = cycleTime[0];
  cycleTimeInterval = t0 - cycleTime2[0];
  cycleTime2[0] = t0;
  cycleTime2[1] = (cycleTime[1] - t0)/us;
  cycleTime2[2] = (cycleTime[2] - t0)/us;
  cycleTime2[3] = (cycleTime[3] - t0)/us;
  cycleTime2[4] = (cycleTime[4] - t0)/us;
  cycleTime2[7] = (t0 - cycleTime[5])/us; // half cycle time is in previous sample
  load = float(cycleTime[4] - t0) / float(cycleTimeInterval);
  cycleTime2[5] = cycleTimeInterval / us;
  cycleTime2[6] = int(load * 1000.0);
}



float URobot::getBatteryVoltage(uint adValue)
{ /// conversion from battery voltage integer to floating point in Volts
  return adValue * batVoltIntToFloat;
}

void URobot::batteryMonitoring()
{ // keep an eye on battery voltage 
  // increased to 10.2V to allow some misbalance of cells.
  const uint16_t batteryIdleVoltageInt = int(10.2 / batVoltIntToFloat);
  const uint16_t batteryGoneInt = int(7 / batVoltIntToFloat);
  // - if on USB, then battery is between 0 and 5.3 Volts - no error
  if (batteryOff)
  { // battery may be back on
    if (batteryGone)
    {
      if (ad.batVoltRawAD > batteryGoneInt)
      powerOn();
    }
    else
    {
      batteryGone = ad.batVoltRawAD < batteryGoneInt;
    }
  }
  else if ((ad.batVoltRawAD < batteryIdleVoltageInt and ad.batVoltRawAD > int(5.3 / batVoltIntToFloat)) or batteryHalt)
  { // battery too low and not on USB-only power (< 5.3V)
    batLowCnt++;
    const float shutDownDelaySec = 22; // ms
    const int shutDownSamples = int(shutDownDelaySec/service.sampleTime_sec());
    if (batLowCnt % 1000 == 100 and batLowCnt < shutDownSamples )
    { // send warning first 22 seconds and stop mission
      // batLowCnt is part of HBT message
      const int MSL = 100;
      char s[MSL];
      snprintf(s, MSL, "# Battery low - going POWER OFF in %.0f second!\r\n", (shutDownSamples - batLowCnt) * service.sampleTime_sec());
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
    if (batLowCnt > shutDownSamples or batteryHalt)
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

void URobot::powerOn()
{ // no warning, just on
  digitalWriteFast(PIN_POWER_ROBOT, HIGH);
  batteryOff = false;
  batteryGone = false;
  display.setLine(deviceName);
  usb.send("# URobot:: power on\r\n");
}

void URobot::powerOff()
{ // no warning, just off
  digitalWriteFast(PIN_POWER_ROBOT, LOW);
  batteryOff = true;
  display.setLine("Power off");
}

void URobot::powerOff(float after)
{ // no warning, just off
  powerOffCntDown = 1000;
  if (after < -0.5)
  { // stop count down
    poweringOff = false;
  }
  else 
  {
    if (after > 0.001)
      powerOffCntDown = int(after/service.sampleTime_sec());
    else
      // default is 1 seconds
      powerOffCntDown = int(1.0/service.sampleTime_sec());
    poweringOff = true;
    powerOffCntMax = powerOffCntDown;
    ledband.setPixel(14, 200, 200, 200);
  }
  // alarm the PC, that we are powering off
  const int MSL = 50;
  char s[MSL];
  snprintf(s, MSL, "power off %.2f\r\n", after);
  usb.send(s);
  if (poweringOff)
    display.setLine(s);
  else
  { // canceled
    powerOn();
  }
}

void URobot::stop()
{ // command motors to stop
  // and set state to manual
  usb.send("# stopping\r\n");
  motor.stopAllMotors();
  motortest.motorTestRunning = false;
}


void URobot::eePromLoad()
{
  if (not eeConfig.isStringConfig())
  {
    uint32_t ts = eeConfig.read32(); // in 10 us unit
    service.setSampleTime(ts); // param in us
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
    // flags introduced without changing ee-flash layout.
    uint8_t flags = deviceName[MAX_NAME_LENGTH-1];
    missionAutoStart = (flags & 0x01) > 0;
    //
    display.setLine(deviceName);
  }
  else
  { // hard coded mission should not change name and ID
    int skip = 2 + 2 + 1 + MAX_NAME_LENGTH;
    eeConfig.skipAddr(skip);
  }
}

void URobot::eePromSave()
{
  eeConfig.push32(service.sampleTime_us);
  eeConfig.pushWord(deviceID);
  eeConfig.pushByte(robotHWversion);
  // flags introduced as last character in device name without changing ee-flash layout.
  // limits the name length to 30 chars (rather than 31)
  uint8_t flags = missionAutoStart & 0x01;
  deviceName[MAX_NAME_LENGTH-1] = flags;
  eeConfig.pushBlock(deviceName, MAX_NAME_LENGTH);
}

