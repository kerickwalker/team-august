/***************************************************************************
 *   Copyright (C) 2024 by DTU
 *   jcan@dtu.dk
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
#include <malloc.h>
#include <IntervalTimer.h>
#include "main.h"
#include "ulog.h"
#include "ulinesensor.h"
#include "ueeconfig.h"
#include "uservo.h"
#include "uusb.h"
#include "urobot.h"
#include "uencoder.h"
#include "umotor.h"
#include "umotortest.h"
#include "uad.h"
#include "ucurrent.h"
#include "uirdist.h"
#include "uimu2.h"
#include "udisplay.h"
#include "uasenc.h"
#include "uledband.h"
#include "uservice.h"
#include "uusbhost.h"

UService service;

IntervalTimer sampleTimer;

void UService::setup()
{ // system sample time
  sampleTime_us = 2000;
  //
  time_us = 0;
  sampleTimer.begin(service.sampleTimeInterrupt, sampleTime_us/2);
  robot.setup();
  ad.setup();
  usb.setup();
  command.setup();
  asenc.setup();
  encoder.setup();
  ls.setup();
  irdist.setup();
  imu2.setup();
  //   usbhost.setup();
  // start 10us timer (heartbeat timer)
  // data logger init
  logger.setup();
  logger.setLogFlagDefault();
  logger.initLogStructure ();
  // read configuration from EE-prom (if ever saved)
  // this overwrites the just set configuration for e.g. logger
  // if a configuration is saved
  eeConfig.setup();
  // configuration changes setup
  current.setup();
  servo.setup();  // set PWM for available servo pins
  motor.setup();  // set motor pins
  //   motortest.setup(); // done already - by ee-load?
  digitalWriteFast(PIN_LED_DEBUG, LOW);
  display.setup();
  ledband.setup();
  usbhost.setup();
  robot.setStatusLed(LOW);
}

bool UService::isSampleTime()
{
  // uint32_t us = micros();
  bool isTime = false;
  // first call after last sample time
  if (cycleStarted)
  { // mostly timing (total sample timing)
    robot.timing(4);
    robot.saveCycleTime();
    cycleStarted = false;
  }
  if (service.sampleTimeNow)
  { // update time and next sample time
    cycleStarted = true;
    isTime = true;
    service.sampleTimeNow = false;
    robot.timing(0);
  }
  else if (service.sampleTimeHalfNow)
  { // at half sample time read all AD values
    // this is needed for the line-sensor
    // to get a light-on and a light-off reading.
    service.sampleTimeHalfNow = false;
    ad.tickHalfTime();
    robot.timing(5);
  }
  else
  { // use idle time to service the USB connection
    usb.tick(); // service commands from USB
  }
  return isTime;
}

void UService::sampleTimeInterrupt()
{ // interrupt at half sample time
  // advance system time in micro seconds
  service.time_us += service.sampleTime_us/2;
  if (service.nextIsHalf)
  { // flip half sample time flag
    service.nextIsHalf = false;
    service.sampleTimeHalfNow = true;
  }
  else
  {
    service.nextIsHalf = true;
    service.sampleTimeNow = true;
  }
  service.timePassed_us(service.sampleTime_us/2);
}


void UService::setSampleTime(int32_t sampleTimeus)
{
  if (sampleTimeus < 20 or sampleTimeus > 500000)
    usb.send("# sample time T out of bounds (19<=T<=500000 (us))\r\n");
  else
  {
    sampleTime_us = sampleTimeus;
    sampleTimer.update(sampleTimeus/2);
    // CONTROL_PERIOD_10us = sampleTimeus/10;
    // SAMPLETIME = (0.00001 * CONTROL_PERIOD_10us);
    //     control.initControl();
    imu2.imuAvailable = 10;
  }
}


void UService::updateSensors()
{
  // AD converter should start as soon as possible, to also get a reading at half time
  // values are not assumed to change faster than this
  ad.tick();
  // robot.timing is to get some statistics on which part uses the CPU time
  robot.timing(1);
  // estimate velocity and pose
  encoder.tick();
  // calculate motor current
  current.tick();
  // net new acc/gyro measurements
  if (not motortest.motorTestRunning)
  {
    imu2.tick();
    asenc.tick();
  }
  // record read sensor time
  robot.timing(2);
  // calculate sensor-related values
  // process line sensor readings and
  // estimate line edge posiitons
  ls.tick();
  // distance sensor (sharp sensor)
  irdist.tick();
  //
  usbhost.tick();
}


void UService::updateActuators()
{
  servo.tick();
  motor.tick();
  // optional, summarize for motor parameter estimate
  motortest.tick();
  // monitor robot state
  robot.tick();
  // record read sensor time + control time
  robot.timing(3);
  // non-critical functions
  // save selected log data to RAM buffer
  logger.tick();
  // update display
  if (not motortest.motorTestRunning)
  {
    display.tick();
  }
  ledband.tick();
}


void UService::timePassed_us(uint32_t dt)
{
  time += float(dt)*1e-6;
}
