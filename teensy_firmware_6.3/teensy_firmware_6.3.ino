/***************************************************************************
*   Copyright (C) 2019-2024 by DTU                             *
*   jcan@dtu.dk                                                    *
*
*   Base Teensy firmware - simplified version from Regbot most control moved to Raspberry Pi.
*   build for Teensy 4.1,
*   intended for 34755 (high level version)
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

#define REV_ID "$Id: teensy_firmware_6.3.ino 1182 2025-03-27 18:18:29Z jcan $"
// #define REV_MINOR 4


#include <malloc.h>
#include <IntervalTimer.h>
#include "src/main.h"
#include "src/ulog.h"
// #include "src/umission.h"
#include "src/ulinesensor.h"
#include "src/ueeconfig.h"
//#include "src/wifi8266.h"
#include "src/uservo.h"
// #include "src/usound.h"

#include "src/uusb.h"
#include "src/ustate.h"
#include "src/uencoder.h"
#include "src/umotor.h"
#include "src/umotortest.h"
#include "src/uad.h"
#include "src/ucurrent.h"
#include "src/uirdist.h"
#include "src/uimu2.h"
#include "src/udisplay.h"
#include "src/uasenc.h"
#include "src/uledband.h"



// main heartbeat timer to service source data and control loop interval
IntervalTimer hbTimer;
// heart beat timer
volatile uint32_t hbTimerCnt = 0; /// heart beat timer count (control_period - typically 1ms)
volatile uint32_t hb10us = 0;     /// heart beat timer count (10 us)
volatile uint32_t tsec = 0; /// time that will not overrun
volatile uint32_t tusec = 0; /// time that will stay within [0...999999]
float missionTime = 0; // time in seconds
// flag for start of new control period
volatile bool startNewCycle = false;
// Heart beat interrupt service routine
void hbIsr ( void );
///
const char * getRevisionString()
{
  const char * p1 = strstr(REV_ID, ".ino");
  if (p1 == nullptr)
    p1 = REV_ID;
  return p1; //strtol(&p1[4], NULL, 10) * 10 + REV_MINOR;
}


// ////////////////////////////////////////

void setup()   // INITIALIZATION
{
  digitalWriteFast(PIN_LED_DEBUG, HIGH);
  state.setStatusLed(HIGH);
  state.setup();
  ad.setup();
  usb.setup();
  command.setup();
  encoder.setup();
  ls.setup();
  irdist.setup();
//   userMission.setup();
//   control.setup();
  imu2.setup();
//   usbhost.setup();
  // start 10us timer (heartbeat timer)
  hbTimer.begin ( hbIsr, ( unsigned int ) 10 ); // heartbeat timer, value in usec
  // data logger init
  logger.setup();
  logger.setLogFlagDefault();
  logger.initLogStructure ( 100000 / state.CONTROL_PERIOD_10us );
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
  // NB! PIN_LED_DEBUG is used as clock for SPI interface
  // NB2! AS5147 interface/SPI fail to work - disabled in uasenc.cpp line 55.
  asenc.setup();
  display.setup();
  ledband.setup();
  // start heartbeat to USB
  state.decode("sub hbt 400\n");
  state.setStatusLed(LOW);
}

int debugSaved = 0;
/**
* Main loop
* primarily for initialization,
* non-real time services and
* synchronisation with heartbeat.*/
void loop ( void )
{
//   control.resetControl();
  bool cycleStarted = false;
  state.setStatusLed(LOW);
  // - listen for incoming commands
  //   and at regular intervals (1ms)
  // - read sensors,
  // - run control
  // - implement on actuators
  // - do data logging
  int loops = 0;
  while ( true ) 
  { // main loop
    loops++;
    usb.tick(); // service commands from USB
    // startNewCycle is set by 10us timer interrupt every 1 ms
    if (startNewCycle ) // start of new control cycle
    { // error detect
      startNewCycle = false;
      cycleStarted = true;
      // AD converter should start as soon as possible, to also get a reading at half time
      // values are not assumed to change faster than this
      ad.tick();
      // state.timing is to get some statistics on which part uses the CPU time
      state.timing(1);
      // estimate velocity and pose
      encoder.tick();
      // calculate motor current
      current.tick();
      // net new acc/gyro measurements
      //if (not motortest.motorTestRunning)
      {
        imu2.tick();
        asenc.tick();
      }
      // record read sensor time
      state.timing(2);
      // calculate sensor-related values
      // process line sensor readings and
      // estimate line edge posiitons
      ls.tick();
      // distance sensor (sharp sensor)
      irdist.tick();
      // advance mission
//       userMission.tick();
      // do control
//       control.tick();
      // Implement on actuators
      servo.tick();
      motor.tick();
      // optional, summarize for motor parameter estimate
      motortest.tick();
      // monitor robot state
      state.tick();
      // record read sensor time + control time
      state.timing(3);
      // non-critical functions
      // save selected log data to RAM buffer
      logger.tick();
      state.idleLoops = loops;
      loops = 0;
      // update display
      if (not motortest.motorTestRunning)
      {
        display.tick();
      }
      ledband.tick();
      // service USB-host plug
//       usbhost.tick();
    }
    // loop end time
    if (cycleStarted)
    { // mostly timing (total sample timing)
      state.timing(4);
      state.saveCycleTime();
      cycleStarted = false;
    }
  }
}

/**
* Heartbeat interrupt routine
* schedules data collection and control loop timing.
* */
void hbIsr ( void ) // called every 10 microsecond
{ // as basis for all timing
  hb10us++;
  tusec += 10;
  if (tusec > 1000000)
  {
    tsec++;
    tusec = 0;
  }
  if (motortest.motorTestRunning)
  { // reduced sample time for motortest
    if (hb10us % motortest.sample_time_10us == 0) // reduced control period start
    { // time to start new processing sample
      missionTime += 1e-5 * motortest.sample_time_10us;
      hbTimerCnt++;
      startNewCycle = true;
      state.timing(0);
    }
  }
  else
  { // normal full sample time
    if (hb10us % state.CONTROL_PERIOD_10us == 0) // main control period start
    { // time to start new processing sample
      missionTime += 1e-5 * state.CONTROL_PERIOD_10us;
      hbTimerCnt++;
      startNewCycle = true;
      state.timing(0);
    }
    if ( int(hb10us % state.CONTROL_PERIOD_10us) == state.CONTROL_PERIOD_10us/2 ) // start half-time ad conversion
    { // Time to read a LEDs off value (and turn LEDs on for next sample)
      ad.tickHalfTime();
    }
  }
}

/////////////////////////////////////////////////////////////////

