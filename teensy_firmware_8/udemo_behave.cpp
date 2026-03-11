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
//#include <string>
//#include <sstream>
#include "src/urobot.h"
#include "src/ucommand.h"
#include "src/ueeconfig.h"
#include "src/ulog.h"
#include "src/umotor.h"
#include "src/uusb.h"
#include "src/uservo.h"
#include "src/umotor.h"
#include "src/uimu2.h"
#include "src/udisplay.h"
#include "src/uledband.h"
#include "src/uservice.h"
#include "src/ucurrent.h"
#include "src/uencoder.h"
#include "src/ulinesensor.h"
#include "udemo_behave.h"


UDemo_Behave dbehave;


bool UDemo_Behave::buttonReleased()
{ // pressed > 5 seconds is power off
  bool released = false;
  //Pressed makes input low
  bool pressed_now = not digitalReadFast(PIN_START_BUTTON);
  if (buttonPressed and not pressed_now and not robot.poweringOff)
    released = true;
  buttonPressed = pressed_now;
  return released;
}


bool UDemo_Behave::tick()
{ // this function is called at every sample time
  // and should never wait in a loop.
  // Update variables as needed and return.
  bool theEnd = false;
  // buffer for debug messages
  const int MSL = 200; // max string buffer length
  char s[MSL]; // 200 chars max
  //
  // this is a state machine
  // state 0: wait for start button press
  // other states are part of a sequence
  switch (state)
  { // run mission, initial value
    case 0:
      if (buttonReleased() or robot.missionStart)
      { // starting a sequence
        // make communication easy
        usb.use_CRC = false;
        robot.missionStart = false;
        // inform USB master
        usb.send("%% starting\n");
        // reset position and trip counters
        encoder.clearPose();
        // controller params
        lead.setup(0.8, 0.15, service.sampleTime_sec());
        // log every 2 ms
        logger.startLogging(2, true);
        ls.lineSensorOn = true;
        // go to next state
        state = 10;
        // do nothing for 0.1 sec
        endTime =encoder.tripBtime + 0.1;
      }
      break;
    case 10:
      // test if this state is finished
      if (encoder.tripBtime > endTime)
      { // change to next values
        motor.setMotorVoltage(2.0, 2.0);
        endTime = encoder.tripBtime + 10.0;
        state = 13;
      }
      break;
    case 11:
      // test if this state is finished
      // tripB is traveled distance in meters
      if (encoder.tripBtime > endTime  or encoder.tripB > 0.3)
      { // change to next values
        motor.setMotorVoltage(1.5, 3.0);
        endTime = encoder.tripBtime + 10.0;
        state = 12;
      }
      break;
    case 12:
      // test if this state is finished
      // tripBh is heading change in radians (positive is left)
      if (encoder.tripBtime > endTime or encoder.tripBh > M_PI/2.0)
      { // change to next values
        motor.setMotorVoltage(2, 2);
        endTime = encoder.tripBtime + 10.00;
        state = 13;
      }
      break;
    case 13:
      // test if this state is finished
      // end when line is found (more than 10 times)
      if (encoder.tripBtime > endTime or ls.lineValidCnt > 15)
      { // change to next values
        if (not ls.lineValid)
        { // no line in time, end
          motor.setMotorVoltage(0, 0);
          state = 90;
          usb.send("%% no line\n");
        }
        else
        { // set to follow line
          state = 14;
          encoder.tripBreset();
          endTime = 10.0;
          usb.send("%% following line\n");
        }
      }
      break;
    case 14:
      // let line determine motor voltage
      followLine(0);
      // end if line is lost (line not valid more than 10 times)
      if (encoder.tripBtime > endTime or ls.lineValidCnt < 5)
      {
        motor.setMotorVoltage(0, 0);
        state = 90;
        snprintf(s, MSL, "%% followed line in %f sec\n", encoder.tripBtime);
        usb.send(s);
      }
      break;
    case 90:
      // test if this state is finished and robot is stopped
      if (encoder.tripBtime > endTime or
         (fabsf(encoder.motorVelocity[0]) < 0.01 and fabsf(encoder.motorVelocity[1]) < 0.01))
      {
        state = 99;
      }
      break;
    default:
      // back to start
      state = 0;
      // or terminate
      theEnd = true;
      break;
  }
  if (state != lastState)
  { // debug print
    snprintf(s, MSL, "%% Demo behave:: state changed from %d to %d at %.4f sec\n",
             lastState, state, service.time_sec());
    usb.send(s);
    lastState = state;
  }
  if (theEnd)
  {
    motor.setMotorVoltage(0, 0);
    logger.stopLogging();
    ls.lineSensorOn = false;
  }
  return theEnd;
}

void UDemo_Behave::followLine(float pos)
{
  float e, u;
  const float kp = 0.5;
  if (ls.lineValid)
  { // line position is a value from
    // about -2 (robot too far right) to +2 (too far left)
    e = pos - ls.linePosition; // error compared to 'pos'
    //u = kp * e;   // P controller
    u = lead.tick(kp * e);  // P-Lead controller
    // get average voltage
    float avgv = (motor.motorVoltage[0] + motor.motorVoltage[1])/2;
    // adjust wheel velocity -- actually voltage,
    // as there is no velocity controller.
    // robot is too far to the right (line negative),
    // left should decrease and right increase
    motor.setMotorVoltage(avgv - u, avgv + u);
  }
}

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

void ULead::setup(float tauZ, float tauP, float sampleTime)
{
  tauP2pT = tauP*2.0 + sampleTime;
  tauP2mT = tauP*2.0 - sampleTime;
  tauZ2pT = tauZ * 2.0 + sampleTime;
  tauZ2mT = tauZ * 2.0 - sampleTime;
  //
  const int MSL = 150;
  char s[MSL];
  snprintf(s, MSL, "%% Lead: tauZ %g sec, tauP = %g sec, T = %g sec\n",
           tauZ, tauP, sampleTime);
  usb.send(s);
  snprintf(s, MSL, "%%       tauZ2pT = %g, tauZ2mT = %g, tauP2pT = %g, tauP2mT = %g\n",
           tauZ2pT, tauZ2mT, tauP2pT, tauP2mT);
  usb.send(s);
}


float ULead::tick(float x)
{
  float y = 0;
  y = (x * tauZ2pT - x1 * tauZ2mT + y1 * tauP2mT)/tauP2pT;
  x1 = x;
  y1 = y;
  return y;
}


