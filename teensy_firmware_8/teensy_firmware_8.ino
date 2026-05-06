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

#include <malloc.h>
#include <IntervalTimer.h>
#include "src/main.h"
#include "src/ulog.h"
#include "src/ulinesensor.h"
#include "src/ueeconfig.h"
#include "src/uservo.h"
#include "src/uusb.h"
#include "src/urobot.h"
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
#include "src/uservice.h"

#include "udemo_behave.h"

//#pragma message("$Id: teensy_firmware_8.ino 1033 2025-01-26 15:41:51Z jcan $")

///
const char * getRevisionString()
{
  return "$Id: teensy_firmware_8.ino 1033 2025-01-26 15:41:51Z jcan $";
}


// ////////////////////////////////////////

void setup()   // INITIALIZATION
{
  robot.setStatusLed(HIGH);
  service.setup();
}

/**
* Main loop
* */
// const int MSL = 100;
// char s[MSL];
void loop ( void )
{
  robot.setStatusLed(LOW);
  int cnt = 0;
  bool done1 = false;
  while ( true )
  { // main loop
    if (service.isSampleTime())
    {
      service.updateSensors();
      // advance mission
      done1 = dbehave.tick();
      if (done1)
        usb.send("%% finished demo 1\n");
      // implement mission actions
      service.updateActuators();
      cnt++;
    }
  }
}


