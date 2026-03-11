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

#pragma once

#include <stdint.h>
#include "main.h"
#include "usubss.h"

class UService
{
public:
  // motor voltage
  /**
  * setup of all sensors and actuators */
  void setup();
  /**
   * Update all sensor values */
  void updateSensors();
  /**
   * implement any actuator decisions */
  void updateActuators();
  /**
   * Sample time service
   * @returns true is 'sampleTime' has passed since last */
  bool isSampleTime();
  /**
   * sample time in micro seconds.
   * normally constant, but can be set for special
   * requirements, e.g. motortest */
  uint32_t sampleTime_us;
  float sampleTime_sec()
  {
    return float(sampleTime_us)/1e6;
  }
  /**
   * function called by interrupt
   * \param dt is the time passed since last call */
  void timePassed_us(uint32_t dt);
  /**
   * Not folding system time in us ~4e9 hours
   * NB! updated at sample time only. */
  uint64_t time_us;
  inline float time_sec()
  {
    return float(time_us)*1e-6;
  }
  static void sampleTimeInterrupt();

  /**
   * Set system sample time, this is the control cycle time and should be
   * slower than the the IMU - if the IMU is needed, i.e. >= 1000 us
   * Minimum is 20us, but will work properly from about 300us */
  void setSampleTime(int32_t sampleTimeus);
  bool sampleTimeNow = false;
  bool sampleTimeHalfNow = false;
  /// time in seconds
  float time = 0.0;

private:
  /**
   * private flag for timing analysis */
  bool cycleStarted = false;
  /**
   * value for micros() to test for next sample time */
  // uint64_t nextsampleTime = 0;
  // uint64_t nextHalfsampleTime = 0;
  bool nextIsHalf = false;
};

extern UService service;

