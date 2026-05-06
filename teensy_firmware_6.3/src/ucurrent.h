/***************************************************************************
 *   Copyright (C) 2014-2024 by DTU
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

#ifndef UCURRENT_H
#define UCURRENT_H

#include <stdint.h>
#include "main.h"
// #include "ucontrol.h"
#include "usubss.h"

class UCurrent : public USubss
{
public:
  /**
   * Setup */
  void setup();
  /**
   * send help */
  void sendHelp();
  /**
   * decode command for this unit */
  bool decode(const char * buf) override;  
  /**
   * Checks for state change
   * to ESC values */
  void tick();
  /**
   * load configuration from flash */
  void eePromLoad();
  /**
   * save current configuration to flash */
  void eePromSave();
  /**
   * get motor current for motor 0 or 1 in amps.
   * NB! no test for valid index.
   * \returns current in amps */
  float getMotorCurrentM(int m, int32_t value);
  inline float getMotorCurrentM(int m)
  { return getMotorCurrentM(m, motorCurrentMLowPass[m]); }
  void logIntervalChanged();
//   void motorDisabled();
  float motorCurrentA[3];
  int32_t motorCurrentMOffset[2] = {0};
  float getSupplyGurrent();

  
protected:
  /**
   * send data to subscriber or requester over USB 
   * @param item is the item number corresponding to the added subscription during setup. */
  void sendData(int item) override;
  
private:
  /**
   * Subscribe service */
  void sendMotorCurrent();
  /**
   * Subscribe service (motor current offset values) */
  void sendMotorCurrentOffset();
  /**
   * Send supply current */
  void sendSupplyCurrent();
  /**
   * Filtered current values (2 x motor + supply) */
  int32_t motorCurrentMLowPass[3] = {0};
  /**
   * supply current - averaged for external use */
  int32_t supplyCurrentAvg = 0;
  int supplyAvgCnt = 0;
  /**
   * Filter constant - out of 300,
   * i.e. 300 is no filter, 1 is filter over 300 samples */
  uint16_t lowPassFactor;
  /**
   * Factor from AD reading to Amps */
  float scale; // scale AD to current in amps
  float scaleSupplyCurrent;
  /// flag for calibration
  bool currentOffsetting = false;
  /** used to find offset, when motor is disabled
   * set to 300 to average over 300 samples */
  int postpondCalibration = 0;
  /**
   * debug */
  int tickCnt = 0;
};

// make class object for current estimate
extern UCurrent current;

#endif
