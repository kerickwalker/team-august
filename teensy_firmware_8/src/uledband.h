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

#ifndef ULIDAR_H
#define ULIDAR_H

#include <stdint.h>
#include "main.h"
#include "usubss.h"
#include "main.h"



class ULedBand : public USubss
{
public:
  /**
   * Setup */
  void setup();
  /**
   * send help */
  void sendHelp() override;
  /**
   * decode command for this unit */
  bool decode(const char * cmd) override;
  /**
   * Checks for state change
   * to ESC values */
  void tick();
  /**
   * Set one LED */
  void setPixel(int n, int r, int g, int b);
  /**
   * load configuration from flash */
  void eePromLoad();
  /**
   * save current configuration to flash */
  void eePromSave();
private:
  // send subscribed data
  void sendData(int item) override;
  // send subscribed data
  void sendLedsData();
  //
  int ledidx = 0;
  int tickCnt = 0;
  uint32_t nextDisplayTime = 0;
  int colorIndex = 0;
  // line sensor
  float line[8] = {0};
  int lineCnt = 0;
  // IMU
  float gyroSum[3] = {0};
  int gyroSumCnt = 0;
  float minIR[2];
};

extern ULedBand ledband;

#endif
