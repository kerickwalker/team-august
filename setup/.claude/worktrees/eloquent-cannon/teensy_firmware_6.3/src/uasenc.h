/***************************************************************************
 *   Copyright (C) 2016 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 * Interface to IMU MPU9150 mounted on spark-fun board
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




#ifndef UASENC_H
#define UASENC_H

#include "usubss.h"
// #include "umat.h"
#include "AS5X47.h"
#include "main.h"

class ULog;

class UAsEnc : public USubss
{
public:
  // Functions
  void setup();
  bool decode(const char * cmd) override;
  void sendHelp();
  void tick();
  /**
   * save scale and offset */
  void eePromSave();
  /**
   * load scale and offset */
  void eePromLoad();

  static const int AS_CNT = 1;
  float encPos[AS_CNT];
  float encVel[AS_CNT];
protected:
  /**
   * send data to subscriber or requester over USB 
   * @param item is the item number corresponding to the added subscription during setup. */
  void sendData(int item) override;
  
private:
  uint16_t initEnc();

  void sendStatus();
  void sendPosition();
  void sendVelocity();
  void sendErr();
  void sendOffset();
  void sendRegister();


private:
  //
  //AS5X47 enc1{CS0};
//   AS5X47 enc2{CS1};
//   AS5X47 enc3{CS2};
  AS5X47 * encs[AS_CNT] = {nullptr}; //, enc2, enc3};
  uint32_t tickCnt = 0;

  bool asencValid[AS_CNT] = {false};
  int encOffset[AS_CNT] = {0};
  //
  uint16_t reg16 = 0x3FF5;
  ReadDataFrame reg16Raw = {0};
  bool reg16Enabled = false;
  unsigned int reg16Interface = 0;
  bool reg16IsSend = false;
  // AS5147U error flags
  Errfl encErr[AS_CNT];
  //
  friend class ULog;
};
  
extern UAsEnc asenc;

#endif // UIMU_H
