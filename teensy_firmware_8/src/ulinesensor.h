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


#ifndef ULINESENSOR_H
#define ULINESENSOR_H

#include "usubss.h"

class ULineSensor : public USubss
{
public:
  bool lsIsWhite = true;
  bool lsPowerHigh = true;
  bool lsTiltCompensate = false;

  /**
  * set PWM port of frekvens */
  void setup();
  /**
   * send command help */
  void sendHelp();
  /**
   * decode commands */
  bool decode(const char * buf) override;
  /**
   * update at sample time */
  void tick();

  /** save line sensor calibration */
  void eePromSaveLinesensor();
  /** load line sensor calibration */
  void eePromLoadLinesensor();

  /**
  * reset filters and stored values
  * as a follow line mission line is finished */
  // void lsPostProcess();

public:
  /* Line sensor result */
  float lsLeftSide;  // could be nice, but pt the same as linePosition
  float lsRightSide; // --"--
  int8_t crossingLineCnt;
  int8_t lineValidCnt;
  /**
   * new line detect */
  float linePosition = 0;
  bool crossing = false;
  float crossingThreshold = 0.8;
  bool lineValid = false;
  float lineValidThreshold = 0.85;
  float reflectAverage = 0;
  /**
  * Use line sensor */
  bool lineSensorOn;
  bool lineSensorIsOn = false;
  uint8_t highPowerPin = PIN_LINE_LED_HIGH;
  int16_t whiteLevel[8] = {600};
  int16_t blackLevel[8] = {0};
  float lsGain[8] = {0.0};
  bool detect[8] = {false};
  /**
   * next 2 probably not relevant */
  //float findCrossingLineVal;
  // float edgeAngle;

  
protected:
  /**
   * send subscripted data 
   * \param item is publish index (called by subscription class) */
  void sendData(int item) override;
  /**
   * line detection function */
  void lineDetect();
  /**
  * Send line sensor difference values,
  * either directly from source, or kompensated for calibration (and tilt if in balance)
  */
  void sendStatusLineSensor(bool normalized);
  /**
  * send normalize gain values */
  void sendLineSensorGain();
  void sendStatusLineSensorLimitsWhite();
  void sendStatusLineSensorLimitsBlack();
  /**
  * Send status for aAD converter values directly
  * \param idx = 1 : ADC, 2: limits, 3:values and edges, 4: gain */
  void sendADLineSensor(int8_t idx);
  /**
   * compensate for different channel-gain */
  void normalize(void);

  /**
  * Send linesensor findings */
  void sendLineSensorPosition();
  /**
   * send line sensor configuration status */
  void sendLineSensorStatus();
public:
  /**
  * difference between illuminated and not,
  * normalized, white = 1.0 */
  float lineSensorValue[8];
protected:
  float lineSensorValueSum[8] = {0};
  int lineSensorValueSumCnt = 0;

private:
  void calibrateWhiteNow();
  // AD values
  int16_t adcLSD[8] =   {600,611,622,633,644,655,666,677};
  int32_t adcLSDA[8] =   {600,611,622,633,644,655,666,677};
  int adcLSDACnt = 0;
  //
  bool swapLeftRight = false;
  //
  bool wideSensor = true; // either 6cm (false) or 10 cm wide (true)
  int calibrateWhite = 0; // requested calibrate white sum
  int32_t calibrateWhiteSum[8];// calibrate white sum
  int calibrateWhiteSumCnt = 0;// number of samples summed for calibrate white
  /// High or low power pin mode
  bool pinModeLed = INPUT;
  int tickCnt = 0;
  int lineSensorOnCnt = 0;
};

extern ULineSensor ls;

#endif
