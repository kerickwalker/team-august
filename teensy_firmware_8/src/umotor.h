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

#ifndef UMOTOR_H
#define UMOTOR_H

#include <stdint.h>
#include "main.h"
#include "usubss.h"

class UMotor : public USubss
{
public:
  // motor voltage
  static const int MOTOR_CNT = 2;
  float maxMotorVoltage = 10.0;
  float motorVoltage[MOTOR_CNT]; // actual value
  bool motorEnable[MOTOR_CNT] = {false};
  /** PWM frequency (100 kHz is probably maximum) */
  int PWMfrq = 68000; // default
  /**
   * if motor gets full power in more than 1 second, then stop */
  int overloadCount;
  bool motorPreEnabled;
  bool motorPreEnabledRestart;
  //
  bool m1ok = true;
  bool m2ok = true;
  // motor reversed
  // Pololu motors not, but big ones from China is opposite (reversed)
  bool motorReversed = false;
  /**
  * set PWM port and frequency */
  void setup();
  /**
   * send command help */
  void sendHelp();
  /**
   * decode commands */
  bool decode(const char * buf) override;
  /**
   * set one esc or pin (mostly for debug) */
  void setPWMfrq(const char * line);
  void setPWMfrq(int frq);
  /**
   * update to motor */
  void tick();
  /**
   * save configuration to (EE)disk */
  void eePromSave();
  /**
   * load configuration from EE-prom */
  void eePromLoad();
  /**
   * Set motor voltage
   * and enable motors if != 0 */
  void setMotorVoltage(float left, float right);
  /**
   * emergency stop */
  void stopAllMotors();
  /**
   * send current motor values (voltage and reference settings)
   * */
  void sendMotorValues();
  /**
   * Send motor direction and PWM (MPW in range [-4096 .. 4096]) */
  void sendMotorPWM();
  /**
   * set enable flag, and for HW 8 disable -> sleep mode for motor driver */
  void motorSetEnable(bool e1, bool e2);
  
  
protected:
  
  void sendData(int item) override;
  
  void motorSetPWM(int m1PWM, int m2PWM);
  
  void motorSetAnchorVoltage();
  
  
private:
  //
  int16_t motorAnkerPWM[MOTOR_CNT];
  int16_t motorAnkerDir[MOTOR_CNT];
  int16_t motorSleeping[MOTOR_CNT];
  int pinMotor2Dir;
  int pinMotor2Pwm;
  int tickCnt = 0;
  int setupCnt = 0;
  float MAX_PWM = 4096;
  int pwmDeadband[MOTOR_CNT] = {0};
};

extern UMotor motor;

#endif // UMOTOR_H
