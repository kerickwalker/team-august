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

#ifndef UENCODER_H
#define UENCODER_H

#include <stdint.h>
#include <math.h>
#include "main.h"
#include "usubss.h"

/**
 * interrupt for motor encoder */
void m1EncoderA();
void m2EncoderA();
void m1EncoderB();
void m2EncoderB();


class UEncoder : public USubss
{
public:
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
   * sample update */
  void tick();
  /**
   * save configuration to (EE)disk */
  void eePromSave();
  /**
   * load configuration from EE-prom */
  void eePromLoad();
  /**
   * encoder status */
  void sendEncStatus();
  /**
   * Send robot config,
   * wheel radius (x2), gear ratio, encoder tics, wheelbase */
  void sendRobotConfig();
  
  /**
   * estimated pose status */
  void sendPose();
  /**
   * estimated pose status */
  void sendVelocity();
  void sendMotorVelocity();
  void sendEncoderErrors();
  /**
   * combine wheel position to a robot pose */
  void updatePose(uint32_t loop);

  /**
   * Estimate motor velocity from time between encoder ticks.
   * Uses CPU clock counter for more accurate timing.
   * Note: difference in encoder magnet size makes estimate noisy */
  void updateVelocityEstimate();

  
  void encoderInterrupt(int m, bool a);
  /**
   * clear pose and distance traveled. */
  void clearPose();
  
protected:
  /**
   * send data to subscriber or requester over USB 
   * @param item is the item number corresponding to the added subscription during setup. */
  void sendData(int item) override;  
  

public:
  static const int MOTOR_CNT = 2;
  const int encApin[MOTOR_CNT] = {M1ENC_A, M2ENC_A};
  const int encBpin[MOTOR_CNT] = {M1ENC_B, M2ENC_B};

  bool encCCV[MOTOR_CNT];
  uint32_t encStartTime_cpu[MOTOR_CNT];
  bool encTimeOverload_cpu[MOTOR_CNT];
  uint32_t encoder[MOTOR_CNT];
  uint32_t encPeriod_cpu[MOTOR_CNT];
  
  /// estimated velocity - same as wheelVelocity times wheel radius
  float wheelVelocityEst[MOTOR_CNT] = {0, 0}; // in meter per second
  float pose[4] = {0,0,0,0}; // x,y,h,tilt
  float distance = 0.0; // distance in meters (forward) reverse is negative.
  float tripA = 0;
  float tripB = 0;
  float tripAh = 0;
  float tripBh = 0;
  float tripAtime = 0;
  float tripBtime = 0;
  void inline tripAreset()
  {
    tripA = 0;
    tripAh = 0;
    tripAtime = 0;
  };
  void inline tripBreset()
  {
    tripB = 0;
    tripBh = 0;
    tripBtime = 0;
  };
  //float robot_delta_velocity = 0; // velocity difference between wheels
  float robotTurnrate = 0.0; // is 1/turn_radius
  float robotVelocity = 0; // linear velocity
  //
  //
  float odoWheelRadius[MOTOR_CNT] = {0.075, 0.075}; // {0.19/2.0, 0.19/2.0};
  //
  float gear = 19; // 9.68 (regbot), RoboBot: 10.0 or 18 or 19 or 30.0
  float odoWheelBase = 0.234; // distance between wheels (regbot ~15cm)
  //
  int intCnt = 0; // debug
  uint16_t pulsPerRev = 68; // using all edges (48 (regbot) or 68 (robobot))

  // including gear and wheel radius
  float wheelVelocity[MOTOR_CNT];
  /** Velocity of motor in radians per second */
  float motorVelocity[MOTOR_CNT];

private:
  int tickCnt = 0;
  // 1ms = frq/12bit
  static const int max_pwm = 4095; // 12 bit
  /// pwm value to give 1ms
  int msPulseCount;
  /// pwm at center position 1.5ms for signed or 1ms for one way only (trust)
  int pulseCountOffset;
  uint32_t encoderLast[2];
  //
  int dEncoder[MOTOR_CNT][4];
  uint32_t lastEncoder[MOTOR_CNT];
  // set by interrupt
  int incrEncoder[MOTOR_CNT][2][4];
  // for print only
  int incrEnc[MOTOR_CNT][4];
  uint32_t transitionTime_cpu[MOTOR_CNT][2][4];
  // active set written by interrupt
  int active = 0;
  // saved value set by tick
  uint32_t lastTransitionTime_cpu[MOTOR_CNT][4];
  // velocity estimate for each A up+down, B up+down
  float velocityPart[MOTOR_CNT][4];
  // angle for one pulse in motor angle (before gear)
  float anglePerPuls = 2.0 * M_PI / (pulsPerRev);
  // debug
  static const int MDV = 100;
  uint8_t eport[MDV][4];
  float edt[MDV];
  int eportCnt = 0;
  int nanCnt = 0;
  // constant factor to get from dt in CPU clocks to us
  const float CPU_us = 1000000.0/float(F_CPU);
  // sample time in micro seconds
  float sampleTime_us = 1000;
  // last sample time (in CPU clocks)
  uint32_t lastSample_CPU = 0;
  // subscribtion average
  float wheelVelocityEstSum[2] = {0};
  float robotTurnrateSum = 0;
  float robotVelocitySum = 0;
  int velSubscribeCnt = 0;
private:
  uint32_t missionStart;
  bool lastA[MOTOR_CNT];
  bool lastB[MOTOR_CNT];
public:
  /** should be 1 or 0 - typically one error at startup */
  int errCntA[MOTOR_CNT][2] = {{0}};
  int errCntB[MOTOR_CNT][2] = {{0}};
};

extern UEncoder encoder;

#endif
