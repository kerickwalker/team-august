 /***************************************************************************
 * 
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
 
#pragma once

class ULead
{
public:
  /**
   * a first order Pole-zero transfer function.
   * \param tauZ is the zero time constant (in seconds)
   * \param tauP is the pole time constant (in seconds)
   * \param sampleTime  is sample time in seconds */
  void setup(float tauZ, float tauP, float sampleTime);
  /**
   * calculate transfer function output for this sample */
  float tick(float x);
  /**
   * Forget old values */
  void reset()
  {
    x1 = 0;
    y1 = 0;
  };
private:
  float tauP2pT; /// 2 tau-pole + sample time
  float tauP2mT; /// 2 tau-pole - sample time
  float tauZ2pT; /// 2 tau-zero + sample time
  float tauZ2mT; /// 2 tau-zero - sample time
  float x1 = 0.0; /// one sample old input
  float y1 = 0.0; /// one sample old output
};

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

class UDemo_Behave
{
public:
  /**
   * test for button pressed and released.
   * Button press in 5 seconds will turn robot off
   * so robot should not start unless
   * button is released within 5 seconds.
   * */
  bool buttonReleased();
  /**
   * update state and behaviour
   * \returns true, when this behaviour has ended */
  bool tick();
  /**
   * set motor voltage to follow line */
  void followLine(float pos);
  //
private:
  ULead lead;

  int state = 0;
  float endTime = 0;
  int lastState = -1;
  bool buttonPressed = false;
};


extern UDemo_Behave dbehave;

