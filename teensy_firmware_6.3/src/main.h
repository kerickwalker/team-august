/***************************************************************************
 *   Copyright (C) 2022 by DTU                                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   Main function for small regulation control object (regbot)
 *   build on a teensy 3.1 72MHz ARM processor MK20DX256 - or any higher,  *
 *   intended for 31300/1 Linear control 1
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

#ifndef REGBOT_MAIN_H
#define REGBOT_MAIN_H


#define MISSING_SYSCALL_NAMES

#include <core_pins.h>
//#include "pins.h"

#define REGBOT_HW41


//#define OLD_PIN_LINE_LED        32 // For version < 3
//#define OLD_PIN_START_BUTTON    11 // For version 1
#define PIN_LED_STATUS_6        0 // on version HW 6.0 and 6.3
#define PIN_LED_DEBUG           13 // (LED_BUILTIN)

// SPI interface
#define CS0                     7
#define CS1                     7
#define CS2                     7

#define PIN_LED_STATUS          6
// #define PIN_LED_DEBUG           13 // (LED_BUILTIN)
#define PIN_START_BUTTON        37
#define PIN_LINE_LED_HIGH       34
//#define PIN_LINE_LED_HIGH_6      6
#define PIN_LINE_LED_LOW        33
#define PIN_DISABLE2            51 // (not used)
#define PIN_POWER_IR            36
#define PIN_POWER_ROBOT         35
#define PIN_MUTE  		          0 // set to 0 for mute (not on HW6)
#define PIN_SUPPLY_CURRENT      A14
#define PIN_IR_RAW_1            A15
#define PIN_IR_RAW_2            A16
#define PIN_BATTERY_VOLTAGE     A17
#define PIN_LEFT_MOTOR_CURRENT  A0
#define PIN_RIGHT_MOTOR_CURRENT A1
#define PIN_LINE_SENSOR_0       A6
#define PIN_LINE_SENSOR_1       A13
#define PIN_LINE_SENSOR_2       A7
#define PIN_LINE_SENSOR_3       A12
#define PIN_LINE_SENSOR_4       A8
#define PIN_LINE_SENSOR_5       A11
#define PIN_LINE_SENSOR_6       A9
#define PIN_LINE_SENSOR_7       A10

// Motor Controller pins
#define PIN_LEFT_PWM            2
#define PIN_LEFT_DIR            3
#define PIN_RIGHT_PWM           4
#define PIN_RIGHT_DIR           5
#define PIN_LEFT_ENCODER_A      29
#define PIN_LEFT_ENCODER_B      28
#define PIN_RIGHT_ENCODER_A     31
#define PIN_RIGHT_ENCODER_B     30
#define PIN_LEFT_FAULT          38
#define PIN_RIGHT_FAULT         32
#define PIN_MOTORS_ENABLE         32
#define M1ENC_A         PIN_LEFT_ENCODER_A
#define M1ENC_B         PIN_LEFT_ENCODER_B
#define M2ENC_A         PIN_RIGHT_ENCODER_A
#define M2ENC_B         PIN_RIGHT_ENCODER_B
#define M1DIR           PIN_LEFT_DIR // M1IN2 - Teensy Digital 2 (direction)
#define M1PWM           PIN_LEFT_PWM // M1IN1 - Teensy Digital 3 (PWM)
#define M2DIR3          PIN_RIGHT_DIR // M2IN2 - Teensy Digital 8 (direction) hardware 3 only
#define M2PWM3          PIN_RIGHT_PWM // M2IN1 - Teensy Digital 4 (PWM) hardware 3
#define M12DIS          PIN_DISABLE2 // M1+M2 D2  enable both motors - hardware 3 only
#define M1DIS1          4 // M1D2  - Teensy Digital 4 (disable (hiz))) - hardware < 3
#define M2DIS1          10 // M2D2  - Teensy Digital 10 (disable (hi-z)) - hardware < 3
#define M2PWM1          9 // M2IN1 - Teensy Digital 9 (PWM) hardware < 3
//#define SLEW            7 // SLEW  - Teensy Digital 7  -  hardware 2, 5 only, hardware 3 fixed high
#define M2DIR1          8 // M2IN2 - Teensy Digital 8 (direction) hardware < 3 only

// Servo pins
// #ifdef REGBOT_HW41
#define PIN_SERVO1      10
#define PIN_SERVO2       9
#define PIN_SERVO3       8
//#define PIN_SERVO4       7

// Global variables

extern volatile uint32_t hb10us;     /// heartbeat timer count (10 us)
extern volatile uint32_t hbTimerCnt; /// sample time count - typically ms (not assumed to overflow)
extern volatile uint32_t tsec;  /// time that will not overrun
extern volatile uint32_t tusec; /// time that will stay within [0...999999]
extern float missionTime;

const char * getRevisionString();



#endif
