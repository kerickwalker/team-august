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
 
#ifndef USTATE_H
#define USTATE_H

#include <stdint.h>
#include "main.h"
// #include "ucontrol.h"
#include "usubss.h"
#include "uad.h"

class URobot : public USubss
{
public:
  /**
   * constructor */
//   URobot();
  /**
   * Setup */
  void setup();
  /**
   * send help */
  void sendHelp() override;
  /**
   * decode command for this unit */
  bool decode(const char * buf) override;  
  /**
   * Checks for state change
   * to ESC values */
  void tick();
  /**
   * send state (heartbeat) */
  void sendState();
  /**
   * send sample timing */
  void sendTiming();
  /**
   * Send device name (id) */
  void sendId();
  /** send digital pin value for debug pin */
  void sendPinValue();
  void sendAutoFlag();
  /**
   * load from flash */
  void eePromLoad();
  /**
   * save to flash */
  void eePromSave();
  /**
   * Turn DC power off after a while
   * \param after is suspend tile in seconds
   * also triggers state change to shut down PC */
  void powerOff(float after);
  /**
   * Turn DC power off (now) */
  void powerOff();
  /**
   * Turn DC power on */
  void powerOn();
  /**
   * stop driving */
  void stop();
  /**
   * get robot name */
  const char * getRobotName()
  {
    return robotname[deviceID];
  }
  
  bool robotIDvalid()
  {
    return deviceID > 0 and deviceID < MAX_ROBOT_NAMES;
  }
  
  inline void timing(int item)
  {
    cycleTime[item] = micros();
  }
 
  void saveCycleTime();
  
  /**
   * Set intensity to 0=off, 1=max or 2..255 PWM intensity
   * */
  inline void setStatusLed(uint8_t value) {
    #if defined(REGBOT_HW4) || defined(REGBOT_HW41) || defined(REGBOT_HW63_35)
    int pin = PIN_LED_STATUS;
    if (robotHWversion == 9)
    { // blue board (PCB version 6.x)
      pin = PIN_LED_STATUS_6;
    }
    if (value == 1)
      // boolean value, so max intensity
      analogWrite(pin, 4095);
    else
      // use value as is - but convert to 12 bit
      analogWrite(pin, value << 3);
    #else
    digitalWriteFast(PIN_LED_DEBUG, value);
    #endif
  }
  
  float batteryVoltage;
  float getBatteryVoltage(unsigned int adValue);
  bool missionStart = false;
  bool missionAutoStart = false;
  int buttonCnt = 0;
  //
  float load;
  
protected:
  /**
   * send data to subscriber or requester over USB 
   * @param item is the item number corresponding to the added subscription during setup. */
  void sendData(int item) override;

private:
  /**
   * monitor battery voltage */
  void batteryMonitoring();
  
  const float lpFilteredMaxADC = pow(2,ad.useADCresolution) * 2;	// ADC returns 0->4095 and 2 samples added
  // assuming voltage divider uses
  //           R1 = 15 kOhm   (47k Teensy 4.1)
  //           R2 = 1.2 kOhm  (12k Teensy 4.1)
  //           AD ref = 1.2V  Teensy 3.2 and 3.5 only
  //           AD ref = 3.3V  Teensy 4.1 only
#if defined(REGBOT_HW41) || defined(REGBOT_HW63_35)
  float batVoltIntToFloat; // = 3.3 / lpFilteredMaxADC * (47.0/2 + 4.7)/4.7;
#else
  const float batVoltIntToFloat = 1.2 / lpFilteredMaxADC * (15.0 + 1.2)/1.2;
#endif
  
  
public:
  /// time in seconds (since power on)
//   float time = 0;
  /// device ID (gives also nickname)
  int deviceID = 0;
  //
  /// hardware version
  /** hw version 1 no line sensor nor wifi
   * hw 2 no power control on board, no wifi, 
   * hw 3 build in power control and wifi, and servo header - big motor controller
   * hw 4 same as 3, but small motor controller
   * hw 5 same as 2, but with sattelite power and wifi boards 
   * hw 6 is for version 4.0 (teensy 3.5) (red PCB)
   * hw 7 is for version 5.1 (teensy 4.1) (purple PCB)
   * hw 8 is for version 8.5 (teensy 4.1) (green PCB)
   * hw 9 is for version 6.2 (teensy 4.1) (blue PCB with build-in display)
   * */
  uint8_t robotHWversion = 8; // regbot (blue 6.3) 9, Robobot (green 8.5) 8
  /// and robot type
  static const int MAX_NAME_LENGTH = 32;
  char deviceName[MAX_NAME_LENGTH] = "Robobot";
  bool robobot = false;
private:
  // system timing
  uint32_t cycleTime[6] = {0}; // cycle time 0=start, 4=end
  uint32_t cycleTimeInterval = 0; // sample interval
  float pressTime = 0;
  // used on Teensy 4.1 only
  void setBatVoltageScale();
public:
  uint32_t cycleTime2[8] = {0}; // cycle time 0=start, 4=end
  bool poweringOff = false;

private:
  static const int MAX_ROBOT_NAMES = 151;
  const char * robotname[MAX_ROBOT_NAMES] = 
  { "invalid", // 0
    "Emma",
    "Sofia",
    "Ida",
    "Freja",
    "Clara", // 5
    "Laura",
    "Anna",
    "Ella",
    "Isabella",
    "Karla", // 10
    "Alma",
    "Josefine",
    "Olivia",
    "Alberte",
    "Maja", // 15
    "Sofie", // 16
    "Mathilde",
    "Agnes",
    "Lily",
    "Caroline", // 20
    "Liva",
    "Emily",
    "Sara",
    "Victoria",
    "Emilie", // 25
    "Mille",
    "Frida",
    "Marie",
    "Ellen",
    "Rosa", // 30
    "Lea",
    "Signe",
    "Filippa",
    "Julie",
    "Nora", // 35
    "Liv",
    "Vigga",
    "Nanna",
    "Naja",
    "Alba", // 40
    "Astrid",
    "Aya",
    "Asta",
    "Luna",
    "Malou", // 45
    "Esther",
    "Celina",
    "Johanne",
    "Andrea", // 49
    "Silje", // 50
    "Thea",
    "Adriana",
    "Dicte",
    "Silke",
    "Eva", // 55
    "Gry",
    "Tania",
    "Susanne",
    "Augusta",  // 59
    "Birte", // 60
    "Dagmar",
    "Leonora",
    "Nova",
    "Molly",
    "Ingrid", // 65
    "Sigrid",
    "Nicoline",
    "Tilde",
    "Europa",
    "Saga", // 70
    "Viola",
    "Emilia",
    "Cecilie",
    "Kim",
    "Clara", // 75
    "Mie",
    "Alex",
    "Melina",
    "Amanda",
    "Hannah", // 80
    "Jasmin",
    "Kaya",
    "Sally",
    "Cleo",
    "Solvej", // 85
    "Nadia",
    "Ronja",
    "Vera",
    "Mary", // 89
    "Hans", // 90
    "Joe",
    "Stub",
    "Rumle",
    "Viking",
    "Ada", // 95
    "Falk",
    "Atlas",
    "Cuba",
    "Gia", // 99
    "Alfred", // 100
    "Oscar",
    "Coco",
    "Jack",
    "William",
    "Oliver", // 105
    "Aksel",
    "Arthur",
    "Kai",
    "Juniper",
    "Noa", //110
    "Emil",
    "August",
    "June",
    "Sky",
    "Nat", // 115
    "Birdie",
    "Gandalf",
    "Hugo",
    "Newton",
    "Theo", // 120
    "Liam",
    "Bode",
    "Verdi",
    "Dot",
    "Theodor", // 125
    "Lauge",
    "Frederik",
    "Liz",
    "Anker",
    "Adam", // 130
    "Loui",
    "Storm",
    "Navy",
    "Johan",
    "Konrad", // 135
    "Flora",
    "Erik",
    "Albert",
    "Murphy", // 139
    "Mars", // 140
    "Uranus",
    "Saturn",
    "Venus",
    "Pluto",
    "Neptune", // 145
    "Titan",
    "Orion",
    "Mercury",
    "Psyche", // 149
    "Tietgen" // 150
  };
  
private:
  bool usbCtrl = false;  /// control signals from USB connection if true
  int tickCnt;
  int debugPin = 0;
  //
  static const int SUBS_CNT = 2;
  USubs * subs[SUBS_CNT] = {nullptr};
  // stop count down (ms)
  int powerOffCntDown = 0;
  int powerOffCntMax = 2;
  int r1[3], r2[3], r3[3];

  bool batteryOff = false;
  bool batteryGone = true;
  int batLowCnt = 0;
  bool batteryHalt = false;

  friend class ULog;
};




extern URobot robot;

#endif
