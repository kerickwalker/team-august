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

#include <stdio.h>
#include <WS2812Serial.h>
#include "ustate.h"
#include "ucommand.h"
#include "ueeconfig.h"
#include "uusb.h"
#include "usubss.h"
#include "umotor.h"
#include "uledband.h"
#include "uad.h"
#include "ulog.h"
#include "ulinesensor.h"
#include "uimu2.h"
#include "uirdist.h"

ULedBand ledband;

const int numled = 18;
// Usable pins:
//   Teensy LC:   1, 4, 5, 24
//   Teensy 3.2:  1, 5, 8, 10, 31   (overclock to 120 MHz for pin 8)
//   Teensy 3.5:  1, 5, 8, 10, 26, 32, 33, 48
//   Teensy 3.6:  1, 5, 8, 10, 26, 32, 33
//   Teensy 4.0:  1, 8, 14, 17, 20, 24, 29, 39
const int pin = 1;

#ifdef BAND_IS_RGBW
// separate white LED area
#define RED    0x00FF0000
#define GREEN  0x0000FF00
#define BLUE   0x000000FF
#define YELLOW 0x00FFD000
#define PINK   0x44F00080
#define ORANGE 0x00FF4200
#define WHITE  0xAA000000
byte drawingMemory[numled*4];         //  4 bytes per LED for RGBW
DMAMEM byte displayMemory[numled*16]; // 16 bytes per LED for RGBW
#else
// then it must be a RGB band
#define RED    0xFF0000
#define GREEN  0x00FF00
#define BLUE   0x0000FF
#define YELLOW 0xFFFF00
#define PINK   0xFF1088
#define ORANGE 0xE05800
#define WHITE  0xFFFFFF
byte drawingMemory[numled*3];         //  3 bytes per LED for RGB
DMAMEM byte displayMemory[numled*12]; // 12 bytes per LED for RGB
#endif

WS2812Serial leds(numled, displayMemory, drawingMemory, pin, WS2812_GRB);


void ULedBand::setup()
{
  leds.begin();
  leds.setBrightness(255); // 0=off, 255=brightest
  leds.setPixel(53, 0x10, 0x20, 0xc0);
  leds.setPixel(54, 0x10, 0x20, 0xc0);
  leds.setPixel(55, 0x10, 0x20, 0xc0);
  // info messages
  addPublistItem("leds", "get leds status (1..n)");
  //   addPublistItem("va", "Get motor current");
  usb.addSubscriptionService(this);
}


void ULedBand::tick()
{ //
  int ledN = 0;
  tickCnt++;
  //
  // alive
  int a = (tickCnt & 0x3ff) >> 2; // 0..255
  if (logger.loggerLogging())
    ledband.setPixel(ledN,a,a,0);
  else
    ledband.setPixel(ledN,0,abs(a - 128)/4,0);
  ledN++;
  //
  // average lone sensor value until used
  for (int i = 0; i < 8; i++)
  {
    line[i] += ad.adcLSH[i] - ad.adcLSL[i];;
  }
  lineCnt++;
  // IMU
  for (int i = 0; i<3; i++)
  {
    gyroSum[i] = imu2.gyro[i];
    accSum[i] = imu2.acc[i];
  }
  gyroSumCnt++;
  accSumCnt++;
  //
  if (tickCnt % 80 == 0)
  { // show line sensor result
    int v[8];
    for (int i = 0; i < 8; i++)
    {
      v[i] = line[i] / lineCnt;
      if (ls.detect[i])
        leds.setPixel(i + ledN, 0, v[i] / 20, 0);
      else
        leds.setPixel(i + ledN, 0, 0, v[i] / 20);
      line[i] = 0;
    }
    lineCnt =0;
    // IMU
    ledN += 8;
    for (int i = 0; i < 3; i++)
    {
      int v = int(gyroSum[i]/gyroSumCnt*50.0);
      gyroSum[i] = 0;
      int g;
      if (v < 0)
      {
        g = -v;
        v = 0;
      }
      else
        g = 0;
      leds.setPixel(i+ledN, g, v, 0);
    }
    ledN += 3;
    for (int i = 0; i < 3; i++)
    {
      int v = int(accSum[i]/accSumCnt*150.0);
      accSum[i] = 0;
      int g;
      if (v < 0)
      {
        g = -v;
        v = 0;
      }
      else
        g = 0;
      leds.setPixel(i+ledN, 0, v, g);
    }
    gyroSumCnt = 0;
    accSumCnt = 0;
    ledN += 3;
    // IR
    if (irdist.useDistSensor)
    {
      int a = int((1.0-irdist.irDistance[0])*200) - 50;
      if (a > 0)
        leds.setPixel(ledN++, a,0,0);
      else
        leds.setPixel(ledN++, 0,20,0);
      a = int((1.0-irdist.irDistance[1])*200) - 50;
      if (a > 0)
        leds.setPixel(ledN++, a,0,0);
      else
        leds.setPixel(ledN++, 0,20,0);
    }
    else
    {
      leds.setPixel(ledN++, 0,0,0);
      leds.setPixel(ledN++, 0,0,0);
    }
    // clear the rest
    while (ledN < numled)
      leds.setPixel(ledN++, 0,0,0);
    // show everything now
    leds.show();
  }


//   if (tickCnt % 100 == 0)
//   {
//     float bat = state.batteryVoltage;
//     int led1 = 60;
//     int ledCnt = 27;
//     float lowVoltage = 9.5;
//     float highVoltage = 20;
//     float dv_per_led = (highVoltage - lowVoltage)/ledCnt;
//     leds.setPixel(led1 + ledCnt, 0x00, 0x30, 0x0);
//     for (int i = 0; i < ledCnt; i++)
//     {
//       float v = lowVoltage + dv_per_led * i;
//       if (v < bat and bat < 10.5)
//       {
//         leds.setPixel(i + led1, 0x99, 0, 0);
//         if ((tickCnt / 200) % 2 == 0)
//           leds.setPixel(led1+ledCnt, 0x00, 0, 0);
//         else
//           leds.setPixel(led1+ledCnt, 0x99, 0, 0);
//       }
//       else if (v < bat)
//         leds.setPixel(i + led1, 0x00, 0x43, 0x25);
//       else
//         leds.setPixel(i + led1, 0x00, 0x00, 0x00);
//     }
//     leds.setPixel(led1, 0x99, 0x00, 0x00);
//     leds.show();
//   }
}


void ULedBand::sendHelp()
{
  usb.send("# Leds -------\r\n");
  usb.send("# -- \tleds N R G B\tSet LED N to this value (e.g. 44 128 25 254 (LED 44 to purple)\r\n");
  usb.send("# -- \tleda N M R G B\tSet LEDs from N through M to value (e.g. 4 6 128 25 254 (LED 4,5,6 to purple)\r\n");
}

bool ULedBand::decode(const char* cmd)
{ // no current commands
  bool found = true;
  if (strncmp(cmd, "leds ", 5) == 0)
  {
    const char * p1 = &cmd[5];
    int n,r,g,b;
    n = strtol(p1, (char**)&p1, 10);
    r = strtol(p1, (char**)&p1, 10);
    g = strtol(p1, (char**)&p1, 10);
    b = strtol(p1, (char**)&p1, 10);
    leds.setPixel(n, r, g, b);
  }
  if (strncmp(cmd, "leda ", 5) == 0)
  {
    const char * p1 = &cmd[5];
    int n,m,r,g,b;
    n = strtol(p1, (char**)&p1, 10);
    m = strtol(p1, (char**)&p1, 10);
    r = strtol(p1, (char**)&p1, 10);
    g = strtol(p1, (char**)&p1, 10);
    b = strtol(p1, (char**)&p1, 10);
    for (int i = n; i <= m; i++)
      leds.setPixel(i, r, g, b);
  }
  else
    found = false;
  return found;
}

void ULedBand::sendData(int item)
{
  if (item == 0)
    sendLedsData();
}

void ULedBand::setPixel(int n, int r, int g, int b)
{
leds.setPixel(n, r, g, b);
}


void ULedBand::sendLedsData()
{
  const int MRL = 500;
  char reply[MRL] = "# todo leds reply";
  usb.send(reply);
}


void ULedBand::eePromLoad()
{
  // deviceID = eeConfig.readWord();
}

void ULedBand::eePromSave()
{
  // eeConfig.pushWord(deviceID);
}


