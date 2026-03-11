/***************************************************************************
 *   Copyright (C) 2014-2024 by DTU
 *   jcan@dtu.dk
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
#include "urobot.h"
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

#define BAND_IS_RGB

#ifdef BAND_IS_RGBW
// separate white LED area
#define RED    0x00FF0000
#define GREEN  0x0000FF00
#define BLUE   0x000000FF
#define YELLOW 0x00FFD000
#define PINK   0x44F00080
#define ORANGE 0x00FF4200
#define WHITE  0xAA000000
#define BytePerLed 4
byte drawingMemory[numled*BytePerLed];          //  4 bytes per LED for RGBW
DMAMEM byte displayMemory[numled*BytePerLed*4]; // 16 bytes per LED for RGBW
#else
// then it must be a RGB band
#define RED    0xFF0000
#define GREEN  0x00FF00
#define BLUE   0x0000FF
#define YELLOW 0xFFFF00
#define PINK   0xFF1088
#define ORANGE 0xE05800
#define WHITE  0xFFFFFF
#define BytePerLed 3
byte drawingMemory[numled*BytePerLed];         //  3 bytes per LED for RGB
DMAMEM byte displayMemory[numled*BytePerLed*4]; // 12 bytes per LED for RGB
#endif

// Create object for DMA transfer to LEDs
WS2812Serial leds(numled, displayMemory, drawingMemory, pin, WS2812_GRB);


void ULedBand::setup()
{
  leds.begin();
  leds.setBrightness(255); // 0=off, 255=brightest
  // info messages
  addPublistItem("leds", "get leds status 'leds N r g b r g b ...' (N=leds, r,g,b in hex)");
  //   addPublistItem("va", "Get motor current");
  usb.addSubscriptionService(this);
  //  clear all
  for(int i = 0; i < numled; i++)
    leds.setPixel(i, 0,0,0);
  // transfer to LEDs
  leds.show();
}


void ULedBand::tick()
{ //
  // average sensors value until used
  for (int i = 0; i < 8; i++)
  { // use normalized values
    line[i] += ls.lineSensorValue[i];
  }
  lineCnt++;
  // IMU
  for (int i = 0; i<3; i++)
  {
    gyroSum[i] = imu2.gyro[i];
  }
  gyroSumCnt++;
  //
  if (millis() > nextDisplayTime)
  { // show line sensor result
    nextDisplayTime += 80;
    // alive
    int a = abs(int((millis() % 2000) - 1000))/10; // 0..100 in 2 seconds
    if (logger.loggerLogging())
      // purple during logging
      ledband.setPixel(0,a,0,a);
    else
      // just white
      ledband.setPixel(0,a,a,a);
    //
    for (int i = 0; i < 8; i++)
    {
      if (ls.lineSensorOn)
      {
        float v = line[i] / lineCnt;
        if (v > 0.8)
          leds.setPixel(i + 1, 0, v * 100, 0);
        else
          leds.setPixel(i + 1, 0, 0, v *100);
      }
      else
        // turn off these leds
        leds.setPixel(i + 1, 0, 0, 0);
      line[i] = 0;
    }
    lineCnt = 0;
    // IMU
    for (int i = 0; i < 3; i++)
    { // gyro is in rad/sec, so multiply with a factor and truncate to int
      int v = int(gyroSum[i]/gyroSumCnt*30.0);
      if (v > 255)
        v = 255;
      else if (v < -255)
        v = -255;
      gyroSum[i] = 0;
      int g;
      if (v < 0)
      { //make negative values red
        g = -v;
        v = 0;
      }
      else
        g = 0;
      leds.setPixel(i + 9, g, v, 0);
    }
    gyroSumCnt = 0;
    // IR
    if (irdist.useDistSensor)
    {
      const float greenDist = 0.50; //m
      const float redDist = 0.13; //m
      for (int i = 0; i < 2; i++)
      {
        float a = minIR[i];
        if (a > greenDist)
        { // > 50cm
          leds.setPixel(i + 12, 0,20,0);
        }
        else if (a < redDist)
        { // show red (<13cm)
          leds.setPixel(i + 12, 40,0,0);
        }
        else
        { // from 13 to 50 show yellow
          leds.setPixel(i + 12, 20,20,0);
        }
        // const int MSL = 100;
        // char s[MSL];
        // snprintf(s, MSL, "# LEDBAND: IR values 1: %.2f, 2: %.2f m\n", minIR[0], minIR[1]);
        // usb.send(s);
        minIR[i] = 2.0;
      }
    }
    else
    { // turn IR LEDs off
      leds.setPixel(12, 0,0,0);
      leds.setPixel(13, 0,0,0);
    }
    // led from 14 and up is
    // assumed to be set by others, but shown here
    //
    // show us starting a DMA transfer, so don't call again until
    // transfer is finished (about 40ms)
    // show everything now
    leds.show();
  }
  else
  { // get minimum dIR distance between blinks
    for (int i = 0; i < 2; i++)
      if (irdist.irDistance[i] < minIR[i])
        minIR[i] = irdist.irDistance[i];
  }
}


void ULedBand::sendHelp()
{
  usb.send("# Leds -------\r\n");
  usb.send("# -- \tleds N R G B\tSet LED N to this value (e.g. 'leds 14 128 25 254' (sets LED 14 to purple))\r\n");
  usb.send("# -- \tleda N M R G B\tSet LEDs from N through M to value (e.g. 'leda 14 16 128 25 254' (sets LED 14,15,16 to purple))\r\n");
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
    // const int MSL = 100;
    // char s[MSL];
    // snprintf(s, MSL, "# set LED %d to %d %d %d\r\n", n, r, g, b);
    // usb.send(s);
  }
  else if (strncmp(cmd, "leda ", 5) == 0)
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
  const int MRL = 1500;
  char reply[MRL];
  snprintf(reply, MRL, "leds %d ", numled);
  int n = strlen(reply);
  char * p1 = &reply[n];
  for (int l = 0; l < numled; l++)
  {
    snprintf(p1, MRL - n, " %02x %02x %02x ",
             drawingMemory[l*BytePerLed+BytePerLed-1],
             drawingMemory[l*BytePerLed+BytePerLed-2],
             drawingMemory[l*BytePerLed+BytePerLed-3]);
    n += strlen(p1);
    p1 = &reply[n];
  }
  snprintf(p1, MRL - n, "\r\n");
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


