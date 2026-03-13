 /***************************************************************************
  *   Copyright (C) 2019-2022 by DTU                             *
  *   jca@elektro.dtu.dk                                                    *
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
 
//#include <core_pins.h>
// #include "udrive.h"
#include "uasenc.h"
#include "ueeconfig.h"
#include "urobot.h"


UAsEnc asenc;


void UAsEnc::setup()
{
  initEnc();
  // subscription
  addPublistItem("asens",  "Get encoder status 'asenc s1 s2 s3' 1=available");
  addPublistItem("aseno",  "Get encoder offset 'aseno o1 o2 o3' in degrees");
  addPublistItem("asenp",  "Get encoder values 'asenv p1 p2 p3' (angle in degrees)");
  addPublistItem("asenv",  "Get encoder values 'asenv v1 v2 v3' (velocity in degrees/s)");
  addPublistItem("asene",  "Get encoder error flags 'asene e1 e2 e3' (Hex, see AS5147E doc)");

  addPublistItem("as16",   "Get register values '#reg16 interface, reg, value'");
  usb.addSubscriptionService(this);
}

uint16_t UAsEnc::initEnc()
{
  ReadDataFrame readDataFrame{0};
  if (encs[0] == nullptr)
    encs[0] = new AS5X47{CS0};
//   if (AS_CNT > 1 and encs[1] == nullptr)
//     encs[1] = new AS5X47{CS1};
//   if (AS_CNT > 2 and encs[2] == nullptr)
//     encs[2] = new AS5X47{CS2};
  for (int i = 0; i < AS_CNT; i++)
  {
    // ERROR SPI interface to AS5147 fail to work 18 may 2024 //
    // failed to find error
    // disabled next line
    readDataFrame = encs[i]->readRegister(DIAG_REG);
    asencValid[i] = readDataFrame.raw > 0;
    encErr[i].raw = 0;
  }
  return readDataFrame.raw;
}


bool UAsEnc::decode(const char* cmd)
{
  bool found = true;
  if (strncmp(cmd, "aseof ", 6) == 0)
  {
    const char * p1 = &cmd[6];
    for (int i = 0; i < AS_CNT; i++)
    {
      encOffset[i] = strtof(p1, (char**)&p1);
    }
  }
  else if (strncmp(cmd, "asi ", 4) == 0)
  {
    const char * p1 = &cmd[4];
    unsigned int i = strtol(p1, (char**)&p1, 10);
    if (i < AS_CNT)
      encs[i]->printDebugString();
    else
      usb.send("# not a valid device\n");
  }
  else if (strncmp(cmd, "as16 ", 5) == 0)
  {
    const char * p1 = &cmd[5];
    reg16Enabled = strtol(p1, (char**)&p1, 10);
    reg16Interface = strtol(p1, (char**)&p1, 10);
    reg16 = strtol(p1, (char**)&p1, 16);
    const int MSL = 100;
    char s[MSL];
    snprintf(s, MSL, "# reg16 interface %d reg %x\r\n", reg16Interface, reg16);
    usb.send(s);
    ReadDataFrame aa;
    aa.values.data=0x7;
    aa.values.ef = 1;
    aa.values.pard = 0;
    snprintf(s, MSL, "# pack test 0x4007 should be = %x\r\n", aa.raw);
    usb.send(s);
  }
  else
    found = false;
  return found;
}

void UAsEnc::sendHelp()
{
  const int MRL = 300;
  char reply[MRL];
  usb.send("# AS5x47 encoder -------\r\n");
  // Settings
  usb.send("# -- \tasi N \tInitialize SPI channel N [0..2]\r\n");
  snprintf(reply, MRL, "# -- \tencof o1 o2 o3 \tSet encoder offset\r\n (is=%d, %d, %d)\r\n",
           encOffset[0], 0, 0);
  usb.send(reply);
  usb.send("# -- \tas16 e i R \tGet register R value from interface i, disable if e=0 (use as16i to see)\r\n");
}

void UAsEnc::tick()
{ // read data - first time will fail
  tickCnt++;
  for (int i = 0; i < AS_CNT; i++)
  {
    if (asencValid[i])
    {
      encPos[i] = encs[i]->readAngle();
      encVel[i] = encs[i]->readVel();
      encErr[i] = encs[i]->readErr();
    }
  }
  if (reg16Enabled and reg16IsSend and reg16Interface < AS_CNT)
  {
    reg16Raw = encs[reg16Interface]->readRegister(reg16);
    reg16IsSend = false;
  }
}

void UAsEnc::sendData(int item)
{
  if (item == 0)
    sendStatus();
  else if (item == 1)
    sendOffset();
  else if (item == 2)
    sendPosition();
  else if (item == 3)
    sendVelocity();
  else if (item == 4)
    sendErr();
  else if (item == 5)
    sendRegister();
}


////////////////////////////////////////////////

void UAsEnc::eePromSave()
{
  uint8_t f = 1;
//   f |= useMadgwich << 1;
  eeConfig.pushByte(f);
  for (int i = 0; i < AS_CNT; i++)
    eeConfig.pushFloat(encOffset[i]);
  // accelerometer
}

void UAsEnc::eePromLoad()
{
  /*uint8_t f =*/ eeConfig.readByte();
  //useMadgwich = (f & 0x02) > 0;
  //
  for (int i = 0; i < AS_CNT; i++)
    encOffset[i] = eeConfig.readFloat();
}

////////////////////////////////////////////

void UAsEnc::sendStatus()
{
  const int MRL = 250;
  char reply[MRL];
//   int16_t * m = mpu.getMag();
  snprintf(reply, MRL, "asens %d %d %d\r\n",
           asencValid[0], 1, 2);
  usb.send(reply);
}

void UAsEnc::sendOffset()
{
  const int MRL = 250;
  char reply[MRL];
  //   int16_t * m = mpu.getMag();
  snprintf(reply, MRL, "aseno %d %d %d\r\n",
           encOffset[0], 1, 2);
  usb.send(reply);
}

void UAsEnc::sendPosition()
{
  const int MRL = 250;
  char reply[MRL];
  //   int16_t * m = mpu.getMag();
  snprintf(reply, MRL, "asenp %.2f %.2f %.2f\r\n",
           encPos[0], 1., 2.);
  usb.send(reply);
}

void UAsEnc::sendVelocity()
{
  const int MRL = 250;
  char reply[MRL];
  //   int16_t * m = mpu.getMag();
  snprintf(reply, MRL, "asenv %.1f %.1f %.1f\r\n",
           encVel[0], 1., 2.);
  usb.send(reply);
}

void UAsEnc::sendErr()
{
  const int MRL = 250;
  char reply[MRL];
  //   int16_t * m = mpu.getMag();
  snprintf(reply, MRL, "asene 0x%04x 0x%04x 0x%04x\r\n",
           encErr[0].raw, 0, 0);
  usb.send(reply);
}

void UAsEnc::sendRegister()
{
  const int MRL = 250;
  char reply[MRL];
  //   int16_t * m = mpu.getMag();
  snprintf(reply, MRL, "# AS5147U: interface %d, reg  %x = warn=%d, err=%d, data= 0x%04x\r\n",
           reg16Interface, reg16, reg16Raw.values.pard, reg16Raw.values.ef, reg16Raw.values.data);
  usb.send(reply);
  reg16IsSend = true;
}
