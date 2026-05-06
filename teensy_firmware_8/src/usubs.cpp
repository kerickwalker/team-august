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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "main.h"
#include "usubs.h"
#include "uusb.h"
#include "urobot.h"
#include "uservice.h"

/**
 * Workaround for linker error
 * https://forum.arduino.cc/t/arduino-due-warning-std-__throw_length_error-char-const/308515
 * */
namespace std {
  void __throw_length_error(char const*) {
  }
}

USubs::USubs(const char * key, const char * help)
{
  msgKey = key;
  helpText = help;
  keySize = strlen(msgKey);
}

bool USubs::decode(const char * keyLine, bool newSubscription)
{
  bool used = false;
  if (newSubscription)
  {
    if (strncmp(msgKey, keyLine, keySize) == 0  and keyLine[keySize] == ' ')
    {
  //     usb.send("# USubs:: set subscription\n");
      const char * p1 = &keyLine[keySize];
      int n = strtol(p1,nullptr, 10);
      // n is in ms. do not convert from ms to tics
      subN = n; //int(n/(float(service.sampleTime_us) / 1000.0));
      // // debug
      // const int MSL = 150;
      // char s[MSL];
      // snprintf(s, MSL, "# USubs:: n=%d, subN=%d, from='%s'\r\n", n, subN, p1);
      // usb.send(s);
      // // debug end
      if (n > 0 and subN == 0)
        // requested faster than sample time, so get sample time.
        subN = 1;
      used = true;
    }
  }
  else if (strncmp(msgKey, keyLine, keySize) == 0  and keyLine[keySize] == 'i')
  { // one-time request for data
    used = true;
    dataRequest = true;
  }
  return used;
}


bool USubs::tick()
{
  bool isTime = dataRequest;
  if (dataRequest)
    dataRequest = false;
  else
  { // not single request, so check for subscriptions
    uint32_t us = micros();
    int32_t dt = us - sendTime;
    if (dt < 0)
    {
      sendTime = us;
      // debug
      // const int MSL = 200;
      // char s[MSL];
      // snprintf(s, MSL, "# USubs::serviceStatus: %s: sendCnt=%d, subN=%d, dt=%ld --- micros() folded %lu, continues\n",
      //          msgKey, sendCnt, subN, (int32_t(micros() - sendTime)/1000), micros());
      // usb.send(s);
      // debug end
    }
    if (subN > 0 and (dt >= subN * 1000 or dt < 0))
    {
      // debug
      // const int MSL = 100;
      // char s[MSL];
      // snprintf(s, MSL, "# USubs:: subN=%d, us=%lu, last=%lu\r\n", subN, us, sendTime);
      // usb.send(s);
      // debug end
      isTime = true;
      sendTime = us;
      sendCnt++;
    }
  }
  return isTime;
}


void USubs::serviceStatus(int me)
{
  if (subN > 0)
  {
    const int MSL = 200;
    char s[MSL];
    snprintf(s, MSL, "# USubs::serviceStatus: %d: %s: sendCnt=%d, subN=%d, dt=%ld\n",
            me, msgKey, sendCnt, subN, (int32_t(micros() - sendTime)/1000));
    usb.send(s);
  }
}


void USubs::sendHelpLine()
{
  const int MSL = 600;
  char s[MSL];
  snprintf(s, MSL, "# -- \t%si and 'sub %s N' \t%s\r\n", msgKey, msgKey, helpText);
//   snprintf(s, MSL, "#   %si and 'sub %s N', %s\r\n", msgKey, msgKey, helpText);
  usb.send(s);
}

void USubs::sendPublishList(int & listNum)
{
  const int MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "pub %d %s %s\r\n", listNum++, msgKey, helpText);
  usb.send(s);
}
