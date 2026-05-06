/***************************************************************************
 *   Copyright (C) 2014-2023 by DTU
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

#include <core_pins.h>
#include <usb_serial.h>
#include "main.h"
#include "uusb.h"
#include "ucommand.h"
#include "ulog.h"
#include "usubss.h"
#include "uservice.h"
#include "urobot.h"

UUSB usb;

void UUSB::setup()
{ // init USB connection (parameter is not used - always 12MB/s)
  Serial.begin ( 115200 ); // USB init serial
  send("# welcome - ready in a moment\r\n");
  //
  addPublistItem("usb", "Get status for USB connection 'usb time inCnt inErr serviced/sec serviceLoopCnt/sec sendFail/sec'");
  addPublistItem("ssv", "Get subscription status (as info with key and interval time)");
  addSubscriptionService(this);
}

void UUSB::tick()
{ // check for messages
  bool done = handleIncoming();
  if (not done)
  {
    for (int i = 0; i < (int)subscriptions.size(); i++)
    {
      done = subscriptions[subscribeServiceState]->subscribeService();
      if (done)
      { // all subscriptions are served here
        // ask the next subscribe service block
        subscribeServiceState++;
        if (subscribeServiceState >= (int)subscriptions.size())
        {
          subscribeServiceState = 0;
          subServiceLoops++;
        }
      }
      else
      {
        subServicedCnt++;
        break;
      }
    }
  }
  if (millis() > lastSec)
  { // a second has passed
    lastSec += 1000;
    if (usbInMsgCnt > 0)
    { // as long as there is incoming messages
      // then we should continue sending
      usbSendFailSumLast = usbSendFailSum;
      if (not usbIsUp)
      {
        // usbIsUp = true;
        usb.send("# USB is up - should never show, as handled by command decode\n");
      }
    }
    else if (usbIsUp)
    { // stop sending, if USB is lost
      // and more then N send errors has occurred.
      // reset, when something new is received.
      const int N = 10;
      if ((usbSendFailSum - usbSendFailSumLast) > N and usbInMsg0CntSecs == 0)
      { // many send errors and no incomming messages
        stopAllSubscriptions();
        usbIsUp = false;
        usbOffReason = "failed sending and no incoming";
      }
      if (usbInMsg0CntSecs > 60 and localEcho == false)
      {  // 60 seconds and no messages
        usbIsUp = false;
        usbOffReason = "too long idle";
      }
      if (not usbIsUp)
        usbOffTime = service.time_sec();
    }
    // debug
    // const int MSL = 100;
    // char s[MSL];
    // snprintf(s, MSL, "# usbUp:: msg %d, msg/s = %d, 0Cnt %d, up %d, echo=%d\n",
    //          usbInMsgCnt, usbInMsgCntInSec, usbInMsg0CntSecs, usbIsUp, localEcho);
    // send(s);
    // debug end
    if (usbInMsgCnt > 0)
    { // message received last second
      usbInMsgCntInSec = usbInMsgCnt;
      usbInMsg0CntSecs = 0;
    }
    else
    { // msg not received in the last second
      usbInMsgCntInSec = usbInMsgCnt;
      usbInMsg0CntSecs++;
    }
    usbInMsgCnt = 0;
    subServiceLoopsSec = subServiceLoops;
    subServiceLoops = 0;
    subServicedCntSec = subServicedCnt;
    subServicedCnt = 0;
    usbSendFailSum += usbSendFail;
    usbSendFailSec = usbSendFail;
    usbSendFail = 0;
    usbSendCntSec = usbSendCnt;
    usbSendCnt = 0;
    // debug
    // sendUSBstatus();
    // debug end
  }
}

void UUSB::usbIsBackUp()
{
  if (not usbIsUp)
  {
    const int MSL = 200;
    char s[MSL];
    snprintf(s, MSL, "# USB back up at %.3fs; went off at %.3fs (since Teensy boot) as %s\r\n", service.time_sec(), usbOffTime, usbOffReason);
    usbIsUp = true;
    usb.send(s);
  }
}


bool UUSB::send(const char* str) // , bool blocking)
{
  bool sendOK;
  if (localEcho == 1 and justSendPrompt)
  { // this is a bad idea, as CRC is added
    client_send_str("\n\r", 2);
    justSendPrompt = false;
  }
  if (usbIsUp)
  {
    // Serial.print("# Uusb:: send 3\n");
    int n = strlen(str);
    sendOK = client_send_str(str, n);
  }
  else
  {
    if (sendOK)
    { // send used to work, someone has pulled the cable
      // @todo - stop all subscriptions
    }
    sendOK = false;
  }
  //
  if (sendOK == false)
    usbSendFail++;
  else
    usbSendCnt++;
  //
  if (logger.logStreamedMsg and not logger.logStreamedMsgFull)
  {
    logger.addMsgLog(str);
  }
  return sendOK;
}

void UUSB::sendData(int item)
{
  if (item == 0)
    sendUSBstatus();
  else if (item == 1)
    sendSubscriptionStatus();
}

void UUSB::sendSubscriptionStatus()
{
  const int MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "# UUSB::tick: (sec) size=%d\n", subscriptions.size());
  usb.send(s);
  for (int i = 0; i < (int)subscriptions.size(); i++)
  { // send full subscription status
    subscriptions[i]->serviceStatus(i);
  }
}

void UUSB::sendUSBstatus()
{
  const int MSL = 120;
  char s[MSL];
  snprintf(s, MSL, "# usb %.3f %d %d %d %d %d %d %d\r\n",
           service.time_sec(),
           usbInMsgCntInSec,
           usbInErrCnt,
           subServicedCntSec,
           subServiceLoopsSec,
           usbSendCntSec,
           usbSendFailSec,
           usbSendFailSum);
  usb.send(s);
}


void UUSB::sendHelp()
{
  const int MRL = 320;
  char reply[MRL];
  snprintf(reply, MRL, "# USB connection ------- \r\n");
  send(reply);
  snprintf(reply, MRL, "# -- \ti V \tInteractive: V=1: local echo (is=%d) (for use with telnet)\r\n", localEcho);
  send(reply);
  snprintf(reply, MRL, "# -- \tnocrc V \tAllow messages without CRC: V=1: allow (is=%d)\r\n", allowNoCRC );
  send(reply);
  snprintf(reply, MRL, "# -- \tsilent V \tShould USB be silent, if no communication (1=auto silent) silent=%d (pt no effect)\r\n", silenceUSBauto);
  send(reply);
  send(                "# -- \talive \tIgnorred, but used to keep communication alive (once a sec is fine)\r\n");
}

bool UUSB::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "i ", 2) == 0)
  {
    const char * p1 = &buf[2];
    localEcho = *p1 == '1';
    // when local echo, there is no need for CRC, so turn it off
    use_CRC = not localEcho;
  }
  else if (strncmp(buf, "silent ", 7) == 0)
  {
    const char * p1 = &buf[7];
    silenceUSBauto = strtol(p1, nullptr, 10);
  }
  else if (strncmp(buf, "nocrc ", 8) == 0)
  {
    const char * p1 = &buf[8];
        allowNoCRC = strtol(p1, nullptr, 10);
  }
  else if (strncmp(buf, "alive", 5) == 0)
  {
    // accepted, but ignored
  }
  else
    used = false;
  return used;
}



bool UUSB::sendInfoAsCommentWithTime(const char* info, const char * msg)
{
  const int MSL = 400;
  char s[MSL];
  bool isOK = false;
  snprintf(s, MSL, "# %.3f %s: %s\r\n", service.time_sec(), info, msg);
  isOK = send(s);
  return isOK;
}


//////////////////////////////////////////////////

bool UUSB::client_send_str(const char * str, int m) // , bool blocking) //, bool toUSB, bool toWifi)
{
  //int n = strlen(str);
  bool okSend = true;
  if (use_CRC)
  { // generate q-code first
    int sum = 0;
    const char * p1 = str;
    for (int i = 0; i < m; i++)
    {
      if (*p1 >= ' ')
        sum += *p1;
      p1++;
    } 
    const int MQL = 4;
    char q[MQL];
    snprintf(q, MQL, ";%02d", (sum % 99) + 1);
    int a = usb_serial_write(q, 3);
    if (a == 3)
    {
      a = usb_serial_write(str, m);
      okSend += a + 2;
    }
    else
      okSend = false;
  }
  else
  { // just send as is
    okSend = usb_serial_write(str, m);
  }
  return okSend;
}

////////////////////////////////////////////////////////////////

bool UUSB::handleIncoming()
{
  int n = 0, m;
  bool dataReceived = false;
  // get number of available chars in USB buffer
  m = usb_serial_available();
  // changed to give
  // incoming more bandwidth (especially for robobot)
  if (false and m > 30)
  { // limit to no more than 20 chars in one 1ms cycle
// ################ debug
//     const int MSL = 100;
//     char s[MSL];
//     snprintf(s, MSL, "# UUSB got %d chars (>30)\n", m);
//     usb.send(s);
// ################ end
    m = 20;
  }
  // 
  if (m > 0)
  { // get characters
    for (int i = 0; i < m; i++)
    { // get pending characters
      n = usb_serial_getchar();
      if (n < 0)
        break;
      // limit to usable part of 7-bit ASCII
      if (n >= '\n' and n < 0x80)
      { // there is data from USB, so it is active
//         usbTimeoutGotData = hbTimerCnt;
        // command arriving from USB
        // usb_send_str("#got a char from USB\r\n");
        dataReceived = receivedCharFromUSB(n) ;
      }
    }
  }
  return dataReceived;
}


/**
 * Got a new character from USB channel
 * Put it into buffer, and if a full line, then intrepid the result.
 * \param n is the new character */
bool UUSB::receivedCharFromUSB(uint8_t n)
{ // got another character from USB (command)
  bool fullMsg = false;
  if (n >= ' ')
  {
    usbRxBuf[usbRxBufCnt] = n;
    if (usbRxBufCnt < RX_BUF_SIZE - 1)
      usbRxBufCnt++;
    else
    {
      usbRxBufOverflow = true;
      usbRxBufCnt = 0;
      usbRxBuf[usbRxBufCnt] = '\0';
    }
  }
  //
  if (localEcho) // and not silentUSB)
    // echo characters back to terminal
    usb_serial_putchar(n);
  if (n == '\n' or n=='\r')
  { // zero terminate
    if (usbRxBufOverflow)
    {
      usbRxBufOverflow = false;
      send("# USB rx-buffer overflow\r\n");
    }
    else
    {
      if (usbRxBufCnt > 0)
      {
//         if (logger.logUSB and logger.isLogging())
//         { // add string with newline to USB log
//           usbRxBuf[usbRxBufCnt] = '\n'; // to make log readable
//           logger.addUSBLogEntry(usbRxBuf, usbRxBufCnt + 1, rxStartHb, -1);
//         }
        usbRxBuf[usbRxBufCnt] = '\0';
        // check CRC
        bool crcOK = false;
        if (usbRxBuf[0] == ';')
        {
          const char * p1 = usbRxBuf;
          int crc = int(p1[1] - '0') * 10 + int(p1[2] - '0');
          int sum = 0;
          int sumCnt = 0;
          for (int i = 3; i < usbRxBufCnt; i++)
          {
            if (usbRxBuf[i] >= ' ')
            {
              sum += usbRxBuf[i];
              sumCnt++;
            }
          }
          crcOK = (sum % 99) + 1 == crc;
          if (crcOK or localEcho)
          {
            fullMsg = true;
            char * msg = &usbRxBuf[3];
            // check for individual confirm character
            bool confirm = msg[0] == '!';
            if (confirm)
              // skip the '!'
              msg++;
            command.parse_and_execute_command(msg);
            usbInMsgCnt++;
            debugCnt = 0;
            if (confirm)
            {
              const int MSL = 250;
              char s[MSL+1];
              snprintf(s, MSL, "confirm %s\n", &usbRxBuf[3]);
              // confirm max first 42 characters
              s[MSL-1] = '\n';
              s[MSL] = '\0';
              send(s);
            }
          }
          else
          {
            const int MSL = 285;
            char s[MSL];
            snprintf(s, MSL, "# CRC failed (crc=%d, found to be %d, sum=%d, %d chars), for '%s'\r\n",
                     crc, (sum % 99) + 1, sum, sumCnt, usbRxBuf);
            send(s);
            usbInErrCnt++;
          }
        }
        else if (not allowNoCRC or not use_CRC)
        { // may be a hand-written message, so accept under protest
          if (use_CRC)
          {
            const int MSL = 300;
            char s[MSL];
            snprintf(s, MSL, "# processing under protest - no CRC '%s'\r\n", usbRxBuf);
            send(s);
          }
          fullMsg = true;
          command.parse_and_execute_command(usbRxBuf);
          usbInMsgCnt++;
          debugCnt = 0;
        }
        else
        { // just ignore - must be a corrupted message
        }
      }
      if (localEcho == 1)
      {
        send("\r\n>>");
        justSendPrompt = true;
      }
    }
    // flush remaining input
    usbRxBufCnt = 0;
  }
  else if (usbRxBufCnt >= RX_BUF_SIZE - 1)
  { // garbage in buffer, just discard
    usbRxBuf[usbRxBufCnt] = 0;
    const char * msg = "** Discarded (missing \\n)\r\n";
    send(msg);
    usbRxBufCnt = 0;
  }
  return fullMsg;
}

void UUSB::addSubscriptionService(USubss* newToBeServiced)
{
  // if (newToBeServiced->isMe("cvel"))
  // {
  //   if (ctrlVel1 == 0)
  //     ctrlVel1 = subscriptions.size();
  //   else
  //     ctrlVel2 = subscriptions.size();
  //   const int MSL = 100;
  //   char s[MSL];
  //   snprintf(s, MSL, "# UUSB::addSubscriptionService: cvel has index %d %d\r\n", ctrlVel1, ctrlVel2);
  //   usb.send(s);
  // }
  subscriptions.push_back(newToBeServiced);
}

void UUSB::stopAllSubscriptions()
{
  for (int i = 0; i < (int)subscriptions.size(); i++)
  {
    subscriptions[i]->stopSubscriptions();
  }
}

void UUSB::sendAllHelp()
{
  const int MRL = 200;
  char reply[MRL];
  snprintf(reply, MRL, "# Command list (host %s %s) ------- \r\n", robot.deviceName, robot.getRobotName());
  usb.send(reply);
  for (int i = 0; i < (int)subscriptions.size(); i++)
  {
    subscriptions[i]->sendHelp();
    subscriptions[i]->subscribeSendHelp();
  }
}

bool UUSB::decodeAll(const char* buf)
{
  bool used = false;
  for (int i = 0; i < (int)subscriptions.size(); i++)
  {
    used = subscriptions[i]->decode(buf);
    if (used)
    { // if velocity control, then copy to other
      if (i == ctrlVel1)
        subscriptions[ctrlVel2]->decode(buf);
      // non-subscribe command found, stop here
      break;
    }
    used = subscriptions[i]->subscribeDecode(buf);
    if (used)
    { // subscribe command found, stop here
      // debug
      // const int MSL = 300;
      // char s[MSL];
      // snprintf(s, MSL, "# UUSB::decodeAll: used by=%d, msg:%s\n", i, buf);
      // usb.send(s);
      // debug end
      break;
    }
  }
  return used;
}
