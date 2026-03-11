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

#ifndef UUSB_H
#define UUSB_H

#include <vector>
#include "usubss.h"

class UUSB : public USubss
{
private:
  bool silenceUSBauto = true; // manuel inhibit of USB silent timeout
  uint32_t lastSend = 0;
  // local echo is used if we are talking to e.g. putty
  // and a command prompt would be nice.
  bool localEcho = false;
  bool justSendPrompt = false;
  /**
   * usb command buffer space */
  static const int RX_BUF_SIZE = 200;
  char usbRxBuf[RX_BUF_SIZE];
  int usbRxBufCnt = 0;
  bool usbRxBufOverflow = false;
  uint32_t usbTimeoutGotData = 0;
  int usbInMsgCnt = 0;
  int usbInMsgCntSec = 0;
  int usbInErrCnt = 0;
  int usbSendFail = 0;
  int usbSendFailSec = 0;
  int usbSendFailSum = 0;
  int usbSendFailSumLast = 0;
  uint32_t lastSec = 0;
  int debugCnt = 0;
  /// index to first cvel controller
  int ctrlVel1 = 0; 
  /// index to last cvel controller
  int ctrlVel2 = 0; 
  
public:
  bool usbIsUp = false;

  void setup();
  /**
   * decode command for this unit */
  bool decode(const char * buf) override;
  /**
   * called as often as time allows, at least one time per sample interval */
  void tick();
  /** send message to USB host
   * \param str : string to send, should end with a '\n'
   * \param blocking : if true, then function will not return until send.
   *                   if false, message will be dropped, if no BW is available
   * return true if send. */
  bool send(const char* str); //, bool blocking = false);
  /** send to USB channel 
  * \param str is string to send
  * \param n is number of bytes to send
  * \param blocking if false, then send if space only, else don't return until send
  */
  inline bool send_block(const char * str, int n) //, bool blocking)
  {
    return client_send_str(str, n); //, blocking);
  }
  
  bool sendInfoAsCommentWithTime(const char* info, const char * msg);
  /**
   * send help and status */
  void sendHelp() override;
  /**
   * add a subscription service block */
  void addSubscriptionService(USubss * newToBeServiced);
  
  void stopAllSubscriptions();
  
  void sendAllHelp();
  
  bool decodeAll(const char * buf);

protected:
  /**
   * send data to subscriber or requester over USB 
   * @param item is the item number corresponding to the added subscription during setup. */
  void sendData(int item) override;
  /**
   * send usb interface status */
  void sendUSBstatus();
  
private:
  /// send a string - if possible - of this length to USB
  bool client_send_str(const char * str, int m); //, bool blocking);
  /// received a character from USB
  bool receivedCharFromUSB(uint8_t n);
  /** USB incomming port handling
   * \return true if something to be handled */
  bool handleIncoming();
  /// subscribe service registrations
  std::vector<USubss*> subscriptions;
  /// next subscription to service
  int subscribeServiceState = 0;
  int subServiceLoops = 0;
  int subServiceLoopsSec = 0;
  int subServicedCnt = 0;
  int subServicedCntSec = 0;
  int usbSendCnt = 0;
  int usbSendCntSec = 0;
  /// reliable transmission over USB connection
  /// set true on first confirmation
  bool allowNoCRC = false;
};
  
extern UUSB usb;
#endif
