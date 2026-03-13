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

#ifndef USUBSS_H
#define USUBSS_H

#include "usubs.h"
#include <vector>

class USubs;

class USubss
{
protected:
  USubss();
    /**
   * @brief sendHelpLine sends help line for this key
   * \param listNum is the publist list number of last item - to be increased
   */
  void sendPublishList(int & listNum);
  /**
   * add subscription key */
  void addPublistItem(const char * key, const char * helpLine);
  /**
   * send data now from one of the subscription items
   * single request or as subscribed */
  virtual void sendData(int item) {};
public:
  /**
   * Service at maximum one subscription
   * \return false if a subscription is serviced, and at next
   * call service the next (if any)
   * \return true if all has been serviced, next call will start from first. */
  bool subscribeService();
  /**
   * send status for subscriptions
   * \param is index to this class */
  void serviceStatus(int me);
  /**
   * @brief Decode a subscription command
   * @param keyline is the keyword for this message and further parameters
   * @returns true if used
   */
  bool subscribeDecode(const char * keyline);
  /**
   * Stop all subscriptions */
  void stopSubscriptions();
  /**
   * @brief sendHelpLine sends help line for this key
   */
  void subscribeSendHelp();
  /**
   * send command help */
  virtual void sendHelp() {};
  /**
   * send data now from one of the subscription items
   * single request or as subscribed */
  virtual bool decode(const char * buf) { return false;};
  
protected:
  /// 
  std::vector<USubs*> subs;
  int subscribeState = 0;
};

#endif // USUBS_H
