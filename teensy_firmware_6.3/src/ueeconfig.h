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

#ifndef REGBOT_EESTRING_H
#define REGBOT_EESTRING_H

#include <string.h>
#include <stdlib.h>
#include "main.h"

#include "ucommand.h"
#include "uusb.h"
#include "usubss.h"



class EEConfig : public USubss
{
public:
  // constructor
  EEConfig();
  /**
   * set PWM port of frekvens */
  void setup();
  /**
   * Send command help */
  void sendHelp() override;
  /**
   * decode commands */
  bool decode(const char * buf) override;
  /** send configuration to USB
   * \param configBuffer set to NULL to use the just saved configuration or to another configuration to fetch.
   * Sends the configuration as byte hex code  */
  void stringConfigToUSB(const uint8_t * configBuffer, int configBufferLength);
  /**
   * is stringbuffer in use, i.e. loaded from a hard-coaded configuration (non-robot specific) */
  bool isStringConfig()
  {
    return stringConfig;
  }
  /** set in use flag and clear buffer */
  inline void setStringBuffer(uint8_t * string2kBuffer,  bool initializeEEprom)
  {
    config = string2kBuffer;
    configAddr = 0;
    configAddrMax = 0;
    if (initializeEEprom)
      eeprom_initialize();
  }
  inline void clearStringBuffer()
  {
    config = NULL;
  }
  /* get 2k config buffer pointer */
//   uint8_t * get2KConfigBuffer()
//   {
//     return config;
//   }
  /**
   * Load configuration from either a config string or from eeProm (flash) 
   * dependent on the "stringConfig" flag
   * \param from2Kbuffer if true, then load from string buffer (must be loaded first), if false, then read for eeprom (flask).
   * */
  void eePromLoadStatus(bool from2Kbuffer);
  /**
   * Save configuration to eeProm (flash) or sent configuration to USB in hex format.
   * \param toUSB set to true to read to USB, or false to save to eeProm */
  void eePromSaveStatus(bool toUSB);
  
public:
  /** save a 32 bit value */
  void push32(uint32_t value);
  /** save a byte */
  void pushByte(uint8_t value);
  /** save a word in configuration stack */
  void pushWord(uint16_t value);
  /** get a 32 bit integer from configuration stack */
  uint32_t read32();
  /** get a byte from configuration stack */
  uint8_t readByte();
  /** get a 16 bit integer from configuration stack */
  uint16_t readWord();
  /**
   * Add a block of data to ee-Prom area 
   * \param data is the byte data block,
   * \param dataCnt is the number of bytes to write 
   * \returns true if space to write all. */
  bool pushBlock(const char * data, int dataCnt);
  /**
   * Read a number of bytes to a string 
   * \param data is a pointer to a byte array with space for at least dataCnt bytes.
   * \param dataCnt is number of bytes to read
   * \returns true if data is added to data array and false if 
   * requested number of bytes is not available */
  bool readBlock(char * data, int dataCnt);
  
  /** save a 32 bit float to configuration stack */
  inline void pushFloat(float value)
  {
    union {float f; uint32_t u32;} u;
    u.f = value;
    push32(u.u32);
  }
  // read 32 bit as float from configuration stack
  inline float readFloat()
  {
    union {float f; uint32_t u32;} u;
    u.u32 = read32();
    return u.f;  
  }
  /** write a word to a specific place in configuration stack
   * typically a size that is not known before pushing all the data */
  inline void write_word(int adr, uint16_t v)
  {
    if (not stringConfig)
      eeprom_write_word((uint16_t*)adr, v);
    else if (config != NULL)
    {
      memcpy(&config[adr], &v, 2);
    }
    else
      usb.send("# failed to save word\n");
    if (adr > configAddr - 2)
      configAddr = adr + 2;
  }
  /**
   * a busy wit if the flash write system is busy */
  inline void busy_wait()
  {
    if (not stringConfig)
    {
      eeprom_busy_wait();
    }
  }
  /** push a block of data to the configuration stack */
  inline void write_block(const char * data, int n)
  {
    if (not stringConfig)
    {
      eeprom_write_block(data, (void*)configAddr, n);
    }
    else
    {
      memcpy(&config[configAddr], data, n);
    }
    configAddr += n;
  }
  /** set the adress for the next push or read operation on the configuration stack */
  void setAddr(int newAddr)
  {
    configAddr = newAddr;
  }
  /** skip some bytes from the configuration stack
   * \param bytes is the number of bytes to skib. */
  void skipAddr(int bytes)
  {
    configAddr+=bytes;
    // debug
//     const int MSL = 100;
//     char s[MSL];
//     snprintf(s, MSL, "# skipped %d bytes\n", bytes);
//     usb.send(s);
    // debug end
  }
  /** get the address of the next push or read operation on the configuration stack */
  int getAddr()
  {
    return configAddr;
  }
  /**
   * Implement one of the hard-coded configurations 
   * \param hardConfigIdx is index to the hardConfig array, as defined in eeconfig.h and set in the constructor.
   * \param andToUsb is a debug flag, that also will return the just loaded configuration to the USB
   * */
  bool hardConfigLoad(int hardConfigIdx, bool andToUsb);
  
protected:
  /**
   * Get hard coded configuration string and load it into buffer in binary form
   * like for the real flash configuration memory. */
   int getHardConfigString(uint8_t * buffer, int configIdx);
  
#ifdef TEENSY35
  const int maxEESize = 4096;
#else
  const int maxEESize = 2048;
#endif
  // public:
//   /**
//    * use hard-coded string values flag - should be false for configurations related to a specific robot
//    * if false, then the flash-version is maintained while this flag is false. */
//   bool eeFromStringuse;
  
private:
  /** full configuration buffer, as real eeProm 
   * is either NULL or points to a 2048 byte array */
  uint8_t * config;
  /** max number of bytes in string buffer */
//   static const int sbufMaxCnt = 64;
//   /** string buffer for data reply to client */
//   uint8_t sbuf[sbufMaxCnt];
  /** number of bytes written to string buffer */
  int sbufCnt;
  /** is string buffer in use - else flash is in use */
  bool stringConfig;
  /** current read/write adress in config array */
  int configAddr;
  /** highest number used in configuration in config array */
  int configAddrMax;
  /** hard coded configuration 
   * for configuration as of 25 dec 2018, NB! must be changed if configuration layout changes
   */
  /*
   thread=2
      vel=0.0, log=10.0: time=0.1
      bal=1: time=6.5
      bal=0: time=0.5
   */
  const char * hardBalance6s = 
  "#cfg00 e7 01 00 00 63 3b 00 00 64 00 18 00 05 72 65 67 62 6f 74 00 00 00 00 00 00 00 00 00 00 00 00 00\
  #cfg01 00 00 00 00 00 00 00 00 00 00 00 00 00 01 80 83 06 40 00 10 5f 3f 00 28 19 be 00 00 00 00 00 00\
  #cfg02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 3f 00 00 00 00 00 00 00 00 00 00\
  #cfg03 00 00 00 00 80 3f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 3f 00 00 80 3f 00 00 80 3f 00 00\
  #cfg04 80 3f 00 00 00 00 14 5c 42 40 00 00 00 00 63 4a 00 00 0a 00 00 00 00 4b 03 00 00 60 41 8f c2 75\
  #cfg05 3d 00 00 a0 40 00 00 00 00 0a d7 a3 3b 00 00 90 40 00 00 10 41 4b 03 00 00 60 41 8f c2 75 3d 00\
  #cfg06 00 a0 40 00 00 00 00 0a d7 a3 3b 00 00 90 40 00 00 10 41 1b 03 00 00 80 3f 00 00 80 3e cd cc cc\
  #cfg07 3d 9a 99 19 3e 0a d7 a3 3c 00 00 00 00 0a d7 23 3c 00 00 00 3f 01 02 00 00 c0 3f 01 03 66 66 46\
  #cfg08 40 cd cc cc 3d 01 02 00 00 80 3f 0d 02 cd cc cc 3e 00 00 00 3f 00 00 c0 3f 00 00 00 00 0a d7 a3\
  #cfg09 3b 2b 03 b8 1e 25 c0 60 e5 50 3d 00 00 00 40 e3 a5 1b 3d a6 9b 44 3b cd cc cc 3d f6 3f 1c 46 00\
  #cfg10 00 00 40 0b 03 1f 85 6b 3e 66 66 86 3f 9a 99 99 3e 00 00 00 00 cd cc cc 3d 9a 99 99 3e 07 01 cd\
  #cfg11 cc cc 3e ec 51 b8 3f 00 00 80 3f 8f c2 95 3f c3 f5 a8 3e 00 00 80 3f 23 00 6f 32 0a 61 30 64 31\
  #cfg12 30 3a 42 3d 30 2e 31 0a 66 31 3a 42 3d 36 2e 35 0a 66 30 3a 42 3d 30 2e 35 0a 00 28 00 00 00 00\
  #cfg13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 6c 1a 6c\
  #cfg14 1a d4 30 d4 30 00 00 00 00 00 00 00 00 00 00 00 8f c2 f5 3c 8f c2 f5 3c 48 e1 1a 41 30 00 52 b8\
  #cfg15 1e 3e 01 00 00 a4 6a";
  
  /*
   thread=2
      vel=0.0, log=50.0: time=0.2
      vel=0.25,label=5: dist = 0.3
      tr=0.2,acc=10 : turn = 90
      : dist = 0.3
      goto=5:count=3
      vel=0: time=1
  */
  const char * hardSquareNoBalance = 
  "#cfg00 05 02 00 00 63 3b 00 00 64 00 18 00 05 72 65 67 62 6f 74 00 00 00 00 00 00 00 00 00 00 00 00 00\
  #cfg01 00 00 00 00 00 00 00 00 00 00 00 00 00 01 80 83 06 40 00 10 5f 3f 00 28 19 be 00 00 00 00 00 00\
  #cfg02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 3f 00 00 00 00 00 00 00 00 00 00\
  #cfg03 00 00 00 00 80 3f 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 3f 00 00 80 3f 00 00 80 3f 00 00\
  #cfg04 80 3f 00 00 00 00 14 5c 42 40 00 00 00 00 63 4a 00 00 32 00 00 00 00 4b 03 00 00 60 41 8f c2 75\
  #cfg05 3d 00 00 a0 40 00 00 00 00 0a d7 a3 3b 00 00 90 40 00 00 10 41 4b 03 00 00 60 41 8f c2 75 3d 00\
  #cfg06 00 a0 40 00 00 00 00 0a d7 a3 3b 00 00 90 40 00 00 10 41 1b 03 00 00 80 3f 00 00 80 3e cd cc cc\
  #cfg07 3d 9a 99 19 3e 0a d7 a3 3c 00 00 00 00 0a d7 23 3c 00 00 00 3f 01 02 00 00 c0 3f 01 03 66 66 46\
  #cfg08 40 cd cc cc 3d 01 02 00 00 80 3f 0d 02 cd cc cc 3e 00 00 00 3f 00 00 c0 3f 00 00 00 00 0a d7 a3\
  #cfg09 3b 2b 03 b8 1e 25 c0 60 e5 50 3d 00 00 00 40 e3 a5 1b 3d a6 9b 44 3b cd cc cc 3d f6 3f 1c 46 00\
  #cfg10 00 00 40 0b 03 1f 85 6b 3e 66 66 86 3f 9a 99 99 3e 00 00 00 00 cd cc cc 3d 9a 99 99 3e 07 01 cd\
  #cfg11 cc cc 3e ec 51 b8 3f 00 00 80 3f 8f c2 95 3f c3 f5 a8 3e 00 00 80 3f 41 00 6f 32 0a 61 30 64 35\
  #cfg12 30 3a 42 3d 30 2e 32 0a 61 30 2e 32 35 6a 35 3a 41 3d 30 2e 33 0a 62 31 30 63 30 2e 32 3a 43 3d\
  #cfg13 39 30 0a 3a 41 3d 30 2e 33 0a 6e 35 3a 44 3d 33 0a 61 30 3a 42 3d 31 0a 00 28 00 00 00 00 00 00\
  #cfg14 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 6c 1a 6c 1a d4\
  #cfg15 30 d4 30 00 00 00 00 00 00 00 00 00 00 00 8f c2 f5 3c 8f c2 f5 3c 48 e1 1a 41 30 00 52 b8 1e 3e\
  #cfg16 01 00 00 a4 6a";
  const char * hardBalance1mOutAndBack = 
  "not implemented";
  
  const char * hardConfigFollowWall = 
  "not implemented";
public:
  static const int hardConfigCnt = 2;
  const char * hardConfig[hardConfigCnt] = {
    hardBalance6s,
    hardSquareNoBalance/*, 
    hardBalance1mOutAndBack*/ } ;
};

/**
 * Instans af ee og string config */
extern EEConfig eeConfig;


#endif
