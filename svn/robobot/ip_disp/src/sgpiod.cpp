/*  
 * 
 * Copyright © 2023 DTU,
 * Author:
 * Christian Andersen jcan@dtu.dk
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

#include <string>
#include <string.h>
#include <unistd.h>
#include <cstdlib>
#include <filesystem>
#include <gpiod.hpp>
#include "uservice.h"
#include "sgpiod.h"

// https://github.com/brgl/libgpiod/blob/master/bindings/cxx/gpiod.hpp
#include "gpiod.h"
#include "umqtt.h"

// create value
SGpiod gpio;

namespace {

  /* Example configuration - customize to suit your situation */
  const ::std::filesystem::path chip_path("/dev/gpiochip4");
  // const ::gpiod::line::offset line_offset = 5;

} /* namespace */

void setPinIn(std::string name, bool up)
{
  printf("# GPIO:: getting chip\n");
  ::gpiod::chip chip(chip_path);
  printf("# GPIO:: got chip, making line config\n");
  gpiod::line_request line_config;
  if (up)
  {
    line_config.flags = line_config.FLAG_BIAS_PULL_UP;
    //line_config.flags = line_config.FLAG_BIAS_DISABLE;
  }
  else
    line_config.flags = line_config.FLAG_BIAS_PULL_DOWN;
  line_config.consumer = "ip_disp";
  line_config.request_type = gpiod::line_request::DIRECTION_INPUT;
  printf("# GPIO:: got chip, finding line\n");
  auto line = chip.find_line(name);
  if (line)
  {
    printf("# GPIO:: got chip, found line, request setting ...\n");
    // printf("# GPIO, got line, is %d, bias=%0x\n",
    //        line.get_value(),
    //        line.bias());
    line.request(line_config);
    printf("# GPIO, line now, is %d, bias=%0x\n",
           line.get_value(),
           line.bias());
    line.release();
  }
  else
  {
    printf("# line not found\n");
  }
}

void setPinOut(std::string name, int value)
{
  // printf("# GPIO:: getting chip\n");
  ::gpiod::chip chip(chip_path);
  // printf("# GPIO:: got chip, making line config\n");
  gpiod::line_request line_config;
  line_config.flags = line_config.FLAG_BIAS_PULL_DOWN;
  line_config.consumer = "ip_disp";
  line_config.request_type = gpiod::line_request::DIRECTION_OUTPUT;
  // printf("# GPIO:: got chip, finding line\n");
  auto line = chip.find_line(name);
  if (line)
  {
    // printf("# GPIO:: got chip, found line, request setting ...\n");
    // printf("# GPIO, got line, is %d, bias=%0x\n",
    //        line.get_value(),
    //        line.bias());
    line.request(line_config, value);
    // printf("# GPIO, line now, is %d, bias=%0x\n",
    //        line.get_value(),
    //        line.bias());
    line.release();
  }
  else
  {
    printf("# line not found\n");
  }
}


// Bridge class:
void SGpiod::setup()
{ // ensure default values
  chip = gpiod_chip_open_by_name(chipname);
  if (chip != nullptr)
  { // get pin handles
    for (int i = 0; i < MAX_PINS; i++)
    { // get handle to relevant pins and set output as specified
      pins[i] = gpiod_chip_get_line(chip, pinNumber[i]);
      if (true)
        // reserve input pin (is no longer used by others)
        gpiod_line_request_input(pins[i], "ip_disp_in");
    }
  }
  else
  {
    printf("# SGpiod::setup there is no GPIO chip found\n");
  }
  // logfiles
  if (true)
  { // open logfile
    std::string fn = "log_gpio.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% gpio logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tPin %d (start)\n", pinNumber[0]);
//     fprintf(logfile, "%% 3 \tPin %2d (stop)\n", pinNumber[1]);
    if (chip == nullptr)
      fprintf(logfile, "%% No GPIO chip found\n");
  }
  if (not service.stop)
    // start listen to the keyboard
    th1 = new std::thread(runObj, this);
}

void SGpiod::terminate()
{
  if (th1 != nullptr)
    th1->join();
  if (logfile != nullptr)
  {
    fclose(logfile);
    printf("# SGpiod:: logfile closed\n");
  }
  if (chip != nullptr)
  {
    for (int i = 0; i < MAX_PINS; i++)
      gpiod_line_release(pins[i]);
  }
}


int SGpiod::getPinIndex(int pinNumber)
{ //   int pinNumber[MAX_PINS] = {13, 6, 12, 16, 19, 26, 21, 20};
  int result = -1;
  switch (pinNumber)
  {
    case 13: result = 0; break; // start
    case  6: result = 1; break; // stop
    case 12: result = 2; break; // close to stop
    default:
      break;
  }
  return result;
}



int SGpiod::readPin(const int pin)
{
  int idx = getPinIndex(pin);
  int val = -1;
  // UTime t("now");
  if (chip != nullptr /*and service.teensyFileFree*/)
  {
    if (idx >= 0)
    {
      int err = 0; // gpiod_line_request_input(pins[idx], "ip_disp");
      if (err == 0)
      {
        // gpiod_line_settings_set_bias(pins[idx], GPIOD_LINE_BIAS_PULL_UP);
        val = gpiod_line_get_value(pins[idx]);
        // gpiod_line_release(pins[idx]);
        //pinReadTook = t.getTimePassed();
        // if (logfile != nullptr)
        //   fprintf(logfile, "%lu.%04ld pin %d pressed\n",
        //           t.getSec(), t.getMicrosec()/100,  pin);
        // if (service.mission_app_running)
        // { // mission app was running, but has stopped using the start button
        //   service.mission_app_running = false;
        //   printf("# SGpiod::readPin: pin %d is no longer reserved by others - no mission running.\n", pin);
        //   if (logfile != nullptr)
        //     fprintf(logfile, "%lu.%04ld pin %d is no longer reserved by others\n",
        //             t.getSec(), t.getMicrosec()/100,  pin);
        // }
      }
      // else
      // { // mission app is running, i.e. uses the start button
      //   if (not service.mission_app_running)
      //   {
      //     service.mission_app_running = true;
      //     printf("# SGpiod::readPin: pin %d is reserved by others - fine (mission app running)\n", pin);
      //     if (logfile != nullptr)
      //       fprintf(logfile, "%lu.%04ld pin %d is reserved by others - fine (mission app running)\n",
      //               t.getSec(), t.getMicrosec()/100,  pin);
      //   }
      // }
    }
    // else
    //   printf("# SGpiod::readPin: pin %d is not valid, use one of: %d, %d\n", pin,
    //       pinNumber[0], pinNumber[1]);
  }
  return val;
}

void SGpiod::run()
{
  int pv[MAX_PINS] = {false};
  bool changed = true;
  UTime t;
  UTime lastScriptCall("now");
  int loop = 0;
  bool firstRead = true;
  // int mr = 0; // mission running
  bool shuttingDown = false;
  if (logfile != nullptr)
    fprintf(logfile, "%% Listening to start button (pin 13)\n");
  while (not service.stop and chip != nullptr)
  {
    loop++;
    changed = false;
    t.now();
    {
      for (int i = 0; i < MAX_PINS; i++)
      { // check all pins on the list
        pv[i] = readPin(pinNumber[i]);
        if (pv[i] >= 0)
        { // valid read
          // printf("# Pin %d read: first=%d, new=%d, old=%d\n",
          //         i, firstRead, pv[i], in_pin_value[i]);
          if (firstRead)
            in_pin_value[i] = pv[i];
          else if (pv[i] != in_pin_value[i])
          { // pin has changed value
            in_pin_value[i] = pv[i];
            changed = true;
            printf("# %lu.%04ld GPIO:: pin %d value changed to %d\n",
                  t.getSec(), t.getMicrosec()/100, pinNumber[i], pv[i]);
            if (logfile != nullptr)
              fprintf(logfile, "# %lu.%04ld GPIO:: pin %d value changed to %d\n",
                  t.getSec(), t.getMicrosec()/100, pinNumber[i], pv[i]);
            if (pv[i] == 1)
              pinPressedTime[i].now();
            else
            { // pin released
              if (i == 0 and not shuttingDown)
              { // call start script
                bool fileOK = service.file_exists(start_script_name);
                if (fileOK)
                { // start script exist
                  mqtt.publish("robobot/cmd/T0/leds","16 100 0 0", nullptr, 0);
                  std::string cmd = "bash " + start_script_name;
                  printf("# SGpio:: start: %s\n", cmd.c_str());
                  std::string o = service.exec(cmd);
                  t.now();
                  if (logfile != nullptr)
                  {
                    fprintf(logfile, "%lu.%04ld GPIO::called start script '%s'\n",
                            t.getSec(), t.getMicrosec()/100, start_script_name.c_str());
                  }
                  lastScriptCall.now();
                }
              }
            }
          }
          if (pinPressedTime[i].getTimePassed() > 4 and pv[i] == 1 and not shuttingDown)
          { // power off request (any of the two (three?) buttons)
            printf("# SGPIO:: power off request - pin %d\n", i);
            // send MQTT message
            shuttingDown = true;
            service.shutdownRequest();
          }
          // else
          //   printf("# %lu.%04ld GPIO:: pin value not changed %d\n",
          //          t.getSec(), t.getMicrosec()/100, pv[i]);
        }
        else
          printf("# pin read not valid %d\n", pinNumber[i]);
      }
      if (firstRead) // and pv[0] >= 0 and pv[1] >= 0)
      {
        firstRead = false;
      }
    }
    if (changed and logfile != nullptr)
    {
      fprintf(logfile,"%lu.%04ld %d\n", t.getSec(), t.getMicrosec()/100,
              pv[0]);
    }
    usleep(15000);
  }
  printf("# Gpio::run terminated\n");
  if (logfile != nullptr)
    fprintf(logfile, "%% Gpio::run terminated\n");
}

