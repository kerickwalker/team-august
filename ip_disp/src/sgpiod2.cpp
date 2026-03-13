/*  
 * 
 * Copyright © 2026 DTU,
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
#include <chrono>
#include <thread>
#include <iostream>
#include "uservice.h"
#include "sgpiod2.h"
#include "umqtt.h"
// #include "srobot.h"

// inspired from https://github.com/brgl/libgpiod/blob/master/bindings/cxx/gpiod.hpp
#include <gpiod.h>

using namespace std::chrono;

// create value
SGpiod gpio;

void SGpiod::setup()
{ // ensure default values
  if (not ini.has("gpio"))
  { // no data yet, so generate some default values
    ini["gpio"]["pins_out"] = "12=0 16=1"; // give value to output pins
    ini["gpio"]["stop_on_stop"] = "true";
    ini["gpio"]["blink_period_ms"] = "600"; // ms
    ini["gpio"]["log"] = "true";
    ini["gpio"]["print"] = "false";
    ini["gpio"]["use"] = "true";
  }
  if (true)
  {
    if (ini["gpio"]["use"] == "true")
    {
      chip = gpiod_chip_open("/dev/gpiochip4");
      if (chip == nullptr)
      {
        chip = gpiod_chip_open("/dev/gpiochip0");
        if (chip != nullptr)
          printf("# SGpiod::setup: found GPIO chip gpiochip0\n");
        else
        {
          chip = gpiod_chip_open("/dev/gpiochip1");
          if (chip != nullptr)
            printf("# SGpiod::setup: found GPIO chip gpiochip1\n");
        }
      }
      else
        printf("# SGpiod::setup: found GPIO chip gpiochip4\n");
    }
    if (chip != nullptr)
    { // set output ports
      // set output pins as specified
      int out_pin_value[MAX_PINS] = {0}; /// default value
      const char * p1 = ini["gpio"]["pins_out"].c_str();
      while (*p1 >= ' ')
      { // set output pins and initial value
        int pin = strtol(p1, (char**)&p1, 10);
        int v = 0;
        int idx = getPinIndex(pin);
        if (idx >= 0)
        {
          while (*p1 == ' ' and *p1 != '\0') p1++;
          if (*p1 == '\0')
            break;
          if (*p1 == '=')
            v = strtol(++p1, (char**)&p1, 10);
          else
          {
            printf("# SGpiod::setup: format 'pins_out=[ P=V]*' P=pin number, V=0|1 (found:%s)\n", ini["gpio"]["pins_out"].c_str());
            break;
          }
          out_pinuse[idx] = true;
          out_pin_value[idx] = v;
        }
        else
          printf("# SGpio::setup: found bad pin number in pin_out (%d)\n", pin);
      }
      // ignore first pin (start), handled by ip_disp
      for (int i = 0; i < MAX_PINS; i++)
      { // get handle to relevant pins and set output as specified
        // pins[i] = gpiod_chip_get_line(chip, pinOffset[i]);
        if (out_pinuse[i])
        { // output pin
          request[i] = request_output_line(pinOffset[i]);
          if (request[i] != nullptr)
            setPin(pinOffset[i], out_pin_value[i]);
          else
            printf("# GPIO::failed to reserve pin %d as output\n", pinOffset[i]);
        }
        else
        { // input pin
          request[i] = request_input_line(pinOffset[i]);
          if (request[i] == nullptr)
            printf("# GPIO::failed to reserve pin %d as input\n", pinOffset[i]);
        }
      }
    }
    else
    {
      if (ini["gpio"]["use"] == "true")
        printf("# SGpiod::setup there is no GPIO chip found - disable function\n");
      else
        printf("# SGpiod::setup GPIO disabled (in robot.ini)\n");
    }
  }
  // logfiles
  toConsole = ini["gpio"]["print"] == "true";
  if (chip != nullptr)
  {
    if (ini["gpio"]["log"] == "true" and logfile == nullptr)
    { // open logfile
      std::string fn = service.logPath + "log_gpio.txt";
      logfile = fopen(fn.c_str(), "w");
      fprintf(logfile, "%% gpio logfile\n");
      fprintf(logfile, "%% pins_out %s\n", ini["gpio"]["pins_out"].c_str());
      fprintf(logfile, "%% 1 \tTime (sec)\n");
  //     fprintf(logfile, "%% 2 \tPin %d (start)\n", pinOffset[0]);
      fprintf(logfile, "%% 2 \tPin %2d\n", pinOffset[0]);
    }
    if (not service.stop and th1 == nullptr)
      // start listen to the keyboard
      th1 = new std::thread(runObj, this);
  }
}

void SGpiod::terminate()
{
  if (th1 != nullptr)
    th1->join();
  if (logfile != nullptr)
  {
    fclose(logfile);
  }
  try
  {
    if (chip != nullptr)
    {
      for (int i = 0; i < MAX_PINS; i++)
        gpiod_line_request_release(request[i]);
    }
  }
  catch (...)
  {
    printf("#### SGPIO had a pin-release error\n");
  }
}


int SGpiod::getPinIndex(int pinOffset)
{ //   int pinOffset[MAX_PINS] = {13, 6, 12, 16, 19, 26, 21, 20};
  int result = -1;
  switch (pinOffset)
  {
    case 13: result = 0; break;
    case  6: result = 1; break;
    case 12: result = 2; break;
    case 16: result = 3; break;
    case 19: result = 4; break;
    case 26: result = 5; break;
    case 21: result = 6; break;
    case 20: result = 7; break;
    default:
      break;
  }
  return result;
}

int SGpiod::readPin(const int pin)
{
  int idx = getPinIndex(pin);
  int val = -1;
  if (chip != nullptr)
  {
    if (idx >= 0)
    {
      val = gpiod_line_request_get_value(request[idx], pin) == GPIOD_LINE_VALUE_ACTIVE;
      // val = gpiod_line_get_value(pins[idx]);
    }
    else
      printf("# SGpiod::readPin: pin %d is not valid, use one of: %d\n", pin,
          pinOffset[0]);
  }
  return val;
}

void SGpiod::setPin(const int pin, bool value)
{ // set one pin to value
  int idx = getPinIndex(pin);
  if (chip != nullptr)
  {
    if (idx >= 0 and out_pinuse[idx])
    {
      if (value)
        gpiod_line_request_set_value(request[idx], pin, GPIOD_LINE_VALUE_ACTIVE);
      else
        gpiod_line_request_set_value(request[idx], pin, GPIOD_LINE_VALUE_INACTIVE);
    }
    else
    { // not valid
      printf("# SGpiod::setPin: pin %d (idx=%d) is not set as output (in robot.ini) or invalid - call ignored\n", pin, idx);
    }
  }
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
    // mission_running = service.isThisProcessRunning("mqtt-client");
    // if (mr != mission_running)
    // {
    //   mr = mission_running;
    //   printf("Mission app has changed status to %d\n", mr);
    //   if (logfile != nullptr)
    //     fprintf(logfile, "# %lu.%04ld GPIO:: Mission app changed status to %d\n",
    //         t.getSec(), t.getMicrosec()/100, mr);
    // }
    // if (mission_running == 0)
    {
      for (int i = 0; i < MAX_PINS; i++)
      { // check all pins on the list
        pv[i] = readPin(pinOffset[i]);
        if (pv[i] >= 0)
        { // valid read
          // printf("# Pin %d read: first=%d, new=%d, old=%d\n",
          //        i, firstRead, pv[i], in_pin_value[i]);
          if (firstRead)
            in_pin_value[i] = pv[i];
          else if (pv[i] != in_pin_value[i])
          { // pin has changed value
            in_pin_value[i] = pv[i];
            changed = true;
            printf("# %lu.%04ld GPIO:: pin %d value changed to %d\n",
                   t.getSec(), t.getMicrosec()/100, pinOffset[i], pv[i]);
            // if (logfile != nullptr)
            //   fprintf(logfile, "# %lu.%04ld GPIO:: pin %d value changed to %d\n",
            //           t.getSec(), t.getMicrosec()/100, pinOffset[i], pv[i]);
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
          printf("# pin read not valid %d\n", pinOffset[i]);
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

int SGpiod::wait4Pin(int pin, uint timeout_ms, int wait4Value)
{
  int value = -1;
  UTime t("now");
  while (true and chip != nullptr)
  {
    int v = readPin(pin);
    if (v == wait4Value)
    {
      value = v;
      break;
    }
    usleep(500);
    float s = t.getTimePassed();
    float w = float(timeout_ms)/1000;
    if (s > w)
      break;
  }
  return value;
}

void SGpiod::toLog(bool pv[])
{ // pv is pin-value
  if (service.stop)
    return;
  UTime t("now");
  if (logfile != nullptr and not service.stop_logging)
  {
    fprintf(logfile,"%lu.%04ld %d\n",
            t.getSec(), t.getMicrosec()/100,
            pv[0]);
  }
  if (toConsole)
  {
    printf("%lu.%04ld %d\n",
            t.getSec(), t.getMicrosec()/100,
            pv[0]);
  }
}


struct gpiod_line_request * SGpiod::request_input_line(unsigned int offset)
{
  struct gpiod_request_config *req_cfg = NULL;
  struct gpiod_line_request *request = NULL;
  struct gpiod_line_settings *settings;
  struct gpiod_line_config *line_cfg;
  int ret;

  settings = gpiod_line_settings_new();
  if (!settings)
    return request;

  gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);

  line_cfg = gpiod_line_config_new();
  if (!line_cfg)
    goto free_settings;

  ret = gpiod_line_config_add_line_settings(line_cfg, &offset, 1,
                                            settings);
  if (ret)
    goto free_line_config;

  req_cfg = gpiod_request_config_new();
  if (!req_cfg)
    goto free_line_config;

  gpiod_request_config_set_consumer(req_cfg, line_user_name);

  request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

  gpiod_request_config_free(req_cfg);

  free_line_config:
  gpiod_line_config_free(line_cfg);

  free_settings:
  gpiod_line_settings_free(settings);

  return request;
}


struct gpiod_line_request * SGpiod::request_output_line(unsigned int offset)
{
  struct gpiod_request_config *req_cfg = NULL;
  struct gpiod_line_request *request = NULL;
  struct gpiod_line_settings *settings;
  struct gpiod_line_config *line_cfg;
  int ret;

  settings = gpiod_line_settings_new();
  if (!settings)
    return request;

  gpiod_line_settings_set_direction(settings,
                                    GPIOD_LINE_DIRECTION_OUTPUT);
  gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

  line_cfg = gpiod_line_config_new();
  if (!line_cfg)
    goto free_settings;

  ret = gpiod_line_config_add_line_settings(line_cfg, &offset, 1,
                                            settings);
  if (ret)
    goto free_line_config;

  req_cfg = gpiod_request_config_new();
  if (!req_cfg)
    goto free_line_config;
  gpiod_request_config_set_consumer(req_cfg, line_user_name);

  request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

  gpiod_request_config_free(req_cfg);

  free_line_config:
  gpiod_line_config_free(line_cfg);

  free_settings:
  gpiod_line_settings_free(settings);

  return request;
}
