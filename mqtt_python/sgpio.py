#/***************************************************************************
#*   Copyright (C) 2024 by DTU
#*   jcan@dtu.dk
#*
#*
#* The MIT License (MIT)  https://mit-license.org/
#*
#* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#* and associated documentation files (the “Software”), to deal in the Software without restriction,
#* including without limitation the rights to use, copy, modify, merge, publish, distribute,
#* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
#* is furnished to do so, subject to the following conditions:
#*
#* The above copyright notice and this permission notice shall be included in all copies
#* or substantial portions of the Software.
#*
#* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
#* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#* THE SOFTWARE. */


import time as t
from datetime import *

gpioFound = False
_busy_pins = []
try:
  import RPi.GPIO as GPIO
  GPIO.setmode(GPIO.BCM)
  for _pin in [6, 12, 13, 16, 19, 26, 21, 20]:  # 6=red/stop, 13=green/start
    try:
      GPIO.setup(_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    except Exception:
      _busy_pins.append(_pin)
  gpioFound = True
  try:
    from uservice import service
    if not service.is_quiet():
      if _busy_pins:
        print(f"% BCM GPIO found OK (busy/skipped pins: {_busy_pins})")
      else:
        print("% BCM GPIO found OK")
  except Exception:
    if _busy_pins:
      print(f"% BCM GPIO found OK (busy/skipped pins: {_busy_pins})")
    else:
      print("% BCM GPIO found OK")
except:
  print("% No GPIO (not on a Pi?) - continue without GPIO support")
  pass


class SGpio:
    onPi = False

    def setup(self):
      from uservice import service
      if gpioFound:
        # import RPi.GPIO as GPIO
        if not service.is_quiet():
          print("% GPIO setup start")
        GPIO.setwarnings(False)
        # list = [13, 12, 16, 19, 26, 21, 20]
        for _p in [6, 13]:  # 6=stop, 13=start
          try:
            GPIO.setup(_p, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
          except Exception:
            if not service.is_quiet():
              print(f"% GPIO pin {_p} busy/unavailable - skipped")
        if not service.is_quiet():
          print("% GPIO setup finished (pin 6=stop, 13=start)")
        self.onPi = True

    def test_stop_button(self):
      """Red button (GPIO 6): returns True when pressed. Stops mission."""
      if self.onPi:
        v = self.get_value(6)
        if v:
          print("% Button/pin 6 (stop) is pressed")
        return v
      else:
        return False

    def test_start_button(self):
      """Green button (GPIO 13): returns True when pressed. Starts mission."""
      if self.onPi:
        return self.get_value(13)
      else:
        return False

    def set_value(self, line, value):
      if self.onPi:
        # import RPi.GPIO as GPIO
        if True: # try:
          isIn = GPIO.gpio_function(line)
          if isIn:
            GPIO.setup(line, GPIO.OUT)
          GPIO.output(line, value)
        # except:
        #   print(f"% GPIO set pin {line} can't be set (got error trying)")
      pass

    def get_value(self, line):
      if self.onPi:
        # import RPi.GPIO as GPIO
        if True: # try:
          isIn = GPIO.gpio_function(line)
          if not isIn:
            GPIO.setup(line, GPIO.IN)
          v = GPIO.input(line)
          if (v == 1):
            print(f"% Button/pin {line} is pressed (high)")
          return v == 1
        # except:
        #   print(f"% GPIO get pin {line} can't be set (got error trying)")
      else:
        return False

    # def print(self):
    #   from uservice import service
    #   print("% GPIO button " + str(self.accTime - service.startTime))

    def decode(self, topic, msg):
      # decode MQTT message
      # none yet (may come, if teensy_interface listens to GPIO too).
      used = False
      return used


    def terminate(self):
      from uservice import service
      if self.onPi:
        # import RPi.GPIO as GPIO
        GPIO.cleanup()
        pass
      if not service.is_quiet():
        print("% GPIO terminated")
      pass

# create the data object
gpio = SGpio()

