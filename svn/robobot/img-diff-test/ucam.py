#/***************************************************************************
#*   Copyright (C) 2025 by DTU
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

import numpy as np
import cv2 as cv
from threading import Thread
import time as t
from datetime import *
import urllib.request

class SCam:

  cap = {} # capture device
  th = {} # thread
  savedFrame = {}
  frameTime = datetime.now()
  getFrame = True
  cnt = 0
  gray = {}
  useCam = True
  imageFailCnt = 0
  stop = False;
  camhost = '192.168.2.251'
  streamOpen = True

  def setup(self, camhost):
    if self.useCam:
      self.camhost = camhost
    try:
      self.stream = urllib.request.urlopen(f'http://{self.camhost}:7123/stream.mjpg')
      print("# camera stream is open")
    except:
      self.streamOpen = False
      print("# camera stream failed to open " + self.camhost)
    if self.streamOpen:
      self.th = Thread(target = cam.run)
      self.th.start()
    else:
      print("% SCam:: Camera disabled (in scam.py)")
    print("# cam setup finished")

  def getImage(self):
    fail = False
    if not self.useCam:
      if self.imageFailCnt == 0:
        print("% SCam:: not using cam")
      fail = True
    if not fail and not self.streamOpen:
      if self.imageFailCnt == 0:
        print("% SCam:: could not open")
      fail = True
    if not fail:
      # from uservice import service
      self.getFrame = True
      cnt = 0 # timeout
      while self.getFrame and cnt < 100 and not self.stop:
        t.sleep(0.01)
        cnt += 1
      fail = self.getFrame
    if fail:
      self.imageFailCnt += 1
      return False, self.savedFrame, self.frameTime
    else:
      self.imageFailCnt = 0
      return True, self.savedFrame, self.frameTime

  def run(self):
    print("# camera thread running")
    cnt = 0;
    frameCnt = 0
    errCnt = 0;
    first = True
    total_bytes = b''
    while self.streamOpen and not self.stop:
      imgOK = False
      b = 0
      while self.streamOpen and not self.stop and not imgOK:
        # print(f"# Trying to get image {b}")
        total_bytes += self.stream.read(1024)
        b = total_bytes.find(b'\xff\xd9') # JPEG end
        if not b == -1:
          a = total_bytes.find(b'\xff\xd8') # JPEG start
          jpg = total_bytes[a:b+2] # actual image
          total_bytes = total_bytes[b+2:] # other informations
          print(f"# got a={a}, b={b}")
          # decode to colored image ( another option is cv.IMREAD_GRAYSCALE )
          #img = cv.imdecode(np.fromstring(jpg, dtype=np.uint8), cv.IMREAD_COLOR)
          img = cv.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv.IMREAD_COLOR)
          errCnt = 0
          imgOK = True
          frameCnt += 1
          print(f" Got image {frameCnt}")
      if imgOK:
        if (self.getFrame or first) and imgOK:
          self.frameTime = datetime.now()
          self.savedFrame = img.copy()
          self.cnt += 1
          self.getFrame = False
          print("# Used image {self.cnt}")
          if first:
            first = False
            h, w, ch = self.savedFrame.shape
            print(f"% Camera available: size ({h}x{w}, {ch} channels)")
          pass
        pass
      pass
    print("% Camera thread stopped")
    pass


  def terminate(self):
    self.stop = True
    try:
      self.th.join()
    except:
      print("% join cam failed")
      pass
    cv.destroyAllWindows()
    print("% Camera terminated")

# create instance of this class
cam = SCam()

