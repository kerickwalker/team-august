# find moving image.
#
# running the program pops up a window to watch the video.
# the program video window shows the first monitor,
# but watch the program video window on second extended monitor

import cv2 as cv
import numpy as np
# from threading import Thread
import time as t
from datetime import *
#from ucam import cam
from scam import cam


####################################################

class ImAna:
  img0 = {}
  imgMerge = {}
  state = 0
  cnt = 0
  baseCnt = 0;
  def imageAnalysis(self, save):
    if cam.useCam:
      ok, img, imgTime = cam.getImage()
      if not ok: # size(img) == 0):
        if cam.imageFailCnt < 5:
          print(f"% Failed to get image ({cam.imageFailCnt}).")
      else:
        h, w, ch = img.shape
        img2 = img.copy()

        # if not service.args.silent:
        #   # print(f"% At {imgTime}, got image {cam.cnt} of size= {w}x{h}")
        #   pass
        # edge.paint(img)
        if True:
          cv.imshow('new frame', img2)
        if self.state == 0:
          # new base image
          self.img0 = img2.copy()
          self.imgMerge = img2.copy()
          self.imgTime = imgTime
          self.cnt = 0
          self.state = 1
          self.baseCnt += 1
        elif self.state == 1:
          # first image with data
          diff = cv.absdiff(self.img0, img2)
          gray = cv.cvtColor(diff, cv.COLOR_BGR2GRAY)
          # cv.imshow("diff image", gray)
          # isum = cv.integral(gray)
          # print(f"integral sum {isum}")
          ret, thresh = cv.threshold(gray, 30, 255, 0)
          mask = cv.merge((thresh, thresh, thresh))
          mask2 = cv.bitwise_not(thresh)
          cv.imshow("thresholded image", thresh)
          fg = cv.bitwise_and(self.imgMerge, self.imgMerge, mask=mask2)
          # cv.imshow("foreground", fg)
          bg = cv.bitwise_and(img2, img2, mask=thresh)
          # cv.imshow("background", bg)
          self.imgMerge = cv.add(fg, bg)
          cv.imshow("merged", self.imgMerge)
          self.cnt += 1
          # self.state = 2
          # contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        pass
      pass
    pass

  def save(self):
    fn = f"image_{self.imgTime.strftime('%Y_%b_%d_%H%M%S_')}{self.cnt:03d}.jpg"
    cv.imwrite(fn, self.imgMerge)
    print(f"% Saved image {fn}")

  def saveImg0(self):
    fn = f"image_base_{self.baseCnt:03d}.jpg"
    cv.imwrite(fn, self.img0)
    print(f"% Saved base image as {fn}")


ana = ImAna()

####################################################

#cam.setup('192.168.2.252')
cam.setup('10.59.9.224')
#cam.setup('10.197.216.186')
while cam.cap.isOpened():
    # function extract frames
    # t.sleep(1)
    ana.imageAnalysis(False)
    print("# Timelap recording into one image")
    print("# press  q to quit")
    print("# press  b to save base image and reset")
    print("# press  r to save merged image and reset")
    print("# press  0 to just reset (no save)")
    aa = cv.waitKey(1)
    if aa & 0xFF == ord("q"):
        t.sleep(0.3)
        break
    if aa & 0xFF == ord("r"):
      ana.save()
      ana.state = 0
    if aa & 0xFF == ord("b"):
      ana.saveImg0()
      ana.state = 0
    if aa & 0xFF == ord("0"):
      ana.state = 0
    pass

# Everything done
cam.terminate()
