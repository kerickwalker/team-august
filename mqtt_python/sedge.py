

from datetime import *
import time as t
from threading import Thread
import cv2 as cv
from ulog import flog

class SEdge:


    defaultLineVelocity = 0.2
    lineValidThreshold = 500
    crossingMinSensors = 4

    yMinSensors = 4
    yMinSpan    = 4
    yMidIndices = (3, 4)

    lineKp = 0.4
    lineKi = 0.0
    lineKd = 0.0
    lineIntegralLimit = 2.0
    lineOutputAlpha = 0.5
    lineDerivativeBeta = 0.0
    lineYMax = 3.0
    lineYMin = -3.0

    recoveryTurnRate = 1.0
    recoveryVelocity = 0.0

    TS_NOMINAL = 0.010
    TS_DRIFT_WARN_FRAC = 0.30
    

    PARAM_SETS = {
        "normal": dict(lineKp=0.3, lineKi=0.0, lineKd=0.05,
                       lineIntegralLimit=2.0, lineOutputAlpha=0.0,
                       lineDerivativeBeta=0.0, lineVelocity=0.20),
        "slow":   dict(lineKp=0.2, lineKi=0.0, lineKd=0.02,
                       lineIntegralLimit=2.0, lineOutputAlpha=0.0,
                       lineDerivativeBeta=0.0, lineVelocity=0.15),
    }

    print_follow_line_block = True
    follow_line_print_every_n = 1
    flog_write_every_n = 1

    print_follow_line_fields = (
        'livn', 'high', 'valid', 'validCnt', 'state', 'aboveCnt',
        'crossingCnt', 'leftMost', 'rightMost', 'center', 'e', 'y'
    )


    edge = [0, 0, 0 , 0, 0, 0, 0, 0]
    edgeUpdCnt = 0
    edgeTime = datetime.now()
    edgeInterval = 0

    edge_n_w = [0, 0, 0 , 0, 0, 0, 0, 0]
    edge_n_wUpdCnt = 0
    edge_n_wTime = datetime.now()

    edge_n = [0, 0, 0 , 0, 0, 0, 0, 0]
    edge_nUpdCnt = 0
    edge_nTime = datetime.now()
    edge_nInterval = 0

    posLeft = 0.0
    posRight = 0.0
    lineCenterWeighted = 0.0
    refPosition = 0.0
    lineValid = False
    lineValidCnt = 0
    crossingLine = False
    crossingLineCnt = 0
    average = 0
    high = 0

    sensorAboveThreshold = [False] * 8
    sensorsAboveCount = 0
    leftmostAboveIndex = None
    rightmostAboveIndex = None
    lineState = "no_line"
    mission_crossing_count = 0

    topicLip = ""
    sendCalibRequest = False
    calibCollecting = False
    calibWhiteMax = [0] * 8


    lineCtrl = False

    lineIntegral = 0.0
    lineE_prev = None
    lineE_filt = 0.0
    lineE_filt_prev = None
    lineY1 = 0.0
    lineY = 0.0

    lastLineSide = 0

    topicCmdT0 = ""
    u = 0


    def setup(self):
      from uservice import service
      sendBlack = False
      loops = 0

      if not service.is_quiet():
        print("% Edge (sedge.py):: turns on line sensor")
      self.topicCmdT0 = "robobot/cmd/T0"
      service.send(self.topicCmdT0, "lip 1")

      service.send(self.topicCmdT0,"sub livn 3")
      service.send(self.topicCmdT0,"sub liv 3")

      while not service.stop:
        t.sleep(0.02)

        if service.args.white:
          if not sendBlack:

            topic = self.topicCmdT0
            param = "litb 0 0 0 0 0 0 0 0"
            sendBlack = service.send(topic, param)
          elif self.edgeUpdCnt < 3:

            service.send(self.topicCmdT0,"livi")
            pass
          elif not self.sendCalibRequest:

            if not service.is_quiet():
              print("% Edge (sedge.py):: Place robot on line; will roll forward then back to capture white level")
            self.calibWhiteMax = [0] * 8
            self.calibCollecting = True

            service.send("robobot/cmd/ti", "rc 0.12 0.0")
            t.sleep(2.5)

            service.send("robobot/cmd/ti", "rc -0.12 0.0")
            t.sleep(2.5)
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            self.calibCollecting = False

            w = [max(1, self.calibWhiteMax[i]) for i in range(8)]
            service.send(self.topicCmdT0, "litw " + " ".join(str(v) for v in w))
            t.sleep(0.1)
            service.send(self.topicCmdT0, "eew")
            self.sendCalibRequest = True
            if not service.is_quiet():
              print(f"% Edge (sedge.py):: white set from rolling calib: {w}")
            service.args.white = False
            service.stop = True
          else:
            t.sleep(0.25)
            service.args.white = False
            if not service.is_quiet():
              print(f"% Edge (sedge.py):: calibration done, terminates.")
            service.stop = True
        elif self.edge_n_wUpdCnt == 0:

          service.send(self.topicCmdT0,"liwi")
          pass
        elif self.edge_nUpdCnt == 0:

          pass
        else:
          if not service.is_quiet():
            print(f"% Edge (sedge.py):: got data stream; after {loops} loops")
          break
        loops += 1
        if loops > 30:
          if not service.is_quiet():
            print(f"% Edge (sedge.py):: got no data after {loops} (continues edge_n_wUpdCnt={self.edge_n_wUpdCnt}, edgeUpdCnt={self.edgeUpdCnt}, edge_nUpdCnt={self.edge_nUpdCnt})")
          break
      pass


    def print(self):
      from uservice import service
      if not service.is_quiet():
        print("% Edge (sedge.py):: " + str(self.edgeTime - service.startTime) +
            f" ({self.edge[0]}, " +
            f"{self.edge[1]}, " +
            f"{self.edge[2]}, " +
            f"{self.edge[3]}, " +
            f"{self.edge[4]}, " +
            f"{self.edge[5]}, " +
            f"{self.edge[6]}, " +
            f"{self.edge[7]})" +
            f" {self.edgeInterval:.2f} ms " +
            str(self.edgeUpdCnt))
    def printn(self):
      from uservice import service
      if not service.is_quiet():
        print("% Edge (sedge.py):: normalized " + str(self.edge_nTime - service.startTime) +
              f" ({self.edge_n[0]}, " +
              f"{self.edge_n[1]}, " +
              f"{self.edge_n[2]}, " +
              f"{self.edge_n[3]}, " +
              f"{self.edge_n[4]}, " +
              f"{self.edge_n[5]}, " +
              f"{self.edge_n[6]}, " +
              f"{self.edge_n[7]})" +
              f" {self.edge_nInterval:.2f} ms " +
              str(self.edge_nUpdCnt))
    def printnw(self):
      from uservice import service
      if not service.is_quiet():
        print("% Edge (sedge.py):: white level " + str(self.edge_n_wTime) +
            f" ({self.edge_n_w[0]}, " +
            f"{self.edge_n_w[1]}, " +
            f"{self.edge_n_w[2]}, " +
            f"{self.edge_n_w[3]}, " +
            f"{self.edge_n_w[4]}, " +
            f"{self.edge_n_w[5]}, " +
            f"{self.edge_n_w[6]}, " +
            f"{self.edge_n_w[7]}) " +
            str(self.edge_n_wUpdCnt))


    def decode(self, topic, msg):

        used = True
        if topic == "T0/liv":
          from uservice import service
          gg = msg.split(" ")
          if (len(gg) >= 4):
            t0 = self.edgeTime;
            self.edgeTime = datetime.fromtimestamp(float(gg[0]))
            self.edge[0] = int(gg[1])
            self.edge[1] = int(gg[2])
            self.edge[2] = int(gg[3])
            self.edge[3] = int(gg[4])
            self.edge[4] = int(gg[5])
            self.edge[5] = int(gg[6])
            self.edge[6] = int(gg[7])
            self.edge[7] = int(gg[8])
            t1 = self.edgeTime;
            if self.edgeUpdCnt == 2:
              self.edgeInterval = (t1 -t0).total_seconds()*1000
            elif self.edgeUpdCnt > 2:
              self.edgeInterval = (self.edgeInterval * 99 + (t1 -t0).total_seconds()*1000) / 100
            self.edgeUpdCnt += 1

            if getattr(self, 'calibCollecting', False):
              for i in range(8):
                self.calibWhiteMax[i] = max(self.calibWhiteMax[i], self.edge[i])

        elif topic == "T0/livn":
          from uservice import service
          gg = msg.split(" ")
          if (len(gg) >= 4):
            t0 = self.edge_nTime;
            self.edge_nTime = datetime.fromtimestamp(float(gg[0]))
            self.edge_n[0] = int(gg[1])
            self.edge_n[1] = int(gg[2])
            self.edge_n[2] = int(gg[3])
            self.edge_n[3] = int(gg[4])
            self.edge_n[4] = int(gg[5])
            self.edge_n[5] = int(gg[6])
            self.edge_n[6] = int(gg[7])
            self.edge_n[7] = int(gg[8])
            t1 = self.edge_nTime;
            if self.edge_nUpdCnt == 2:
              self.edge_nInterval = (t1 -t0).total_seconds()*1000
            elif self.edge_nUpdCnt > 2:
              self.edge_nInterval = (self.edge_nInterval * 99 + (t1 -t0).total_seconds()*1000) / 100
            self.edge_nUpdCnt += 1


            self.LineDetect()


            if self.lineCtrl:
              self.followLine()


        elif topic == "T0/liw":
          from uservice import service
          gg = msg.split(" ")
          if (len(gg) >= 4):
            self.edge_n_wTime = datetime.fromtimestamp(float(gg[0]))
            self.edge_n_w[0] = int(gg[1])
            self.edge_n_w[1] = int(gg[2])
            self.edge_n_w[2] = int(gg[3])
            self.edge_n_w[3] = int(gg[4])
            self.edge_n_w[4] = int(gg[5])
            self.edge_n_w[5] = int(gg[6])
            self.edge_n_w[6] = int(gg[7])
            self.edge_n_w[7] = int(gg[8])
            self.edge_n_wUpdCnt += 1

        else:
          used = False
        return used


    def LineDetect(self):
      total = 0
      high = int(1)


      for i in range(8):
        total += self.edge_n[i]
        if self.edge_n[i] > high:
          high = self.edge_n[i]
      self.high = high


      self.average = total / 8.0

      for i in range(8):
        self.sensorAboveThreshold[i] = self.edge_n[i] >= self.lineValidThreshold
      self.sensorsAboveCount = sum(1 for b in self.sensorAboveThreshold if b)
      self.leftmostAboveIndex = None
      self.rightmostAboveIndex = None
      for i in range(8):
        if self.sensorAboveThreshold[i]:
          if self.leftmostAboveIndex is None:
            self.leftmostAboveIndex = i
          self.rightmostAboveIndex = i

      T_pattern = self.sensorsAboveCount >= self.crossingMinSensors
      Y_pattern = False
      if (self.sensorsAboveCount >= self.yMinSensors
          and self.leftmostAboveIndex is not None
          and self.rightmostAboveIndex is not None
          and (self.rightmostAboveIndex - self.leftmostAboveIndex) >= self.yMinSpan):

        Y_pattern = any(not self.sensorAboveThreshold[i] for i in self.yMidIndices)
      self.crossingLine = T_pattern or Y_pattern

      if self.sensorsAboveCount == 0:
        self.lineState = "no_line"
      elif self.crossingLine:
        self.lineState = "crossing"
      else:
        self.lineState = "line"

      self.lineValid = self.high >= self.lineValidThreshold
      active = 0
      sumPos = 0.0
      for i in range(8):
        if self.sensorAboveThreshold[i]:
          active += 1
          sumPos += i - 3.5
      if active > 0:
        self.lineCenterWeighted = sumPos / active

      if self.lineValid:
        posLeft = -3.5
        if self.edge_n[0] < self.lineValidThreshold:
          posLeft = -3
          for i in range(1,8):
            if self.edge_n[i] < self.lineValidThreshold:
              posLeft += 1;
            else:
              break;
        posRight = 3.5
        if self.edge_n[7] < self.lineValidThreshold:
          posRight = 3
          for i in range(1,8):
            if self.edge_n[7-i] < self.lineValidThreshold:
              posRight -= 1;
            else:
              break;
        self.posLeft = posLeft
        self.posRight = posRight
      else:

        pass

      if self.lineValid and self.lineValidCnt < 20:
        self.lineValidCnt += 1
      elif not self.lineValid:
        if self.lineValidCnt > 0:
          self.lineValidCnt -= 1
        else:
          self.lineValidCnt = 0
      if self.crossingLine and self.crossingLineCnt < 20:
        self.crossingLineCnt += 1
      elif not self.crossingLine:
        self.crossingLineCnt -= 1
        if self.crossingLineCnt < 0:
          self.crossingLineCnt = 0
      pass


    def lineControl(self, velocity=None, refPosition=0, params="normal"):

      if velocity is None:
        velocity = self.defaultLineVelocity
      self.velocity = velocity
      self.refPosition = refPosition


      self.lineCtrl = abs(velocity) > 0.001
      if params in self.PARAM_SETS:
        for k, v in self.PARAM_SETS[params].items():
          setattr(self, k, v)
        self.lineIntegral = 0.0
        self.lineE_filt_prev = None


    def followLine(self):
      from uservice import service

      lineCenter = self.lineCenterWeighted
      e = self.refPosition - lineCenter


      Tsec = self.TS_NOMINAL
      if (self.edge_nInterval > 0
          and self.edge_nUpdCnt > 100
          and not getattr(self, '_ts_warned', False)
          and abs(self.edge_nInterval / 1000.0 - self.TS_NOMINAL) / self.TS_NOMINAL
              > self.TS_DRIFT_WARN_FRAC):
        if not service.is_quiet():
          print(f"% sedge: measured Ts {self.edge_nInterval:.1f} ms differs "
                f">{self.TS_DRIFT_WARN_FRAC*100:.0f}% from TS_NOMINAL "
                f"{self.TS_NOMINAL*1000:.0f} ms — review TS_NOMINAL")
        self._ts_warned = True


      if not self.lineValid:
        self.lineIntegral = 0.0
        self.lineE_prev = e
        self.lineE_filt = e
        self.lineE_filt_prev = None
      elif self.lineDerivativeBeta > 0.0:

        if self.lineE_filt_prev is None:
          self.lineE_filt = e
        else:
          b = self.lineDerivativeBeta
          self.lineE_filt = b * self.lineE_filt + (1.0 - b) * e
      else:
        self.lineE_filt = e


      if self.lineE_filt_prev is not None:
        dTerm = (self.lineE_filt - self.lineE_filt_prev) / Tsec
      else:
        dTerm = 0.0
      self.lineE_filt_prev = self.lineE_filt
      self.lineE_prev = e


      if self.lineValid:
        self.lineIntegral += e * Tsec
        if self.lineIntegral > self.lineIntegralLimit:
          self.lineIntegral = self.lineIntegralLimit
        elif self.lineIntegral < -self.lineIntegralLimit:
          self.lineIntegral = -self.lineIntegralLimit


      pTerm = self.lineKp * e
      iTerm = self.lineKi * self.lineIntegral
      self.u = pTerm + iTerm + self.lineKd * dTerm
      raw_y = self.u
      if raw_y > self.lineYMax:
        raw_y = self.lineYMax
      elif raw_y < self.lineYMin:
        raw_y = self.lineYMin

      alpha = max(0.0, min(1.0, self.lineOutputAlpha))
      self.lineY = (1.0 - alpha) * raw_y + alpha * self.lineY1
      self.lineY1 = self.lineY

      if self.lineValid:
        if e > 0:
          self.lastLineSide = 1
        elif e < 0:
          self.lastLineSide = -1


      if not self.lineValid and self.lineCtrl:
        recovery_turn = self.recoveryTurnRate * (self.lastLineSide if self.lastLineSide != 0 else 1)
        if recovery_turn > self.lineYMax:
          recovery_turn = self.lineYMax
        elif recovery_turn < self.lineYMin:
          recovery_turn = self.lineYMin
        self.lineY = recovery_turn
        sent_velocity = self.recoveryVelocity
        sent_turn = self.lineY
      else:
        sent_velocity = self.velocity
        sent_turn = self.lineY


      if sent_velocity < 0.0:
        sent_turn = -sent_turn
      par = f"rc {sent_velocity:.3f} {sent_turn:.3f} {t.time()}"
      service.send("robobot/cmd/ti", par)

      if self.print_follow_line_block and (getattr(service.args, 'test', False) or not getattr(service.args, 'silent', True)) and self.edge_nUpdCnt > 0 and self.edge_nUpdCnt % self.follow_line_print_every_n == 0:
        enabled = set(self.print_follow_line_fields)
        parts = []
        if 'livn' in enabled:
          norm = " ".join(f"{self.edge_n[i]:4d}" for i in range(8))
          parts.append(f"livn [{norm}]")
        if 'avg' in enabled:
          parts.append(f"avg={self.average:6.0f}")
        if 'high' in enabled:
          parts.append(f"high={self.high:4d}")
        if 'valid' in enabled:
          parts.append(f"valid={str(self.lineValid):5}")
        if 'validCnt' in enabled:
          parts.append(f"validCnt={self.lineValidCnt:2d}")
        if 'posL' in enabled:
          parts.append(f"posL={self.posLeft:5.2f}")
        if 'posR' in enabled:
          parts.append(f"posR={self.posRight:5.2f}")
        if 'center' in enabled:
          parts.append(f"center={lineCenter:5.2f}")
        if 'cross' in enabled:
          parts.append(f"cross={str(self.crossingLine):5}")
        if 'state' in enabled:
          parts.append(f"state={self.lineState}")
        if 'aboveCnt' in enabled:
          parts.append(f"aboveCnt={self.sensorsAboveCount}")
        if 'crossingCnt' in enabled:
          parts.append(f"crossingCnt={self.mission_crossing_count}")
        if 'leftMost' in enabled:
          parts.append(f"leftMost={self.leftmostAboveIndex if self.leftmostAboveIndex is not None else '-'}")
        if 'rightMost' in enabled:
          parts.append(f"rightMost={self.rightmostAboveIndex if self.rightmostAboveIndex is not None else '-'}")
        if any(k in enabled for k in ('e', 'p', 'i', 'd', 'dTerm', 'integral', 'u', 'y', 'rc', 'lastLineSide')):
          parts.append("|")
        if 'e' in enabled:
          parts.append(f"e={e:6.3f}")
        if 'p' in enabled:
          parts.append(f"p={pTerm:6.3f}")
        if 'i' in enabled:
          parts.append(f"i={iTerm:6.3f}")
        if 'd' in enabled:
          parts.append(f"d={self.lineKd*dTerm:6.3f}")
        if 'dTerm' in enabled:
          parts.append(f"dTerm={dTerm:6.3f}")
        if 'integral' in enabled:
          parts.append(f"integral={self.lineIntegral:6.3f}")
        if 'u' in enabled:
          parts.append(f"u={self.u:6.3f}")
        if 'y' in enabled:
          parts.append(f"y={self.lineY:6.3f}")
        if 'rc' in enabled:
          parts.append(f"-> rc {sent_velocity:.3f} {sent_turn:.3f}")
        if 'lastLineSide' in enabled:
          parts.append(f"lastSide={self.lastLineSide:+d}")
        if parts:
          print("% line: " + " ".join(parts))


    def terminate(self):
      from uservice import service
      self.need_data = False
      if not service.is_quiet():
        print("% Edge (sedge.py):: turn off line sensor")
      service.send(self.topicCmdT0, "lip 0")
      if not service.is_quiet():
        print("% Edge (sedge.py):: terminated")
      pass


    def paint(self, img):
      h, w, ch = img.shape
      pl = int(h - h/4)
      st = int(w/10)
      gh = int(h/2)
      x = st
      y = pl
      dtuGreen = (0x35, 0x88, 0)
      dtuBlue = (0xea, 0x3e, 0x2f)
      dtuRed = (0x00, 0x00, 0x99)
      dtuPurple = (0x8e, 0x23, 0x77)

      cv.line(img, (x,y), (int(x + 7*st), int(y)), dtuGreen, thickness=1, lineType=8)

      cv.line(img, (x,int(y-gh)), (int(x + 7*st), int(y-gh)), dtuGreen, thickness=1, lineType=8)

      cv.line(img, (x,int(y-gh*self.lineValidThreshold/1000.0)), (int(x + 7*st), int(y-gh*self.lineValidThreshold/1000.0)), dtuBlue, thickness=1, lineType=4)

      for i in range(8):
        y = int(pl - self.edge_n[i]/1000 * gh)
        cv.drawMarker(img, (x,y), dtuRed, markerType=cv.MARKER_STAR, thickness=2, line_type=8, markerSize = 10)
        x += st

      from uservice import service
      if not service.is_quiet():
        print(f" Edge::paint: posLeft {self.posLeft}, right {self.posRight}")
      pixP = int((self.posLeft + 4.5)*st)
      cv.line(img, (pixP, int(pl)), (pixP, int(pl-gh)), dtuRed, thickness=3, lineType=4)
      pixP = int((self.posRight + 4.5)*st)
      cv.line(img, (pixP, int(pl)), (pixP, int(pl-gh)), dtuGreen, thickness=3, lineType=4)

      pixL = pl - int(gh * 0.0)
      cv.line(img, (st, pixL), (st*8, pixL), dtuRed, thickness=1, lineType=4)

      cv.putText(img, "Left", (st,pl - 2), cv.FONT_HERSHEY_PLAIN, 1, dtuPurple, thickness=2)
      cv.putText(img, "Right", (int(st+6*st),pl - 2), cv.FONT_HERSHEY_PLAIN, 1, dtuPurple, thickness=2)
      cv.putText(img, "White (1000)", (int(st),pl - gh - 2), cv.FONT_HERSHEY_PLAIN, 1, dtuPurple, thickness=2)
      if self.crossingLine:
        cv.putText(img, "Crossing", (int(st),int(pl - 20)), cv.FONT_HERSHEY_PLAIN, 1, dtuRed, thickness=2)


edge = SEdge()

