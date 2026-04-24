# sedge_core.py — stripped, commented line-following module
# See notes/code_map_line_following.md for where each value comes from / goes to.
# Original: sedge.py (DTU/MIT). This file is for understanding only; run with sedge.py.

from datetime import *
import time as t
from threading import Thread
import cv2 as cv
from ulog import flog

class SEdge:
    # ═══════════════════════════════════════════════════════════════════════════
    # SECTION: Raw and normalized line sensor state (INPUT from MQTT)
    # ═══════════════════════════════════════════════════════════════════════════
    # Source: Teensy → teensy_interface → MQTT robobot/drive/T0/edge_livn (and liv, liw)
    # Filled in decode() when message on T0/livn (or T0/liv, T0/liw)
    edge = [0] * 8           # raw AD (T0/liv)
    edgeUpdCnt = 0
    edgeTime = datetime.now()
    edgeInterval = 0.0       # ms between updates (measured from timestamps)
    edge_n_w = [0] * 8       # white calibration values (T0/liw)
    edge_n_wUpdCnt = 0
    edge_n_wTime = datetime.now()
    edge_n = [0] * 8         # normalized 0–1000 (T0/livn) — MAIN INPUT for control
    edge_nUpdCnt = 0
    edge_nTime = datetime.now()
    edge_nInterval = 0.0
    edgeIntervalSetup = 0.1  # expected interval (sec) for Lead filter tuning

    # ═══════════════════════════════════════════════════════════════════════════
    # SECTION: Line detection thresholds and outputs (set in LineDetect())
    # ═══════════════════════════════════════════════════════════════════════════
    lineValidThreshold = 750   # 1000 = calibrated white; above this = "on line"
    crossingThreshold = 700    # average above this = crossing (e.g. perpendicular line)
    low = 0                   # darkest in current sample (line 61 overwrites class low)
    posLeft = 0.0             # left edge position in sensor index space (-3.5 .. 3.5)
    posRight = 0.0             # right edge position
    followLeft = True         # True = follow left edge, False = right (set in lineControl)
    refPosition = 0.0          # setpoint: desired distance from chosen edge
    lineValid = False         # true if high >= lineValidThreshold
    lineValidCnt = 0          # smoothed 0..20; used by mqtt-client to decide "on line" / "lost"
    crossingLine = False
    crossingLineCnt = 0
    average = 0.0             # mean of edge_n[0..7]
    high = 0                  # max(edge_n)

    # Calibration / one-shot
    topicLip = ""
    sendCalibRequest = False

    # ═══════════════════════════════════════════════════════════════════════════
    # SECTION: Line controller (P + Lead) — OUTPUT to MQTT rc
    # ═══════════════════════════════════════════════════════════════════════════
    lineCtrl = False          # when True, each livn causes followLine() → rc
    lineKp = 1.0              # rad/s per unit position error
    lineTauZ = 0.8            # Lead zero (sec)
    lineTauP = 0.25           # Lead pole (sec)
    tauP2pT = 1.0             # Lead coeffs (recomputed in PIDrecalculate from sample interval)
    tauP2mT = 0.0
    tauZ2pT = 1.0
    tauZ2mT = 0.0
    lineE1 = 0.0              # previous error * Kp (for Lead)
    lineY1 = 0.0              # previous control output (rad/s)
    lineY = 0.0               # current turn rate (rad/s) — sent in rc
    topicCmdT0 = "robobot/cmd/T0"
    lostLineCnt = 0
    u = 0.0                   # turn rate before saturation

    # ═══════════════════════════════════════════════════════════════════════════
    # setup() — called once from uservice.setup()
    # Sends: lip 1, sub livn 10; if --white: litb, livi, liwi, licw 100, eew, liwi
    # Waits until edge_n (or edge_n_w) data has arrived, then returns.
    # ═══════════════════════════════════════════════════════════════════════════
    def setup(self):
        from uservice import service
        sendBlack = False
        loops = 0
        self.topicCmdT0 = "robobot/cmd/T0"
        service.send(self.topicCmdT0, "lip 1")           # turn line sensor ON on Teensy
        service.send(self.topicCmdT0, "sub livn 10")     # request normalized values every 10 ms
        while not service.stop:
            t.sleep(0.02)
            if service.args.white:
                if not sendBlack:
                    service.send(self.topicCmdT0, "litb 0 0 0 0 0 0 0 0")
                    sendBlack = True
                elif self.edgeUpdCnt < 3:
                    service.send(self.topicCmdT0, "livi")
                elif not self.sendCalibRequest:
                    service.send(self.topicCmdT0, "liwi")
                    t.sleep(0.02)
                    service.send(self.topicCmdT0, "licw 100")
                    t.sleep(0.25)
                    service.send(self.topicCmdT0, "eew")
                    self.sendCalibRequest = True
                    service.send(self.topicCmdT0, "liwi")
                    t.sleep(0.02)
                else:
                    t.sleep(0.25)
                    service.args.white = False
                    service.stop = True
            elif self.edge_n_wUpdCnt == 0:
                service.send(self.topicCmdT0, "liwi")
            elif self.edge_nUpdCnt == 0:
                pass
            else:
                break
            loops += 1
            if loops > 30:
                break

    # ═══════════════════════════════════════════════════════════════════════════
    # decode(topic, msg) — called from uservice.decode() for each MQTT message
    # topic is subtopic after stripping "robobot/drive/", e.g. "T0/livn", "T0/liv", "T0/liw"
    # Parses payload into edge[] or edge_n[] or edge_n_w[], then LineDetect(); if lineCtrl, followLine()
    # ═══════════════════════════════════════════════════════════════════════════
    def decode(self, topic, msg):
        used = True
        if topic == "T0/liv":
            gg = msg.split(" ")
            if len(gg) >= 9:
                t0 = self.edgeTime
                self.edgeTime = datetime.fromtimestamp(float(gg[0]))
                for i in range(8):
                    self.edge[i] = int(gg[i + 1])
                t1 = self.edgeTime
                if self.edgeUpdCnt == 2:
                    self.edgeInterval = (t1 - t0).total_seconds() * 1000
                elif self.edgeUpdCnt > 2:
                    self.edgeInterval = (self.edgeInterval * 99 + (t1 - t0).total_seconds() * 1000) / 100
                self.edgeUpdCnt += 1
        elif topic == "T0/livn":
            gg = msg.split(" ")
            if len(gg) >= 9:
                t0 = self.edge_nTime
                self.edge_nTime = datetime.fromtimestamp(float(gg[0]))
                for i in range(8):
                    self.edge_n[i] = int(gg[i + 1])
                t1 = self.edge_nTime
                if self.edge_nUpdCnt == 2:
                    self.edge_nInterval = (t1 - t0).total_seconds() * 1000
                elif self.edge_nUpdCnt > 2:
                    self.edge_nInterval = (self.edge_nInterval * 99 + (t1 - t0).total_seconds() * 1000) / 100
                self.edge_nUpdCnt += 1
                self.LineDetect()
                if self.lineCtrl:
                    self.followLine()
                if self.edge_nUpdCnt % 10 == 0:
                    flog.write()
        elif topic == "T0/liw":
            gg = msg.split(" ")
            if len(gg) >= 9:
                self.edge_n_wTime = datetime.fromtimestamp(float(gg[0]))
                for i in range(8):
                    self.edge_n_w[i] = int(gg[i + 1])
                self.edge_n_wUpdCnt += 1
        else:
            used = False
        return used

    # ═══════════════════════════════════════════════════════════════════════════
    # LineDetect() — uses edge_n[] only. Computes:
    #   average, high, lineValid (high >= threshold), crossingLine (average >= crossingThreshold),
    #   posLeft/posRight by scanning from left/right until above lineValidThreshold,
    #   lineValidCnt and crossingLineCnt (smoothed 0..20).
    # ═══════════════════════════════════════════════════════════════════════════
    def LineDetect(self):
        s = sum(self.edge_n)
        high = 1
        for i in range(8):
            if self.edge_n[i] > high:
                high = self.edge_n[i]
        self.high = high
        self.average = s / 8.0
        self.crossingLine = self.average >= self.crossingThreshold
        self.lineValid = self.high >= self.lineValidThreshold
        if self.lineValid:
            posLeft = -3.5
            if self.edge_n[0] < self.lineValidThreshold:
                posLeft = -3
                for i in range(1, 8):
                    if self.edge_n[i] < self.lineValidThreshold:
                        posLeft += 1
                    else:
                        break
            posRight = 3.5
            if self.edge_n[7] < self.lineValidThreshold:
                posRight = 3
                for i in range(1, 8):
                    if self.edge_n[7 - i] < self.lineValidThreshold:
                        posRight -= 1
                    else:
                        break
            self.posLeft = posLeft
            self.posRight = posRight
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
            self.crossingLineCnt = max(0, self.crossingLineCnt - 1)

    # ═══════════════════════════════════════════════════════════════════════════
    # lineControl(velocity, followLeft, refPosition) — called from mqtt-client
    # Sets: self.velocity, self.followLeft, self.refPosition; lineCtrl = (velocity > 0.001)
    # ═══════════════════════════════════════════════════════════════════════════
    def lineControl(self, velocity, followLeft=True, refPosition=0):
        self.velocity = velocity
        self.followLeft = followLeft
        self.refPosition = refPosition
        self.lineCtrl = velocity > 0.001

    # ═══════════════════════════════════════════════════════════════════════════
    # followLine() — called from decode() after LineDetect() when lineCtrl is True
    # Error e = refPosition - posLeft (or posRight); P-Lead → lineY (rad/s); saturate ±4;
    # Sends: service.send("robobot/cmd/ti", f"rc {velocity} {lineY} {time}")
    # ═══════════════════════════════════════════════════════════════════════════
    def followLine(self):
        from uservice import service
        if abs(self.edge_nInterval - self.edgeIntervalSetup) > 2.0:
            self.PIDrecalculate()
            self.edgeIntervalSetup = self.edge_nInterval
        if self.followLeft:
            e = self.refPosition - self.posLeft
        else:
            e = self.refPosition - self.posRight
        self.u = self.lineKp * e
        self.lineY = (self.u * self.tauZ2pT - self.lineE1 * self.tauZ2mT + self.lineY1 * self.tauP2mT) / self.tauP2pT
        if self.lineY > 4:
            self.lineY = 4
        elif self.lineY < -4:
            self.lineY = -4
        self.lineE1 = self.u
        self.lineY1 = self.lineY
        par = f"rc {self.velocity:.3f} {self.lineY:.3f} {t.time()}"
        service.send("robobot/cmd/ti", par)

    def PIDrecalculate(self):
        Tsec = self.edge_nInterval / 1000
        self.tauP2pT = self.lineTauP * 2.0 + Tsec
        self.tauP2mT = self.lineTauP * 2.0 - Tsec
        self.tauZ2pT = self.lineTauZ * 2.0 + Tsec
        self.tauZ2mT = self.lineTauZ * 2.0 - Tsec

    # ═══════════════════════════════════════════════════════════════════════════
    # terminate() — called from uservice.terminate(). Sends lip 0 to turn sensor off.
    # ═══════════════════════════════════════════════════════════════════════════
    def terminate(self):
        from uservice import service
        self.need_data = False
        service.send(self.topicCmdT0, "lip 0")

    # paint() — optional OpenCV overlay on camera image; not needed for control path.
    def paint(self, img):
        pass

edge = SEdge()
