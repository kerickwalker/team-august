/***************************************************************************
 *   Copyright (C) 2019-2022 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
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

#include "uimu2.h"
#include "ueeconfig.h"
#include "uencoder.h"
#include "urobot.h"
#include "uservice.h"


UImu2 imu2;



void UImu2::setup()
{
  initMpu();
  // subscription
  addPublistItem("gyro",  "Get calibrated gyro value as 'gyro gx gy gz' (deg/s)");
  addPublistItem("gyroo", "Get gyro offset 'gyroo ox oy oz'");
  addPublistItem("acc",   "Get accelerometer values 'acc ax ay az' (m/s^2)");
  addPublistItem("gyro0",  "Get calibrated gyro value as 'gyro0 gx gy gz' (deg/s) averaged for subscriber");
  addPublistItem("acc0",   "Get calibrated acc  value as 'acc0 ax ay az' (m/s^2) averaged for subscriber");
  usb.addSubscriptionService(this);
}

void UImu2::initMpu()
{
  // #if defined(REGBOT_HW4)
  // Wire.begin ( I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_1000 );
  // #else
  // #if defined(REGBOT_HW41) || defined(REGBOT_HW63_35)
  Wire.begin();
  //Wire.setClock(1000000);
  Wire.setClock(400000);
  // #else
  // Wire.begin ( I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_1000 );
  // #endif
  // #endif
  /* Initialize and configure IMU */
  mpu.setWire(&Wire);
  uint8_t id = 0;
  uint8_t retval = mpu.readId(&id);
  if (retval != 0)
  { // could not read from IPU
    usb.send("# Error initializing communication with IMU\n");
    imuAvailable = 0;
  }
  else
  { // print who am i
    const int MSL = 100;
    char s[MSL];
    snprintf(s, MSL, "# MPU9250 'who_am_i'=%d (0x%x)\n", id, id);
    usb.send(s);
    // set sensor scale scale
    mpu.beginAccel(ACC_FULL_SCALE_4_G);
    mpu.beginGyro(GYRO_FULL_SCALE_1000_DPS);
    if (useMag)
      mpu.beginMag(MAG_MODE_CONTINUOUS_100HZ);
  }
}


bool UImu2::decode(const char* cmd)
{
  bool found = true;
  if (strncmp(cmd, "gyrocal ", 8) == 0)
  {
    const char * p1 = &cmd[8];
    offsetGyro[0] = strtof(p1, (char**)&p1);
    offsetGyro[1] = strtof(p1, (char**)&p1);
    offsetGyro[2] = strtof(p1, (char**)&p1);
  }
  else if (strncmp(cmd, "gyroc", 5) == 0)
  {
    gyroOffsetDone = false;
  }
  else if (strncmp(cmd, "imuon ", 6) == 0)
  {
    char * p1 = (char*)&cmd[6];
    int e = strtol(p1, &p1, 10);
    /*int m =*/ strtol(p1, &p1, 10);
    // int mag = strtol(p1, &p1, 10);
    if (e == 1)
    { // start IMU (initialize)
      if (imuAvailable == 0)
      { // not available, try again
        initMpu();
        imuAvailable = 10;
        tickCnt = 0;
        usb.send("# initializing MPU9250\n");
      }
    }
    else
    {
      imuAvailable = 0;
      usb.send("# Stopped using MPU9250\n");
    }
  }
  else
    found = false;
  return found;
}

void UImu2::sendHelp()
{
  const int MRL = 300;
  char reply[MRL];
  usb.send("# IMU -------\r\n");
  // info requests
  usb.send(            "# -- \tgyrocal \tSet gyro calibration values (offset[3])\n");
  snprintf(reply, MRL, "# -- \tgyroc \tStart gyro calibration (finished=%d)\r\n", gyroOffsetDone);
  usb.send(reply);
  //   usb.send(            "# -- \tmagcal \tSet magnetometer calibration values (offset[3] rotate/scale[9])\n");
  //   usb.send(            "# -- \tacccal \tSet accelerometer calibration values (offset[3] scale[3])\n");
  //   usb.send(            "# -- \tacccal2 \tmake simple acceleration calibration with horizontal board\n");
  //   usb.send(            "# -- \tboard rX rY rZ \tSet IMU board orientation angles (in radians)\r\n");
  usb.send(            "# -- \timuon E F M \tEnable IMU (E=1), Madgwick (F=1), use magnetometer (M=1) \r\n");
}

void UImu2::tick()
{ // read data - first time will fail
  tickCnt++;
  if (tickCnt < 20)
  {
    const int MSL = 100;
    char s[MSL];
    snprintf(s, MSL,"# UImu2::tick %lu, sampleTime = %dus, imuavail=%d\n", tickCnt, sampleTime_us, imuAvailable);
    usb.send(s);
  }
  if (imuAvailable > 0)
  {
    uint32_t nt = micros();
    int dt = nt - lastRead;
    if (true /*dt >= 90 and dt < 10000*/)
    { // data should be imuAvailable
      // there seems to be a problem, if dt <= 100 (1ms).
      lastRead = nt;
      sampleTime_us = dt;
      //       imuSec = tsec;
      imuuSec = service.time_us;
      if (mpu.accelUpdate() == 0)
      { // rectify raw data - except magnetometer
        // - seems like z-acceleration has a wrong sign
        //   from the MPU9250_asukiaaa library
        //   as z is up, then z acceleration should be positive
        float accw[3];
        accw[0] = mpu.accelX();
        accw[1] = mpu.accelY();
        accw[2] = -mpu.accelZ();
        // average for raw data costumer
        acc0[0] += accw[0];
        acc0[1] += accw[1];
        acc0[2] += accw[2];
        acc0Cnt++;
        //
        for (int i = 0; i < 3; i++)
          acc[i] = accw[i] * accScale[i]; // - accOffset[i];
      }
      // Gyro
      if (mpu.gyroUpdate() == 0)
      { // gyro
        if (gyroOffsetDone)
        { // production
          // - seems like z-rotation has a wrong sign
          //   from the MPU9250_asukiaaa library
          // as z is up, then turning left should be positive
          gyro[0] = mpu.gyroX() - offsetGyro[0];
          gyro[1] = mpu.gyroY() - offsetGyro[1];
          gyro[2] = -mpu.gyroZ() - offsetGyro[2];
          // average for subscriber
          gyro0[0] += mpu.gyroX()  - offsetGyro[0];
          gyro0[1] += mpu.gyroY()  - offsetGyro[1];
          gyro0[2] += -mpu.gyroZ() - offsetGyro[2];
          gyro0Cnt++;
        }
        else
        { // calibrate
          // use raw values
          gyro[0] = mpu.gyroX();
          gyro[1] = mpu.gyroY();
          gyro[2] = -mpu.gyroZ();
          // start calibration
          if (tickCnt < gyroOffsetStartCnt)
          { // zero offset before 1000 summations
            offsetGyro[0] = 0;
            offsetGyro[1] = 0;
            offsetGyro[2] = 0;
          }
          else if (tickCnt <= gyroOffsetStartCnt + 1000)
          { // not finished offset calculation
            // summation over 1 second
            offsetGyro[0] += gyro[0];
            offsetGyro[1] += gyro[1];
            offsetGyro[2] += gyro[2];
            if ((tickCnt - gyroOffsetStartCnt) % 100 == 0)
            {
              const int MSL = 150;
              char s[MSL];
              snprintf(s, MSL,"# UImu2::gyrooffset n=%lu, gx=%g sumgx=%g\n", tickCnt - gyroOffsetStartCnt, gyro[0], offsetGyro[0]);
              usb.send(s);
            }
            if (tickCnt == gyroOffsetStartCnt + 1000)
            { // set average offset
              offsetGyro[0] /= 1000;
              offsetGyro[1] /= 1000;
              offsetGyro[2] /= 1000;
              usb.send("# gyro offset finished\r\n");
              gyroOffsetDone  = true;
            }
          }
          else
            // redo of calibrate requested
            gyroOffsetStartCnt = tickCnt + 10;
        }
        if (imuAvailable < 10)
          imuAvailable++;
      }
      else
      {
        imuAvailable--;
        if (imuAvailable == 0)
          usb.send("# message failed to read from MPU9250 10 times in a row, stopped trying\n");
      }
    }
    // if (not useMadgwich)
    { //  complementary tilt filter only
      estimateTilt();
      tiltOnly = true;
    }
  }
}

void UImu2::sendData(int item)
{
  if (item == 0)
    sendStatusGyro();
  else if (item == 1)
    sendGyroOffset();
  else if (item == 2)
    sendStatusAcc();
  else if (item == 3)
    sendRawGyro();
  else if (item == 4)
    sendRawAcc();
}

////////////////////////////////////////////////

void UImu2::eePromSave()
{
  uint8_t f = 1;
  // f |= useMadgwich << 1;
  eeConfig.pushByte(f);
  eeConfig.pushFloat(offsetGyro[0]);
  eeConfig.pushFloat(offsetGyro[1]);
  eeConfig.pushFloat(offsetGyro[2]);
}

void UImu2::eePromLoad()
{
  /*uint8_t f =*/ eeConfig.readByte();
  //bool useImu = f & 0x01;
  //
  offsetGyro[0] = eeConfig.readFloat();
  offsetGyro[1] = eeConfig.readFloat();
  offsetGyro[2] = eeConfig.readFloat();
  gyroOffsetDone = true;
}

////////////////////////////////////////////


void UImu2::sendStatusGyro()
{
  const int MRL = 250;
  char reply[MRL];
  snprintf(reply, MRL, "gyro %f %f %f %.3f\r\n",
           gyro[0], gyro[1], gyro[2], service.time_sec());
  usb.send(reply);
}

void UImu2::sendStatusAcc()
{
  const int MRL = 250;
  char reply[MRL];
  snprintf(reply, MRL, "acc %f %f %f %.3f\r\n",
           acc[0], acc[1], acc[2], service.time_sec());
  usb.send(reply);
}

void UImu2::sendGyroOffset()
{
  const int MRL = 250;
  char reply[MRL];
  snprintf(reply, MRL, "gyroo %f %f %f\r\n",
           offsetGyro[0],
           offsetGyro[1],
           offsetGyro[2]);
  usb.send(reply);
}

void UImu2::sendImuPose()
{
  const int MRL = 150;
  char reply[MRL];
  // if (tiltOnly)
  snprintf(reply, MRL, "imupose %f %f %f\r\n",
           0.0, encoder.pose[3], 0.0);
  // else
  //   snprintf(reply, MRL, "imupose %f %f %f\r\n",
  //            getRollRadians(), getPitchRadians(), getYawRadians());
  usb.send(reply);
}

void UImu2::sendRawAcc()
{
  float div = acc0Cnt;
  const int MSL = 250;
  char s[MSL];
  if (acc0Cnt == 0)
    snprintf(s, MSL, "acc0 %g %g %g %d\n", mpu.accelX(), mpu.accelY(), mpu.accelZ(), acc0Cnt);
  else
    snprintf(s, MSL, "acc0 %g %g %g %d\n", acc0[0]/div, acc0[1]/div, acc0[2]/div, acc0Cnt);
  usb.send(s);
  acc0[0] = 0;
  acc0[1] = 0;
  acc0[2] = 0;
  acc0Cnt = 0;
}

void UImu2::sendRawGyro()
{
  float div = gyro0Cnt;
  const int MSL = 250;
  char s[MSL];
  if (gyro0Cnt == 0)
    snprintf(s, MSL, "gyro0 %g %g %g %d\n", mpu.gyroX(), mpu.gyroY(), mpu.gyroZ(), gyro0Cnt);
  else
    snprintf(s, MSL, "gyro0 %g %g %g %d\n", gyro0[0]/div, gyro0[1]/div, gyro0[2]/div, gyro0Cnt);
  usb.send(s);
  gyro0[0] = 0;
  gyro0[1] = 0;
  gyro0[2] = 0;
  gyro0Cnt = 0;
}


/**
 * estimate tilt angle, as complementary filter with gyro and acc
 *       1     tau s                     1
 * Gyro ---  ----------  + acc_pitch --------
 *       s    tau s + 1              tau s + 1
 *
 *     1        T/(T+2.*tau) + *T/(T+2.*tau) * z^-1
 * --------- = -------------------------------------
 *  tau s + 1     1 + (T-2.*tau)/(T+2.*tau) * z^-1
 *
 * T = 0.001;
 * tau = 1.0;
 *  */
/// tilt angle estimator
// float tiltu1  = 0; // old value for complementary filter
// float accAng;   // for debug
// float gyroTiltRate;
void UImu2::estimateTilt()
{ // use actual sample time
  float T = sampleTime_us * 1e-6;
  float tau = 1.0; // seems to give good response
  float b = T/(T + 2 * tau);
  float a = -(T - 2 * tau)/(T + 2 * tau);
  float u; // input to filter
  float est; // estimated angle
  // gyro mounted on top plate!
  accAng = atan2f(-float(acc[0]),-float(acc[2]));
  // offset with value that makes the robot balance
  // accAng -= rY; // rY is board rotation on yAxis (tilt offset)
  // New and old angle must be in same revolution
  if ((accAng - encoder.pose[3]) > M_PI)
    accAng -= 2*M_PI;
  else if ((accAng - encoder.pose[3]) < -M_PI)
    accAng += 2*M_PI;
  // gyro is running in mode 2 (0= 250 grader/sek, 1 = 500 deg/s, 2=1000 deg/s 3=2000 deg/s)
  gyroTiltRate = gyro[1] * M_PI / 180.0; // radianer pr sekund
  // add gyro and accelerometer reading
  u = accAng + gyroTiltRate * tau;
  if (true) // imuGyro[0] < 245 and imuGyro[0] > -245)
  { // gyro not saturated
    // filter
    if (accAng > 0.0 and encoder.pose[3] < -M_PI/2.0)
      est = a * (encoder.pose[3] + 2 * M_PI) + b * u + b * tiltu1;
    else if (accAng < 0.0 and encoder.pose[3] > M_PI/2.0)
      est = a * (encoder.pose[3] - 2 * M_PI) + b * u + b * tiltu1;
    else
      est = a * encoder.pose[3] + b * u + b * tiltu1;
  }
  else
    // else use angle as is from accelerometer
    est = accAng;
  // debug
  if (false and tickCnt % 100 == 0)
  {
    const int MSL = 200;
    char s[MSL];
    snprintf(s, MSL, "# est tilt:: at %.3f accAng=%.4f, gyro=%.4f, u=%.4f, est=%.4f, u1=%.4f, a=%.4f, b=%.4f, T=%.4f\n",
             service.time_sec(),
             accAng, gyroTiltRate, u, est, tiltu1, a, b, T);
    usb.send(s);
  }
  // debug end
  //
  if (est > M_PI)
  { // folded
    est -= 2 * M_PI;
    // save last value of u in right angle space
    tiltu1 = accAng - 2 * M_PI + gyroTiltRate * tau;
  }
  else if (est < -M_PI)
  { // folded
    est += 2 * M_PI;
    tiltu1 = accAng - 2 * M_PI + gyroTiltRate * tau;
  }
  else
  { // no folding
    tiltu1 = u;
  }
  //
  encoder.pose[3] = est;
}
