/** @file AS5X47.cpp
 *
 * @brief A library for Arduino boards that reads angles from AS5047 and AS5147 sensors.
 * 		  Also support configuration of the sensor parameters.
 *
 * @par
 * COPYRIGHT NOTICE: MIT License
 *
 * 	Copyright (c) 2020 Adrien Legrand <contact@adrien-legrand.com>
 *
 * 	Permission is hereby granted, free of charge, to any person obtaining a copy
 * 	of this software and associated documentation files (the "Software"), to deal
 * 	in the Software without restriction, including without limitation the rights
 * 	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * 	copies of the Software, and to permit persons to whom the Software is
 * 	furnished to do so, subject to the following conditions:
 *
 * 	The above copyright notice and this permission notice shall be included in all
 * 	copies or substantial portions of the Software.
 *
 * 	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * 	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * 	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * 	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * 	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * 	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * 	SOFTWARE.
 *
*/

#include "AS5X47.h"
#include "uusb.h"

AS5X47::AS5X47(uint8_t chipSelectPin) : spi(chipSelectPin) {
}

ReadDataFrame AS5X47::readRegister(uint16_t registerAddress)
{
  ReadDataFrame receivedFrame;
  receivedFrame.raw = 0;
  CommandFrame command;
  command.values.commandFrame = registerAddress;
  command.values.rw = READ;
  command.values.parc = 0; //isEven(command.raw); // actually don't care

  CommandFrame nopCommand;
  nopCommand.values.commandFrame = NOP_REG;
  nopCommand.values.rw = READ;
  nopCommand.values.parc = 0; //isEven(nopCommand.raw);
  receivedFrame.raw = spi.readData(command.raw, nopCommand.raw);
	return receivedFrame;
}



void AS5X47::writeRegister(uint16_t registerAddress, uint16_t registerValue)
{ // form write word to register
	CommandFrame command;
	command.values.commandFrame = registerAddress;
  command.values.rw = WRITE;
  command.values.parc = 0; //isEven(command.raw);
  // write data
	WriteDataFrame contentFrame;
	contentFrame.values.data = registerValue;
	contentFrame.values.low = 0;
	contentFrame.values.pard = 0; //isEven(contentFrame.raw);
	spi.writeData(command.raw, contentFrame.raw);
}

float AS5X47::readAngle() {
	ReadDataFrame readDataFrame = readRegister(ANGLE_COMPSATED);
	Angle angle;
	angle.raw = readDataFrame.values.data;
	return angle.values.cordicang/16384.*360.;
}

float AS5X47::readVel()
{
  ReadDataFrame readDataFrame = readRegister(ANGLE_VEL);
  Angle angle;
  angle.raw = readDataFrame.values.data;
  // 14 bit signed - default is +/-28000 RPM or 24.141 deg/s per LSB
  // can be changed in settings?
  // convert to deg/s
  // extend to get sign
  int16_t a = angle.values.cordicang << 2;
  return a*24.141 / 4.;
}

Errfl AS5X47::readErr()
{
  ReadDataFrame readDataFrame = readRegister(ERRFL_REG);
  Errfl err;
  err.raw = readDataFrame.values.data;
  // 14 bit signed - default is +/-28000 RPM or 24.141 deg/s per LSB
  // can be changed in settings?
  // convert to deg/s
  return err;
}

Diaagc AS5X47::readDiag()
{
  ReadDataFrame readDataFrame = readRegister(DIAG_REG);
  Diaagc dia;
  dia.raw = readDataFrame.values.data;
  // 14 bit signed - default is +/-28000 RPM or 24.141 deg/s per LSB
  // can be changed in settings?
  // convert to deg/s
  return dia;
}


void AS5X47::writeSettings1(Settings1 values) {
	writeRegister(SETTINGS1_REG, values.raw);
}
void AS5X47::writeSettings2(Settings2 values){
	writeRegister(SETTINGS2_REG, values.raw);
}
void AS5X47::writeZeroPosition(Zposm zposm, Zposl zposl){
	writeRegister(ZPOSM_REG, zposm.raw);
	writeRegister(ZPOSL_REG, zposl.raw);
}

void AS5X47::printDebugString() {
	ReadDataFrame readDataFrame;
	readDataFrame = readRegister(ERRFL_REG);
	Errfl errfl;
	errfl.raw = readDataFrame.values.data;
  const int MSL = 200;
  char s[MSL];
  usb.send("#======== AS5X47 Debug ========\r\n");
  snprintf(s, MSL, "#------- ERRFL Register :\r\n");
  usb.send(s);
	snprintf(s, MSL, "#   Read Warning %d, Read Error: %d, flags 0x%04x (see AS5247U manual reg 0x0001)\r\n",
           readDataFrame.values.pard, readDataFrame.values.ef, errfl.raw);
  usb.send(s);

	readDataFrame = readRegister(DIAG_REG);
	Diaagc diaagc;
	diaagc.raw = readDataFrame.values.data;
  usb.send("#------- DIAG Register: \r\n");
  snprintf(s, MSL, "#   Read Error: %d, Flags: 0x%04x (hex, see AS5147U manual, reg=0x3FF5)\r\n",
           readDataFrame.values.ef, diaagc.raw);
  usb.send(s);

	readDataFrame = readRegister(MAG_REG);
	Mag mag;
	mag.raw = readDataFrame.values.data;
  usb.send("#------- MAG Register: \r\n");
  snprintf(s, MSL, "#   (%x) Read Error: %d, CMAG: %d\r\n", MAG_REG,
           readDataFrame.values.ef, mag.values.cmag);
  usb.send(s);

	readDataFrame = readRegister(ANGLE_REG);
	Angle angle;
	angle.raw = readDataFrame.values.data;
  usb.send("#------- Angle Register: \r\n");
  snprintf(s, MSL, "#   (%x) Read Error: %d, CORDIANG: %d\r\n", ANGLE_REG,
           readDataFrame.values.ef, angle.values.cordicang);
  usb.send(s);


  readDataFrame = readRegister(ANGLE_VEL);
  Angle velocity;
  velocity.raw = readDataFrame.values.data;
  angle.raw = readDataFrame.values.data;
  usb.send("#------- Velocity Register: \r\n");
  snprintf(s, MSL, "#   (%x) Read Error: %d, CORDIANG: %d\r\n", ANGLE_VEL,
           readDataFrame.values.ef, velocity.values.cordicang);
  usb.send(s);

  readDataFrame = readRegister(ANGLECOM_REG);
	Anglecom anglecom;
	anglecom.raw = readDataFrame.values.data;
  usb.send("#------- AngleCom Register: \r\n");
  snprintf(s, MSL, "#   (%x) Read Error: %d, DAECANG: %d\r\n", ANGLECOM_REG,
           readDataFrame.values.ef, anglecom.values.daecang);
  usb.send(s);


// 	readDataFrame = readRegister(ZPOSM_REG);
// 	Zposm zposm;
// 	zposm.raw = readDataFrame.values.data;
// 	Serial.println("|------- ZPOSM Register: ");
// 	Serial.print("|   Reading Error: ");
// 	Serial.println(readDataFrame.values.ef);
// 	Serial.print("|   ZPOSM: ");
// 	Serial.println(zposm.values.zposm);
// 	Serial.println("|");
//
// 	readDataFrame = readRegister(ZPOSL_REG);
// 	Zposl zposl;
// 	zposl.raw = readDataFrame.values.data;
// 	Serial.println("|------- ZPOSL Register: ");
// 	Serial.print("|   Reading Error: ");
// 	Serial.println(readDataFrame.values.ef);
// 	Serial.print("|   ZPOSL: ");
// 	Serial.println(zposl.values.zposl);
// 	Serial.print("|   COMP_L_ERROR_EN: ");
// 	Serial.println(zposl.values.compLerrorEn);
// 	Serial.print("|   COMP_H_ERROR_EN: ");
// 	Serial.println(zposl.values.compHerrorEn);
// 	Serial.println("|");

	readDataFrame = readRegister(SETTINGS1_REG);
	Settings1 settings1;
	settings1.raw = readDataFrame.values.data;
  usb.send("#------- Settings1 Register: \r\n");
  snprintf(s, MSL, "#   (%x) Read Error: %d, settings1: 0x%x\r\n", SETTINGS1_REG,
           readDataFrame.values.ef, settings1.raw);
  usb.send(s);

//   Serial.println("|------- SETTINGS1 Register: ");
// 	Serial.print("|   Reading Error: ");
// 	Serial.println(readDataFrame.values.ef);
// 	Serial.print("|   NOISESET: ");
// 	Serial.println(settings1.values.noiseset);
// 	Serial.print("|   DIR: ");
// 	Serial.println(settings1.values.dir);
// 	Serial.print("|   UVW_ABI: ");
// 	Serial.println(settings1.values.uvw_abi);
// 	Serial.print("|   DAECDIS: ");
// 	Serial.println(settings1.values.daecdis);
// 	Serial.print("|   ABIBIN: ");
// 	Serial.println(settings1.values.abibin);
// 	Serial.print("|   DATASELECT: ");
// 	Serial.println(settings1.values.dataselect);
// 	Serial.print("|   PWMON: ");
// 	Serial.println(settings1.values.pwmon);
// 	Serial.println("|");

	readDataFrame = readRegister(SETTINGS2_REG);
	Settings2 settings2;
	settings2.raw = readDataFrame.values.data;
  usb.send("#------- Settings2 Register: \r\n");
  snprintf(s, MSL, "#   (%x) Read Error: %d, settings1: 0x%x\r\n", SETTINGS2_REG,
           readDataFrame.values.ef, settings2.raw);
  usb.send(s);

//   Serial.println("|------- SETTINGS2 Register: ");
// 	Serial.print("|   Reading Error: ");
// 	Serial.println(readDataFrame.values.ef);
// 	Serial.print("|   UVWPP: ");
// 	Serial.println(settings2.values.uvwpp);
// 	Serial.print("|   HYS: ");
// 	Serial.println(settings2.values.hys);
// 	Serial.print("|   ABIRES: ");
// 	Serial.println(settings2.values.abires);
//
// 	Serial.println("==============================");


/*ANGLECOM_REG 	0x3FFF

// Non-Volatile Registers Addresses
#define ZPOSM_REG 		0x0016
#define ZPOSL_REG 		0x0017
#define SETTINGS1_REG 	0x0018
#define SETTINGS2_REG 	0x0019*/
}


bool AS5X47::isEven(uint16_t data) {
	int count=0;
	unsigned int b = 1;
	for (unsigned int i=0; i<15; i++) {
		if (data & (b << i)) {
			count++;
		}
	}
	if (count%2==0) {
		return false;
	} else {
		return true;
	}
}
