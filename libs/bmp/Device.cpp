/*
	I2C Version - mac
  Device is an I2C/SPI compatible base class library.
	
	Copyright (C) Martin Lindupp 2020
	
	V1.0.0 -- Initial release
	V1.0.1 -- Modification to allow user-defined pins for I2C operation on the ESP8266
	V1.0.2 -- Modification to allow user-defined pins for I2C operation on the ESP32
	V1.0.3 -- Initialise "device" constructor member variables in the same order they are declared
	V1.0.4 -- Fix incorrect oversampling definition for x1, thanks to myval for raising the issue
	V1.0.5 -- Modification to allow ESP8266 SPI operation, thanks to Adam9850 for the generating the pull request
	V1.0.6 -- Fix compilation issue with Arduino Due
	V1.0.7 -- Allow for additional TwoWire instances
	V1.0.8 -- Fix compilation issue with STM32 Blue Pill
	V1.0.9 -- Fixed uninitialised "Wire" pointer for ESP8266/ESP32 with user defined I2C pins 
	
	The MIT License (MIT)
	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:
	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#include "Device.h"

////////////////////////////////////////////////////////////////////////////////
// Device Class Constructors
////////////////////////////////////////////////////////////////////////////////

Device::Device(TwoWire& twoWire) : i2c(&twoWire) {}								// Initialise constructor for I2C communications

////////////////////////////////////////////////////////////////////////////////
// Device Public Member Function
////////////////////////////////////////////////////////////////////////////////

void Device::setClock(uint32_t clockSpeed)													// Set the I2C or SPI clock speed
{
	i2c->setClock(clockSpeed);
}


////////////////////////////////////////////////////////////////////////////////
// Device I2C (Protected) Member Functions
////////////////////////////////////////////////////////////////////////////////

void Device::initialise()																						// Initialise device communications
{
	i2c->begin();																										// Initialise I2C communication
	i2c->setClock(400000);																					// Set the SCL clock to default of 400kHz
}

void Device::setI2CAddress(uint8_t addr)														// Set the Device's I2C address
{	
	address = addr;
}

void Device::writeByte(uint8_t subAddress, uint8_t data)
{
	i2c->beginTransmission(address);  															// Write a byte to the sub-address using I2C
	i2c->write(subAddress);          
	i2c->write(data);                 
	i2c->endTransmission();          
}

uint8_t Device::readByte(uint8_t subAddress)												// Read a byte from the sub-address using I2C
{
  uint8_t data = 0x00;
	i2c->beginTransmission(address);         
	i2c->write(subAddress);                  
	i2c->endTransmission(false);             
	i2c->requestFrom(address, (uint8_t)1);	 
	data = i2c->read();                      
  return data;                             													// Return data read from sub-address register
}

void Device::readBytes(uint8_t subAddress, uint8_t* data, uint16_t count)
{  
  // Read "count" bytes into the "data" buffer using I2C
	i2c->beginTransmission(address);          
	i2c->write(subAddress);                   
	i2c->endTransmission(false);              
	uint8_t i = 0;
	i2c->requestFrom(address, (uint8_t)count);  
	while (i2c->available()) 
	{
		data[i++] = i2c->read();          
	}
}
