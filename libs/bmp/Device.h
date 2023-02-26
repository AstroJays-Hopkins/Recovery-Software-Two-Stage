/*
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

#ifndef Device_h
#define Device_h

#include <Arduino.h>
#include <Wire.h>


////////////////////////////////////////////////////////////////////////////////
// Device Communications
////////////////////////////////////////////////////////////////////////////////
					 
// Only I2c

////////////////////////////////////////////////////////////////////////////////
// Device Class definition
////////////////////////////////////////////////////////////////////////////////

class Device{
	public:
		Device(TwoWire& twoWire);																		// Device object for I2C operation
	
		void setClock(uint32_t clockSpeed);													// Set the I2C clock speed
	protected:
		void initialise();																					// Initialise communications	
		void setI2CAddress(uint8_t addr);											  		// Set the Device I2C address
		void writeByte(uint8_t subAddress, uint8_t data);						// I2C write byte wrapper function
		uint8_t readByte(uint8_t subAddress);												// I2C read byte wrapper function
		void readBytes(uint8_t subAddress, uint8_t* dest, uint16_t count);		// I2C  read bytes wrapper function	
	private:
		uint8_t address;																						// The device I2C address
		TwoWire* i2c;																								// Pointer to the Wire class
};
#endif
