/*
  This is a library written for the MMC5983MA High Performance Magnetometer.
  SparkFun sells these at its website:
  https://www.sparkfun.com/products/19034

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, February 2nd, 2022.
  This file implements all functions used in the MMC5983MA High Performance Magnetometer Arduino Library IO layer.

  SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
  See LICENSE.md for more information.
*/

#include "SparkFun_MMC5983MA_IO.h"
#include "SparkFun_MMC5983MA_Arduino_Library_Constants.h"

// Read operations must have the most significant bit set
#define READ_REG(x) (0x80 | x)

bool SFE_MMC5983MA_IO::begin(TwoWire &i2cPort)
{
    useSPI = false;
    _i2cPort = &i2cPort;
    return isConnected();
}


bool SFE_MMC5983MA_IO::isConnected()
{
    bool result;
    _i2cPort->beginTransmission(I2C_ADDR);
    result = (_i2cPort->endTransmission() == 0);
    if (result)
    {
        uint8_t id = 0;
        result &= readSingleByte(PROD_ID_REG, &id);
        result &= id == PROD_ID;
    }
    return result;
}

bool SFE_MMC5983MA_IO::writeMultipleBytes(const uint8_t registerAddress, uint8_t *const buffer, uint8_t const packetLength)
{
    bool success = true;
    _i2cPort->beginTransmission(I2C_ADDR);
    _i2cPort->write(registerAddress);
    for (uint8_t i = 0; i < packetLength; i++)
        _i2cPort->write(buffer[i]);
    success = _i2cPort->endTransmission() == 0;
    return success;
}

bool SFE_MMC5983MA_IO::readMultipleBytes(const uint8_t registerAddress, uint8_t *const buffer, const uint8_t packetLength)
{
    bool success = true;
    _i2cPort->beginTransmission(I2C_ADDR);
    _i2cPort->write(registerAddress);
    success = _i2cPort->endTransmission() == 0;

    uint8_t returned = _i2cPort->requestFrom(I2C_ADDR, packetLength);
    for (uint8_t i = 0; (i < packetLength) && (i < returned); i++)
        buffer[i] = _i2cPort->read();
    success &= returned == packetLength;
    return success;
}

bool SFE_MMC5983MA_IO::readSingleByte(const uint8_t registerAddress, uint8_t *buffer)
{
    bool success = true;
    
    _i2cPort->beginTransmission(I2C_ADDR);
    _i2cPort->write(registerAddress);
    success = _i2cPort->endTransmission() == 0;
    
    uint8_t returned = _i2cPort->requestFrom(I2C_ADDR, 1U);
    if (returned == 1)
        *buffer = _i2cPort->read();
    success &= returned == 1;
    
    return success;
}

bool SFE_MMC5983MA_IO::writeSingleByte(const uint8_t registerAddress, const uint8_t value)
{
    bool success = true;
    _i2cPort->beginTransmission(I2C_ADDR);
    _i2cPort->write(registerAddress);
    _i2cPort->write(value);
    success = _i2cPort->endTransmission() == 0;
    return success;
}

bool SFE_MMC5983MA_IO::setRegisterBit(const uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t value = 0;
    bool success = readSingleByte(registerAddress, &value);
    value |= bitMask;
    success &= writeSingleByte(registerAddress, value);
    return success;
}

bool SFE_MMC5983MA_IO::clearRegisterBit(const uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t value = 0;
    bool success = readSingleByte(registerAddress, &value);
    value &= ~bitMask;
    success &= writeSingleByte(registerAddress, value);
    return success;
}

bool SFE_MMC5983MA_IO::isBitSet(const uint8_t registerAddress, const uint8_t bitMask)
{
    uint8_t value = 0;
    readSingleByte(registerAddress, &value);
    return (value & bitMask);
}