/**
 * @file ADIS16467.cpp
 * @author Rita Yang
 * @brief Software Driver for ADIS16467-3 Precision MEMS IMU Module
 *
 * @copyright Copyright (c) 2021
 *
 * Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/adis16467.pdf
 */

#include <inttypes.h>

#include "ADIS16467.h"

#define ADIS_DEBUG 1

ADIS16467::ADIS16467(
    PinName DIN, PinName DOUT, PinName SCLK, PinName CS, PinName RST)
    : spi(DIN, DOUT, SCLK)
    , cs(CS)
    , rst(RST)
{
}

void ADIS16467::initADIS()
{
    // reset ADIS
    deselect();
    rst.write(0);
    wait_us(10);

    resetTimer.reset();
    rst.write(1);
    resetTimer.start(); // start the reset timer
    spi.frequency(SPI_FREQ);
    spi.format(16, 3); // transmits 16 bits at a time, mode 3, master

    begin();
}

bool ADIS16467::checkExistence() { return (readRegister(Register::PROD_ID) == 16467); }

bool ADIS16467::hasNewData()
{
    int32_t curr_DC = readDataCounter();
    // check for data counter roll overs
    if (curr_DC < lastDC)
    {
        lastDC -= 0xFFFF;
    }
    return curr_DC > lastDC;
}

void ADIS16467::setErrorFlags(uint16_t statusDiagVal)
{
    clockError |= static_cast<bool>(statusDiagVal & (1 << 7));
    memoryFailure |= static_cast<bool>(statusDiagVal & (1 << 6));
    sensorFailure |= static_cast<bool>(statusDiagVal & (1 << 5));
    standbyMode |= static_cast<bool>(statusDiagVal & (1 << 4));
    comSPIError |= static_cast<bool>(statusDiagVal & (1 << 3));
    flashMemUpdateFailure |= static_cast<bool>(statusDiagVal & (1 << 2));
    datapathOverrun |= static_cast<bool>(statusDiagVal & (1 << 1));
}

void ADIS16467::resetAllFlags()
{
    clockError = false;
    memoryFailure = false;
    sensorFailure = false;
    standbyMode = false;
    comSPIError = false;
    flashMemUpdateFailure = false;
    datapathOverrun = false;
}

uint32_t ADIS16467::readResetTimer() { return std::chrono::duration_cast<std::chrono::milliseconds>(resetTimer.elapsed_time()).count(); }

uint16_t ADIS16467::readStatDiag() { return readRegister(Register::DIAG_STAT); }

int32_t ADIS16467::readGyroX()
{
    lastDC = readDataCounter();
    return read32BitValue(Register::X_GYRO_OUT, Register::X_GYRO_LOW);
}

int32_t ADIS16467::readGyroY()
{
    lastDC = readDataCounter();
    return read32BitValue(Register::Y_GYRO_OUT, Register::Y_GYRO_LOW);
}

int32_t ADIS16467::readGyroZ()
{
    lastDC = readDataCounter();
    return read32BitValue(Register::Z_GYRO_OUT, Register::Z_GYRO_LOW);
}

int32_t ADIS16467::readAccelX()
{
    lastDC = readDataCounter();
    return read32BitValue(Register::X_ACCL_OUT, Register::X_ACCL_LOW);
}

int32_t ADIS16467::readAccelY()
{
    lastDC = readDataCounter();
    return read32BitValue(Register::Y_ACCL_OUT, Register::Y_ACCL_LOW);
}

int32_t ADIS16467::readAccelZ()
{
    lastDC = readDataCounter();
    return read32BitValue(Register::Z_ACCL_OUT, Register::Z_ACCL_LOW);
}

uint16_t ADIS16467::readInternalTemp()
{
    lastDC = readDataCounter();
    return readRegister(Register::TEMP_OUT);
}

uint16_t ADIS16467::readTimeStamp()
{
    lastDC = readDataCounter();
    return readRegister(Register::TIME_STAMP);
}

uint16_t ADIS16467::readDataCounter() { return readRegister(Register::DATA_CNTR); }

void ADIS16467::burstRead(BurstReadResult& newResult)
{
    lastDC = readDataCounter();
    if (readResetTimer() < 275)
    {
        ThisThread::sleep_for(std::chrono::milliseconds(275 - readResetTimer()));
    }

    select();
    spi.write(BURST_READ_COMMAND);
    uint16_t rxBuf[10];

    for (int i = 0; i < 10; i++)
    {
        rxBuf[i] = static_cast<uint16_t>(spi.write(0));
    }

    deselect();

    uint16_t verify = 0;
    for (int i = 0; i < 9; i++)
    {
        uint16_t temp = rxBuf[i];
        verify += (temp & 0x00FF);        // add in [7:0]
        verify += ((temp >> 8) & 0x00FF); // add in [15:8]
    }
    if (verify != rxBuf[9])
    {
#if ADIS_DEBUG
        printf("Wrong checksum value\r\n");
#endif
        return;
    }

    // reading the values into each variable respectively
    newResult.statusDiag = rxBuf[0];
    newResult.gyroX = static_cast<int32_t>(rxBuf[1]) << 16;
    newResult.gyroY = static_cast<int32_t>(rxBuf[2]) << 16;
    newResult.gyroZ = static_cast<int32_t>(rxBuf[3]) << 16;
    newResult.accelX = static_cast<int32_t>(rxBuf[4]) << 16;
    newResult.accelY = static_cast<int32_t>(rxBuf[5]) << 16;
    newResult.accelZ = static_cast<int32_t>(rxBuf[6]) << 16;
    newResult.internalTemp = rxBuf[7];
    newResult.dataCounter = rxBuf[8];
    newResult.checkSum = rxBuf[9];
}

void ADIS16467::getFirmwareInformation(FirmwareInfo& info)
{
    uint16_t firmRev = readRegister(Register::FIRM_REV);

    info.firmRevMajor = static_cast<uint8_t>(firmRev >> 8);
    info.firmRevMinor = static_cast<uint8_t>(firmRev & 0x00FF);

    uint16_t firmDM = readRegister(Register::FIRM_DM);

    info.firmwareMonth = static_cast<uint8_t>((firmDM >> 8 & 0xF) + (firmDM >> 12 & 0xF) * 10);
    info.firmwareDay = static_cast<uint8_t>((firmDM & 0xF) + (firmDM >> 4 & 0xF) * 10);

    uint16_t firmY = readRegister(Register::FIRM_Y);

    info.firmwareYear = static_cast<uint16_t>((firmY & 0xF) + (firmY >> 4 & 0xF) * 10
        + (firmY >> 8 & 0xF) * 100 + (firmY >> 12 & 0xF) * 1000);

    info.serialNum = readRegister(Register::SERIAL_NUM);
}

void ADIS16467::setPollRate(uint16_t hz)
{
    if (hz > 2000)
        hz = 2000;
    if (hz <= 0)
        hz = 1;

    uint16_t decRateVal = 2000 / hz - 1;
    curr_DecRate = decRateVal;

    writeConfigurationRegister(Register::DEC_RATE_1, Register::DEC_RATE_2, decRateVal);
}

void ADIS16467::setGyroBiases(float xBias, float yBias, float zBias)
{
    int32_t xBiasRegister = static_cast<int32_t>((-1 * xBias) / GYRO_CONV);
    int32_t yBiasRegister = static_cast<int32_t>((-1 * yBias) / GYRO_CONV);
    int32_t zBiasRegister = static_cast<int32_t>((-1 * zBias) / GYRO_CONV);

    writeConfigurationRegister(Register::XG_BIAS_LOW_1,
        Register::XG_BIAS_LOW_2,
        static_cast<int16_t>(xBiasRegister & 0xFFFF));
    writeConfigurationRegister(Register::XG_BIAS_HIGH_1,
        Register::XG_BIAS_HIGH_2,
        static_cast<int16_t>(xBiasRegister >> 16));

    writeConfigurationRegister(Register::YG_BIAS_LOW_1,
        Register::YG_BIAS_LOW_2,
        static_cast<int16_t>(yBiasRegister & 0xFFFF));
    writeConfigurationRegister(Register::YG_BIAS_HIGH_1,
        Register::YG_BIAS_HIGH_2,
        static_cast<int16_t>(yBiasRegister >> 16));

    writeConfigurationRegister(Register::ZG_BIAS_LOW_1,
        Register::ZG_BIAS_LOW_2,
        static_cast<int16_t>(zBiasRegister & 0xFFFF));
    writeConfigurationRegister(Register::ZG_BIAS_HIGH_1,
        Register::ZG_BIAS_HIGH_2,
        static_cast<int16_t>(zBiasRegister >> 16));
}

void ADIS16467::sensorSelfTest()
{
    writeConfigurationRegister(Register::GLOB_CMD_1, Register::GLOB_CMD_2, (1 << 2));
}

bool ADIS16467::updateDeltaAngles()
{
    int32_t curr_DC = readDataCounter();
    // check for data counter roll overs
    if (curr_DC < lastDC_DeltaAngle)
    {
        lastDC_DeltaAngle -= 0xFFFF;
    }

    if ((curr_DC - lastDC_DeltaAngle) > 1)
    {
        printf("Delta Angles polling too slow, missed %" PRIi32 " updates\n\r",
            curr_DC - lastDC_DeltaAngle);
    }
    else if ((curr_DC - lastDC_DeltaAngle) < 1)
    {
        // no new data
        return false;
    }

    readDeltaAngleX();
    readDeltaAngleY();
    readDeltaAngleZ();

    lastDC_DeltaAngle = curr_DC;

    return true;
}

void ADIS16467::resetDeltaAngle()
{
    lastDC_DeltaAngle = readDataCounter();
    deltaAngleX_sum = 0;
    deltaAngleY_sum = 0;
    deltaAngleZ_sum = 0;
}

int64_t ADIS16467::getDeltaAngleXSum()
{
    if (readResetTimer() < 275)
    {
        ThisThread::sleep_for(std::chrono::milliseconds(275 - readResetTimer()));
    }

    return deltaAngleX_sum;
}

int64_t ADIS16467::getDeltaAngleYSum()
{
    if (readResetTimer() < 275)
    {
        ThisThread::sleep_for(std::chrono::milliseconds(275 - readResetTimer()));
    }

    return deltaAngleY_sum;
}

int64_t ADIS16467::getDeltaAngleZSum()
{
    if (readResetTimer() < 275)
    {
        ThisThread::sleep_for(std::chrono::milliseconds(275 - readResetTimer()));
    }

    return deltaAngleZ_sum;
}

void ADIS16467::updateDeltaVels()
{
    int32_t curr_DC = readDataCounter();
    // check for data counter roll overs
    if (curr_DC < lastDC_DeltaVel)
    {
        lastDC_DeltaVel -= 0xFFFF;
    }

    if ((curr_DC - lastDC_DeltaVel) > 1)
        printf("Delta Velocity polling too slow\n\r");
    else if ((curr_DC - lastDC_DeltaVel) < 1)
    {
        printf("Delta Velocity polling too fast\n\r");
        return;
    }

    readDeltaVelX();
    readDeltaVelY();
    readDeltaVelZ();

    lastDC_DeltaVel = curr_DC;
}

void ADIS16467::resetDeltaVel()
{
    lastDC_DeltaVel = readDataCounter();
    deltaVelocityX_sum = 0;
    deltaVelocityY_sum = 0;
    deltaVelocityZ_sum = 0;
}

int64_t ADIS16467::getDelVelXSum()
{
    if (readResetTimer() < 275)
    {
        ThisThread::sleep_for(std::chrono::milliseconds(275 - readResetTimer()));
    }

    return deltaVelocityX_sum;
}

int64_t ADIS16467::getDelVelYSum()
{
    if (readResetTimer() < 275)
    {
        ThisThread::sleep_for(std::chrono::milliseconds(275 - readResetTimer()));
    }

    return deltaVelocityY_sum;
}

int64_t ADIS16467::getDelVelZSum()
{
    if (readResetTimer() < 275)
    {
        ThisThread::sleep_for(std::chrono::milliseconds(275 - readResetTimer()));
    }

    return deltaVelocityZ_sum;
}

bool ADIS16467::getClockError() { return clockError; }

bool ADIS16467::getMemFailure() { return memoryFailure; }

bool ADIS16467::getSensorFailure() { return sensorFailure; }

bool ADIS16467::getStandbyMode() { return standbyMode; }

bool ADIS16467::getSPIError() { return comSPIError; }

bool ADIS16467::getFlashUpFail() { return flashMemUpdateFailure; }

bool ADIS16467::getDatapathOverrun() { return datapathOverrun; }

//-----------------------private-------------------------------------------

void ADIS16467::begin()
{
    clockError = false;
    memoryFailure = false;
    sensorFailure = false;
    standbyMode = false;
    comSPIError = false;
    flashMemUpdateFailure = false;
    datapathOverrun = false;
    deltaAngleX_sum = 0;
    deltaAngleY_sum = 0;
    deltaAngleZ_sum = 0;
    deltaVelocityX_sum = 0;
    deltaVelocityY_sum = 0;
    deltaVelocityZ_sum = 0;
    lastDC_DeltaAngle = 0;
    lastDC_DeltaVel = 0;
    lastDC = 0;
    curr_DecRate = 1999; // factory default, 2000Hz
}

int32_t ADIS16467::readDeltaAngleX()
{
    int32_t value = read32BitValue(Register::X_DELTANG_OUT, Register::X_DELTANG_LOW);
    // value = value / (curr_DecRate + 1) * 2000;
    deltaAngleX_sum += value;

    return value;
}

int32_t ADIS16467::readDeltaAngleY()
{
    int32_t value = read32BitValue(Register::Y_DELTANG_OUT, Register::Y_DELTANG_LOW);
    // value = value / (curr_DecRate + 1) * 2000;
    deltaAngleY_sum += value;

    return value;
}

int32_t ADIS16467::readDeltaAngleZ()
{
    int32_t value = read32BitValue(Register::Z_DELTANG_OUT, Register::Z_DELTANG_LOW);
    // value = value / (curr_DecRate + 1) * 2000;
    deltaAngleZ_sum += value;

    return value;
}

int32_t ADIS16467::readDeltaVelX()
{
    int32_t value = read32BitValue(Register::X_DELTVEL_OUT, Register::X_DELTVEL_LOW);
    value = value / (curr_DecRate + 1) * 2000;
    deltaVelocityX_sum += value;

    return value;
}

int32_t ADIS16467::readDeltaVelY()
{
    int32_t value = read32BitValue(Register::Y_DELTVEL_OUT, Register::Y_DELTVEL_LOW);
    value = value / (curr_DecRate + 1) * 2000;
    deltaVelocityY_sum += value;

    return value;
}

int32_t ADIS16467::readDeltaVelZ()
{
    int32_t value = read32BitValue(Register::Z_DELTVEL_OUT, Register::Z_DELTVEL_LOW);
    value = value / (curr_DecRate + 1) * 2000;
    deltaVelocityZ_sum += value;

    return value;
}

void ADIS16467::writeConfigurationRegister(Register addressLow, Register addressHigh, int16_t data)
{
    uint16_t command1 = (1 << 15) | (static_cast<uint8_t>(addressLow) << 8) | (data & 0xFF);
    uint16_t command2 = (1 << 15) | (static_cast<uint8_t>(addressHigh) << 8) | ((data >> 8) & 0xFF);

    select();
    spi.write(command1);
    deselect();

    wait_us(20);

    select();
    spi.write(command2);
    deselect();

    setErrorFlags(readStatDiag());
}

uint16_t ADIS16467::readRegister(Register address)
{
    if (readResetTimer() < 275)
    {
        ThisThread::sleep_for(std::chrono::milliseconds(275 - readResetTimer()));
    }

    uint16_t readCommand = static_cast<int>(address) << 8;
    select();
    spi.write(readCommand);

    // if (address != DIAG_STAT)
    // {
    //     setErrorFlags(readStatDiag());
    // }

    deselect();
    wait_us(20);
    select();
    uint16_t result = static_cast<uint16_t>(spi.write(0));
    deselect();

    return result;
}

int32_t ADIS16467::read32BitValue(Register highAddress, Register lowAddress)
{
    uint16_t lowWord, highWord;
    highWord = readRegister(highAddress);
    lowWord = readRegister(lowAddress);
    uint32_t val2Complement = (static_cast<uint32_t>(highWord) << 16) | lowWord;
    int32_t val = static_cast<int32_t>(val2Complement);

    return val;
}

void ADIS16467::select()
{
    spi.select();
    cs.write(0);
}

void ADIS16467::deselect() { cs.write(1); }
