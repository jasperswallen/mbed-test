/**
 * @file ADIS16467.h
 * @author Rita Yang
 * @brief Software Driver for ADIS16467-3 Precision MEMS IMU Module
 *
 * @copyright Copyright (c) 2021
 *
 * Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/adis16467.pdf
 */

#ifndef ADIS16467_H
#define ADIS16467_H

#include "Stream.h"
#include "mbed.h"

/**
 * @brief The ADIS16467 Driver
 */

class ADIS16467
{
public:
    /**
     * @brief The data read by a burst read.
     *
     * Contains gyro, accel, temp, status, a data counter, and a checksum
     */
    typedef struct BurstReadResult
    {
        int32_t gyroX;
        int32_t gyroY;
        int32_t gyroZ;
        int32_t accelX;
        int32_t accelY;
        int32_t accelZ;
        uint16_t statusDiag;
        uint16_t internalTemp;
        uint16_t dataCounter;
        uint16_t checkSum;
    } BurstReadResult;

    /**
     * @brief Contains information about the firmware, including the firmware revision, date, and
     * serial number
     */
    typedef struct FirmwareInfo
    {
        uint8_t firmRevMajor;
        uint8_t firmRevMinor;

        uint8_t firmwareDay;
        uint8_t firmwareMonth;
        uint16_t firmwareYear;
        uint16_t serialNum;
    } FirmwareInfo;

public:
    /**
     * @brief Construct a new ADIS16467 driver
     *
     * Sets all error flag variables to false
     *
     * @param[in] DIN SPI data input (MOSI)
     * @param[in] DOUT SPI data output (MISO)
     * @param[in] SCLK Clock pin
     * @param[in] CS Chip select pin
     * @param[in] RST Reset pin
     */
    ADIS16467(PinName DIN, PinName DOUT, PinName SCLK, PinName CS, PinName RST);

    /**
     * @brief Initialize ADIS
     *
     * Must be called after each reset of the chip
     */
    void initADIS();

    /**
     * @brief Checks if the ADIS is connected and working
     *
     * Reads the PROD_ID register, from page 28 of datasheet
     *
     * @return true if the PROD_ID is correct (16467), false otherwise
     */
    bool checkExistence();

    /**
     * @brief Checks if there is new data
     *
     * Checks the data counter to see if there is an update available
     *
     * @return true if there is updated data since the last read, false otherwise
     */
    bool hasNewData();

    /**
     * @brief Sets the error flags
     *
     * Accepts a given reading from a DIAG_STAT reading and sets the error flags to their
     * corresponding value. The error flags will maintain their value until cleared, so if an error
     * occurs, it will not be overridden until it has been read.
     *
     * Refer to Table 10 in the datasheet
     * @param[in] statusDiagVal The DIAG_STAT reading to set error values from
     */
    void setErrorFlags(uint16_t statusDiagVal);

    /**
     * @brief Resets the variables that keep track of the error flags to false
     */
    void resetAllFlags();

    /**
     * @name Read Data
     * @{
     */

    /**
     * @brief Returns the time since the last reset
     *
     * Should be called before calling initADIS()
     *
     * @return The time since the last reset, in ms
     */
    uint32_t readResetTimer();

    /**
     * @brief Reads the DIAG_STAT register and returns its value
     *
     * Reading this register will cause all of its bits to reset to 0
     *
     * @return The value of the DIAG_STAT register
     */
    uint16_t readStatDiag();

    /**
     * @brief Reads the current value of the x-axis gyroscope
     *
     * Reads both the high and low words, and returns the data as a 2s complement 32-bit LSB value.
     * To get a human-readable format, multiply by GYRO_CONV
     *
     * @return The current value of the x-axis gyroscope
     */
    int32_t readGyroX();

    /**
     * @brief Reads the current value of the y-axis gyroscope
     *
     * Reads both the high and low words, and returns the data as a 2s complement 32-bit LSB value.
     * To get a human-readable format, multiply by GYRO_CONV
     *
     * @return The current value of the y-axis gyroscope
     */
    int32_t readGyroY();

    /**
     * @brief Reads the current value of the z-axis gyroscope
     *
     * Reads both the high and low words, and returns the data as a 2s complement 32-bit LSB value.
     * To get a human-readable format, multiply by GYRO_CONV
     *
     * @return The current value of the z-axis gyroscope
     */
    int32_t readGyroZ();

    /**
     * @brief Reads the current value of the x-axis accelerometer
     *
     * Reads both the high and low words, and returns the data as a 2s complement 32-bit LSB value.
     * To get a human-readable format, multiply by ACCEL_CONV
     *
     * @return The current value of the x-axis accelerometer
     */
    int32_t readAccelX();

    /**
     * @brief Reads the current value of the y-axis accelerometer
     *
     * Reads both the high and low words, and returns the data as a 2s complement 32-bit LSB value.
     * To get a human-readable format, multiply by ACCEL_CONV
     *
     * @return The current value of the y-axis accelerometer
     */
    int32_t readAccelY();

    /**
     * @brief Reads the current value of the z-axis accelerometer
     *
     * Reads both the high and low words, and returns the data as a 2s complement 32-bit LSB value.
     * To get a human-readable format, multiply by ACCEL_CONV
     *
     * @return The current value of the z-axis accelerometer
     */
    int32_t readAccelZ();

    /**
     * @brief Reads the current coarse measurement of internal temperature
     *
     * Reads the TEMP_OUT register as a 2s complement 32-bit LSB value. To get a human-readable
     * format, multiply by TEMP_CONV
     *
     * @return The current internal temperature reading
     */
    uint16_t readInternalTemp();

    /**
     * @brief Reads the time stamp
     *
     * Reads the time stamp from the last pulse on the SYNC pin from the TIME_STAMP register, which
     * works in conjuction with the Scaled Sync Mode. To get a human-readable value, multiply by
     * TIME_CONV
     *
     * @return The time stamp
     */
    uint16_t readTimeStamp();

    /**
     * @brief Reads the data counter
     *
     * Reads the DATA_CNTR register, which increments each time new data is loaded into the output
     * registers. Once the value reaches 0xFFFF, it will wrap around to 0x0000
     *
     * @return The data counter's value
     */
    uint16_t readDataCounter();

    /**
     * @brief Performs a burst read on several regsiters
     *
     * Reads DIAG_STAT, all gyros axes, all accel axes, TEMP_OUT, and DATA_CNTR
     *
     * Only the high word of the gyros and accel will be read (16-bit value). A 32-bit checksum
     * value will also be given at the end of the read.
     *
     * @param[out] newResult The result of the burst read
     */
    void burstRead(BurstReadResult& newResult);

    /**
     * @brief Gets the firmware information
     *
     * Reads the firmware revision, data, and serial number.
     *
     * All the values formatted with 4 bits per number
     *
     * @param[out] info The result of the firmware info read
     */
    void getFirmwareInformation(FirmwareInfo& info);

    /**
     * @}
     */

    /**
     * @brief Sets the ADIS poll rate
     *
     * Calculates the decimation factor from the input Hz polling rate.
     *
     * @param[in] hz The desired polling rate in Hz, must be between 1 and 2000
     */
    void setPollRate(uint16_t hz);

    /**
     * @brief Set the gyro calibration biases
     *
     * As would be logical, a positive bias value increases the gyro's
     * reading in that axis (the register setting is the other way).
     *
     * Arguments are in degrees per second.
     *
     * @param[in] xBias x-axis bias in degrees/s
     * @param[in] yBias y-axis bias in degrees/s
     * @param[in] zBias z-axis bias in degrees/s
     */
    void setGyroBiases(float xBias, float yBias, float zBias);

    /**
     * @brief Performs an internal self test of the ADIS
     *
     * Writes a 1 to GLOB_CMD bit 2.
     *
     * A pass or fail result will be reported to DIAG_STAT bit 5. Error flags will automatically be
     * checked after the function is called, so no need to call setErrorFlags - instead, directly
     * call getSensorFailure()
     */
    void sensorSelfTest();

    /**
     * @name Delta Angles
     * @{
     */

    /**
     * @brief Helper function to read all 3 delta angles and update sums
     *
     * @return true if the angles were updated, false if there is no new data
     */
    bool updateDeltaAngles();

    /**
     * @brief Resets the delta angle sums to 0
     *
     * Updates lastDC_DeltaAngle to latest data counter value
     */
    void resetDeltaAngle();

    /**
     * @brief Gets the cumulative sum of the x-axis delta angles from calls to updateDeltaAngles()
     *
     * To get a human readable result, multiply by DELTA_ANGLE_CONV
     *
     * @return The cumulative sum of angles, or 0 if the chip is not ready yet
     */
    int64_t getDeltaAngleXSum();

    /**
     * @brief Gets the cumulative sum of the y-axis delta angles from calls to updateDeltaAngles()
     *
     * To get a human readable result, multiply by DELTA_ANGLE_CONV
     *
     * @return The cumulative sum of angles, or 0 if the chip is not ready yet
     */
    int64_t getDeltaAngleYSum();

    /**
     * @brief Gets the cumulative sum of the z-axis delta angles from calls to updateDeltaAngles()
     *
     * To get a human readable result, multiply by DELTA_ANGLE_CONV
     *
     * @return The cumulative sum of angles, or 0 if the chip is not ready yet
     */
    int64_t getDeltaAngleZSum();

    /**
     * @}
     * @name Delta Velocities
     * @{
     */

    /**
     * @brief Helper function to read all 3 delta velocities and update sums
     */
    void updateDeltaVels();

    /**
     * @brief Resets the delta velocity sums to 0
     *
     * Updates lastDC_DeltaVel to latest data counter value
     */
    void resetDeltaVel();

    /**
     * @brief Gets the cumulative sum of the x-axis delta velocities from calls to updateDeltaVels()
     *
     * To get a human readable result, multiply by DELTA_VEL_CONV
     *
     * @return The cumulative sum of velocities, or 0 if the chip is not ready yet
     */
    int64_t getDelVelXSum();

    /**
     * @brief Gets the cumulative sum of the y-axis delta velocities from calls to updateDeltaVels()
     *
     * To get a human readable result, multiply by DELTA_VEL_CONV
     *
     * @return The cumulative sum of velocities, or 0 if the chip is not ready yet
     */
    int64_t getDelVelYSum();

    /**
     * @brief Gets the cumulative sum of the z-axis delta velocities from calls to updateDeltaVels()
     *
     * To get a human readable result, multiply by DELTA_VEL_CONV
     *
     * @return The cumulative sum of velocities, or 0 if the chip is not ready yet
     */
    int64_t getDelVelZSum();

    /**
     * @}
     * @name Error Flag Getters
     * @{
     */
    /**
     * @brief Gets the clock error flag
     *
     * @return The clockError flag value
     */
    bool getClockError();

    /**
     * @brief Gets the memory failure flag
     *
     * @return The memoryFailure flag value
     */
    bool getMemFailure();

    /**
     * @brief Gets the sensor failure flag
     *
     * @return The sensorFailure flag value
     */
    bool getSensorFailure();

    /**
     * @brief Gets the standy mode flag
     *
     * @return The standbyMode flag value
     */
    bool getStandbyMode();

    /**
     * @brief Gets the SPI error flag
     *
     * @return The comSPIError flag value
     */
    bool getSPIError();

    /**
     * @brief Gets the flash memory update failure flag
     *
     * @return The flashMemUpdateFailure flag value
     */
    bool getFlashUpFail();

    /**
     * @brief Gets the datapath overrun error flag
     *
     * @return The datapathOverrun flag value
     */
    bool getDatapathOverrun();

    /**
     * @}
     * @name Conversion Factors
     * @{
     */

    /**
     * @brief Gyro conversion factor
     *
     * 10 LSB/degree/sec
     */
    static constexpr float GYRO_CONV = 0.1f / (1 << 16);

    /**
     * @brief Accel conversion factor
     *
     * 1 LSB = 1.25 mg
     */
    static constexpr float ACCEL_CONV = 1.25f / (1 << 16);

    /**
     * @brief Temp conversion factor
     *
     * 1 LSB = 0.1 degrees C
     */
    static constexpr float TEMP_CONV = 0.1f;

    /**
     * @brief Time conversion factor
     *
     * 1 LSB = 49.02 us
     */
    static constexpr float TIME_CONV = 49.02f;

    /**
     * @brief Delta angle conversion factor
     *
     * degrees per LSB
     */
    static constexpr float DELTA_ANGLE_CONV = 2160.0f / (1 << 31);

    /**
     * @brief Delta velocity conversion factor
     *
     * meters/sec per LSB
     */
    static constexpr float DELTA_VEL_CONV = 400.0f / (1 << 15);

    /**
     * @}
     */

private:
    /**
     * @brief List of ADIS registers
     */
    enum class Register : uint8_t
    {
        // Output Register Memory Map (Table 8)
        DIAG_STAT = 0x02,     ///< R, output, system error flag
        X_GYRO_LOW = 0x04,    ///< R, output, x-axis gyro, low word
        X_GYRO_OUT = 0x06,    ///< R, output, x-axis gyro, high word
        Y_GYRO_LOW = 0x08,    ///< R, output, y-axis gyro, low word
        Y_GYRO_OUT = 0x0A,    ///< R, output, y-axis gyro, high word
        Z_GYRO_LOW = 0x0C,    ///< R, output, z-axis gyro, low word
        Z_GYRO_OUT = 0x0E,    ///< R, output, z-axis gyro, high word
        X_ACCL_LOW = 0x10,    ///< R, output, x-axis accel, low word
        X_ACCL_OUT = 0x12,    ///< R, output, x-axis accel, high word
        Y_ACCL_LOW = 0x14,    ///< R, output, y-axis accel, low word
        Y_ACCL_OUT = 0x16,    ///< R, output, y-axis accel, high word
        Z_ACCL_LOW = 0x18,    ///< R, output, z-axis accel, low word
        Z_ACCL_OUT = 0x1A,    ///< R, output, z-axis accel, high word
        TEMP_OUT = 0x1C,      ///< R, output, temperature
        TIME_STAMP = 0x1E,    ///< R, output, time stamp
        DATA_CNTR = 0x22,     ///< R, output, new data counter
        X_DELTANG_LOW = 0x24, ///< R, output, x-axis delta angle, low word
        X_DELTANG_OUT = 0x26, ///< R, output, x-axis delta angle, high word
        Y_DELTANG_LOW = 0x28, ///< R, output, y-axis delta angle, low word
        Y_DELTANG_OUT = 0x2A, ///< R, output, y-axis delta angle, high word
        Z_DELTANG_LOW = 0x2C, ///< R, output, z-axis delta angle, low word
        Z_DELTANG_OUT = 0x2E, ///< R, output, z-axis delta angle, high word
        X_DELTVEL_LOW = 0x30, ///< R, output, x-axis delta velocity, low word
        X_DELTVEL_OUT = 0x32, ///< R, output, x-axis delta velocity, high word
        Y_DELTVEL_LOW = 0x34, ///< R, output, y-axis delta velocity, low word
        Y_DELTVEL_OUT = 0x36, ///< R, output, y-axis delta velocity, high word
        Z_DELTVEL_LOW = 0x38, ///< R, output, z-axis delta velocity, low word
        Z_DELTVEL_OUT = 0x3A, ///< R, output, z-axis delta velocity, high word
        FLSHCNT_LOW = 0x7C,   ///< R, output, flash memory write cycle counter, lower word
        FLSHCNT_HIGH = 0x7E,  ///< R, output, flash memory write cycle counter, upper word

        // Calibration Registers (Table 8)
        XG_BIAS_LOW_1 = 0x40,  ///< R/W, calib, offset, gyro, x-axis, low word, lower byte
        XG_BIAS_LOW_2 = 0x41,  ///< higher byte
        XG_BIAS_HIGH_1 = 0x42, ///< R/W, calib, offset, gyro, x-axis, high word, lower byte
        XG_BIAS_HIGH_2 = 0x43, ///< higher byte
        YG_BIAS_LOW_1 = 0x44,  ///< R/W, calib, offset, gyro, y-axis, low word, lower byte
        YG_BIAS_LOW_2 = 0x45,  ///< higher byte
        YG_BIAS_HIGH_1 = 0x46, ///< R/W, calib, offset, gyro, y-axis, high word, lower byte
        YG_BIAS_HIGH_2 = 0x47, ///< higher byte
        ZG_BIAS_LOW_1 = 0x48,  ///< R/W, calib, offset, gyro, z-axis, low word, lower byte
        ZG_BIAS_LOW_2 = 0x49,  ///< higher byte
        ZG_BIAS_HIGH_1 = 0x4A, ///< R/W, calib, offset, gyro, z-axis, high word, lower byte
        ZG_BIAS_HIGH_2 = 0x4B, ///< higher byte
        XA_BIAS_LOW_1 = 0x4C,  ///< R/W, calib, offset, accel, x-axis, low word, lower byte
        XA_BIAS_LOW_2 = 0x4D,  ///< higher byte
        XA_BIAS_HIGH_1 = 0x4E, ///< R/W, calib, offset, accel, x-axis, high word, lower byte
        XA_BIAS_HIGH_2 = 0x4F, ///< higher byte
        YA_BIAS_LOW_1 = 0x50,  ///< R/W, calib, offset, accel, y-axis, low word, lower byte
        YA_BIAS_LOW_2 = 0x51,  ///< higher byte
        YA_BIAS_HIGH_1 = 0x52, ///< R/W, calib, offset, accel, y-axis, high word, lower byte
        YA_BIAS_HIGH_2 = 0x53, ///< higher byte
        ZA_BIAS_LOW_1 = 0x54,  ///< R/W, calib, offset, accel, z-axis, low word, lower byte
        ZA_BIAS_LOW_2 = 0x55,  ///< higher byte
        ZA_BIAS_HIGH_1 = 0x56, ///< R/W, calib, offset, accel, z-axis, high word, lower byte
        ZA_BIAS_HIGH_2 = 0x57, ///< higher byte

        // Control Registers
        FILT_CTRL_1 = 0x5C, ///< R/W, control, Bartlett window FIR filter, lower byte
        FILT_CTRL_2 = 0x5D, ///< higher byte
        RANG_MDL = 0x5E,    ///< R, measurement range (model specific) identifier
        MSC_CTRL_1 = 0x60,  ///< R/W, control, input/output & other, lower byte
        MSC_CTRL_2 = 0x61,  ///< higher byte
        UP_SCALE_1 = 0x62,  ///< R/W, control, scale factor for input clock, PPS mode, lower byte
        UP_SCALE_2 = 0x63,  ///< higher byte
        DEC_RATE_1 = 0x64,  ///< R/W, control, decimation filter (output data rate), lower byte
        DEC_RATE_2 = 0x65,  ///< higher byte
        NULL_CNFG_1 = 0x66, ///< R/W, control, bias estimation period, lower byte
        NULL_CNFG_2 = 0x67, ///< higher byte
        GLOB_CMD_1 = 0x68,  ///< W, control, global commands, lower byte
        GLOB_CMD_2 = 0x69,  ///< higher byte

        // Identification
        FIRM_REV = 0x6C,        ///< R, id, firmware revision
        FIRM_DM = 0x6E,         ///< R, id, date code, day and month
        FIRM_Y = 0x70,          ///< R, id, date code, year
        PROD_ID = 0x72,         ///< R, id, device number
        SERIAL_NUM = 0x74,      ///< R, id, serial number
        USER_SCR_1_LOW = 0x76,  ///< R/W, user scratch register 1, lower byte
        USER_SCR_1_HIGH = 0x77, ///< higher byte
        USER_SCR_2_LOW = 0x78,  ///< R/W, user scratch register 2, lower byte
        USER_SCR_2_HIGH = 0x79, ///< higher byte
        USER_SCR_3_LOW = 0x7A,  ///< R/W, user scratch register 3, lower byte
        USER_SCR_3_HIGH = 0x7B  ///< higher byte
    };

private:
    /**
     * @brief Reset all private variables
     *
     * Called from initADIS()
     */
    void begin();

    /**
     * @name Read Delta Angles
     * @{
     */

    /**
     * @brief Reads the x-axis delta angle registers
     *
     * To get a human readable format, multiply by DELTA_ANGLE_CONV
     *
     * @return 2s complement 32-bit value in LSB
     */
    int32_t readDeltaAngleX();

    /**
     * @brief Reads the y-axis delta angle registers
     *
     * To get a human readable format, multiply by DELTA_ANGLE_CONV
     *
     * @return 2s complement 32-bit value in LSB
     */
    int32_t readDeltaAngleY();

    /**
     * @brief Reads the z-axis delta angle registers
     *
     * To get a human readable format, multiply by DELTA_ANGLE_CONV
     *
     * @return 2s complement 32-bit value in LSB
     */
    int32_t readDeltaAngleZ();

    /**
     * @}
     * @name Read Delta Velocities
     * @{
     */

    /**
     * @brief Reads the x-axis delta velocity registers
     *
     * To get a human readable format, multiply by DELTA_VEL_CONV
     *
     * @return 2s complement 32-bit value in LSB
     */
    int32_t readDeltaVelX();

    /**
     * @brief Reads the y-axis delta velocity registers
     *
     * To get a human readable format, multiply by DELTA_VEL_CONV
     *
     * @return 2s complement 32-bit value in LSB
     */
    int32_t readDeltaVelY();

    /**
     * @brief Reads the z-axis delta velocity registers
     *
     * To get a human readable format, multiply by DELTA_VEL_CONV
     *
     * @return 2s complement 32-bit value in LSB
     */
    int32_t readDeltaVelZ();

    /**
     * @}
     */

    /**
     * @brief Writes a value to a configuration register
     *
     * Combines low and high addresses with respective part of the data
     *
     * @param[in] addressLow The lower byte of the configuration register
     * @param[in] addressHigh The upper byter of the configuration register
     * @param[in] data The data to write to the configuration register
     */
    void writeConfigurationRegister(Register addressLow, Register addressHigh, int16_t data);

    /**
     * @brief Reads a register at a given address
     *
     * @param[in] address Register address to read from
     * @return The data at the register, or 0 along with an error message if the chip is not ready
     */
    uint16_t readRegister(Register address);

    /**
     * @brief Reads values at two addresses and combines them into a signed 32-bit value
     *
     * @param[in] highAddress The upper address to read from
     * @param[in] lowAddress The lower address to read from
     * @return The 32-bit signed value from combining the results
     */
    int32_t read32BitValue(Register highAddress, Register lowAddress);

    /**
     * @brief Assert the Chip Select pin
     */
    void select();

    /**
     * @brief Deassert the Chip Select pin
     */
    void deselect();

private:
    /** @brief The SPI interface */
    SPI spi;

    /** @brief The Chip Select pin */
    DigitalOut cs;

    /** @brief The reset pin */
    DigitalOut rst;

    /** @brief Keep track of reset time */
    Timer resetTimer;

    /**
     * @name Error Flags
     * @{
     */

    /**
     * @brief Clock error
     *
     * A 1 indicates that the internal data sampling clock (fSM, see Figure 20 and Figure 21) does
     * not synchronize with the external clock, which only applies when using scaled sync mode
     * (Register MSC_CTRL, Bits[4:2] = 010, see Table 105. When this error occurs, adjust the
     * frequency of the clock signal on the SYNC pin to operate within the appropriate range.
     */
    bool clockError;

    /**
     * @brief Memory failure
     *
     * A 1 indicates a failure in the flash memory test (Register GLOB_CMD, Bit 4, see Table 113),
     * which involves a comparison between a cyclic redundancy check (CRC) calculation of the
     * present flash memory and a CRC calculation from the same memory locations at the time of
     * initial programming (during the production process). If this error occurs, repeat the same
     * test. If this error persists, replace the ADIS16467.
     */
    bool memoryFailure;

    /**
     * @brief Sensor failure
     *
     * A 1 indicates failure of at least one sensor, at the conclusion of the self test (Register
     * GLOB_CMD, Bit 2, see Table 113). If this error occurs, repeat the same test. If this error
     * persists, replace the ADIS16467. Motion, during the execution of this test, can cause a false
     * failure.
     */
    bool sensorFailure;

    /**
     * @brief Standby mode
     *
     * A 1 indicates that the voltage across VDD and GND is <2.8 V, which causes data processing to
     * stop. When VDD ≥ 2.8 V for 250 ms, the ADIS16467 reinitializes and starts producing data
     * again.
     */
    bool standbyMode;

    /**
     * @brief SPI communication error
     *
     * A 1 indicates that the total number of SCLK cycles is not equal to an integer multiple of 16.
     * When this error occurs, repeat the previous communication sequence. Persistence in this error
     * may indicate a weakness in the SPI service that the ADIS16467 is receiving from the system it
     * is supporting.
     */
    bool comSPIError;

    /**
     * @brief Flash memory update failure
     *
     * A 1 indicates that the most recent flash memory update (Register GLOB_CMD, Bit 3, see Table
     * 113) failed. If this error occurs, ensure that VDD ≥ 3 V and repeat the update attempt. If
     * this error persists, replace the ADIS16467.
     */
    bool flashMemUpdateFailure;

    /**
     * @brief Datapath overrun
     *
     * A 1 indicates that one of the datapaths experienced an overrun condition. If this error
     * occurs, initiate a reset using the RST pin (see Table 5, Pin 8) or Register GLOB_CMD, Bit 7
     * (see Table 113). See the Serial Port Operation section for more details on conditions that
     * may cause this bit to be set to 1.
     */
    bool datapathOverrun;

    /**
     * @}
     */

    /**
     * @brief The SPI frequency
     *
     * Supports burst read (<= 1 MHz)
     */
    const int SPI_FREQ = 1000000;

    /**
     * @brief The command to perform a burst read
     */
    const int BURST_READ_COMMAND = 0x6800;

    /**
     * @name Delta Angle Sums
     * @{
     */

    /**
     * @brief The current sum of every delta angle read along the x-axis
     */
    int64_t deltaAngleX_sum;

    /**
     * @brief The current sum of every delta angle read along the y-axis
     */
    int64_t deltaAngleY_sum;

    /**
     * @brief The current sum of every delta angle read along the z-axis
     */
    int64_t deltaAngleZ_sum;

    /**
     * @brief The data counter read for the last delta angle read
     */
    int32_t lastDC_DeltaAngle;

    /**
     * @}
     * @name Delta Velocity Sums
     * @{
     */

    /**
     * @brief The current sum of every delta velocity read along the x-axis
     */
    int64_t deltaVelocityX_sum;

    /**
     * @brief The current sum of every delta velocity read along the y-axis
     */
    int64_t deltaVelocityY_sum;

    /**
     * @brief The current sum of every delta velocity read along the z-axis
     */
    int64_t deltaVelocityZ_sum;

    /**
     * @brief The data counter read for the last delta velocity read
     */
    int32_t lastDC_DeltaVel;

    /**
     * @}
     */

    /**
     * @brief The data counter read for the last overall read
     */
    int32_t lastDC;

    /**
     * @brief The current decimation rate
     *
     * The averaging decimating filter, which averages and decimates the gyroscope and accelerometer
     * data; it also extends the time that the delta angle and the delta velocity track between each
     * update.
     */
    uint16_t curr_DecRate;
};

#endif
