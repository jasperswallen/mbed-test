/**
 * @author Kyle Marino
 * @filename ADS124S0x.cpp
 *
 * @section DESCRIPTION
 *
 * Driver for ADS124S0x 24-bit ADC
 *
 * Datasheet:
 *
 * https://www.ti.com/product/ADS124S06
 */
#include "ADS124S0x.h"

#include <cinttypes>

#define ADS124S0X_SPI_FREQ 2000000
#define ADS_DEBUG          0
/**
 * ADS124S06 - 0
 * ADS124S08 - 1
 */
#define ADS_VERSION 0

/** ========= Commands (9.5.3) =========
 * Read and write commands need to be ORed with the address to perform the
 * operation.
 */
#define READ_COMMAND  0x20
#define WRITE_COMMAND 0x40
#define RESET_COMMAND 0x06
#define START_COMMAND 0x08
#define STOP_COMMAND  0x0A
#define RDATA_COMMAND 0x12

/** ========= DeviceID (ID) Register (9.6.1.1) =========
 * Device ID is 0 for ADS124S08 and 1 for ADS124S06
 */
#define DEVICE_ID_REG  0x00
#define DEVICE_ID_MASK 0x07

/** ========= Device Status (STATUS) Register (9.6.1.2) =========
 * Device status register. All 0 indicates no error
 *  7: Power on Reset flag
 *  6: ADC ready flag (active low)
 *  5: Positive PGA output is within 0.15 V of AVDD
 *  4: Positive PGA output is within 0.15 V of AVSS
 *  3: Negative PGA output is within 0.15 V of AVDD
 *  2: Negative PGA output is within 0.15 V of AVSS
 *  1: External differential reference is lower than 1/3 of AVDD
 *  0: External differential reference is lower than 0.3 V
 */
#define STATUS_REG   0x01
#define POR_MASK     0x80
#define RDY_MASK     0x40
#define P_RAILP_MASK 0x20
#define P_RAILN_MASK 0x10
#define N_RAILP_MASK 0x08
#define N_RAILN_MASK 0x04
#define REF_L1_MASK  0x02
#define REF_L0_MASK  0x01

/**
 * Poll ready status bit when device config is ready. The device should be ready
 * after 1 ms (4096 ADC clocks at 4.096 MHz), but try 10 times in case it needs
 * longer to power up.
 */
#define RDY_POLL_TIMEOUT 10

/**
 * Wait between ready bit polls. There's no set time for this
 */
#define RDY_POLL_WAIT 1ms

/** ========= Input Multiplexer (INPMUX) Register (9.6.1.3) =========
 * Controls internal muxes for which pin is the analog positive input (AINP)
 * and the analog negative input (AINN). AIN6-11 are only available for the
 * ADS124S08 model.
 */
#define INPUT_MUX_REG 0x02

/** ========= Gain Setting (PGA) Register (9.6.1.4) =========
 * Controls if the programmable gain amplifier (PGA) is enabled or bypassed,
 * and what the gain is. Gain can be 1, 2, 4, 8, 16, 32, 64, or 128.
 */
#define PGA_REG 0x03

/** ========= System Control Register (9.6.1.10) =========
 * Enable sending status with each conversion.
 * Can also send a crc, configure calibration sample size, and measure internal
 * rails and the internal temperature sensor.
 */
#define SYS_CONTROL_REG 0x09
#define CRC_BIT_MASK    0x02
#define STATUS_BIT_MASK 0x01

/** ========= DATARATE Register (9.6.1.5) =========
 * Controls Global Chop, continuous or single-shot conversion mode selection,
 * sinc3 or low-latency filter selection, and sample rate.
 */
#define DATARATE_REG         0x04
#define SAMPLE_RATE_MASK     0x0F
#define GCHOP_MASK           0x80
#define CONVERSION_MODE_MASK 0x20
#define FILTER_MASK          0x10

/** ========= Reference Control Register (9.6.1.6) =========
 * Enables reference monitor, controls positive and negative reference buffers,
 * reference selection, and the internal reference configuration.
 */
#define REF_CONTROL_REG     0x05
#define DEFAULT_REF_CONTROL 0x10
#define EN_REF_MONITOR      0x80

/** ========= Gain Calibration Register 3 (9.6.1.16) =========
 * Is intended to calibrate the PGA, but we will use this as a register with
 * a non-zero value that does not change to validate device power up.
 */
#define GAIN_CALIB_3_REG     0x0F
#define DEFAULT_GAIN_CALIB_3 0x40

/** ========= GPIO Configuration Registers (9.6.1.17) =========
 * Enables pins 22-19 as GPIO pins 0-3. Pins can be used as inputs
 * and outputs.
 */
#define GPIO_DAT_REG        0x10
#define GPIO_CONF_REG       0x11
#define ENABLE_GPIO         0x0F
#define GPIO_DIRECTION_MASK 0xF0
#define GPIO_DAT_MASK       0x0F

/** ========= Two's complement sign extension =========
 * Constants for extending the 24 bit result into an int32_t
 */
#define BIT_24_MASK 0x00800000
#define BYTE1_EXT   0xFF000000

/** ========= CRC Calculation Constants =========
 * Explanation and code from here:
 * https://barrgroup.com/embedded-systems/how-to/crc-calculation-c-code
 */
#define CRC_POLYNOMIAL 0x07
#define CRC_WIDTH      8
#define CRC_TOPBIT     (1 << (CRC_WIDTH - 1))

/**
 * Constructor
 * Note: A chip select line is not included because that is controlled externally
 * via a shift register. A reset line is also not included because it is shared.
 * @param debug Serial output pointer for debugging
 * @param mosi Pin for MOSI line
 * @param miso Pin for MISO line
 * @param sclk Pin for SCLK line
 * @param ssel Pin for CS line
 */
ADS124S0x::ADS124S0x(PinName mosi, PinName miso, PinName sclk, PinName ssel) :
    spi(mosi, miso, sclk, ssel, use_gpio_ssel)
{
    spi.frequency(ADS124S0X_SPI_FREQ);
    spi.format(8, 1);
    spi.set_default_write_value(0);
}

/**
 * Initializes the ADS124S0x.
 * @return True if init success
 */
bool ADS124S0x::init()
{
    // Reset all ADCS before initializing to ensure 2.2 ms wait for power-on reset

    // Reset internal error flags
    resetAllFlags();

    // Send reset command
    spi.select();
    spi.write(RESET_COMMAND);
    spi.deselect();

    // Delay 1 ms (4096 ADC clocks at 4.096 MHz)
    ThisThread::sleep_for(1ms);

    uint8_t dataRead;

    // Poll the ready bit
    for (int statusCounter = 0; statusCounter < RDY_POLL_TIMEOUT; statusCounter++)
    {

        dataRead = readRegister(STATUS_REG);

        // ADC is ready when this bit is low
        if ((dataRead & RDY_MASK) == 0)
        {
            break;
        }
        else
        {
            // Try until ADC ready flag to go low or times out
            if (statusCounter > RDY_POLL_TIMEOUT)
            {

#if ADS_DEBUG
                printf("ADS ready flag failed to activate\r\n");
#endif
                return false;
            }

            ThisThread::sleep_for(RDY_POLL_WAIT);
        }
    }

    // Read gain control register 3 for a non-zero value to validate device
    dataRead = readRegister(GAIN_CALIB_3_REG);

    if (dataRead != DEFAULT_GAIN_CALIB_3)
    {
#if ADS_DEBUG
        printf(
            "ADS failed to read the Gain Calibration Register!  Expected 0x%x, got 0x%" PRIx8
            "\r\n",
            GAIN_CALIB_3_REG,
            dataRead);
#endif
        return false;
    }
    else
    {
        printf("ADS gain calibration read correctly\n\r");
    }

    // Clear power on reset (POR) flag
    writeRegister(STATUS_REG, 0x00);

    // Set default input muxes
    setInputMux(InputMux::AIN0, InputMux ::AIN1);

    // Set default data rate register
    writeRegister(DATARATE_REG, DEFAULT_DATARATE);

    // Set system control data with crc enabled and status disabled
    enableCRC(true);
    enableStatus(false);

    // Enable rail monitors
    enableRailMonitors(true);

    return true;
}

/**
 * Command to start ADC conversions. Used if the start/sync pin
 * is not being used.
 */
void ADS124S0x::startConversions()
{
    spi.select();
    spi.write(START_COMMAND);
    spi.deselect();
}

/**
 * Command to stop ADC conversions. Used if the start/sync pin
 * is not being used.
 */
void ADS124S0x::stopConversions()
{
    spi.select();
    spi.write(STOP_COMMAND);
    spi.deselect();
}

/**
 * Reads the ADC conversion.
 * @return The ADC conversion. 24 bits in two's complement. Returns
 * std::numeric_limits<int32_t>::max() on CRC failure
 */
int32_t ADS124S0x::readADC()
{
    // max size: 1 don't care byte + 1 optional status byte + 3 data bytes + 1 CRC byte
    const size_t rxBufLen = 6;
    uint8_t rxData[rxBufLen];

    // Increase response size if crc and/or status bytes are sent
    size_t rxDataLen = 4;

    if (crcEnabled)
    {
        rxDataLen++;
    }

    if (statusEnabled)
    {
        rxDataLen++;
    }

    // RDATA command is used to read when not synced to conversions
    uint8_t txData[1] = {RDATA_COMMAND};

    spi.write(
        reinterpret_cast<const char *>(txData),
        1,
        reinterpret_cast<char *>(rxData),
        rxDataLen);

    if (crcEnabled)
    {

        uint32_t calculatedCRC;
        uint32_t transmittedCRC;

        if (statusEnabled)
        {
            // compute CRC on 32 bits starting with the status byte
            transmittedCRC = rxData[5];
            crcCalculator.compute(rxData + 1, 4, &calculatedCRC);
        }
        else
        {
            // Datasheet here seems to be wrong.
            // It says that we should compute CRC on 32 bits starting with the three data bytes then
            // a zero. However, experiments show that you actually just want to use the 24 bit data
            // to get a matching CRC. Maybe it's some kind of subtlety with Mbed's CRC
            // implementation?
            transmittedCRC = rxData[4];
            crcCalculator.compute(rxData + 1, 3, &calculatedCRC);
        }

        if (calculatedCRC != transmittedCRC)
        {

#if ADS_DEBUG
            printf(
                "CRC Check Failed! Calculated = 0x%" PRIx32 ", Transmitted = 0x%" PRIx32
                ", Data recieved:",
                calculatedCRC,
                transmittedCRC);
            for (size_t i = 1; i <= 4; i++)
            {
                printf(" %" PRIx8, rxData[i]);
            }
            printf("\n");
#endif

            // Return max value if crc is invalid
            return std::numeric_limits<int32_t>::max();
        }
    }

    int32_t adcData = 0;

    if (statusEnabled)
    {
        // rxData[0] is the undefined response to the status command
        // Set status flags from status byte
        setErrorFlags(rxData[1]);

        adcData |= (static_cast<uint32_t>(rxData[2]) << 16);
        adcData |= (static_cast<uint32_t>(rxData[3]) << 8);
        adcData |= static_cast<uint32_t>(rxData[4]);
    }
    else
    {
        // rxData[0] is the undefined response to the status command
        adcData |= (static_cast<uint32_t>(rxData[1]) << 16);
        adcData |= (static_cast<uint32_t>(rxData[2]) << 8);
        adcData |= static_cast<uint32_t>(rxData[3]);
    }

    // Sign extend 24 bit adc data to 32 bits
    if ((adcData & BIT_24_MASK) != 0)
    {
        adcData |= BYTE1_EXT;
    }

    // printf("Read returned: %" PRIi32 "\n", adcData);

    return adcData;
}

/**
 * Reads a register at a given address.
 * @param address Address to read from
 * @return The data at the address
 */
uint8_t ADS124S0x::readRegister(uint8_t address)
{

    spi.select();
    /*
     * No delay required between SPI command.
     * Or READ_COMMAND to read address
     */
    spi.write(READ_COMMAND | address);
    /*
     * Second command byte is number of additional bytes to receive, but we will just use
     * one byte transmissions.
     */
    spi.write(0);
    uint8_t result = spi.write(0);
    spi.deselect();

    return result;
}

/**
 * Writes a value to a register
 * @param address Address to write to
 * @param data Data to write
 */
void ADS124S0x::writeRegister(uint8_t address, uint8_t data)
{

    spi.select();
    // No delay required between SPI commands
    // Or WRITE_COMMAND to writeRegister address
    spi.write(WRITE_COMMAND | address);
    // Second command byte is number of additional bytes to receive
    spi.write(0);

    spi.write(data);
    spi.deselect();
}

/**
 * Sets the muxes for the analog input pins. Each pin can be used
 * for either the positive or negative input, but pins AIN8-AIN11
 * are only for the ADS124S0x model. The positive and negative
 * inputs should be routed to different pins.
 * @param AINP The positive input pin
 * @param AINN The negative input pin
 */
void ADS124S0x::setInputMux(InputMux AINP, InputMux AINN)
{
    // Upper 4 bits are AINP, lower 4 bits are AINN
    writeRegister(INPUT_MUX_REG, (static_cast<uint8_t>(AINP) << 4) | static_cast<uint8_t>(AINN));
}

/**
 * Sets the programmable gain amplifier (PGA) setting. This can
 * be bypassed, 1, 2, 4, 8, 16, 32, 64, or 128. Note that bypassing
 * the PGA can extend the measurement range. (9.3.2.3)
 * @param newPGA The PGA value to set.
 */
void ADS124S0x::setPGA(PGA newPGA)
{
    writeRegister(PGA_REG, static_cast<uint8_t>(newPGA));
}

/**
 * Turns on FL_REF_L0 and FL_REF_L1 monitors. L0 is if the voltage
 * reference difference is less than 0.3V, and L1 is if the voltage
 * reference difference is less than 1/3 * (AVDD - AVSS)
 * @param enable Enables the reference monitors.
 */
void ADS124S0x::enableRailMonitors(bool enable)
{
    if (enable)
    {
        writeRegister(REF_CONTROL_REG, EN_REF_MONITOR | DEFAULT_REF_CONTROL);
    }
    else
    {
        writeRegister(REF_CONTROL_REG, DEFAULT_REF_CONTROL);
    }
}

/**
 * Sets the system control register. Can enable a crc to be sent
 * after each data transmission.
 * @param enable Enables the crc.
 */

void ADS124S0x::enableCRC(bool crcEnable)
{

    // Store internally if the crc byte is being sent
    crcEnabled = crcEnable;

    // Read sys control register
    uint8_t sysControlData = readRegister(SYS_CONTROL_REG);

    if (crcEnable)
    {
        writeRegister(SYS_CONTROL_REG, CRC_BIT_MASK | sysControlData);
    }
    else
    {
        writeRegister(SYS_CONTROL_REG, ~CRC_BIT_MASK & sysControlData);
    }
}

/**
 * Sets the system control register. Can enable a crc to be sent
 * after each data transmission.
 * @param enable Enables the crc.
 */

void ADS124S0x::enableStatus(bool statusEnable)
{

    // Store internally if the status byte is being sent
    statusEnabled = statusEnable;

    // Read sys control register
    uint8_t sysControlData = readRegister(SYS_CONTROL_REG);

    if (statusEnable)
    {
        writeRegister(SYS_CONTROL_REG, STATUS_BIT_MASK | sysControlData);
    }
    else
    {
        writeRegister(SYS_CONTROL_REG, ~STATUS_BIT_MASK & sysControlData);
    }
}

/**
 * Sets the sample rate for the ADC. Note: This is not necessarily the
 * output data rate. Check the design calculator to get the exact data rate.
 * @param sampleRate The sample rate to set.
 */
void ADS124S0x::setSampleRate(SampleRates sampleRate)
{
    dataRate &= ~SAMPLE_RATE_MASK;
    dataRate |= static_cast<uint8_t>(sampleRate);
    writeRegister(DATARATE_REG, dataRate);
}

/**
 * Enables global chop. This switches the positive and negative inputs
 * on alternate measurements to reduce the voltage reference offset drift.
 * See 9.3.6.5 for more information.
 * @param newGChop Enables global chop.
 */
void ADS124S0x::setGChop(bool GChop)
{
    dataRate &= ~GCHOP_MASK;
    if (GChop)
    {
        dataRate |= GCHOP_MASK;
    }
    writeRegister(DATARATE_REG, dataRate);
}

/**
 * Sets the low latency (default) or sinc3 filter. See 9.3.6 for more
 * information on both filters.
 * @param filter Filter to set.
 */
void ADS124S0x::setFilter(FilterType filter)
{
    dataRate &= ~FILTER_MASK;
    if (filter == FilterType::LOW_LATENCY)
    {
        dataRate |= FILTER_MASK;
    }
    writeRegister(DATARATE_REG, dataRate);
}

/**
 * Configures the device to either continuous (default) or single-shot
 * conversion mode.
 * @param mode Mode to set.
 */
void ADS124S0x::setMode(ConversionMode mode)
{
    dataRate &= ~CONVERSION_MODE_MASK;
    if (mode == ConversionMode::SINGLE_SHOT)
    {
        dataRate |= CONVERSION_MODE_MASK;
    }
    writeRegister(DATARATE_REG, dataRate);
}

/**
 * Enables GPIOs on the ADS124S06.
 * @param enable Enables the GPIO pins.  Some packages share ADC inputs on the same pins as GPIOs,
 * so on these packages enabling the GPIOs disables those ADC inputs
 */
void ADS124S0x::enableGPIO(bool enable)
{
    if (enable)
    {
        writeRegister(GPIO_CONF_REG, ENABLE_GPIO);
    }
    else
    {
        // Register default value 0x00 disables GPIOs
        writeRegister(GPIO_CONF_REG, 0x00);
    }
}

/**
 * Sets the pin directions for the gpio. 0 is output and 1 is input.
 * @param direction Bits 3-0 control the direction of GPIO 3-0
 */
void ADS124S0x::setGPIODirection(uint8_t direction)
{
    uint8_t datReg = readRegister(GPIO_DAT_REG);
    datReg &= ~(GPIO_DIRECTION_MASK); // clear top 4 bits
    datReg |= direction << 4;
    writeRegister(GPIO_DAT_REG, datReg);
}

/**
 * Reads the value of the GPIO_DAT register. Bits 3-0 are the values of
 * GPIO 3-0.
 */
uint8_t ADS124S0x::readGPIO()
{
    return (readRegister(GPIO_DAT_REG) & GPIO_DAT_MASK);
}

/**
 * Writes values to the GPIO pins.
 * @param gpio_dat Bits 3-0 control the values of GPIO 3-0
 */
void ADS124S0x::writeGPIO(uint8_t gpio_dat)
{
    uint8_t datReg = readRegister(GPIO_DAT_REG);
    datReg &= ~(GPIO_DAT_MASK); // clear bottom 4 bits
    datReg |= gpio_dat;
    writeRegister(GPIO_DAT_REG, datReg);
}

/**
 * Sets error flags from the data received from the status register.
 * @param status Status register data.
 */
void ADS124S0x::setErrorFlags(uint8_t status)
{
    powerOnReset |= static_cast<bool>(status & POR_MASK);
    rdyFlag |= static_cast<bool>(status & RDY_MASK);
    pRailPError |= static_cast<bool>(status & P_RAILP_MASK);
    pRailNError |= static_cast<bool>(status & P_RAILN_MASK);
    nRailPError |= static_cast<bool>(status & N_RAILP_MASK);
    nRailNError |= static_cast<bool>(status & N_RAILN_MASK);
    refL1Error |= static_cast<bool>(status & REF_L1_MASK);
    refL0Error |= static_cast<bool>(status & REF_L0_MASK);
}

/**
 * Resets all error flags.
 */
void ADS124S0x::resetAllFlags()
{
    powerOnReset = false;
    rdyFlag      = false;
    pRailPError  = false;
    pRailNError  = false;
    nRailPError  = false;
    nRailNError  = false;
    refL1Error   = false;
    refL0Error   = false;
}

/**
 * Getter method. Only updated when readADC is called.
 * @return Power on reset flag.
 */
bool ADS124S0x::getPowerOnReset()
{
    return powerOnReset;
}

/**
 * Getter method. Only updated when readADC is called.
 * @return ADC ready flag, active low.
 */
bool ADS124S0x::getRdyFlag()
{
    return rdyFlag;
}

/**
 * Getter method. Only updated when readADC is called.
 * @return Positive PGA output is within 0.15 V of AVDD.
 */
bool ADS124S0x::getPRailPError()
{
    return pRailPError;
}

/**
 * Getter method. Only updated when readADC is called.
 * @return Positive PGA output is within 0.15 V of AVSS.
 */
bool ADS124S0x::getPRailNError()
{
    return pRailNError;
}

/**
 * Getter method. Only updated when readADC is called.
 * @return Negative PGA output is within 0.15 V of AVDD.
 */
bool ADS124S0x::getNRailPError()
{
    return nRailPError;
}

/**
 * Getter method. Only updated when readADC is called.
 * @return Negative PGA output is within 0.15 V of AVSS.
 */
bool ADS124S0x::getNRailNError()
{
    return nRailNError;
}

/**
 * Getter method. Only updated when readADC is called.
 * @return External differential reference is lower than 1/3 of AVDD.
 */
bool ADS124S0x::getRefL1Error()
{
    return refL1Error;
}

/**
 * Getter method. Only updated when readADC is called.
 * @return External differential reference is lower than 0.3 V.
 */
bool ADS124S0x::getRefL0Error()
{
    return refL0Error;
}
