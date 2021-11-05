/**
 * @author Kyle Marino
 * @filename ADS124S0x.h
 *
 * @section DESCRIPTION
 *
 * Driver for ADS124S0x 24-bit ADC
 *
 * Datasheet:
 *
 * https://www.ti.com/product/ADS124S06
 */

#ifndef ADS124S0X_H
#define ADS124S0X_H

#include <Stream.h>
#include <mbed.h>

#define DEFAULT_DATARATE 0x14
#define CRC_TABLE_LEN    256

class ADS124S0x
{
public:
    /**
     * Enum listing input mux configurations.
     * See 9.6.1.3 for more info.
     */
    enum class InputMux : uint8_t
    {
        AIN0   = 0x00,
        AIN1   = 0x01,
        AIN2   = 0x02,
        AIN3   = 0x03,
        AIN4   = 0x04,
        AIN5   = 0x05,
        AIN6   = 0x06,
        AIN7   = 0x07,
        AIN8   = 0x08,
        AIN9   = 0x09,
        AIN10  = 0x0A,
        AIN11  = 0x0B,
        AINCOM = 0x0C
    };

    /**
     * Enum listing PGA configurations. This
     * controls the PGA_EN and GAIN registers.
     * See 9.6.1.4 for more info.
     */
    enum class PGA : uint8_t
    {
        PGA_BYPASSED = 0x00,
        PGA_1        = 0x08,
        PGA_2        = 0x09,
        PGA_4        = 0x0A,
        PGA_8        = 0x0B,
        PGA_16       = 0x0C,
        PGA_32       = 0x0D,
        PGA_64       = 0x0E,
        PGA_128      = 0x0F
    };

    /**
     * Enum listing sample rates.
     * See 9.6.1.5 for more info.
     */
    enum class SampleRates : uint8_t
    {
        SPS_2_5  = 0x00,
        SPS_5    = 0x01,
        SPS_10   = 0x02,
        SPS_16_6 = 0x03,
        SPS_20   = 0x04,
        SPS_50   = 0x05,
        SPS_60   = 0x06,
        SPS_100  = 0x07,
        SPS_200  = 0x08,
        SPS_400  = 0x09,
        SPS_800  = 0x0A,
        SPS_1000 = 0x0B,
        SPS_2000 = 0x0C,
        SPS_4000 = 0x0D
    };

    /**
     * Enum of filter types.
     */
    enum class FilterType
    {
        LOW_LATENCY,
        SINC3
    };

    /**
     * Enum of conversion modes.
     */
    enum class ConversionMode
    {
        CONTINUOUS,
        SINGLE_SHOT
    };

    ADS124S0x(PinName mosi, PinName miso, PinName sclk, PinName ssel);
    bool init();
    int32_t readADC();
    void setInputMux(InputMux AINP, InputMux AINN);
    void setPGA(PGA);
    void setSampleRate(SampleRates sampleRate);
    void setGChop(bool GChop);
    void setMode(ConversionMode mode);
    void enableGPIO(bool enable);
    void setGPIODirection(uint8_t direction);
    uint8_t readGPIO();
    void writeGPIO(uint8_t gpio_dat);
    void setFilter(FilterType filter);
    void enableRailMonitors(bool enable);
    void enableCRC(bool enable);
    void enableStatus(bool enable);
    void startConversions();
    void stopConversions();
    void resetAllFlags();
    bool getPowerOnReset();
    bool getRdyFlag();
    bool getPRailPError();
    bool getPRailNError();
    bool getNRailPError();
    bool getNRailNError();
    bool getRefL1Error();
    bool getRefL0Error();

private:
    SPI spi;
    DigitalOut *rst;
    uint8_t dataRate = DEFAULT_DATARATE;
    MbedCRC<POLY_8BIT_CCITT, 8> crcCalculator;

    uint8_t readRegister(uint8_t address);
    void writeRegister(uint8_t, uint8_t);
    void setErrorFlags(uint8_t status);
    bool crcEnabled;
    bool statusEnabled;
    bool powerOnReset;
    bool rdyFlag;
    bool pRailPError;
    bool pRailNError;
    bool nRailPError;
    bool nRailNError;
    bool refL1Error;
    bool refL0Error;
};

#endif
