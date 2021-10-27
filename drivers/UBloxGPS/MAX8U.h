#ifndef MAX8U_H
#define MAX8U_H

#include "UBloxGPS.h"
#include "UBloxGPSConstants.h"
#include "mbed.h"

#include <chrono>
namespace chrono = std::chrono;

namespace UBlox
{
class MAX8U : public UBloxGPS
{

public:
    /**
     * Construct a MAX8U, providing pins and parameters.
     *
     * This doesn't actually initialize the chip, you will need to call begin() for that.
     *
     * @param sdaPin Hardware I2C SDA pin connected to the MAX8
     * @param sclPin Hardware I2C SCL pin connected to the MAX8
     * @param rstPin Output pin connected to NRST
     * @param i2cAddress I2C address.  The MAX8 defaults to 0x42
     * @param i2cPortSpeed I2C frequency.
     */
    MAX8U(PinName sdaPin, PinName sclPin, PinName rstPin,
        uint8_t i2cAddress = UBloxGPS_I2C_DEF_ADDRESS, int i2cPortSpeed = 100000)
        : UBloxGPS(sdaPin, sclPin, rstPin, i2cAddress, i2cPortSpeed)
    {
    }

    /**
     * @brief see UBloxGPS::configure
     */
    bool configure() override;

    /**
     * Enables timepulse functionality for the sensor
     */
    bool configureTimepulse(uint32_t frequency, float onPercentage, chrono::nanoseconds delayTime);

protected:
    const char* getName() override { return "MAX-8"; };

private:
    /**
     * Tells the GPS to enable the message indicated by messageClass and messageID
     * This is done by sending a message to the GPS and then waiting for an ack message
     * @return true if an ack message has been recieved from the gps
     */
    bool setMessageEnabled(uint8_t messageClass, uint8_t messageID, bool enabled);

    /**
     * @brief Save all the current settings to the GPS's flash memory so that they will
     * be loaded when it boots.
     * @return true if the operation was successful, false otherwise.
     */
    bool saveSettings();
};

}

#endif
