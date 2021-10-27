//
// Author: Adhyyan Sekhsaria
//
//

#ifndef HAMSTER_ZEDFP9_H
#define HAMSTER_ZEDFP9_H

#include "UBloxGPS.h"
#include "UBloxGPSConstants.h"
#include "mbed.h"

namespace UBlox
{

class ZEDF9P : public UBloxGPS
{

public:
    /**
     * Construct a ZEDF9P for SPI use, providing pins and parameters.
     *
     * This doesn't actually initialize the chip, you will need to call begin() for that.
     *
     * @param user_SDApin Hardware I2C SDA pin connected to the MAX8
     * @param user_SCLpin Hardware I2C SCL pin connected to the MAX8
     * @param user_RSTPin Output pin connected to NRST
     * @param i2cAddress I2C address.  The MAX8 defaults to 0x42
     * @param i2cPortSpeed I2C frequency.
     */
    ZEDF9P(PinName user_MOSIpin, PinName user_MISOpin, PinName user_RSTPin,
        PinName user_SCLKPin, PinName user_CSPin, int spiClockRate = 1000000)
        : UBloxGPS(user_MOSIpin, user_MISOpin, user_RSTPin, user_SCLKPin, user_CSPin,
            spiClockRate)
    {
    }

    /**
     * @brief see UBloxGPS::configure
     */
    bool configure() override;

    /**
     * Platform model selection.  Allows one to choose the environment that the GPS is in.
     * Provides a tradeoff between accuracy and tolerance against motion.
     *
     * See ZED-F9P integration manual section 3.1.7.1 for more info.
     */
    enum class PlatformModel : uint8_t
    {
        PORTABLE = 0,
        STATIONARY = 2,
        PEDESTRIAN = 3,
        AUTOMOT = 4,
        SEA = 5,
        AIR_1G = 6,
        AIR_2G = 7,
        AIR_4G = 8,
        WRIST = 9
    };

    /**
     * Set the platform model in use.  Default is PORTABLE on new units.
     * @param model
     * @return
     */
    bool setPlatformModel(PlatformModel model);

private:
    const char* getName() override { return "ZED-F9P"; };

    /**
     * Implmentation of UBX-SET-VAl
     * Used to configure the sensor
     * @param key config key
     * @param value The value associated with the key. It takes care of the size of the int
     * @param layers bitmask which indicates the layer to save the config on the GPS. Flash, BBR,
     * and RAM
     * @return true if setting was successful and ACK is received.
     */
    bool setValue(uint32_t key, uint64_t value, uint8_t layers = 0x7);
};

}

#endif // HAMSTER_ZEDFP9_H
