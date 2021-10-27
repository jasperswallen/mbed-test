//
// Author: Adhyyan Sekhsaria
// GPS Class. Almost identical to MAX8U.

#include "ZEDFP9.h"

namespace UBlox
{

bool ZEDF9P::setValue(uint32_t key, uint64_t value, uint8_t layers)
{

    int sizeBits = (key >> 28) & 0x7;
    int valueLen
        = 1 << ((
              sizeBits == 1 ? 0 : (sizeBits - 2))); // converts the sizeBits to the byte len of val
    int totalLen = valueLen + 4 + 4;
    const int maxDataLen = 4 + 4 + 8; // 4 for setup, 4 for key, 8 max for value
    uint8_t data[maxDataLen];

    data[0] = 0;
    data[1] = layers;

    memcpy(data + 4, &key, sizeof(key));
    memcpy(data + 4 + sizeof(key), &value, totalLen); // Assuming little endinaness

    if (!sendCommand(UBX_CLASS_CFG, UBX_CFG_VALSET, data, totalLen, true, false, 1))
    {
        printf("Ublox GPS: Failed to set value!\r\n");
        return false;
    }
    DEBUG("UBX GPS: Set value successfully\r\n");
    return true;
}

bool ZEDF9P::configure()
{
    // switch to UBX mode
    bool ret = true;

    ret &= setValue(CFG_SPIINPROT_NMEA, 0);
    ret &= setValue(CFG_SPIINPROT_UBX, 1);

    ret &= setValue(CFG_SPIOUTPROT_NMEA, 0);
    ret &= setValue(CFG_SPIOUTPROT_UBX, 1);
    ret &= setValue(CFG_MSGOUT_UBX_NAV_PVT + MSGOUT_OFFSET_SPI, 1);

    // Explicity disable raw gps logging
    ret &= setValue(CFG_MSGOUT_UBX_RXM_RAWX + MSGOUT_OFFSET_SPI, 0);

    ret &= setValue(CFG_HW_ANT_CFG_VOLTCTRL, 1);
    return ret;
}

bool ZEDF9P::setPlatformModel(ZEDF9P::PlatformModel model)
{
    return setValue(CFG_NAVSPG_DYNMODEL, static_cast<uint8_t>(model));
}

}
