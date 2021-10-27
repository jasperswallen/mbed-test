#pragma clang diagnostic push
#pragma ide diagnostic ignored "readability-magic-numbers"
// MAX 8U GPS Driver

// Author: Adhyyan Sekhsaria, Jamie Smith, Jay Sridharan

// Written with the help of the following data sheets:
// https://www.u-blox.com/sites/default/files/MAX-8-M8-FW3_HardwareIntegrationManual_%28UBX-15030059%29.pdf
// https://www.u-blox.com/en/docs/UBX-13003221

#include "UBloxGPS.h"
#include "ScopeGuard.h"
#include <algorithm>
#include <cstdarg>

namespace UBlox
{

UBloxGPS::UBloxGPS(PinName user_SDApin, PinName user_SCLpin,
    PinName user_RSTPin, uint8_t i2cAddress, int i2cPortSpeed)
    : i2cAddress(i2cAddress)
    , isSPI(false)
    , i2cPort_(user_SDApin, user_SCLpin)
    , spiPort_(NC, NC, NC)
    , spiCS_(NC)
    , reset_(user_RSTPin, 1)
{
    // Get user settings
    i2cPortSpeed_ = i2cPortSpeed;

    const int UBloxGPS_I2c_MAX_SPEED = 4000000; // 400 kHz
    if (i2cPortSpeed_ > UBloxGPS_I2c_MAX_SPEED)
    {
        i2cPortSpeed_ = UBloxGPS_I2c_MAX_SPEED;
    }
    i2cPort_.frequency(i2cPortSpeed_);
}

UBloxGPS::UBloxGPS(PinName user_MOSIpin, PinName user_MISOpin,
    PinName user_RSTPin, PinName user_SCLKPin, PinName user_CSPin, int spiClockRate)
    : isSPI(true)
    , i2cPort_(NC, NC)
    , spiPort_(user_MOSIpin, user_MISOpin, user_SCLKPin)
    , spiCS_(user_CSPin)
    , reset_(user_RSTPin, 1)
{
    // Get SPI clock rate
    spiClockRate_ = spiClockRate;

    if (spiClockRate_ > UBloxGPS_SPI_MAX_SPEED)
    {
        spiClockRate_ = UBloxGPS_SPI_MAX_SPEED;
    }

    spiCS_ = 1;            // deselect chip
    spiPort_.format(8, 0); // Setup SPI for 8 bit data, SPI Mode 0. UBLox8 default is SPI Mode 0
    spiPort_.frequency(spiClockRate_);
}

void UBloxGPS::softwareReset(SWResetType type)
{
    constexpr int dataLen = 4;
    uint8_t data[dataLen];
    data[0] = static_cast<uint16_t>(type) & 0xFF;
    data[1] = (static_cast<uint16_t>(type) >> 8) & 0xFF;
    data[2] = 0x1; // controlled SW reset
    data[3] = 0;

    // this command is not acknowledged
    sendCommand(UBX_CLASS_CFG, UBX_CFG_RST, data, dataLen, false, false, 0);

    // set the reset flags, and start the reset timer.
    resetInProgress_ = true;
    resetTimer_.reset();
    resetTimer_.start();
}

bool UBloxGPS::begin(bool shouldConfigure)
{
    // Reset if not currently in reset
    const auto wait_time = std::chrono::milliseconds(BOOT_TIME
        - std::chrono::duration_cast<std::chrono::milliseconds>(resetTimer_.elapsed_time())
              .count());
    if (!resetInProgress_)
    {
        softwareReset(SWResetType::HOT_START);
        DEBUG("UBloxGPS::begin() was called without starting a reset.  You could save "
              "time by starting one beforehand.\r\n");
        ThisThread::sleep_for(wait_time);
    }
    else if (wait_time.count() > 0)
    {
        // wait for reset to finish
        DEBUG("UBloxGPS::begin() was called %d ms before the reset was done.  You "
              "could save time by calling it later.\r\n",
            wait_time.count());
        ThisThread::sleep_for(wait_time);
    }

    resetInProgress_ = false;
    resetTimer_.stop();

    if (checkVersion(false, false))
    {
        DEBUG("%s booted up!\r\n", getName());
    }
    else
    {
        DEBUG("%s not detected!\r\n", getName());
        return false;
    }

    if (shouldConfigure)
    {
        if (!configure())
        {
            DEBUG("%s: failed to configure comm settings!\r\n", getName());
            return false;
        }
    }

    return true;
}

int UBloxGPS::update(float timeout)
{
    Timer timeoutTimer;
    timeoutTimer.start();

    int packetsRead = 0;

    while (std::chrono::duration<float>(timeoutTimer.elapsed_time()).count() <= timeout || timeout == 0)
     {
         switch (readMessage())
         {
             case ReadStatus::DONE:
                packetsRead++;

                if(timeout == 0)
                {
                    return packetsRead;
                }
                break;

             case ReadStatus::ERR:
                 return packetsRead;

             case ReadStatus::NO_DATA:
                // if we still haven't read a packet,
                // try again (if timeout allows). Otherwise, we have emptied the message
                // queue, so return the number of packets we have read.
                if (packetsRead == 0 && timeout != 0)
                {
                    continue;
                }
                else
                {
                    return packetsRead;
                }
                break;
         }
     }
    return packetsRead;
}

void UBloxGPS::printGNSSConfig()
{
    if (!sendCommand(UBX_CLASS_CFG, UBX_CFG_GNSS, nullptr, 0, false, true, 0.5))
    {
        printf("Could not send UBX_CFG_GNSS message");
        return;
    }

    uint8_t numTrkChHw = static_cast<uint8_t>(rxBuffer[UBX_DATA_OFFSET + 1]);
    uint8_t usedTracks = static_cast<uint8_t>(rxBuffer[UBX_DATA_OFFSET + 2]);
    uint8_t blocks = static_cast<uint8_t>(rxBuffer[UBX_DATA_OFFSET + 3]);
    printf("CHANNELS: %" PRIx8 " , USED: %" PRIx8 " , LEN: %" PRIx8 "\r\n",
        numTrkChHw,
        usedTracks,
        blocks);

    for (int i = 0; i < blocks; i++)
    {
        uint8_t gnssID = static_cast<uint8_t>(rxBuffer[UBX_DATA_OFFSET + 4 + 8 * i]);
        uint32_t flag = static_cast<uint32_t>(rxBuffer[UBX_DATA_OFFSET + 8 + 8 * i]);
        bool enabled = flag & 1;
        printf(
            "GNSS ID: %" PRIx8 ", NAME: %s, ENABLED: %d \r\n", gnssID, GNSSNames[gnssID], enabled);
    }
}

ssize_t UBloxGPS::getSatelliteInfo(SatelliteInfo* satelliteInfos, size_t infoLen)
{
    if (!sendCommand(UBX_CLASS_NAV, UBX_NAV_SAT, nullptr, 0, false, true, 1))
    {
        printf("Could not send UBX_NAV_SAT message");
        return -1;
    }

    uint8_t satellitesReturned = static_cast<uint8_t>(rxBuffer[UBX_DATA_OFFSET + 5]);

    for (size_t i = 0; i < std::min(static_cast<size_t>(satellitesReturned), infoLen); i++)
    {
        // detect a buffer overrun in the case where more satellites were returned than could
        // fit in the buffer
        size_t flagOffset = UBX_DATA_OFFSET + 16 + 12 * i;
        if (flagOffset >= MAX_MESSAGE_LEN)
        {
            printf("Error: NAV-SAT message truncated by receive buffer size!\r\n");

            // keep the part that was valid
            return i;
        }

        satelliteInfos[i].gnss
            = static_cast<GNSSID>(static_cast<uint8_t>(rxBuffer[UBX_DATA_OFFSET + 8 + 12 * i]));
        satelliteInfos[i].satelliteID
            = static_cast<uint8_t>(rxBuffer[UBX_DATA_OFFSET + 9 + 12 * i]);
        satelliteInfos[i].signalStrength
            = static_cast<uint8_t>(rxBuffer[UBX_DATA_OFFSET + 10 + 12 * i]);

        uint32_t flag = 0;
        memcpy(&flag, rxBuffer + flagOffset, 4);

        satelliteInfos[i].signalQuality = (flag & 0x0007);
        satelliteInfos[i].svUsed = (flag & (1 << 3));

        DEBUG("NAV_SAT Strength for %s %" PRIu8 ":: %" PRIu8 " dBHz; Quality: %" PRIu8 "\r\n",
            satelliteInfos[i].getGNSSName(),
            satelliteInfos[i].satelliteID,
            satelliteInfos[i].signalStrength,
            satelliteInfos[i].signalQuality);
    }

    return satellitesReturned;
}

AntennaPowerStatus UBloxGPS::getAntennaPowerStatus()
{
    if (!sendCommand(UBX_CLASS_MON, UBX_MON_HW, nullptr, 0, false, true, 0.5))
    {
        antennaPowerStatus = AntennaPowerStatus::NO_MESSAGE_RCVD;
    }
    else
    {
        antennaPowerStatus
            = AntennaPowerStatus(static_cast<uint8_t>(rxBuffer[UBX_DATA_OFFSET + 21]));
    }

    // else print out whether its on or off
    return antennaPowerStatus;
}

bool UBloxGPS::checkVersion(bool printVersion, bool printExtraInfo)
{
    if (!sendCommand(UBX_CLASS_MON, UBX_MON_VER, nullptr, 0, false, true, 0.5))
    {
        return false;
    }

    if (printVersion || UBloxGPS_DEBUG)
    {
        printf("-> %s Software Version: \r\n", getName());
        printf("-> %s\r\n", rxBuffer + UBX_DATA_OFFSET);
        printf("-> %s\r\n", rxBuffer + UBX_DATA_OFFSET + 30);

        // print additional data
        if (printExtraInfo)
        {
            size_t numAdditionalLines = (currMessageLength_ - UBX_HEADER_FOOTER_LENGTH - 40) / 30;
            if (numAdditionalLines > 0)
            {
                printf("-> Extra Info: \r\n");
            }
            for (size_t line = 0; line < numAdditionalLines; line++)
            {
                printf("-> %s\r\n", rxBuffer + UBX_DATA_OFFSET + 40 + 30 * line);
            }
        }
    }

    return true;
}

void UBloxGPS::hardwareReset()
{
    reset_ = 0;                   // Reset UBloxGPS
    ThisThread::sleep_for(100ms); // Requires at least 100ms
    reset_ = 1;                   // Bring out of reset

    resetInProgress_ = true;
    resetTimer_.reset();
    resetTimer_.start();
}

void UBloxGPS::requestTimepulseUpdate()
{
    sendCommand(UBX_CLASS_TIM, UBX_TIM_TP, nullptr, 0, false, false, 0);
}

bool UBloxGPS::sendCommand(uint8_t messageClass, uint8_t messageID, const uint8_t* data,
    uint16_t dataLen, bool shouldWaitForACK, bool shouldWaitForResponse, float timeout)
{
    // Prohibit sending commands with a payload larger than 500 bytes.
    if (dataLen > MAX_MESSAGE_LEN)
    {
        printf(
            "ERROR: sendCommand recieved a command longer than %d bytes (%d)\r\n",
            MAX_MESSAGE_LEN,
            dataLen
        );
        return false;
    }
    // make array to add header and footer
    uint16_t packetLen = dataLen + 8;
    uint8_t packet[packetLen];

    // send the sync chars
    packet[0] = UBX_SYNC_CHAR_1;
    packet[1] = UBX_SYNC_CHAR_2;

    // send the header
    packet[2] = messageClass;
    packet[3] = messageID;
    packet[4] = dataLen & 0xFF;
    packet[5] = dataLen >> 8;

    for(uint16_t i = 0; i < dataLen ; i++)
    {
        packet[i + 6] = data[i];
    }

    // compute checksum on header and data. Refer to datasheet
    calcChecksum(
        packet, 
        packetLen, 
        packet[dataLen + 6], 
        packet[dataLen + 7]
    );

    DEBUG("Sending: ");
    for (uint16_t i = 0; i < packetLen; i++)
    {
        DEBUG(" %02" PRIx8, packet[i]);
    }
    DEBUG("\r\n");

    bool status;
    if (isSPI)
    {
        status = performSPITransaction(packet, packetLen) == ReadStatus::DONE;
    }
    else
    {
        status = sendMessageI2C(packet, packetLen);
    }

    if (shouldWaitForACK)
    {
        bool ret = waitForACK(messageClass, messageID, timeout);
        status &= ret;
    }

    if (shouldWaitForResponse)
    {
        bool ret = waitForMessage(messageClass, messageID, timeout);
        status &= ret;
    }

    return status;
}

bool UBloxGPS::waitForACK(uint8_t sentMessageClass, uint8_t sentMessageID, float timeout)
{
    // NOTE: we assume that we wait for an ACK before sending another message, so
    // there will never be two ACKs in play at once

    if (!waitForMessage(UBX_CLASS_ACK, 0xFF, timeout))
    {
        printf("Timeout waiting for ACK for message 0x%02" PRIx8 " 0x%02" PRIx8 "\r\n",
            sentMessageClass,
            sentMessageID);
        return false;
    }

    // check the byte IDs
    if (rxBuffer[UBX_BYTE_CLASS] == UBX_CLASS_ACK && rxBuffer[UBX_BYTE_ID] == UBX_ACK_NACK)
    {
        printf(
            "NACK rcvd for message: %" PRIx8 " , %" PRIx8 "\r\n", sentMessageClass, sentMessageID);
        return false;
    }

    if (rxBuffer[UBX_DATA_OFFSET] != sentMessageClass
        || rxBuffer[UBX_DATA_OFFSET + 1] != sentMessageID)
    {
        printf("Ack rcvd for wrong message\r\n");
        return false;
    }

    DEBUG("ACK rcvd for message: %" PRIx8 " , %" PRIx8 "\r\n", sentMessageClass, sentMessageID);
    return true;
}

bool UBloxGPS::waitForMessage(uint8_t messageClass, uint8_t messageID, float timeout)
{
    Timer timeoutTimer;
    timeoutTimer.start();
    while (std::chrono::duration<float>(timeoutTimer.elapsed_time()).count() <= timeout)
    {
        auto ret = readMessage();
        if (ret != ReadStatus::DONE)
        {
            ThisThread::sleep_for(1ms);
            continue;
        }

        if (messageClass == rxBuffer[UBX_BYTE_CLASS]
            && (messageID == rxBuffer[UBX_BYTE_ID] || messageID == 0xFF))
        {
            // messageID == 0xFF implies we only want to wait for a message for the given class
            return true;
        }
    }

    printf(
        "Timeout after %.03fs waiting for message 0x%02" PRIx8 " 0x%02" PRIx8 ".\r\n", timeout, messageClass, messageID);
    return false;
}

// decides whether using SPI or I2C
UBloxGPS::ReadStatus UBloxGPS::readMessage()
{
    if (isSPI)
    {
        return performSPITransaction(nullptr, 0);
    }
    else
    {
        return readMessageI2C();
    }
}

UBloxGPS::ReadStatus UBloxGPS::performSPITransaction(uint8_t* packet, uint16_t packetLen)
{

    DEBUG_TR("Beginning SPI transaction ----------------------------------\r\n");

    auto init_spi = [this]()
    {
        spiPort_.select();
        spiCS_ = 0;
    };

    auto cleanup_spi = [this]() { spiCS_ = 1; };

    ScopeGuard<decltype(init_spi), decltype(cleanup_spi)> spiManager(init_spi, cleanup_spi);

    // If we are recieving a UBX message, this variable gets filled with the expected length.
    uint32_t ubxMsgLen = 0;

    // If we are receiving a message in this transaction, this index is nonzero and indicates where
    // in the RX buffer to save the current byte of the message
    uint32_t rxIndex = 0;

    // True if this is the first loop of an RX-only transaction.
    bool isRXOnly = packetLen == 0;

    /* CONTINUE WHILE:
     * we still have data to send OR
     * we are in the middle of receiving a packet OR
     * we are trying to start an RX only transaction.
     *
     * QUIT IF:
     * we have been going for 10000 cycles
     */
    for (int i = 0; (i < packetLen || rxIndex > 0 || isRXOnly) && i < 10000; i++)
    {
        uint8_t dataToSend = (i < packetLen) ? packet[i] : 0xFF;
        uint8_t incoming = spiPort_.write(dataToSend);

        DEBUG_TR(
            "SPI 0x%" PRIx8 " <--> 0x%" PRIx8 " (rxIndex = %d)\r\n", incoming, dataToSend, rxIndex);

        // last byte of original packet?
        if (i == packetLen - 1)
        {
            DEBUG_TR("Sent packet (% " PRIu16 " bytes): ");
            for (uint16_t j = 0; j < packetLen; j++)
            {
                DEBUG_TR(" %02" PRIx8, packet[j]);
            }
            DEBUG_TR("\r\n");
        }

        if (rxIndex < MAX_MESSAGE_LEN)
        {
            rxBuffer[rxIndex] = incoming;
        }

        // check for the start of a packet
        if (rxIndex == 0)
        {
            switch (incoming)
            {
                case NMEA_MESSAGE_START_CHAR:
                    isNMEASentence = true;
                    break;

                case UBX_MESSAGE_START_CHAR:
                    isNMEASentence = false;
                    break;

                case 0xFF:
                    // 0xFF is sent to indicate no data
                    if (isRXOnly)
                    {
                        return ReadStatus::NO_DATA;
                    }
                    else
                    {
                        continue;
                    }
                default:
                    printf("Received unknown byte 0x%" PRIx8
                                       ", not the start of a UBX or NMEA message.\r\n",
                        incoming);
                    rxIndex = 0;
                    continue;
            }
        }
        else if (rxIndex == 5 && !isNMEASentence)
        {
            // Populate ubxMsgLen with the size of the incoming UBX message
            // Add 8 to account for the sync(2) bytes, class, id, length(2) and checksum(2) bytes
            ubxMsgLen = (static_cast<uint16_t>(rxBuffer[rxIndex] << 8) | rxBuffer[rxIndex - 1]) + 8;
        }

        // if it's an NMEA sentence, there is a CRLF at the end
        // if it's an UBX  sentence, there is a length passed before the payload
        if (isNMEASentence && incoming == '\n')
        {
            currMessageLength_ = rxIndex + 1;
            rxIndex = 0;
            if (i >= packetLen)
            {
                return ReadStatus::DONE;
            }
        }
        else if (!isNMEASentence && ubxMsgLen != 0 && rxIndex == ubxMsgLen - 1)
        {
            DEBUG("Received packet (% " PRIu16 " bytes): ", ubxMsgLen);
            for (uint16_t j = 0; j < ubxMsgLen; j++)
            {
                DEBUG(" %02" PRIx8, rxBuffer[j]);
            }
            DEBUG("\r\n");

            if (rxIndex < MAX_MESSAGE_LEN)
            {
                rxBuffer[rxIndex + 1] = 0;
            }

            if (!verifyChecksum(ubxMsgLen))
            {
                printf("Checksums for UBX message don't match!\r\n");
                if (i >= packetLen)
                {
                    return ReadStatus::ERR;
                }
            }

            processMessage();
            currMessageLength_ = rxIndex + 1;
            if (i >= packetLen)
            {
                return ReadStatus::DONE;
            }
        }
        rxIndex++;
    }

    DEBUG_TR("\r\n\r\n");
    return ReadStatus::DONE;
}

// reads next message through I2C
UBloxGPS::ReadStatus UBloxGPS::readMessageI2C()
{
    int i2cOutputSize = readLenI2C();
    if (i2cOutputSize == -1)
    {
        printf("Didn't rcv ack from %s when reading length\r\n", getName());
        return ReadStatus::ERR;
    }

    if (i2cOutputSize == 0)
    {
        // nothing to do
        return ReadStatus::NO_DATA;
    }

    i2cPort_.start();
    int readResult = i2cPort_.write((i2cAddress << 1) | 0x01);

    if (readResult != 1)
    {
        printf("Didn't receive ack from %s\r\n", getName());
		i2cPort_.stop();
        return ReadStatus::ERR;
    }

    bool isLastByte = false;
	currMessageLength_ = 0;
	int ubxMsgLen = 100000; // large value to stop loop exit condition, will read real value later

    // for loop in case there's a data error and we don't detect the last byte
    for (int rxIndex = 0; rxIndex < i2cOutputSize; rxIndex++)
    {
        int value = i2cPort_.read(!isLastByte);
        if (value == -1)
        {
			i2cPort_.stop();
            return ReadStatus::ERR;
        }
        uint8_t incoming = static_cast<uint8_t>(value);

        if (rxIndex == 0)
        {
            if (incoming == NMEA_MESSAGE_START_CHAR)
            {
                // NMEA sentences start with a dollars sign
                isNMEASentence = true;
            }
            else if (incoming == UBX_MESSAGE_START_CHAR)
            {
                // UBX sentences start with a 0xB5
                isNMEASentence = false;
            }
            else if (incoming == 0xFF)
            {
                DEBUG("Recieved 0xFF despite output buffer length > 0\r\n");
            }
            else
            {
                printf("Unknown first character \r\n");
				i2cPort_.stop();
				return ReadStatus::ERR;
            }
        }
        else if (rxIndex == 5 && !isNMEASentence)
        {
            // read length and change that to msg length
            ubxMsgLen = (static_cast<uint16_t>(rxBuffer[rxIndex] << 8) | rxBuffer[rxIndex - 1]) + 8;
            // non-payload body of a ubx message is 8
        }

        if (rxIndex <= MAX_MESSAGE_LEN)
        {
            rxBuffer[rxIndex] = incoming;
            currMessageLength_++;
        }

        // if it's an NMEA sentence, there is a CRLF at the end
        // if it's an UBX  sentence, there is a length passed before the payload
        if ((isNMEASentence && incoming == '\n') || (!isNMEASentence && rxIndex == ubxMsgLen - 1))
        {
            break;
        }
    }

    if (currMessageLength_ <= MAX_MESSAGE_LEN)
    {
        // add null terminator
        rxBuffer[currMessageLength_] = 0;
    }

    i2cPort_.stop();

    DEBUG("Read stream of %s: ", getName());
    for (size_t j = 0; j < currMessageLength_; j++)
    {
        DEBUG("%02" PRIx8, rxBuffer[j]);
    }
    DEBUG(";\r\n");

    if (!verifyChecksum(currMessageLength_))
    {
        printf("Checksums for UBX message don't match!\r\n");
        return ReadStatus::ERR;
    }

	processMessage();

    return ReadStatus::DONE;
}

// send the header then the data to the Max8 on the default address 0x42
// Returns false if sensor does not ACK
bool UBloxGPS::sendMessageI2C(uint8_t* packet, uint16_t packetLen)
{

    // start the transaction and contact the GPS
    i2cPort_.start();

    // to indicate an i2c read, shift the 7 bit address up 1 bit and keep bit 0 as a 0
    int writeResult = i2cPort_.write(i2cAddress << 1);

    if (writeResult != 1)
    {
        printf("%s I2C write failed!\r\n", getName());
        i2cPort_.stop();
        return false;
    }

    DEBUG("%s I2C write acked!\r\n", getName());

    for (uint16_t i = 0; i < packetLen; i++)
    {
        i2cPort_.write(packet[i]);
    }

    i2cPort_.stop();

    return true;
}

void UBloxGPS::processMessage()
{
    switch (rxBuffer[UBX_BYTE_CLASS])
    {
        case UBX_CLASS_NAV:
            {
                switch (rxBuffer[UBX_BYTE_ID])
                {
                    case UBX_NAV_POSLLH:
                        position = parseNAV_POSLLH(rxBuffer);
                        break;
                    case UBX_NAV_VELNED:
                        velocity = parseNAV_VELNED(rxBuffer);
                        break;
                    case UBX_NAV_SOL:
                        fixQuality = parseNAV_SOL(rxBuffer);
                        break;
                    case UBX_NAV_TIMEUTC:
                        time = parseNAV_TIMEUTC(rxBuffer);
                        break;
                    case UBX_NAV_PVT:
                        parseNAV_PVT(rxBuffer, position, velocity, fixQuality, time);
                        break;
                    default:
                        return;
                }
                break;
            }
        case UBX_CLASS_TIM:
            {
                switch (rxBuffer[UBX_BYTE_ID])
                {
                    case UBX_TIM_TP:
                        timePulse = parseTIM_TP(rxBuffer);
                        break;
                }
                break;
            }
        default:
            return;
    }
}

int32_t UBloxGPS::readLenI2C()
{

    i2cPort_.start();
    int i2cStatus = i2cPort_.write((i2cAddress << 1) | 0x00);
    if (i2cStatus != 1)
    {
		i2cPort_.stop();
        return -1;
    }
    i2cPort_.write(0xFD);

    i2cPort_.start();
    i2cStatus = i2cPort_.write((i2cAddress << 1) | 0x01);
    if (i2cStatus != 1)
    {
		i2cPort_.stop();
        return -1;
    }

    uint8_t highByte = static_cast<uint8_t>(i2cPort_.read(true));
    uint8_t lowByte = static_cast<uint8_t>(i2cPort_.read(false));

    i2cPort_.stop();

    return (static_cast<uint16_t>(highByte << 8) | lowByte);
}

bool UBloxGPS::calcChecksum(const uint8_t* packet, uint32_t packetLen, uint8_t& chka, uint8_t& chkb) const
{
    
    // Start the checksum calculation after the sync bytes,
    // and continue until the checksum bytes
    chka = 0;
    chkb = 0;

    if(packetLen < 2)
    {
        return false;
    }

    for(uint32_t i = 2; i < packetLen - 2; i++){
        chka += packet[i];
        chkb += chka;
    }

    return true;
}

bool UBloxGPS::verifyChecksum(uint32_t messageLength)
{
    if (rxBuffer[0]  == UBX_MESSAGE_START_CHAR)
    {
        if(messageLength < 2){
            return false;
        }

        uint8_t chka = 0;
        uint8_t chkb = 0;

        if(!calcChecksum(rxBuffer, messageLength, chka, chkb))
        {
            return false;
        }
        return (chka == rxBuffer[messageLength - 2]) && (chkb == rxBuffer[messageLength - 1]);
    }

    // If the packet is NOT UBX, then it doesnt have a checksum. 
    // Simply return true. 
    return true;
}


int UBloxGPS::DEBUG_TR(const char* format, ...)
{
#if UBloxGPS_TRANSACTION_DEBUG
    std::va_list arg;
    va_start(arg, format);
    int r = vprintf(format, arg);
    va_end(arg);
    return r;
#else
    return 0;
#endif
}

int UBloxGPS::DEBUG(const char* format, ...)
{
#if UBloxGPS_DEBUG
    std::va_list arg;
    va_start(arg, format);
    int r = vprintf(format, arg);
    va_end(arg);
    return r;
#else
    return 0;
#endif
}

}

#pragma clang diagnostic pop
