/**
 * @file ConsoleRedirection.h
 * @author Jasper Swallen
 * @brief Redirect the console outputs over Ethernet & serial
 * @date 2021-09-21
 *
 * @copyright Copyright (c) 2021 USC RPL
 */

#include <mbed.h>

struct ConsoleOutput
{
    uint32_t length;
    uint8_t buf[1024];
};

/**
 * @brief Implement a file handle to send printfs to the NetworkCommunicator and over serial
 */
class ConsoleRedirection : public FileHandle
{
public:
    ConsoleRedirection(ConsoleOutput &_networkConsole) :
        serial(USBTX, USBRX, 115200),
        networkConsole(_networkConsole)
    {
        flushNetworkConsole();
    }

    virtual ssize_t write(const void *buffer, size_t size) final
    {
        /* Write to the serial */
        ssize_t serial_write = serial.write(buffer, size);

        const int32_t previousLength = networkConsole.length;

        /* Don't write past the end of the buffer */
        if (previousLength + size > MAX_CONSOLE_LENGTH)
        {
            size = MAX_CONSOLE_LENGTH - previousLength;
        }

        /* Copy the data from `buffer` to the ConsolePrints buffer */
        void *result = memcpy(&networkConsole.buf[previousLength], buffer, size);
        if (result == nullptr)
        {
            return -ENOMEM;
        }

        networkConsole.length += size;

        return std::min(static_cast<ssize_t>(size), serial_write);
    }

    /**
     * @brief Defer to BufferedSerial::read
     *
     * Read the contents of a file into a buffer
     *
     * Follows POSIX semantics:
     *
     * * if no data is available, and non-blocking set return -EAGAIN
     * * if no data is available, and blocking set, wait until data is
     *   available
     * * If any data is available, call returns immediately
     *
     * @param buffer   The buffer to read in to
     * @param length   The number of bytes to read
     * @return         The number of bytes read, 0 at end of file, negative
     *                 error on failure
     */
    ssize_t read(void *buffer, size_t size) final
    {
        /* Reading is not supported over Ethernet, so just accept over serial */
        return serial.read(buffer, size);
    }

    /**
     * @brief Defer to BufferedSerial::seek
     *
     * @param offset   The offset from whence to move to
     * @param whence   The start of where to seek
     *     SEEK_SET to start from beginning of file,
     *     SEEK_CUR to start from current position in file,
     *     SEEK_END to start from end of file
     * @return         The new offset of the file, negative error code on
     *                 failure
     */
    off_t seek(off_t offset, int whence = SEEK_SET) final
    {
        return serial.seek(offset, whence);
    }

    off_t size() final
    {
        return networkConsole.length;
    }

    /**
     * @brief Close a file/connection
     *
     * Close local file streams and defer to BufferedSerial::close
     *
     * @return         0 on success, negative error code on failure
     */
    int close() final
    {
        return serial.close();
    }

    /***********************************************************************
     * Network-specific functionality
     */

    ConsoleOutput &getNetworkConsole()
    {
        return networkConsole;
    }

    void flushNetworkConsole()
    {
        memset(networkConsole.buf, '\0', MAX_CONSOLE_LENGTH);
        networkConsole.length = 0;
    }

private:
    BufferedSerial serial;

    ConsoleOutput networkConsole;

    static constexpr size_t MAX_CONSOLE_LENGTH = sizeof(ConsoleOutput::buf) /
                                                 sizeof(ConsoleOutput::buf[0]);
};
