#include <mbed.h>

#include "CustomFileHandle.hpp"

static CustomFileHandle cfh;

namespace mbed
{
/**
 * @brief Override the default console (for printf) to use a CustomFileHandle
 */
FileHandle *mbed_override_console(int fd)
{
    return &cfh;
}
}; // namespace mbed

/* Allow for manually printing over USBTX and RX for testing */
BufferedSerial serial(USBTX, USBRX, 115200);
FILE *pc;

int main()
{
    pc = fdopen(&serial, "r+");

    fprintf(pc, "%s\n", (cfh.get_internal_buf().empty()) ? "empty" : "has element");
    ThisThread::sleep_for(1s);

    printf("Hello\r\n");

    fprintf(pc, "%s\n", (cfh.get_internal_buf().empty()) ? "empty" : "has element");

    printf("hi\rhello\tjksdflkjsd");

    /* Note that without a newline, this does not get added to the internal buffer */
    for (const auto &command : cfh.get_internal_buf())
    {
        fprintf(pc, "Got data: %s\n", command.data());
    }
    fprintf(pc, "Total Num of Calls: %d\r\n", cfh.get_internal_buf().size());
    ThisThread::sleep_for(1s);

    /* Flush the buffer, appending to the internal buffer (and not clearing) */
    printf("With a newline:\r\n");

    for (const auto &command : cfh.get_internal_buf())
    {
        fprintf(pc, "Got data: %s\n", command.data());
    }
    fprintf(pc, "Total Num of Calls: %d\r\n", cfh.get_internal_buf().size());
    ThisThread::sleep_for(1s);
}
