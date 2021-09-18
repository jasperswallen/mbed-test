#include <mbed.h>

#include "CustomFileHandle.hpp"

static CustomFileHandle cfh;

namespace mbed
{
FileHandle *mbed_override_console(int fd)
{
    return &cfh;
}
}; // namespace mbed

BufferedSerial serial(USBTX, USBRX, 115200);

int main()
{
    const char *msg = (cfh.get_internal_buf() == nullptr) ? "null" : "full";
    serial.write((void *)msg, 4);

    printf("Hello\r\n");

    const char *msg_2 = (cfh.get_internal_buf() == nullptr) ? "null" : "full";
    serial.write((void *)msg_2, 4);
}
