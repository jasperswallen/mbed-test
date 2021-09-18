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
FILE * pc;

int main()
{
    pc = fdopen(&serial, "r+");

    fprintf(pc, "%s\n", (cfh.get_internal_buf() == nullptr) ? "null" : "full");
    ThisThread::sleep_for(1s);

    printf("Hello\r\n");

    fprintf(pc, "%s\n", (cfh.get_internal_buf() == nullptr) ? "null" : "full");

    ThisThread::sleep_for(1s);
}
