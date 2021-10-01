#include <mbed.h>

#include "ConsoleRedirection.hpp"

static ConsoleOutput networkConsole;
static ConsoleRedirection console(networkConsole);

namespace mbed
{
/**
 * @brief Override the default console (for printf) to use a CustomFileHandle
 */
FileHandle *mbed_override_console(int fd)
{
    return &console;
}
}; // namespace mbed

/* Allow for manually printing over USBTX and RX for testing */
BufferedSerial serial(USBTX, USBRX, 115200);
FILE *pc;

int main()
{
    pc = fdopen(&serial, "r+");

    fprintf(pc, "1st: %u\n", console.getNetworkConsole().length);

    printf("Hello\r\n");

    fprintf(pc, "2nd: %u\n", console.getNetworkConsole().length);

    printf("Hello Again!\r\n");

    fprintf(pc, "3rd: %u\n", console.getNetworkConsole().length);

    fprintf(pc, "%.*s", console.getNetworkConsole().length, reinterpret_cast<const char *>(console.getNetworkConsole().buf));
}
