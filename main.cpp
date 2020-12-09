/* This is a small test file to see what works and what doesn't on the LPC1768
 * processor.
 */

#include "SerialStream.h"
#include "mbed.h"

#define BAUDRATE 115200
#define CMD_BUFFER_SIZE 32

BufferedSerial serial(USBTX, USBRX, 115200);
char buf[64];
Thread t;
EventQueue eventQueue;

void onSerialReceived(void)
{
    char *p_buf = buf;

    memset(buf, 0, sizeof(buf));
    while(serial.readable())
    {
        p_buf += serial.read(p_buf, sizeof(buf) - (p_buf - buf));
    }
    if(p_buf > buf)
    {
        printf("Received: %s\r\n", buf);
    }
}

void onSigio(void)
{
    eventQueue.call(onSerialReceived);
}

int main()
{
    printf("Starting...\r\n");
    t.start(callback(&eventQueue, &EventQueue::dispatch_forever));
    serial.sigio(callback(onSigio));
    while(1)
    {
    }
}