/* This is a small test file to see what works and what doesn't on the LPC1768
 * processor.
 */

#include "SerialStream.h"
#include "mbed.h"

#define BAUDRATE 115200
#define CMD_BUFFER_SIZE 32

BufferedSerial serial(USBTX, USBRX, BAUDRATE);
SerialStream<BufferedSerial> pc(serial);
char buf[64];
Thread t;
EventQueue eventQueue;

void onSerialReceived(void)
{
    char *p_buf = buf;

    memset(buf, 0, sizeof(buf));
    while(pc.readable())
    {
        p_buf += pc.read(p_buf, sizeof(buf) - (p_buf - buf));
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
    pc.printf("Starting...\r\n");
    t.start(callback(&eventQueue, &EventQueue::dispatch_forever));
    pc.sigio(callback(onSigio));

    int i = 0;
    while(1)
    {
        pc.printf("%i\r\n", i++);
        ThisThread::sleep_for(1s);
    }
}