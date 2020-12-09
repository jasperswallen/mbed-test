/* This is a small test file to see what works and what doesn't on the LPC1768
 * processor.
 */

#include "SerialStream.h"
#include "mbed.h"

#define BAUDRATE 115200
#define CMD_BUFFER_SIZE 32

UnbufferedSerial serial(USBTX, USBRX, BAUDRATE);
SerialStream<UnbufferedSerial> pc(serial);

char cmdStr[CMD_BUFFER_SIZE];
int currPos = 0;
volatile bool pendingCmd = false;
volatile bool charAvailable = false;

void rxCallback(char c)
{
    static char prev_char = '\0';
    bool bufferFull = (currPos == (CMD_BUFFER_SIZE - 1));

    if(c == '\n' || c == '\r' || bufferFull)
    {
        if(prev_char == '\r')
        {
            prev_char = c;
            return;
        }

        cmdStr[currPos] = '\0';
        pendingCmd = true;
    }
    else
    {
        pc.putc(c);
        cmdStr[currPos] = c;
    }
    currPos++;
    prev_char = c;
}

// A function that echoes any received data back
void updateCommand(char *cmd)
{
    pc.printf("\r\nReceived CMD ");
    pc.printf(cmd);
    pc.printf("\r\n");

    pendingCmd = false;
    memset(cmdStr, 0, strlen(cmdStr));
    currPos = 0;
}

void charToRead()
{
    charAvailable = true;
}

int main(void)
{
    pc.printf("Starting\r\n");
    pc.attach(&charToRead, UnbufferedSerial::RxIrq);

    while(1)
    {
        if(charAvailable)
        {
            char c;
            pc.read(&c, 1);
            rxCallback(c);
            charAvailable = false;
        }

        if(pendingCmd)
        {
            updateCommand(cmdStr);
        }
        ThisThread::sleep_for(1s);
    }
}