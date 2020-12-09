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
volatile bool pendingCmd = false;
volatile bool charAvailable = false;

void rxCallback(char c)
{
    static char prev_char = '\0';
    static int currPos = 0;

    bool bufferFull = (currPos == (CMD_BUFFER_SIZE - 1));

    // if we got a newline (handling \n, \r, and \r\n equally)
    if(c == '\n' || c == '\r' || bufferFull)
    {
        // Case 1 (\n recieved)   : n > 1, command is copied to cmdStr
        // Case 2 (\r recieved)   : n > 1, command is copied to cmdStr
        // Case 3 (\r\n recieved) : \r is received first (Case 2),
        //                          then \n is recieved but n == 1, so it is
        //                          ignored.
        if(prev_char == '\r')
        {
            prev_char = c;
            pc.sync();
            return; // skip this \r or \n, since it's prob part of a CRLF (or an
                    // \r\r. Which would be weird.)
        }

        // insert string null terminator
        cmdStr[currPos] = '\0';
        pendingCmd = true;
        if(bufferFull)
        {
            prev_char = c;
            pc.sync();
        }
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
            pc.printf("charAvailable\r\n");
            char c;
            pc.read(&c, 1);
            rxCallback(c);
        }

        if(pendingCmd)
        {
            updateCommand(cmdStr);
        }
        ThisThread::sleep_for(1s);
    }
}