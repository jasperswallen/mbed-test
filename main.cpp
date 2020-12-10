/* This is a small test file to see what works and what doesn't on the LPC1768
 * processor.
 */

#include "SerialStream.h"
#include "mbed.h"

#define BAUDRATE 115200
#define CMD_BUFFER_SIZE 32

BufferedSerial serial(USBTX, USBRX, BAUDRATE);
SerialStream<BufferedSerial> pc(serial);
char buf[CMD_BUFFER_SIZE];
char cmdStr[CMD_BUFFER_SIZE];
volatile bool pendingCmd = false;
volatile bool charsAvailable = false;
volatile int currPos = 0;

void rxCallback(char c)
{
    static char prev_char = '\0';

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
    currPos = 0;
}

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
        for(size_t i = 0; i < strlen(buf); ++i)
        {
            rxCallback(buf[i]);
        }
    }
}

void onSigio(void)
{
    charsAvailable = true;
}

int main()
{
    pc.printf("Starting...\r\n");
    pc.sigio(callback(onSigio));

    int i = 0;
    while(1)
    {
        pc.printf("%i\r\n", i++);
        if(charsAvailable)
        {
            onSerialReceived();
        }

        if(pendingCmd)
        {
            updateCommand(cmdStr);
        }
        ThisThread::sleep_for(1s);
    }
}