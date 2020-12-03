/* This is a small test file to see what works and what doesn't on the LPC1768
 * processor.
 */
#include "mbed.h"

int main()
{
    int i = 0;
    while(1)
    {
        printf("%i", i++);
        ThisThread::sleep_for(1s);
    }
}