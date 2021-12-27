#include "mbed.h"

int main()
{
    printf("Hello, world\r\n");

    int counter = 0;
    while (1)
    {
        printf("%d\r\n", counter++);
        ThisThread::sleep_for(1s);
    }
}
