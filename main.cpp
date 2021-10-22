#include <mbed.h>

int main()
{
    int counter = 0;
    while (true)
    {
        counter++;
        printf("Hello, world! %d\r\n", counter);
        ThisThread::sleep_for(1000ms);
    }
}
