#include <mbed.h>

#include "KX134SPI.h"
#include "ZEDFP9.h"

constexpr PinName ACC_MOSI = PF_9;
constexpr PinName ACC_MISO = PF_8;
constexpr PinName ACC_SCLK = PF_7;
constexpr PinName ACC_CS   = PF_10;

constexpr PinName GPS_CS  = PD_15;
constexpr PinName GPS_RST = PD_14;

constexpr PinName MISC_MOSI = PE_14;
constexpr PinName MISC_MISO = PE_13;
constexpr PinName MISC_SCLK = PE_12;

void connect_to_accel()
{

    KX134SPI accel(ACC_MOSI, ACC_MISO, ACC_SCLK, ACC_CS);

    printf("Connecting to accelerometer...\r\n");
    if (accel.init())
    {
        printf("Successfully Connected!\r\n");
    }
    else
    {
        printf("Failed to connect\r\n");
    }

    if (accel.checkExistence())
    {
        printf("Exists!\r\n");
    }
    else
    {
        printf("No existence\r\n");
    }
}

void connect_to_gps()
{
    UBlox::ZEDF9P gps(MISC_MOSI, MISC_MISO, GPS_RST, MISC_SCLK, GPS_CS);

    printf("Connecting to GPS...\r\n");
    if (gps.begin(true))
    {
        printf("Successfully Connected!\r\n");
    }
    else
    {
        printf("Failed to connect\r\n");
    }
}

int main()
{
    connect_to_accel();
    connect_to_gps();

    int counter = 0;

    while (true)
    {
        counter++;
        printf("Hello, world! %d\r\n", counter);
        ThisThread::sleep_for(1000ms);
    }
}
