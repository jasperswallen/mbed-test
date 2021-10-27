#include <mbed.h>

#include "KX134SPI.h"
#include "LM75B.h"
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

constexpr PinName I2C3_SDA = PC_9;
constexpr PinName I2C3_SCL = PA_8;

void connect_to_accel()
{

    KX134SPI accel(ACC_MOSI, ACC_MISO, ACC_SCLK, ACC_CS);

    printf("\r\n\r\nConnecting to accelerometer...\r\n");
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

    printf("\r\n\r\nConnecting to GPS...\r\n");
    if (gps.begin(true))
    {
        printf("Successfully Connected!\r\n");
    }
    else
    {
        printf("Failed to connect\r\n");
    }
}

void connect_to_temp_sensor()
{
    LM75B temp(I2C3_SDA, I2C3_SCL);

    printf("\r\n\r\nConnecting to temp sensor...\r\n");
    if (temp.open())
    {
        printf("Successfully Connected!\r\n");

        ThisThread::sleep_for(1s);

        printf("Read %fC temperature\r\n", temp.temp());
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
