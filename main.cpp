#include <mbed.h>

#include "ADS124S0x.h"

constexpr PinName ADS_MOSI = NC;
constexpr PinName ADS_MISO = NC;
constexpr PinName ADS_SCLK = NC;
constexpr PinName ADS_CS   = NC;

void connect_to_adc()
{
    ADS124S0x ads(ADS_MOSI, ADS_MISO, ADS_SCLK, ADS_CS);

    printf("Created ADC\r\n");

    bool inited = ads.init();

    printf("Init result: %s\r\n", inited ? "Succeeded" : "Failed");
}

int main()
{
    connect_to_adc();

    int counter = 0;
    while (true)
    {
        counter++;
        printf("Hello, world! %d\r\n", counter);
        ThisThread::sleep_for(1000ms);
    }
}
