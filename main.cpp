#include <mbed.h>

#include "ADIS16467.h"
#include "BNO080.h"
#include "KX134SPI.h"
#include "LM75B.h"
#include "MS5607SPI.h"
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

constexpr PinName ALT_CS = PE_7;

constexpr PinName ADIS_CS  = PE_9;
constexpr PinName ADIS_RST = PE_8;

constexpr PinName BNO_RST  = PD_9;
constexpr PinName BNO_INT  = PD_10;
constexpr PinName BNO_WAKE = PD_11;
constexpr PinName BNO_MISO = PB_14;
constexpr PinName BNO_MOSI = PB_15;
constexpr PinName BNO_SCLK = PB_10;
constexpr PinName BNO_CS   = PD_8;

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

void connect_to_altimeter()
{
    MS5607SPI alt(MISC_MOSI, MISC_MISO, MISC_SCLK, ALT_CS);

    printf("\r\n\r\nConnecting to altimeter...\r\n");
    if (alt.init())
    {
        printf("Successfully Connected!\r\n");

        /* Perform a temperature conversion */
        alt.startTempConversion();
        while (alt.conversionInProgress())
        {
            ThisThread::sleep_for(100ms);
        }
        int rawTemp = alt.getConversionResult();

        /* Perform a pressure conversion */
        alt.startPressureConversion();
        while (alt.conversionInProgress())
        {
            ThisThread::sleep_for(100ms);
        }
        int rawPressure = alt.getConversionResult();

        printf(
            "Read temp = %f, pressure = %f, altitude = %f\r\n",
            alt.convertToTemp(rawTemp),
            alt.convertToPressure(rawTemp, rawPressure),
            alt.convertToAltitude(rawTemp, rawPressure));
    }
    else
    {
        printf("Failed to connect\r\n");
    }
}

void connect_to_adis()
{
    ADIS16467 adis(MISC_MOSI, MISC_MISO, MISC_SCLK, ADIS_CS, ADIS_RST);

    adis.initADIS();

    printf("\r\n\r\nConnecting to ADIS...\r\n");
    if (adis.checkExistence())
    {
        printf("Successfully Connected!\r\n");

        adis.resetDeltaAngle();
        while (!adis.hasNewData())
        {
            ThisThread::sleep_for(100ms);
        }

        ADIS16467::BurstReadResult result = {};
        adis.burstRead(result);

        float accelX = (float)(result.accelX) * adis.ACCEL_CONV;
        float accelY = (float)(result.accelY) * adis.ACCEL_CONV;
        float accelZ = (float)(result.accelZ) * adis.ACCEL_CONV;

        printf("Read accel mg: %.02f | %.02f | %.02f\r\n", accelX, accelY, accelZ);
    }
    else
    {
        printf("Failed to connect\r\n");
    }
}

void connect_to_bno()
{
    BNO080I2C bno_i2c(I2C3_SDA, I2C3_SCL, BNO_INT, BNO_RST);

    printf("\r\n\r\nConnecting to BNO over I2C...\r\n");

    ThisThread::sleep_for(1s);

    if (bno_i2c.begin())
    {
        printf("Successfully Connected!\r\n");
    }
    else
    {
        printf("Failed to connect\r\n");
    }

    BNO080SPI bno_spi(BNO_RST, BNO_INT, BNO_WAKE, BNO_MISO, BNO_MOSI, BNO_SCLK, BNO_CS, 2000000);

    printf("\r\n\r\nConnecting to BNO over SPI...\r\n");
    if (bno_spi.begin())
    {
        printf("Successfully Connected!\r\n");
    }
    else
    {
        printf("Failed to connect\r\n");
    }

    ThisThread::sleep_for(1s);
}

int main()
{
    connect_to_accel();
    connect_to_gps();
    connect_to_temp_sensor();
    connect_to_altimeter();
    connect_to_adis();
    connect_to_bno();

    int counter = 0;

    while (true)
    {
        counter++;
        printf("Hello, world! %d\r\n", counter);
        ThisThread::sleep_for(1000ms);
    }
}
