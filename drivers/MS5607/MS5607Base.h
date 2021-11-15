/*
Copyright (c) 2012, Senio Networks, Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef MS5607_BASE_H
#define MS5607_BASE_H

#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#include "mbed.h"

class MS5607Base {
public:
    void printCoefficients() {
        printf("%" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32 ", %" PRId32
               ", %" PRId32 "\n",
               c1, c2, c3, c4, c5, c6);
    }

    void startTempConversion() {
        curConversion = TEMP;
        startConversion(ADC_D2 | OSR_4096);
    }

    void startPressureConversion() {
        curConversion = PRESSURE;
        startConversion(ADC_D1 | OSR_4096);
    }

    /**
     * Reads the temperature read by the sensor.
     * @return Temperature in degrees celcius.
     */
    float convertToTemp(int rawTemp) {
        #if HAMSTER_SIMULATOR == 1
        return HamsterInterface::getInterface().msInterface.msData.temp;
        #endif

        if(rawTemp < 0){
            return 0;
        }
        int dT = rawTemp - (c5 << 8);
        int temp = 2000 + ((dT * c6) >> 23);
        
        // 2nd order temperature compensation
        if (temp < 2000) {
            int t2 = (int64_t) dT * dT >> 31;
            temp -= t2;
        }

        return float(temp) / 100;
    }

    /**
     * Get the pressure read by the sensor.
     * @return Pressure in pascals.
     */
    float convertToPressure(int rawTemp, int rawPressure) {
        #if HAMSTER_SIMULATOR == 1
        return HamsterInterface::getInterface().msInterface.msData.pressure;
        #endif

        if(rawTemp < 0 || rawPressure < 0){
            return 0;
        }
        int dT = rawTemp - (c5 << 8);
        int temp = 2000 + ((dT * c6) >> 23);
        int64_t off = ((int64_t) c2 << 17) + ((int64_t) dT * c4 >> 6);
        int64_t sens = ((int64_t) c1 << 16) + ((int64_t) dT * c3 >> 7);

        // 2nd order temperature compensation
        if (temp < 2000) {
            int64_t off2 = (int64_t) 61 * (temp - 2000) * (temp - 2000) >> 4;
            int64_t sens2 = (int64_t) 2 * (temp - 2000) * (temp - 2000);
            if (temp < -1500) {
                off2 += (int64_t) 15 * (temp + 1500) * (temp + 1500);
                sens2 += (int64_t) 8 * (temp + 1500) * (temp + 1500);
            }
            off -= off2;
            sens -= sens2;
        }

        return float((((int64_t) rawPressure * sens >> 21) - off) >> 15);
    }

    /**
     * Get the altitude currently read by the sensor
     * @param pressure
     * @return
     */
    float convertToAltitude(int rawTemp, int rawPressure) {
        #if HAMSTER_SIMULATOR
        return HamsterInterface::getInterface().msInterface.msData.altitude;
        #endif

        if(rawTemp < 0 || rawPressure < 0){
            return 0;
        }
        return toAltitude(static_cast<int>(convertToPressure(rawTemp, rawPressure)));
    }

    MS5607Base(){
        curConversion = NONE;
        waitTime = 0;

    }
    virtual ~MS5607Base() = default;

    typedef enum {
        NONE, 
        TEMP, 
        PRESSURE
    } ConversionType;


protected:
    int32_t c1, c2, c3, c4, c5, c6;

    enum {
        RESET     = 0x1E,
        ADC_READ  = 0x00,
        ADC_CONV  = 0x40,
        ADC_D1    = 0x00,
        ADC_D2    = 0x10,
        OSR_256   = 0x00,
        OSR_512   = 0x02,
        OSR_1024  = 0x04,
        OSR_2048  = 0x06,
        OSR_4096  = 0x08,
        PROM_READ = 0xA0
    };

    /**
     * Resets the chip
     * @return whether or not the reset was successful
     */
    virtual bool sendReset() = 0;

    virtual int readPROM(int address) = 0;
    
    virtual void startConversion(int command) = 0;

    Timer timer;
    int waitTime;
    ConversionType curConversion;

public:
    /**
     * Intializes the altimeter.  This is already called by the constructor,
     * but it's here in case you need to initialize the chip again.
     * @return whether or not the altimeter was detected after reset
     */
    bool init()
    {
        bool success = false;
        success = sendReset();

        c1 = readPROM(1);
        c2 = readPROM(2);
        c3 = readPROM(3);
        c4 = readPROM(4);
        c5 = readPROM(5);
        c6 = readPROM(6);

        return success;
    }

    virtual bool conversionInProgress() = 0;
    virtual int getConversionResult() = 0;

    ConversionType getCurConversion(){
        return curConversion;
    }


protected:
    float toAltitude(float pressure) {
        // Ref. 29124-AltimeterAppNote1.pdf
        const float R = 287.052; // specific gas constant R*/M0
        const float g = 9.80665; // standard gravity 
        const float t_grad = 0.0065; // gradient of temperature
        const float t0 = 273.15 + 15; // temperature at 0 altitude
        const float p0 = 101325; // pressure at 0 altitude

        return t0 / t_grad * (1 - exp((t_grad * R / g) * log(pressure / p0)));
    }
};

#endif
