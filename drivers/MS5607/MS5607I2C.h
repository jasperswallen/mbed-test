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

#ifndef MS5607_I2C_H
#define MS5607_I2C_H

#include "MS5607Base.h"
#include "mbed.h"

class MS5607I2C : public MS5607Base {
public:
    MS5607I2C(PinName sda, PinName scl, int csb = 0) : i2c(sda, scl), i2cAddr(csb ? 0xEC : 0xEE)
            {
        init();
    }

    virtual bool conversionInProgress(){
        if(curConversion == NONE){
           return false;
        }
        if(timer.elapsed_time().count() > waitTime){
           waitTime = 0;
           timer.stop();
           timer.reset();
           return false;
        }
        return true;
    }
    virtual int getConversionResult(){
        char buf[3];
        curConversion = NONE;
        if (i2c.write(i2cAddr, ADC_READ, sizeof(cmd)) == 0 && i2c.read(i2cAddr, buf, sizeof(buf)) == 0){
            return buf[0] << 16 | buf[1] << 8 | buf[2];
        }
        else{
            return -1;
        }
    }


private:
    I2C i2c;
    int i2cAddr;

    virtual bool sendReset()
    {
        char buf[1] = {RESET};
        bool successful = i2c.write(i2cAddr, buf, 1) == 0;

        // per the datasheet, sending a reset takes exactly 2.8 ms
        ThisThread::sleep_for(3ms);

        return successful;
    }

    virtual int readPROM(int address) {
        char buf[2] = {PROM_READ | (address << 1), 0};

        if (i2c.write(i2cAddr, buf, 1) == 0 &&
            i2c.read(i2cAddr, buf, 2) == 0)
            return buf[0] << 8 | buf[1];

        return -1;
    }

    virtual void startConversion(int command){
        static int duration[] = {500, 1100, 2100, 4100, 8220};
        i2c.write(i2cAddr, cmd, sizeof(ADC_CONV | command));
        waitTime = duration[(command & 0x0F) >> 1];
        timer.stop();
        timer.reset();
        timer.start();   
    }

};

#endif
