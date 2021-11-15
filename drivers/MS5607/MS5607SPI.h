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

#ifndef MS5607_SPI_H
#define MS5607_SPI_H

#include "MS5607Base.h"

class MS5607SPI : public MS5607Base {
#if HAMSTER_SIMULATOR == 1
public:
    MS5607SPI(PinName mosi, PinName miso, PinName sclk, PinName csb)
    {
    }

    virtual bool conversionInProgress()
    {
        return false;
    }

    virtual int getConversionResult()
    {
        return 0;
    }

private:
    virtual void writeCommand(int command, int ms = 0)
    {
        
    }
    virtual bool sendReset()
    {
        return true;
    }
    virtual int readPROM(int address)
    {
        return 0;
    }
    virtual int readADC(int command)
    {
        return 0;
    }

    virtual void startConversion(int command) {}
#else
public:
    MS5607SPI(PinName _mosi, PinName _miso, PinName _sclk, PinName _csb)
    : spi(new SPI(_mosi, _miso, _sclk, _csb, use_gpio_ssel_t()))
    , mosi(_mosi)
    , miso(_miso)
    , sclk(_sclk)
    , csb(_csb)
    {
    }
    
    virtual bool conversionInProgress(){
        if(curConversion == NONE){
            return false;
        }
        if(timer.elapsed_time().count() > waitTime){
            //! spi->deselect(); // i think this is wrong
            waitTime = 0;
            timer.stop();
            timer.reset();
            return false;
        }
        return true;
    }

    virtual int getConversionResult(){
        spi->select();
        spi->write(ADC_READ);
        int hi = spi->write(0);
        int mid = spi->write(0);
        int low = spi->write(0);

        spi->deselect();
        curConversion = NONE;
        return hi << 16 | mid << 8 | low;
    }

private:
    SPI* spi;
    PinName mosi;
    PinName miso;
    PinName sclk;
    PinName csb;

    virtual void writeCommand(int command, int ms = 0) {
        spi->select();
        spi->write(command);
        if (ms) ThisThread::sleep_for(std::chrono::milliseconds(ms));
        spi->deselect();
    }

  
    virtual bool sendReset()
    {
        spi->select();
        spi->write(RESET);

        delete spi;

        // per the datasheet, sending a reset takes exactly 2.8 ms
        // Empirically, it only takes 2.012 ms
        // Set the timeout to 3.5ms, just in case

        ThisThread::sleep_for(4ms);
        
        DigitalIn* misoReader = new DigitalIn(miso);

        bool success;
        // Return true if MISO goes high after 4ms (MISO is low by default)
        if(misoReader->read() == 0)
        {
            success = false;
        }
        else
        {
            success = true;
        }

        delete misoReader;
        spi = new SPI(mosi, miso, sclk, csb, use_gpio_ssel_t());
        return success;
    }


    virtual int readPROM(int address) {
        spi->select();
        spi->write(PROM_READ | address << 1);
        int hi = spi->write(0);
        int low = spi->write(0);
        spi->deselect();
        return hi << 8 | low;
    }

    virtual void startConversion(int command){
        static int duration[] = {500, 1100, 2100, 4100, 8220};
        spi->select();
        spi->write(ADC_CONV | command);
        
        waitTime = duration[(command & 0x0F) >> 1];
        timer.stop();
        timer.reset();
        timer.start();
        spi->deselect();
    }
#endif
};

#endif
