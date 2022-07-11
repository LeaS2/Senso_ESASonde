/**
 * @author Sergii Doroshenko
 *
 * @section LICENSE
 *
 * Copyright (c) 2010 ARM Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * MS5611_01BA03 Barometric Pressure Sensor, Variometer.
 *
 * Datasheet:
 *
 * http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
 */
 
#ifndef MS5611_01BA03_SPI_H
#define MS5611_01BA03_SPI_H

/**
 * Includes
 */
#include "mbed.h"

/**
 * Defines
 */
#define ENABLE_BARO *m_cs = 0
#define DISABLE_BARO *m_cs = 1

/**
 * Registers
 */
#define MS561101BA_PRESSURE     0x40
#define MS561101BA_TEMPERATURE  0x50
#define MS561101BA_RESET        0x1E

/**
 * OSR (Over Sampling Ratio) constants
 */
#define MS561101BA_OSR_256      0x00
#define MS561101BA_OSR_512      0x02
#define MS561101BA_OSR_1024     0x04
#define MS561101BA_OSR_2048     0x06
#define MS561101BA_OSR_4096     0x08

#define OSR MS561101BA_OSR_4096 



class MS5611_01BA03 {

public:

    /**
     * Constructor.
     *
     * Send reset sequence at first
     *
     * @param mosi - mbed pin to use for the MOSI/SDI spi line.
     * @param miso - mbed pin to use for the MISO/SDO spi line.
     * @param sclk - mbed pin to use for the SCL/SCLK  spi line.
     * @param cs - mbed pin to use for the CS spi line. (for sensor, not for mbed spi)
     */
	MS5611_01BA03(SPI *spi, PinName cs);
    ~MS5611_01BA03();
    bool init();
    int32_t getPressure();
    int32_t getAltitude();
    int32_t getTemperature();
    void start_conv_temp();
	void read_conv_temp();
	void start_conv_press();
	void read_conv_press();
	virtual void calculate(void);
	void calculateAltitude(void);
protected:

    // sensor registers from the MS561101BA datasheet
    struct {
        uint16_t c[8];
        union {uint32_t val; uint8_t raw[4]; } ut; //uncompensated T
        union {uint32_t val; uint8_t raw[4]; } up; //uncompensated P
        uint8_t  state;
        uint32_t deadline;
    } ms5611_01ba_ctx;

    uint8_t baro_read_reg(char reg);
    void baro_write_reg(uint8_t reg, uint8_t val);
    
    void ms5611_01ba_reset();
    void ms5611_01ba_readCalibration();
    
    
    

    
    SPI *m_spi;
    DigitalOut *m_cs;
    Timer timer;
    int32_t  pressure;
    int32_t  altitude;
    int32_t  temperature;
};
    
#endif // MS5611_01BA03_SPI_H
