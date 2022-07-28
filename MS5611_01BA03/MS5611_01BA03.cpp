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
 
#include "MS5611_01BA03.h"


uint8_t MS5611_01BA03::baro_read_reg(char reg)
{
    uint8_t byte;
   
    ENABLE_BARO;
    m_spi->write(reg);
    byte =m_spi->write(0x00);
    DISABLE_BARO;

    return byte;
}

void MS5611_01BA03::baro_write_reg(uint8_t reg, uint8_t val)
{
    ENABLE_BARO;
    m_spi->write(reg);
    m_spi->write(val);
    DISABLE_BARO;
}

void MS5611_01BA03::ms5611_01ba_reset() {
    ENABLE_BARO;
    m_spi->write(MS561101BA_RESET);
    ThisThread::sleep_for(500);
    DISABLE_BARO;
}

static bool ms5611_01ba_crc(uint16_t* n_prom) {
    uint8_t cnt;                            // simple counter  
    uint16_t n_rem;                         // crc reminder 
    uint8_t crc_read;                       // original value of the crc 
    uint8_t  n_bit; 

    n_rem = 0x00; 
    crc_read = n_prom[7] & 0xF;             //save read CRC 
    n_prom[7]=(0xFFF0 & (n_prom[7]));       //CRC byte is replaced by 0 - 0xFF00 // but 4 bit CRC!!!!!!
    for (cnt = 0; cnt < 16; cnt++) {        // operation is performed on bytes 
        // choose LSB or MSB 
        if (cnt % 2 == 1) 
            n_rem ^= (uint16_t)((n_prom[cnt>>1]) & 0x00FF); 
        else 
            n_rem ^= (uint16_t) (n_prom[cnt>>1]>>8); 
            
        for (n_bit = 8; n_bit > 0; n_bit--) { 
            if (n_rem & (0x8000)) { 
                n_rem = (n_rem << 1) ^ 0x3000; 
            } 
            else { 
                n_rem = (n_rem << 1); 
            } 
        } 
    } 

    n_rem = (0x000F & (n_rem >> 12));   // final 4-bit reminder is CRC code 
    n_prom[7] = crc_read;               // restore the crc_read to its original place 

    return (crc_read == (n_rem ^ 0x00)) ;
}

static bool ms5611_01ba_crc2(uint16_t prom[]) {
    int32_t i, j;
    uint32_t res = 0;
    uint8_t crc = prom[7] & 0xF;
    prom[7] &= 0xFFF0;

    for (i = 0; i < 16; i++) {
        if (i & 1) 
            res ^= ((prom[i>>1]) & 0x00FF);
        else 
            res ^= (prom[i>>1]>>8);

        for (j = 8; j > 0; j--) {
            if (res & 0x8000) res ^= 0x1800;
            res <<= 1;
        }
    }
    prom[7] |= crc;

    return (crc == ((res >> 12) & 0xF)); 
}

void MS5611_01BA03::ms5611_01ba_readCalibration() {
    union { uint16_t val; uint8_t raw[2]; } data;
    uint8_t i;

    //read first prom settings
    for(i=0;i<8;i++) {
        ENABLE_BARO;
        m_spi->write(0xA0+2*i);
        data.raw[1] = m_spi->write(0x0);  // read a 16 bit register
        data.raw[0] = m_spi->write(0x0);
        ms5611_01ba_ctx.c[i] = data.val;
        DISABLE_BARO;
        ThisThread::sleep_for(10);
    }
}

bool MS5611_01BA03::init() {
    DISABLE_BARO;
    ThisThread::sleep_for(50);
    ms5611_01ba_reset();
    ms5611_01ba_readCalibration();

    return true; //ms5611_01ba_crc(ms5611_01ba_ctx.c); 
}
 
// read uncompensated temperature value: send command first
void MS5611_01BA03::start_conv_temp() {
    ENABLE_BARO;
    m_spi->write(MS561101BA_TEMPERATURE + OSR);
    DISABLE_BARO;
} 

// read uncompensated pressure value: send command first
void MS5611_01BA03::start_conv_press() {
    ENABLE_BARO;
    m_spi->write(MS561101BA_PRESSURE + OSR);
    DISABLE_BARO;
}

// read uncompensated pressure value: read result bytes
void MS5611_01BA03::read_conv_press() {
    ENABLE_BARO;
    m_spi->write(0x0);
    ms5611_01ba_ctx.up.raw[3] = 0x00;
    ms5611_01ba_ctx.up.raw[2] = m_spi->write(0x0);
    ms5611_01ba_ctx.up.raw[1] = m_spi->write(0x0);
    ms5611_01ba_ctx.up.raw[0] = m_spi->write(0x0);
    DISABLE_BARO;
}

// read uncompensated temperature value: read result bytes
void MS5611_01BA03::read_conv_temp() {
    ENABLE_BARO;
    m_spi->write(0x0);
    ms5611_01ba_ctx.ut.raw[3] = 0x00;
    ms5611_01ba_ctx.ut.raw[2] = m_spi->write(0x0);
    ms5611_01ba_ctx.ut.raw[1] = m_spi->write(0x0);
    ms5611_01ba_ctx.ut.raw[0] = m_spi->write(0x0);
    DISABLE_BARO;
}

void MS5611_01BA03::calculate(void) {
    int32_t off2 = 0, sens2 = 0, delt;

    int32_t dT   = ms5611_01ba_ctx.ut.val - ((uint32_t)ms5611_01ba_ctx.c[5] << 8);
    int64_t off  = ((uint32_t)ms5611_01ba_ctx.c[2] << 16) + (((int64_t)dT * ms5611_01ba_ctx.c[4]) >> 7);
    int64_t sens = ((uint32_t)ms5611_01ba_ctx.c[1] << 15) + (((int64_t)dT * ms5611_01ba_ctx.c[3]) >> 8);
    temperature  = 2000U + (((int64_t)dT * (int64_t)ms5611_01ba_ctx.c[6]) >> 23);

    if (temperature < 2000) { // temperature lower than 20st.C 
        delt = temperature - 2000;
        delt  = delt * delt;
        off2  = (5 * delt) >> 1; 
        sens2 = (5 * delt) >> 2; 
        if (temperature < -1500) { // temperature lower than -15st.C
            delt = temperature + 1500;
            delt = delt * delt;
            off2 += 7 * delt; 
            sens2 += (11 * delt) >> 1; 
        }
    } 
    off  -= off2; 
    sens -= sens2;
    pressure = (( (ms5611_01ba_ctx.up.val * sens ) >> 21) - off) >> 15;
}

MS5611_01BA03::MS5611_01BA03(SPI *spi, PinName cs): m_spi(spi)
{
    m_cs = new DigitalOut(cs);
    DISABLE_BARO;
    altitude = 0;
}

MS5611_01BA03::~MS5611_01BA03()
{
    timer.stop();
    delete(m_cs);
    m_cs = NULL;
}

int32_t MS5611_01BA03::getPressure()
{
    return pressure;
}

void MS5611_01BA03::calculateAltitude(void)
{
    altitude = (1.0f - pow((float)pressure/101325.0f, 0.190295f)) * 4433000.0f; //centimeter
}

int32_t MS5611_01BA03::getAltitude() {
    return altitude;
}

int32_t MS5611_01BA03::getTemperature() {
    return temperature;
}
