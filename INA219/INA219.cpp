/*
 * mbed library program
 *  INA219 High-Side Measurement,Bi-Directional CURRENT/POWER MONITOR with I2C Interface
 *  by Texas Instruments
 *
 * Copyright (c) 2015,'17 Kenji Arai / JH1PJL
 *  http://www.page.sannet.ne.jp/kenjia/index.html
 *  http://mbed.org/users/kenjiArai/
 *      Created: January    25th, 2015
 *      Revised: August     23rd, 2017
 */

#include    "mbed.h"
#include    "INA219.h"

INA219::INA219 (PinName p_sda, PinName p_scl, const INA219_TypeDef *ina219_parameter)
 : _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p)
{
    _i2c.frequency(400000);
    ina219_set_data = *ina219_parameter;
    initialize();
}

INA219::INA219 (PinName p_sda, PinName p_scl, uint8_t addr)
 : _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p)
{
    _i2c.frequency(400000);
    // Use standard setting
    ina219_set_data = ina219_std_paramtr;
    // Change user defined address
    ina219_set_data.addr = addr;
    initialize();
}

INA219::INA219 (PinName p_sda, PinName p_scl)
 : _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p)
{
    _i2c.frequency(400000);
    // Use standard setting
    ina219_set_data = ina219_std_paramtr;
    initialize();
}

INA219::INA219 (I2C& p_i2c, const INA219_TypeDef *ina219_parameter)
 : _i2c(p_i2c)
{
    _i2c.frequency(400000);
    ina219_set_data = *ina219_parameter;
    initialize();
}

INA219::INA219 (I2C& p_i2c, uint8_t addr)
 : _i2c(p_i2c)
{
    _i2c.frequency(400000);
    // Use standard setting
    ina219_set_data = ina219_std_paramtr;
    // Change user defined address
    ina219_set_data.addr = addr;
    initialize();
}

INA219::INA219 (I2C& p_i2c) : _i2c(p_i2c)
{
    _i2c.frequency(400000);
    // Use standard setting
    ina219_set_data = ina219_std_paramtr;
    initialize();
}

/////////////// Read Current //////////////////////////////
float INA219::read_current()
{
    dt[0] = INA219_CURRENT;
    _i2c.write((int)ina219_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina219_set_data.addr, (char *)dt, 2, false);
    int16_t data = (dt[0] << 8) | dt[1];
    return (float)data * 25 /1000;
}

/////////////// Read Power ////////////////////////////////
float INA219::read_power()
{
    dt[0] = INA219_POWER;
    _i2c.write((int)ina219_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina219_set_data.addr, (char *)dt, 2, false);
    int16_t data = (dt[0] << 8) | dt[1];
    return (float)data / 2000;
}

/////////////// Read Bus_volt /////////////////////////////
float INA219::read_bus_voltage()
{
    dt[0] = INA219_BUS_VOLT;
    _i2c.write((int)ina219_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina219_set_data.addr, (char *)dt, 2, false);
    int16_t data = ((dt[0] << 8) | dt[1]) >> 3;
    return (float)data * 4 / 1000;
}

/////////////// Read Shunt volt ///////////////////////////
float INA219::read_shunt_voltage()
{
    dt[0] = INA219_SHUNT_V;
    _i2c.write((int)ina219_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina219_set_data.addr, (char *)dt, 2, false);
    int16_t data = (dt[0] << 8) | dt[1];
    return (float)data;
}

float INA219::read_current_by_shuntvolt()
{
    dt[0] = INA219_SHUNT_V;
    _i2c.write((int)ina219_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina219_set_data.addr, (char *)dt, 2, false);
    int16_t data = (dt[0] << 8) | dt[1];
    return (float)data / 10;
//    return ((float)data / ina219_set_data.shunt_register) / 1000;
}

/////////////// Read configulation ////////////////////////
uint16_t INA219::read_config()
{
    dt[0] = INA219_CONFIG;
    _i2c.write((int)ina219_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina219_set_data.addr, (char *)dt, 2, false);
    uint16_t data = (dt[0] << 8) | dt[1];
    return data;
}

/////////////// Set configulation /////////////////////////
uint16_t INA219::set_config(uint16_t cfg)
{
    uint16_t data = cfg;
    dt[0] = INA219_CONFIG;
    dt[1] = data >> 8;    // MSB 1st
    dt[2] = data & 0xff;  // LSB 2nd
    _i2c.write((int)ina219_set_data.addr, (char *)dt, 3, false);
    return data;
}

/////////////// Read Calibration reg. /////////////////////
uint16_t INA219::read_calb(void)
{
    dt[0] = INA219_CALBLATION;
    _i2c.write((int)ina219_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina219_set_data.addr, (char *)dt, 2, false);
    uint16_t data = (dt[0] << 8) | dt[1];
    return data;
}

/////////////// Set Calibration reg. //////////////////////
uint16_t INA219::set_calb(uint16_t clb)
{
    uint16_t data = clb;
    dt[0] = INA219_CALBLATION;
    dt[1] = data >> 8;    // MSB 1st
    dt[2] = data & 0xff;  // LSB 2nd
    _i2c.write((int)ina219_set_data.addr, (char *)dt, 3, false);
    return data;
}

/////////////// Read/Write specific register //////////////
uint8_t INA219::read_reg(uint8_t addr)
{
    dt[0] = addr;
    _i2c.write((int)ina219_set_data.addr, (char *)dt, 1, true);
    _i2c.read((int)ina219_set_data.addr, (char *)dt, 1, false);
    return dt[0];
}

uint8_t INA219::write_reg(uint8_t addr, uint8_t data)
{
    dt[0] = addr;
    dt[1] = data;
    _i2c.write((int)ina219_set_data.addr, (char *)dt, 2, false);
    return dt[1];
}

/////////////// Initialize ////////////////////////////////
void INA219::initialize()
{
    uint16_t data = 0;
    data  = (ina219_set_data.v_max & 0x01) << 13;
    data |= (ina219_set_data.gain & 0x03) << 11;
    data |= (ina219_set_data.bus_adc_resolution & 0x0f) << 6;
    data |= (ina219_set_data.Shunt_adc_resolution & 0x0f) << 3;
    data |= (ina219_set_data.mode & 0x07);
    dt[0] = INA219_CONFIG;
    dt[1] = data >> 8;    // MSB 1st
    dt[2] = data & 0xff;  // LSB 2nd
    _i2c.write((int)ina219_set_data.addr, (char *)dt, 3, false);
    dt[0] = INA219_CALBLATION;
    dt[1] = ina219_set_data.calibration_data >> 8;    // MSB 1st
    dt[2] = ina219_set_data.calibration_data & 0xff;  // LSB 2nd
    _i2c.write((int)ina219_set_data.addr, (char *)dt, 3, false);
    scale_factor = 0;
}

/////////////// I2C Freq. /////////////////////////////////
void INA219::frequency(int hz)
{
    _i2c.frequency(hz);
}


