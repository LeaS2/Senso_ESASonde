#include "Honeywell_RSC.h"
#include 	"USBSerial.h"

Honeywell_RSC::Honeywell_RSC(SPI *spi, PinName cs_ee_pin, PinName cs_adc_pin)
{
    m_cs_ee_pin = new DigitalOut(cs_ee_pin);
    m_cs_adc_pin = new DigitalOut(cs_adc_pin);

    // deselect both EEPROM and ADC
    *m_cs_ee_pin = 1u;
    *m_cs_adc_pin = 1u;

    m_spi = spi;
}

void Honeywell_RSC::init(RSC_DATA_RATE rate, RSC_MODE mode)
{
    // read and store constants from EEPROM
    get_catalog_listing();
    get_serial_number();
    get_pressure_range();
    get_pressure_minimum();
    get_pressure_unit();
    get_pressure_type();

    // setup ADC
    uint8_t adc_init_values[4];
    get_initial_adc_values(adc_init_values);
    setup_adc(adc_init_values);

    get_coefficients();

    set_data_rate(rate);
    set_mode(mode);
    rtos::ThisThread::sleep_for(5);
}

void Honeywell_RSC::select_eeprom()
{
    m_spi->format(8, 0);
    // enable CS_EE
    *m_cs_ee_pin = 0u;

    // make sure CS_ADC is not active
    *m_cs_adc_pin = 1u;
}

void Honeywell_RSC::deselect_eeprom()
{
    *m_cs_ee_pin = 1u;
}

void Honeywell_RSC::select_adc()
{
    m_spi->format(8, 1);

    // make sure CS_EE is not active
    *m_cs_ee_pin = 1u;

    // enable CS_ADC
    *m_cs_adc_pin = 0u;
}

void Honeywell_RSC::deselect_adc()
{
    *m_cs_adc_pin = 1u;
}

//////////////////// EEPROM read ////////////////////

void Honeywell_RSC::eeprom_read(uint16_t address, uint8_t num_bytes, uint8_t *data)
{
    // generate command (refer to sensor datasheet section 2.2)
    uint8_t command[2] = {0};
    command[0] = RSC_READ_EEPROM_INSTRUCTION | ((address & RSC_EEPROM_ADDRESS_9TH_BIT_MASK) >> 5);
    command[1] = address & 0xFF;
    // send command
    // select EEPROM
    select_eeprom();

    m_spi->write(command[0]);
    m_spi->write(command[1]);

    // receive results
    // - results are transmitted back after the last bit of the command is sent
    // - to get results, just transfer dummy data, as subsequent bytes will not used by sensor
    for (int i = 0; i < num_bytes; i++)
    {
        data[i] = m_spi->write(0x00);
    }

    // deselect EEPROM
    // - after command is sent, the sensor will keep sending bytes from EEPROM,
    //   in ascending order of address. Resetting the CS_EE pin at the end of
    //   the function means that when reading from EEPROM next time, the result
    //   would start at the correct address.
    deselect_eeprom();
}

void Honeywell_RSC::get_catalog_listing()
{
    eeprom_read(RSC_CATALOG_LISTING_MSB, RSC_SENSOR_NAME_LEN, _catalog_listing);
}

void Honeywell_RSC::get_serial_number()
{
    eeprom_read(RSC_SERIAL_NO_YYYY_MSB, RSC_SENSOR_NUMBER_LEN, _serial_number);
}

void Honeywell_RSC::get_pressure_range()
{
    uint8_t buf[RSC_PRESSURE_RANGE_LEN];
    eeprom_read(RSC_PRESSURE_RANGE_LSB, RSC_PRESSURE_RANGE_LEN, buf);
    // convert byte array to float (buf[0] is LSB)
    memcpy(&_pressure_range, &buf, sizeof(_pressure_range));
}

void Honeywell_RSC::get_pressure_minimum()
{
    uint8_t buf[RSC_PRESSURE_MINIMUM_LEN];
    eeprom_read(RSC_PRESSURE_MINIMUM_LSB, RSC_PRESSURE_MINIMUM_LEN, buf);
    // convert byte array to float (buf[0] is LSB)
    memcpy(&_pressure_minimum, &buf, sizeof(_pressure_minimum));
}

void Honeywell_RSC::get_pressure_unit()
{
    uint8_t buf[RSC_PRESSURE_UNIT_LEN] = {0};
    eeprom_read(RSC_PRESSURE_UNIT_MSB, RSC_PRESSURE_UNIT_LEN, buf);
    buf[RSC_PRESSURE_UNIT_LEN - 1] = '\0';
    if (buf[RSC_PRESSURE_UNIT_LEN - 2] == 'O')
    {
        _pressure_unit = INH2O;
        _pressure_unit_name = "inH2O";
    }
    else if (buf[RSC_PRESSURE_UNIT_LEN - 2] == 'a')
    {
        if (buf[RSC_PRESSURE_UNIT_LEN - 4] == 'K')
        {
          _pressure_unit = KPASCAL;
          _pressure_unit_name = "kilopascal";
        }
        else if (buf[RSC_PRESSURE_UNIT_LEN - 4] == 'M')
        {
          _pressure_unit = MPASCAL;
          _pressure_unit_name = "megapascal";
        }
        else
        {
          _pressure_unit = PASCAL;
          _pressure_unit_name = "pascal";
        }
    }
    else if (buf[RSC_PRESSURE_UNIT_LEN - 2] == 'r')
    {
        if (buf[RSC_PRESSURE_UNIT_LEN - 5] == 'm')
        {
          _pressure_unit = mBAR;
          _pressure_unit_name = "millibar";
        }
        else
        {
          _pressure_unit = BAR;
          _pressure_unit_name = "bar";
        }
    }
    else if (buf[RSC_PRESSURE_UNIT_LEN - 2] == 'i')
    {
        _pressure_unit = PSI;
        _pressure_unit_name = "psi";
    }
}

void Honeywell_RSC::get_pressure_type()
{
    uint8_t buf[RSC_SENSOR_TYPE_LEN];
    eeprom_read(RSC_PRESSURE_REFERENCE, RSC_SENSOR_TYPE_LEN, buf);
    switch (buf[0])
    {
        case 'D':
            _pressure_type = DIFFERENTIAL;
            _pressure_type_name = "differential";
            break;
        case 'A':
            _pressure_type = ABSOLUTE;
            _pressure_type_name = "absolute";
            break;
        case 'G':
            _pressure_type = GAUGE;
            _pressure_type_name = "gauge";
            break;
        default:
            _pressure_type = DIFFERENTIAL;
            _pressure_type_name = "differential";
    }
}

void Honeywell_RSC::get_coefficients()
{
    int base_address = RSC_OFFSET_COEFFICIENT_0_LSB;
    uint8_t buf[4] = {0};
    int i, j = 0;

    // coeff matrix structure
    // _coeff_matrix[i][j]
    //  i\j   0                     1                     2                     3
    //  0   OffsetCoefficient0    OffsetCoefficient1    OffsetCoefficient2    OffsetCoefficient3
    //  1   SpanCoefficient0      SpanCoefficient1      SpanCoefficient2      SpanCoefficient3
    //  2   ShapeCoefficient0     ShapeCoefficient1     ShapeCoefficient2     ShapeCoefficient3

    // storing all the coefficients
    for (i = 0; i < RSC_COEFF_T_ROW_NO; i++)
    {
        for (j = 0; j < RSC_COEFF_T_COL_NO; j++)
        {
            // 80 is the number of bytes that separate the beginning
            // of the address spaces of all the 3 coefficient groups
            // refer the datasheet for more info
            base_address = RSC_OFFSET_COEFFICIENT_0_LSB + i * 80 + j * 4;
            eeprom_read(base_address, 4, buf);
            memcpy(&_coeff_matrix[i][j], (&buf), sizeof(_coeff_matrix[i][j]));
        }
    }
}

void Honeywell_RSC::get_initial_adc_values(uint8_t* adc_init_values)
{
    eeprom_read(RSC_ADC_CONFIG_00, 1, &adc_init_values[0]);
    rtos::ThisThread::sleep_for(2);
    eeprom_read(RSC_ADC_CONFIG_01, 1, &adc_init_values[1]);
    rtos::ThisThread::sleep_for(2);
    eeprom_read(RSC_ADC_CONFIG_02, 1, &adc_init_values[2]);
    rtos::ThisThread::sleep_for(2);
    eeprom_read(RSC_ADC_CONFIG_03, 1, &adc_init_values[3]);
    rtos::ThisThread::sleep_for(2);
}

/*
 * write command and read in the same time the new value
 */
void Honeywell_RSC::write_command_adc_read(READING_T type, int32_t *value)
{

    uint8_t data[3];
    static uint8_t command[2] = {0};
    // WREG byte
    command[0] = RSC_ADC_WREG
               | ((1 << 2) & RSC_ADC_REG_MASK);
    // configuration byte, which includes DataRate, Mode, Pressure/Temperature choice
    command[1] = (((_data_rate << RSC_DATA_RATE_SHIFT) & RSC_DATA_RATE_MASK)
                | ((_mode << RSC_OPERATING_MODE_SHIFT) & RSC_OPERATING_MODE_MASK)
                | (((type & 0x01) << 1) | RSC_SET_BITS_MASK));
    // send command
    select_adc();
    data[0] = m_spi->write(0x00u);
    data[1] = m_spi->write(command[0]);
    data[2] = m_spi->write(command[1]);
    m_spi->write(0x08u);
    deselect_adc();

    *value = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | ((uint32_t)data[2]);
}



//////////////////// ADC read ////////////////////

void Honeywell_RSC::adc_read(READING_T type, uint8_t *data)
{
    // refer to datasheet section 3

    // need to configure the ADC to temperature mode first
    // generate command
    static uint8_t command[2] = {0};
    // WREG byte
    command[0] = RSC_ADC_WREG
               | ((1 << 2) & RSC_ADC_REG_MASK);
    // configuration byte, which includes DataRate, Mode, Pressure/Temperature choice
    command[1] = (((_data_rate << RSC_DATA_RATE_SHIFT) & RSC_DATA_RATE_MASK)
                | ((_mode << RSC_OPERATING_MODE_SHIFT) & RSC_OPERATING_MODE_MASK)
                | (((type & 0x01) << 1) | RSC_SET_BITS_MASK));
    // send command
    select_adc();
    m_spi->write(command[0]);
    m_spi->write(command[1]);
    m_spi->write(0x08);
    deselect_adc();

    add_dr_delay();

    // receive results
    select_adc();
    // send 0x10 command to start data conversion on ADC
    for (int i = 0; i < 4; i++)
    {
        data[i] = m_spi->write(0x00);
    }
    deselect_adc();
}

int32_t Honeywell_RSC::calculate_temperature(int32_t t_raw)
{
    // first 14 bits represent temperature
    // following 10 bits are random thus discarded
    t_raw >>= 10;
    t_raw &= 0x3FFF;
    if(t_raw & 0x2000)
    {
        t_raw |= 0xFFFFC000;
    }
    _t_raw = t_raw;

    float temp = (float)t_raw * 0.03125f;
    temp = temp * 100u;
    return temp;
}


int32_t Honeywell_RSC::calculate_pressure(int32_t p_raw)
{
    //if value nagativ, fill with 1
    if(p_raw & 0x800000UL)
    {
        p_raw |= 0xFF000000;
    }

    // calculate compensated pressure
    // refer to datasheet section 1.3 Compensation Mathematics
    float x = (_coeff_matrix[0][3] * _t_raw * _t_raw * _t_raw);
    float y = (_coeff_matrix[0][2] * _t_raw * _t_raw);
    float z = (_coeff_matrix[0][1] * _t_raw);
    float p_int1 = (float)p_raw - (x + y + z + _coeff_matrix[0][0]);

    x = (_coeff_matrix[1][3] * _t_raw * _t_raw * _t_raw);
    y = (_coeff_matrix[1][2] * _t_raw * _t_raw);
    z = (_coeff_matrix[1][1] * _t_raw);
    float p_int2 = p_int1 / (x + y + z + _coeff_matrix[1][0]);

    x = (_coeff_matrix[2][3] * p_int2 * p_int2 * p_int2);
    y = (_coeff_matrix[2][2] * p_int2 * p_int2);
    z = (_coeff_matrix[2][1] * p_int2);
    float p_comp_fs = x + y + z + _coeff_matrix[2][0];

    float p_comp = (p_comp_fs * _pressure_range) + _pressure_minimum;

    //1mbar ^= 100 pa
    //set the value in pa
    p_comp = p_comp * 100.0;
    return p_comp;
}

int32_t Honeywell_RSC::get_temperature()
{
    // reads temperature from ADC, stores raw value in sensor object, but returns the temperature in Celsius
    // refer to datasheet section 3.5 ADC Programming and Read Sequence – Temperature Reading

    uint8_t sec_arr[4] = {0};

    adc_read(TEMPERATURE, sec_arr);

    // first 14 bits represent temperature
    // following 10 bits are random thus discarded
    _t_raw = (((int32_t)sec_arr[0] << 8) | (int32_t)sec_arr[1]) >> 2;
    float temp = (float)_t_raw * 0.03125f;
    temp = temp * 100u;
    return temp;
}


int32_t Honeywell_RSC::get_pressure()
{
    // reads uncompensated pressure from ADC, then use temperature reading to convert it to compensated pressure
    // refer to datasheet section 3.6 ADC Programming and Read Sequence – Pressure Reading

    // read the 24 bits uncompensated pressure
    uint8_t sec_arr[4];
    adc_read(PRESSURE, sec_arr);
    int32_t p_raw = ((uint32_t)sec_arr[0] << 16) | ((uint32_t)sec_arr[1] << 8) | ((uint32_t)sec_arr[2]);


    //if value nagativ, fill with 1
    if(p_raw & 0x800000UL)
    {
        p_raw |= 0xFF000000;
    }

    // calculate compensated pressure
    // refer to datasheet section 1.3 Compensation Mathematics
    float x = (_coeff_matrix[0][3] * _t_raw * _t_raw * _t_raw);
    float y = (_coeff_matrix[0][2] * _t_raw * _t_raw);
    float z = (_coeff_matrix[0][1] * _t_raw);
    float p_int1 = (float)p_raw - (x + y + z + _coeff_matrix[0][0]);

    x = (_coeff_matrix[1][3] * _t_raw * _t_raw * _t_raw);
    y = (_coeff_matrix[1][2] * _t_raw * _t_raw);
    z = (_coeff_matrix[1][1] * _t_raw);
    float p_int2 = p_int1 / (x + y + z + _coeff_matrix[1][0]);

    x = (_coeff_matrix[2][3] * p_int2 * p_int2 * p_int2);
    y = (_coeff_matrix[2][2] * p_int2 * p_int2);
    z = (_coeff_matrix[2][1] * p_int2);
    float p_comp_fs = x + y + z + _coeff_matrix[2][0];

    float p_comp = (p_comp_fs * _pressure_range) + _pressure_minimum;

    //1mbar ^= 100 pa
    //set the value in pa
    p_comp = p_comp * 100.0;
    return p_comp;
}

void Honeywell_RSC::adc_write(uint8_t reg, uint8_t num_bytes, uint8_t* data)
{
    // check if the register and the number of bytes are valid
    // The number of bytes to write has to be - 1,2,3,4
    if (num_bytes <= 0 || num_bytes > 4)
        return;

    // the ADC registers are 0,1,2,3
    if (reg > 3)
        return;

    // generate command
    // the ADC REG Write (WREG) command is as follows: 0100 RRNN
    //   RR - Register Number             (0,1,2,3)
    //   NN - Number of Bytes to send - 1 (0,1,2,3)
    uint8_t command[num_bytes + 1];
    command[0] = RSC_ADC_WREG
               | ((reg << 2) & RSC_ADC_REG_MASK)
               | ((num_bytes - 1) & RSC_ADC_NUM_BYTES_MASK);

    for (int i = 0; i < num_bytes; i++)
    {
        command[i + 1] = data[i];
    }

    // send command
    select_adc();
    for (int i = 0; i < num_bytes + 1; i++)
    {
        m_spi->write(command[i]);
    }
    deselect_adc();
}

void Honeywell_RSC::add_dr_delay() {
    float delay_duration = 0;
    // calculating delay based on the Data Rate
    switch (_data_rate)
    {
        case N_DR_20_SPS:
              delay_duration = MSEC_PER_SEC / 20;
              break;
        case N_DR_45_SPS:
              delay_duration = MSEC_PER_SEC / 45;
              break;
        case N_DR_90_SPS:
              delay_duration = MSEC_PER_SEC / 90;
              break;
        case N_DR_175_SPS:
              delay_duration = MSEC_PER_SEC / 175;
              break;
        case N_DR_330_SPS:
              delay_duration = MSEC_PER_SEC / 330;
              break;
        case N_DR_600_SPS:
              delay_duration = MSEC_PER_SEC / 600;
              break;
        case N_DR_1000_SPS:
              delay_duration = MSEC_PER_SEC / 1000;
              break;
        case F_DR_40_SPS:
              delay_duration = MSEC_PER_SEC / 40;
              break;
        case F_DR_90_SPS:
              delay_duration = MSEC_PER_SEC / 90;
              break;
        case F_DR_180_SPS:
              delay_duration = MSEC_PER_SEC / 180;
              break;
        case F_DR_350_SPS:
              delay_duration = MSEC_PER_SEC / 350;
              break;
        case F_DR_660_SPS:
              delay_duration = MSEC_PER_SEC / 660;
              break;
        case F_DR_1200_SPS:
              delay_duration = MSEC_PER_SEC / 1200;
              break;
        case F_DR_2000_SPS:
              delay_duration = MSEC_PER_SEC / 2000;
              break;
        default:
            delay_duration = 50;
    }
    rtos::ThisThread::sleep_for((int)delay_duration + 2);
}

void Honeywell_RSC::wait(RSC_DATA_RATE rate)
{
    float delay_duration = 0;
    // calculating delay based on the Data Rate
    switch (rate)
    {
        case N_DR_20_SPS:
            delay_duration = MSEC_PER_SEC / 20;
            break;
        case N_DR_45_SPS:
            delay_duration = MSEC_PER_SEC / 45;
            break;
        case N_DR_90_SPS:
            delay_duration = MSEC_PER_SEC / 90;
            break;
        case N_DR_175_SPS:
            delay_duration = MSEC_PER_SEC / 175;
            break;
        case N_DR_330_SPS:
            delay_duration = MSEC_PER_SEC / 330;
            break;
        case N_DR_600_SPS:
            delay_duration = MSEC_PER_SEC / 600;
            break;
        case N_DR_1000_SPS:
            delay_duration = MSEC_PER_SEC / 1000;
            break;
        case F_DR_40_SPS:
            delay_duration = MSEC_PER_SEC / 40;
            break;
        case F_DR_90_SPS:
            delay_duration = MSEC_PER_SEC / 90;
            break;
        case F_DR_180_SPS:
            delay_duration = MSEC_PER_SEC / 180;
            break;
        case F_DR_350_SPS:
            delay_duration = MSEC_PER_SEC / 350;
            break;
        case F_DR_660_SPS:
            delay_duration = MSEC_PER_SEC / 660;
            break;
        case F_DR_1200_SPS:
            delay_duration = MSEC_PER_SEC / 1200;
            break;
        case F_DR_2000_SPS:
            delay_duration = MSEC_PER_SEC / 2000;
            break;
        default:
            delay_duration = 50;
    }
    rtos::ThisThread::sleep_for((int)delay_duration + 2);
}

void Honeywell_RSC::set_data_rate(RSC_DATA_RATE dr)
{
    _data_rate = dr;
    switch (dr)
    {
        case N_DR_20_SPS:
        case N_DR_45_SPS:
        case N_DR_90_SPS:
        case N_DR_175_SPS:
        case N_DR_330_SPS:
        case N_DR_600_SPS:
        case N_DR_1000_SPS:
              set_mode(NORMAL_MODE);
              break;
        case F_DR_40_SPS:
        case F_DR_90_SPS:
        case F_DR_180_SPS:
        case F_DR_350_SPS:
        case F_DR_660_SPS:
        case F_DR_1200_SPS:
        case F_DR_2000_SPS:
              set_mode(FAST_MODE);
              break;
        default:
            set_mode(NA_MODE);
    }
}

void Honeywell_RSC::set_mode(RSC_MODE mode)
{
    RSC_MODE l_mode;

    switch (mode)
    {
        case NORMAL_MODE:
            if (_data_rate < N_DR_20_SPS || _data_rate > N_DR_1000_SPS) {
            //Serial.println("RSC: Normal mode not supported with the current selection of data rate\n");
            //Serial.println("RSC: You will see erronous readings\n");
            l_mode = NA_MODE;
            } else
            l_mode = NORMAL_MODE;
            break;
        case FAST_MODE:
            if (_data_rate < F_DR_40_SPS || _data_rate > F_DR_2000_SPS) {
            //Serial.println("RSC: Fast mode not supported with the current selection of data rate\n");
            //Serial.println("RSC: You will see erronous readings\n");
            l_mode = NA_MODE;
            } else
            l_mode = FAST_MODE;
            break;
        default:
            l_mode = NA_MODE;
    }
    _mode = l_mode;
}

void Honeywell_RSC::setup_adc(uint8_t* adc_init_values)
{
    // refer to datasheet section 3.4 ADC Programming Sequence – Power Up
    select_adc();
    m_spi->write(RSC_ADC_RESET_COMMAND);
    deselect_adc();
    rtos::ThisThread::sleep_for(10);
    uint8_t command[4] = {adc_init_values[0], adc_init_values[1], adc_init_values[2], adc_init_values[3]};
    adc_write(0, 4, command);

}
/*
 * get raw temperatur
 */
int32_t Honeywell_RSC::getRawTemperatur()
{
    return _t_raw;
}
