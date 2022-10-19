#ifndef HONEYWELL_RSC_H
#define HONEYWELL_RSC_H

#include "mbed.h"
#include "rtos.h"
#include "rsc_regs.h"

class Honeywell_RSC {
public:
  Honeywell_RSC(SPI *spi, PinName cs_ee_pin, PinName cs_adc_pin);

  void init(RSC_DATA_RATE rate, RSC_MODE mode);

  // chip selection
  void select_eeprom();
  void deselect_eeprom();
  void select_adc();
  void deselect_adc();

  // read number of bytes stored in EEPROM, starting from an address
  void eeprom_read(uint16_t address, uint8_t num_bytes, uint8_t *data);
  void get_catalog_listing();
  void get_serial_number();
  void get_pressure_range();
  void get_pressure_minimum();
  void get_pressure_unit();
  void get_pressure_type();
  void get_coefficients();
  void get_initial_adc_values(uint8_t *adc_init_values);

  // read from ADC
  void adc_read(READING_T type, uint8_t *data);
  int32_t get_temperature();
  int32_t get_pressure();

  // write to ADC
  void adc_write(uint8_t reg, uint8_t num_bytes, uint8_t *data);

  // other ADC related functions
  void setup_adc(uint8_t* adc_init_values);
  void add_dr_delay();
  
  // setter functions
  void set_data_rate(RSC_DATA_RATE dr);
  void set_mode(RSC_MODE mode);
  
  // getter functions
  const uint8_t* catalog_listing() const {return _catalog_listing;}
  const uint8_t* serial_number() const {return _serial_number;}
  float pressure_range() const {return _pressure_range;}
  float pressure_minimum() const {return _pressure_minimum;}
  char* pressure_unit_name() const {return _pressure_unit_name;}
  char* pressure_type_name() const {return _pressure_type_name;}
  void write_command_adc_read(READING_T type, int32_t* data);
  int32_t calculate_temperature(int32_t t_raw);
  int32_t calculate_pressure(int32_t p_raw);
  static void wait(RSC_DATA_RATE rate);
  int32_t getRawTemperatur();
  float getCoefficients(uint8_t row, uint8_t column);

private:
  // physical pin connections
  DigitalIn *m_drdy_pin;
  DigitalOut *m_cs_ee_pin;
  DigitalOut *m_cs_adc_pin;

  //communication
  SPI *m_spi;

  // from EEPROM
  uint8_t _catalog_listing[RSC_SENSOR_NAME_LEN];
  uint8_t _serial_number[RSC_SENSOR_NUMBER_LEN];
  float _pressure_range;
  float _pressure_minimum;
  PRESSURE_U _pressure_unit;
  char* _pressure_unit_name;
  PRESSURE_T _pressure_type;
  char* _pressure_type_name;

  // ADC
  RSC_DATA_RATE _data_rate;
  RSC_MODE _mode;

  // for calculating compensated pressure
  float _coeff_matrix[RSC_COEFF_T_ROW_NO][RSC_COEFF_T_COL_NO];
  int32_t _t_raw;
};

#endif // HONEYWELL_RSC_H
