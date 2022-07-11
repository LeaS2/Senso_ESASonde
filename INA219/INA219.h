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
/*
 *---------------- REFERENCE ----------------------------------------------------------------------
 * Original Information
 *  http://www.ti.com/product/INA219/description
 *  http://www.ti.com/lit/ds/sbos448f/sbos448f.pdf
 * Device kit
 *  https://learn.adafruit.com/adafruit-ina219-current-sensor-breakout/overview
 *  http://akizukidenshi.com/catalog/g/gM-08221/
 */

#ifndef        MBED_INA219
#define        MBED_INA219

/////////// ADDRESS /////////////////////////////
//  7bit address = 0b1000000(0x40)
//  G=GND, V=VS+, A=SDA, L=SCL
//  e.g. _VG: A1=VS+, A0=GND
//    -> Please make sure your H/W configuration
// Set data into "addr"
#define INA219_ADDR_GG             (0x40 << 1)
#define INA219_ADDR_GV             (0x41 << 1)
#define INA219_ADDR_GA             (0x42 << 1)
#define INA219_ADDR_GL             (0x43 << 1)
#define INA219_ADDR_VG             (0x44 << 1)
#define INA219_ADDR_VV             (0x45 << 1)
#define INA219_ADDR_VA             (0x46 << 1)
#define INA219_ADDR_VL             (0x47 << 1)
#define INA219_ADDR_AG             (0x48 << 1)
#define INA219_ADDR_AV             (0x49 << 1)
#define INA219_ADDR_AA             (0x4a << 1)
#define INA219_ADDR_AL             (0x4b << 1)
#define INA219_ADDR_LG             (0x4c << 1)
#define INA219_ADDR_LV             (0x4d << 1)
#define INA219_ADDR_LA             (0x4e << 1)
#define INA219_ADDR_LL             (0x4f << 1)

/////////// REGISTER DEFINITION /////////////////
#define INA219_CONFIG              0x00
#define INA219_SHUNT_V             0x01
#define INA219_BUS_VOLT            0x02
#define INA219_POWER               0x03
#define INA219_CURRENT             0x04
#define INA219_CALBLATION          0x05

/////////// PARAMETER SETTING ///////////////////
// Set data into "shunt_register"
#define INA219_PAR_R_100MOHM       100
#define INA219_PAR_R_200MORM       200
#define INA219_PAR_R_MORM(x)       (x)
// Set data into "v_max"
#define INA219_PAR_V_16V           0
#define INA219_PAR_V_32V           1    // Default
// Set data into "gain"
#define INA219_PAR_G_40MV          0    // 400[mA] max if R=0.1[Ohm]
#define INA219_PAR_G_80MV          1
#define INA219_PAR_G_160MV         2
#define INA219_PAR_G_320MV         3    // Default
// Set data into "bus_adc_resolution"
#define INA219_PAR_B_9B_X1_84US    0x0
#define INA219_PAR_B_10B_X1_148US  0x1
#define INA219_PAR_B_11B_X1_276US  0x2
#define INA219_PAR_B_12B_X1_532US  0x3  // Default
#define INA219_PAR_B_12B_X2_1R06MS 0x9
#define INA219_PAR_B_12B_X4_2R13MS 0xa
#define INA219_PAR_B_12B_X8_4R26MS 0xb
#define INA219_PAR_B_12B_X16_8MS   0xc
#define INA219_PAR_B_12B_X32_17MS  0xd
#define INA219_PAR_B_12B_X64_34MS  0xe
#define INA219_PAR_B_12B_X128_68MS 0xf
// Set data into "shunt_adc_resolution"
#define INA219_PAR_S_9B_X1_84US    0x0
#define INA219_PAR_S_10B_X1_148US  0x1
#define INA219_PAR_S_11B_X1_276US  0x2
#define INA219_PAR_S_12B_X1_532US  0x3  // Default
#define INA219_PAR_S_12B_X2_1R06MS 0x9
#define INA219_PAR_S_12B_X4_2R13MS 0xa
#define INA219_PAR_S_12B_X8_4R26MS 0xb
#define INA219_PAR_S_12B_X16_8MS   0xc
#define INA219_PAR_S_12B_X32_17MS  0xd
#define INA219_PAR_S_12B_X64_34MS  0xe
#define INA219_PAR_S_12B_X128_68MS 0xf
// Set data into "mode"
#define INA219_PAR_M_PDWN          0
#define INA219_PAR_M_SHNT_TRG      1
#define INA219_PAR_M_BUS_TRG       2
#define INA219_PAR_M_SHNTBUS_TRG   3
#define INA219_PAR_M_ADC_OFF       4
#define INA219_PAR_M_SHNT_CONT     5
#define INA219_PAR_M_BUS_CONT      6
#define INA219_PAR_M_SHNTBUS_CONT  7    // Default

////////////// DATA TYPE DEFINITION ///////////////////////
typedef struct {
    // I2C Address
    uint8_t addr;
    // CONFIG REG
    uint8_t shunt_register;
    uint8_t v_max;
    uint8_t gain;
    uint8_t bus_adc_resolution;
    uint8_t Shunt_adc_resolution;
    uint8_t mode;
    // CALBLATION REG
    uint16_t calibration_data;
} INA219_TypeDef;

////////////// DEFAULT SETTING ////////////////////////////
// Standard parameter for easy set-up
const INA219_TypeDef ina219_std_paramtr = {
    // I2C Address
    INA219_ADDR_VV,
    // CONFIG REG
    INA219_PAR_R_100MOHM,       // 100 milli-ohm
    INA219_PAR_V_16V,           // 16V max
    INA219_PAR_G_40MV,          // Gain x1 (40mV -> 400mA max with 100 milliOhm)
    INA219_PAR_B_12B_X1_532US,  // Bus/resolution 12bit & one time convertion
    INA219_PAR_S_12B_X1_532US,  // Shunt/resolution 12bit & one time convertion
    INA219_PAR_M_SHNTBUS_CONT,  // Measure continuously both Shunt voltage and Bus voltage
    // CALBLATION REG
    16384                       // Calibration data
};

/** INA219 High-Side Measurement,Bi-Directional CURRENT/POWER MONITOR with I2C Interface
 *
 * @code
 * //--------- Default setting -----------------
 * #include "mbed.h"
 * #include "INA219.h"
 *
 * // I2C Communication
 * INA219 current(dp5, dp27, INA219_ADDR_GG);
 * // If you connected I2C line not only this device but also other devices,
 * //     you need to declare following method.
 * I2C    i2c(dp5, dp27);
 * INA219 current(I2C& p_i2c, INA219_ADDR_GG);
 *
 * int main() {
 *     while(1){
 *         printf("I=%+6.3f [mA]\r\n", current.read_current());
 *         wait(1.0):
 *     }
 * }
 * //--------- Detail setting -----------------
 * #include "mbed.h"
 * #include "INA219.h"
 *
 * const INA219_TypeDef ina219_my_paramtr = {
 *    // I2C Address
 *    INA219_ADDR_GG,
 *    // CONFIG Reg.
 *    INA219_PAR_R_100MOHM,     // 100 milli-ohm
 *    INA219_CFG_B16V,          // 16V max
 *    INA219_PAR_G_40MV,        // Gain x1
 *    INA219_PAR_M_SHNTBUS_CONT,// Measure continuously
 *    // CALIB
 *    16384                     // Calibration data is nothing
 * };
 *
 * I2C    i2c(dp5,dp27);
 * INA219 current(I2C& p_i2c, &ina219_my_paramtr);
 *
 * int main() {
 *     while(1){
 *         printf("I=%+6.3f [mA]\r\n", current.read_current());
 *         wait(1.0):
 *     }
 * }
 * @endcode
 */

class INA219
{
public:
    /** Configure data pin
      * @param data SDA and SCL pins
      * @param parameter address chip (INA219_TypeDef)
      * @param or just set address or just port
      */
    INA219(PinName p_sda, PinName p_scl, const INA219_TypeDef *ina219_parameter);
    INA219(PinName p_sda, PinName p_scl, uint8_t addr);
    INA219(PinName p_sda, PinName p_scl);

    /** Configure data pin (with other devices on I2C line)
      * @param I2C previous definition
      * @param parameter address chip (INA219_TypeDef)
      * @param or just set address or just port
      */
    INA219(I2C& p_i2c, const INA219_TypeDef *ina219_parameter);
    INA219(I2C& p_i2c, uint8_t addr);
    INA219(I2C& p_i2c);

    /** Read Current data
      * @param none
      * @return current [mA]
      */
    float read_current(void);
    float read_current_by_shuntvolt(void);

    /** Read Power data
      * @param none
      * @return power [w]
      */
    float read_power(void);

    /** Read Bus voltage
      * @param none
      * @return voltage [v]
      */
    float read_bus_voltage(void);

    /** Read Shunt voltage data
      * @param none
      * @return voltage [v]
      */
    float read_shunt_voltage(void);

    /** Read configration reg.
      * @param none
      * @return configrartion register value
      */
    uint16_t read_config(void);

    /** Set configration reg.
      * @param
      * @return configrartion register value
      */
    uint16_t set_config(uint16_t cfg);

    /** Read calibration reg.
      * @param none
      * @return calibration register value
      */
    uint16_t read_calb(void);

    /** Set calibration reg.
      * @param
      * @return calibration register value
      */
    uint16_t set_calb(uint16_t clb);

    /** Set I2C clock frequency
      * @param freq.
      * @return none
      */
    void frequency(int hz);

    /** Read register (general purpose)
      * @param register's address
      * @return register data
      */
    uint8_t read_reg(uint8_t addr);

    /** Write register (general purpose)
      * @param register's address
      * @param data
      * @return register data
      */
    uint8_t write_reg(uint8_t addr, uint8_t data);

protected:
    I2C *_i2c_p;
    I2C &_i2c;

    void initialize(void);

private:
    INA219_TypeDef ina219_set_data;
    int32_t scale_factor;
    uint8_t dt[4];

};

//-------------------------------------------------------------------------------------------------
// Following parts are only internal use and you only use it if you manage an internal config reg.
//-------------------------------------------------------------------------------------------------
/////////// BIT DEFINITION ////////////////////////////////
#define INA219_CFG_RESET           (1UL << 15)

#define INA219_CFG_B32V            (1UL << 13)
#define INA219_CFG_B16V            (0UL << 13)

#define INA219_CFG_PGA_DIV_1       (0UL << 11)
#define INA219_CFG_PGA_DIV_2       (1UL << 11)
#define INA219_CFG_PGA_DIV_4       (2UL << 11)
#define INA219_CFG_PGA_DIV_8       (3UL << 11)

#define INA219_CFG_BADC            (0xf << 7)

#define INA219_CFG_SADC_9B_84U     (0x0 << 3)
#define INA219_CFG_SADC_10B_148U   (0x1 << 3)
#define INA219_CFG_SADC_11B_276U   (0x2 << 3)
#define INA219_CFG_SADC_12B_532U   (0x3 << 3)
#define INA219_CFG_SADC_2S_1R06M   (0x9 << 3)
#define INA219_CFG_SADC_4S_2R13M   (0xa << 3)
#define INA219_CFG_SADC_8S_4R26M   (0xb << 3)
#define INA219_CFG_SADC_16S_8R51M  (0xc << 3)
#define INA219_CFG_SADC_32S_17M    (0xd << 3)
#define INA219_CFG_SADC_64S_34M    (0xe << 3)
#define INA219_CFG_SADC_128S_68M   (0xf << 3)

#define INA219_CFG_MODE_PDWN       (0UL << 0)
#define INA219_CFG_MODE_SHNTTRG    (1UL << 0)
#define INA219_CFG_MODE_BUSTRG     (2UL << 0)
#define INA219_CFG_MODE_SBTRG      (3UL << 0)
#define INA219_CFG_MODE_ADC_OFF    (4UL << 0)
#define INA219_CFG_MODE_SHNT_CONT  (5UL << 0)
#define INA219_CFG_MODE_BUS_CONT   (6UL << 0)
#define INA219_CFG_MODE_SB_CONT    (7UL << 0)

#endif  //  MBED_INA219

