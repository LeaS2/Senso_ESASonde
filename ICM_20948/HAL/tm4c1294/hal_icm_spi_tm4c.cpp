/**
 *  hal_mpu_spi_tm4c.c
 *
 *  SPI drivers for MPU9250 on TM4C1294NCPDT
 *  This file implements communication with MPU9250 IMU by utilizing SPI bus.
 *  SPI2 bus is used at 1MHZ speed with PN2 as slave select (configured as GPIO),
 *  PA5 as data-ready signal, and PL4 as power-control pin.
 *
 *  Created on: Mar 4, 2017
 *      Author: Vedran
 */
#include "hal_icm_tm4c.h"




/**     ICM20948 - related macros        */
#define ICM20948_SPI_BASE SSI2_BASE


static SPI* m_spi;
static DigitalOut* m_spi_cs;
/**
 * Initializes SPI2 bus for communication with MPU
 *   * SPI Bus frequency 1MHz, PD0 as MISO, PD1 as MOSI, PD3 as SCLK
 *   * Pin PL4 as power switch (control external MOSFET to cut-off power to MPU)
 *   * Pin PA5 as input, to receive data-ready signal from MPU
 *   * Pin PN2 as slave select, but configured as GPIO and manually toggled
 */
void HAL_MPU_Init(SPI* spi, DigitalOut* spi_cs)
{
	m_spi = spi;
	m_spi_cs = spi_cs;
}
/**
 * Check if MPU has raised an interrupt to notify it has new data ready
 * @return true if interrupt pin is high, false otherwise
 */
bool HAL_MPU_DataAvail()
{
    //return (MAP_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5) != 0);
	return 1;
}

/**
 * Write one byte of data to SPI bus and wait until transmission is over (blocking)
 * @param I2Caddress (NOT USED) Here for compatibility with I2C HAL implementation
 * @param regAddress Address of register in MPU to write into
 * @param data Data to write into the register
 */
void HAL_MPU_WriteByte(uint8_t I2Caddress, uint8_t regAddress, uint8_t data)
{
	uint8_t ret = 0u;
	*m_spi_cs = 0u;
	m_spi->write(regAddress);
	m_spi->write(data);
	*m_spi_cs = 1u;
}

/**
 * Send a byte-array of data through SPI bus (blocking)
 * @param I2Caddress (NOT USED) Here for compatibility with I2C HAL implementation
 * @param regAddress Address of a first register in MPU to start writing into
 * @param data Buffer of data to send
 * @param length Length of data to send
 * @return 0 to verify that function didn't hang somewhere
 */
int HAL_MPU_WriteBytes(void * context, uint8_t regAddress,
                       const uint8_t *data, uint32_t length)
{
	uint8_t ret = 0u;
	uint8_t ret_bytes = 0u;
	*m_spi_cs = 0u;
	m_spi->write(regAddress);
	ret_bytes = m_spi->write((const char*) data, length,(char*) data, length);
	*m_spi_cs = 1u;
	if(ret_bytes == 0u)
	{
		ret = 1u;
	}
	return ret;
}

/**
 * Read one byte of data from SPI device (performs dummy write as well)
 * @param I2Caddress (NOT USED) Here for compatibility with I2C HAL implementation
 * @param regAddress Address of register in MPU to read from
 * @return Byte of data received from SPI device
 */
uint8_t HAL_MPU_ReadByte(uint8_t I2Caddress, uint8_t regAddress)
{
	uint8_t ret_bytes = 0u;

	regAddress = 0x80u | regAddress;
	*m_spi_cs = 0u;
	m_spi->write(regAddress);
	ret_bytes = m_spi->write(0x00);
	*m_spi_cs = 1u;
	return ret_bytes;
}

/**
 * Read several bytes from SPI device (performs dummy write as well)
 * @param I2Caddress (NOT USED) Here for compatibility with I2C HAL implementation
 * @param regAddress Address of register in MPU to read from
 * @param count Number of bytes to red
 * @param dest Pointer to data buffer in which data is saved after reading
 */
int HAL_MPU_ReadBytes(void * context, uint8_t regAddress, uint8_t* data, uint32_t length)
{
	uint8_t ret = 0u;
		uint8_t ret_bytes = 0u;

		regAddress = 0x80u | regAddress;
		*m_spi_cs = 0u;
		m_spi->write(regAddress);
		ret_bytes = m_spi->write((const char*) data,length, (char*) data, length);
		*m_spi_cs = 1u;
		if(ret_bytes == 0u)
		{
			ret = 1u;
		}
		return ret;
}
