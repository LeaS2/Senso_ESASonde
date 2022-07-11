#ifndef __TEENSY_ICM_20948_H__
#define __TEENSY_ICM_20948_H__

#include "mbed.h"
#include "USBSerial.h"
/*************************************************************************
  Defines
*************************************************************************/

typedef struct {
  int spi_speed;
  int mode;
  bool enable_gyroscope;
  bool enable_accelerometer;
  bool enable_magnetometer;
  bool enable_quaternion;
  int gyroscope_frequency;
  int accelerometer_frequency;
  int magnetometer_frequency;
  int quaternion_frequency;

} ICM20948_DMPSettings;

/*************************************************************************
  Class
*************************************************************************/

class ICM20948_DMP
{
	public:
		ICM20948_DMP(SPI *spi, PinName cs);

		bool init(ICM20948_DMPSettings settings);
		void task();

		bool gyroDataIsReady();
		bool accelDataIsReady();
		bool magDataIsReady();
		bool quatDataIsReady();

		void readGyroData(float *x, float *y, float *z);
		void readAccelData(float *x, float *y, float *z);
		void readMagData(float *x, float *y, float *z);
		void readQuatData(float *w, float *x, float *y, float *z);

  private:
         SPI* m_spi;
         DigitalOut* m_spi_cs;
};


void icm20948_set_serial(USBSerial* usbSerial);

#endif // __TEENSY_ICM_20948_H__
