/*
 * sensor_config.h
 *
 *  Created on: 15.07.2022
 *      Author: Marks_Markus
 */

#ifndef SENSOR_CONFIG_H_
#define SENSOR_CONFIG_H_
/*
 * Parameter for the rsc sensors
 */
struct sensor_config
{
	RSC_DATA_RATE datarate;
	RSC_MODE mode;
	uint8_t delay;
};



#endif /* SENSOR_CONFIG_H_ */
