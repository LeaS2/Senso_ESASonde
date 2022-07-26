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
 * 1. byte datarate
 * 2. byte mode
 * 3. byte delay in ms
 */
struct sensor_config
{
	RSC_DATA_RATE datarate:8;
	RSC_MODE mode:8;
	uint8_t delay;
};



#endif /* SENSOR_CONFIG_H_ */
