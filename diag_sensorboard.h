/*
 * diag_sensorbaord.h
 *
 *  Created on: 18.07.2022
 *      Author: Marks_Markus
 */

#ifndef DIAG_SENSORBOARD_H_
#define DIAG_SENSORBOARD_H_

#define GET_SESSION               0x30
#define START_BOOTLOADER          0x40
#define GET_BOARD_ID              0x50
#define START_SENDING_SENSOR_DATA 0x60
#define STOP_SENDING_SENSOR_DATA  0x70
#define GET_ERROR_INFORMATION     0x80
#define SET_SENSOR_CONFIG         0x90

#define CONTROL_WORD              0x20
/*
 * frame for diagnostic data
 */
struct diag_data
{
	uint8_t id;
	uint8_t data[16];
};



#endif /* DIAG_SENSORBOARD_H_ */
