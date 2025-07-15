/*
 * BNO055_config.c
 *
 *  Created on: Nov 9, 2024
 *      Author: tim
 */

#include "BNO055_config.h"


BNO055_Offsets BNO055_off = {
		.accel_offset_x = BNO_ACC_OFF_X,
		.accel_offset_y = BNO_ACC_OFF_Y,
		.accel_offset_z = BNO_ACC_OFF_Z,
		.mag_offset_x = BNO_MAG_OFF_X,
		.mag_offset_y = BNO_MAG_OFF_Y,
		.mag_offset_z = BNO_MAG_OFF_Z,
		.gyro_offset_x = BNO_GYRO_OFF_X,
		.gyro_offset_y = BNO_GYRO_OFF_Y,
		.gyro_offset_z = BNO_GYRO_OFF_Z,
		.accel_radius = BNO_ACC_RAD,
		.mag_radius = BNO_MAG_RAD
};

#ifdef BNO_CALIB_ON
BNO055_Calibration_Status BNO055_stat;
#endif
