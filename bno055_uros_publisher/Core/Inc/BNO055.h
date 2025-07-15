/*
 * BNO055.h
 *
 *  Created on: Apr 18, 2024
 *      Author: 08809
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include "main.h"
#include "math.h"



#ifndef INC_DATA_CONVERT_H_
	typedef union {
		uint16_t u16;
		int16_t i16;
		uint8_t u8[2];
		int8_t i8[2];
	}data16_t;
#endif


#define BNO055_ADD_H 0x29 << 1
#define BNO055_ADD_L 0x28 << 1

/* Page ID = 0x00 */

#define CHIP_ID 0x00
#define ACC_ID 0x01
#define MAG_ID 0x02
#define GYR_ID 0x03

#define SW_REV_ID_LSB 0x04
#define SW_REV_ID_MSB 0x05
#define BL_REV_ID 0x06
#define PAGE_ID 0x07

#define ACC_DATA_X_LSB 0x08
#define ACC_DATA_X_MSB 0x09
#define ACC_DATA_Y_LSB 0x0A
#define ACC_DATA_Y_MSB 0x0B
#define ACC_DATA_Z_LSB 0x0C
#define ACC_DATA_Z_MSB 0x0D

#define MAG_DATA_X_LSB 0x0E
#define MAG_DATA_X_MSB 0x0F
#define MAG_DATA_Y_LSB 0x10
#define MAG_DATA_Y_MSB 0x11
#define MAG_DATA_Z_LSB 0x12
#define MAG_DATA_Z_MSB 0x13

#define GYR_DATA_X_LSB 0x14
#define GYR_DATA_X_MSB 0x15
#define GYR_DATA_Y_LSB 0x16
#define GYR_DATA_Y_MSB 0x17
#define GYR_DATA_Z_LSB 0x18
#define GYR_DATA_Z_MSB 0x19

#define EUL_DATA_HEADING_LSB 0x1A
#define EUL_DATA_HEADING_MSB 0x1B
#define EUL_DATA_ROLL_LSB 0x1C
#define EUL_DATA_ROLL_MSB 0x1D
#define EUL_DATA_PITCH_LSB 0x1E
#define EUL_DATA_PITCH_MSB 0x1F

#define QUA_DATA_W_LSB 0x20
#define QUA_DATA_W_MSB 0x21
#define QUA_DATA_X_LSB 0x22
#define QUA_DATA_X_MSB 0x23
#define QUA_DATA_Y_LSB 0x24
#define QUA_DATA_Y_MSB 0x25
#define QUA_DATA_Z_LSB 0x26
#define QUA_DATA_Z_MSB 0x27

#define LIA_DATA_X_LSB 0x28
#define LIA_DATA_X_MSB 0x29
#define LIA_DATA_Y_LSB 0x2A
#define LIA_DATA_Y_MSB 0x2B
#define LIA_DATA_Z_LSB 0x2C
#define LIA_DATA_Z_MSB 0x2D

#define GRV_DATA_X_LSB 0x2E
#define GRV_DATA_X_MSB 0x2F
#define GRV_DATA_Y_LSB 0x30
#define GRV_DATA_Y_MSB 0x31
#define GRV_DATA_Z_LSB 0x32
#define GRV_DATA_Z_MSB 0x33

#define TEMP 0x34

#define CALIB_STAT 0x35
#define ST_RESULT 0x36
#define INT_STA 0x37
#define SYS_CLK_STATUS 0x38
#define SYS_STATUS 0x39
#define SYS_ERR 0x3A
#define OPR_MODE 0x3D
#define UNIT_SEL 0x3B
#define PWR_MODE 0x3E
#define SYS_TRIGGER 0x3F
#define TEMP_SOURCE 0x40
#define AXIS_MAP_CONFIG 0x41
#define AXIS_MAP_SIGN 0x42

#define ACC_OFFSET_X_LSB 0x55
#define ACC_OFFSET_X_MSB 0x56
#define ACC_OFFSET_Y_LSB 0x57
#define ACC_OFFSET_Y_MSB 0x58
#define ACC_OFFSET_Z_LSB 0x59
#define ACC_OFFSET_Z_MSB 0x5A

#define MAG_OFFSET_X_LSB 0x5B
#define MAG_OFFSET_X_MSB 0x5C
#define MAG_OFFSET_Y_LSB 0x5D
#define MAG_OFFSET_Y_MSB 0x5E
#define MAG_OFFSET_Z_LSB 0x5F
#define MAG_OFFSET_Z_MSB 0x60

#define GYR_OFFSET_X_LSB 0x61
#define GYR_OFFSET_X_MSB 0x62
#define GYR_OFFSET_Y_LSB 0x63
#define GYR_OFFSET_Y_MSB 0x64
#define GYR_OFFSET_Z_LSB 0x65
#define GYR_OFFSET_Z_MSB 0x66

#define ACC_RADIUS_LSB 0x67
#define ACC_RADIUS_MSB 0x68
#define MAG_RADIUS_LSB 0x69
#define MAG_RADIUS_MSB 0x6A

/* Page ID = 0x01 */

#define ACC_Config 0x08
#define MAG_Config 0x09
#define GYR_Config_0 0x0A
#define GYR_Config_1 0x0B

#define ACC_Sleep_Config 0x0C
#define GYR_Sleep_Config 0x0D

#define INT_MSK 0x0F
#define INT_EN 0x10

#define ACC_AM_THRES 0x11
#define ACC_INT_Settings 0x12
#define ACC_HG_DURATION 0x13
#define ACC_HG_THRES 0x14
#define ACC_NM_THRES 0x15
#define ACC_NM_SET 0x16

#define GYR_INT_SETTING 0x17
#define GYR_HR_X_SET 0x18
#define GYR_DUR_X 0x19
#define GYR_HR_Y_SET 0x1A
#define GYR_DUR_Y 0x1B
#define GYR_HR_Z_SET 0x1C
#define GYR_DUR_Z 0x1D
#define GYR_AM_THRES 0x1E
#define GYR_AM_SET 0x1F

typedef enum {
	Page_ID_00 = 0,
	Page_ID_01
}Page_ID;

typedef enum {
	CONFIGMODE = 0,
	ACCONLY,
	MAGONLY,
	GYROONLY,
	ACCMAG,
	ACCGYRO,
	MAGGYRO,
	AMG,
	IMU,
	COMPASS,
	M4G,
	NDOF_FMC_OFF,
	NDOF
}OPRMode;

typedef enum {
	Normal_Mode = 0,
	Low_Power_Mode,
	Suspend_Mode
}PWRMode;

typedef enum {
	P0_Config = 0x21,
	P1_Config = 0x24,
	P2_Config = 0x24,
	P3_Config = 0x21,
	P4_Config = 0x24,
	P5_Config = 0x21,
	P6_Config = 0x21,
	P7_Config = 0x24
}Remap_Config;

typedef enum {
	P0_Sign = 0x04,
	P1_Sign = 0x00,
	P2_Sign = 0x06,
	P3_Sign = 0x02,
	P4_Sign = 0x03,
	P5_Sign = 0x01,
	P6_Sign = 0x07,
	P7_Sign = 0x05
}Remap_Sign;

typedef enum {
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
}Ascale;

typedef enum {
	ABW_7_81Hz = 0,
	ABW_15_63Hz,
	ABW_31_25Hz,
	ABW_62_5Hz,
	ABW_125Hz,
	ABW_250Hz,
	ABW_500Hz,
	ABW_1000Hz
}Abandwidth;

typedef enum {
	APM_Normal = 0,
	APM_Suspend,
	APM_Low_power_1,
	APM_Standby,
	APM_Low_Power_2,
	APM_Deep_Suspend
}APwrMode;

typedef enum {
	GFS_2000DPS = 0,
	GFS_1000DPS,
	GFS_500DPS,
	GFS_250DPS,
	GFS_125DPS
}Gscale;

typedef enum {
	GBW_523Hz = 0,
	GBW_230Hz,
	GBW_116Hz,
	GBW_47Hz,
	GBW_23Hz,
	GBW_12Hz,
	GBW_64Hz,
	GBW_32Hz
}Gbandwidth;

typedef enum {
	GPM_Normal = 0,
	GPM_Fast_Power_Up,
	GPM_Deep_Suspend,
	GPM_Suspend,
	GPM_Advanced_Powersave
}GPwrMode;

typedef enum {
	MBW_2Hz = 0,
	MBW_6Hz,
	MBW_8Hz,
	MBW_10Hz,
	MBW_15Hz,
	MBW_20Hz,
	MBW_25Hz,
	MBW_30Hz
}Mbandwidth;

typedef enum {
	MOM_Low_Power = 0,
	MOM_Regular,
	MOM_Enhanced_Regular,
	MOM_High_Accuracy
}MOprMode;

typedef enum {
	MPM_Normal = 0,
	MPM_Sleep,
	MPM_Suspend,
	MPM_Force_Mode
}MPwrMode;

typedef struct {
	double x;
	double y;
	double z;
}Vector_3D;

typedef struct {
	double x;
	double y;
	double z;
	double w;
}Vector_4D;

typedef struct {
	double roll;
	double pitch;
	double yaw;
}Vector_Euler;

typedef enum {
	MAGNETOMETER,
	GYROSCOPE,
	EULER,
	ACCELEROMETER,
	LINEARACCEL,
	GRAVITY,
	QUATERNION
}Vector_Type;

typedef struct {
	HAL_StatusTypeDef accel_stat;
	HAL_StatusTypeDef mag_stat;
	HAL_StatusTypeDef gyro_stat;
}BNO055_Calibration_Status;

typedef struct {
	data16_t accel_offset_x;
	data16_t accel_offset_y;
	data16_t accel_offset_z;
	data16_t mag_offset_x;
	data16_t mag_offset_y;
	data16_t mag_offset_z;
	data16_t gyro_offset_x;
	data16_t gyro_offset_y;
	data16_t gyro_offset_z;
	data16_t accel_radius;
	data16_t mag_radius;
}BNO055_Offsets;

typedef struct {
	I2C_HandleTypeDef *hi2cx;
	uint8_t address;
	uint8_t mode;
	HAL_StatusTypeDef flag;
	Vector_3D accel;
	Vector_3D mag;
	Vector_3D gyro;
	Vector_Euler euler;
	Vector_4D quat;
	Vector_3D lin_acc;
	Vector_3D grav;
	data16_t DataBuffer[22];
}BNO055_t;

HAL_StatusTypeDef BNO055_Init(BNO055_t *bno, I2C_HandleTypeDef *hi2cx, uint8_t addr, OPRMode mode);

void BNO055_Read(BNO055_t *bno, Vector_Type type);

void BNO055_Read_DMA(BNO055_t *bno, uint8_t fast_mode);

void BNO055_Calibrated(BNO055_t *bno, BNO055_Calibration_Status *calib_stat, BNO055_Offsets *bno_off);

void BNO055_SetOffsets(BNO055_t *bno, BNO055_Offsets *bno_offset);

void BNO055_SetAxis(BNO055_t *bno, Remap_Config config, Remap_Sign sign);


#endif /* INC_BNO055_H_ */
