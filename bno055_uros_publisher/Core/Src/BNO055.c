/*
 * BNO055.c
 *
 *  Created on: Apr 18, 2024
 *      Author: 08809
 */

#include "BNO055.h"

HAL_StatusTypeDef BNO055_Init(BNO055_t *bno, I2C_HandleTypeDef *hi2cx, uint8_t addr, OPRMode mode)
{
	uint8_t txbuffer;
	uint8_t rxbuffer;

	bno->hi2cx = hi2cx;
	bno->address = BNO055_ADD_H;
	if (addr == 0) bno->address = BNO055_ADD_L;

	HAL_I2C_Mem_Read(bno->hi2cx, bno->address, CHIP_ID, 1, &rxbuffer, 1, 10);
	if (rxbuffer != 0xA0) {
		HAL_Delay(1000);
		HAL_I2C_Mem_Read(bno->hi2cx, bno->address, CHIP_ID, 1, &rxbuffer, 1, 10);
		if (rxbuffer != 0xA0) return HAL_ERROR;
	}

	txbuffer = CONFIGMODE;
	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &txbuffer, 1, 10);
	HAL_Delay(10);

	txbuffer = 0x20;
	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, SYS_TRIGGER, 1, &txbuffer, 1, 10);
	HAL_Delay(10);

	do {
		HAL_I2C_Mem_Read(bno->hi2cx, bno->address, CHIP_ID, 1, &rxbuffer, 1, 10);
	} while (rxbuffer != 0xA0);
	HAL_Delay(50);

	txbuffer = Normal_Mode;
	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, PWR_MODE, 1, &txbuffer, 1, 10);
	HAL_Delay(10);

	txbuffer = Page_ID_00;
	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, PAGE_ID, 1, &txbuffer, 1, 10);
	HAL_Delay(10);

	txbuffer = 0x00;
	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, SYS_TRIGGER, 1, &txbuffer, 1, 2);
	HAL_Delay(10);

	bno->mode = mode;
	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &bno->mode, 1, 2);

	bno->flag = HAL_OK;

	return HAL_OK;
}

void BNO055_Read(BNO055_t *bno, Vector_Type type)
{
	uint8_t read_mode;
	HAL_I2C_Mem_Read(bno->hi2cx, bno->address, OPR_MODE, 1, &read_mode, 1, 10);
	if (read_mode != bno->mode) {
		HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &bno->mode, 1, 10);
	}

	if (type != QUATERNION) {
		uint8_t rxbuffer[6];
		uint8_t data_reg;
		switch (type) {
		case ACCELEROMETER:
			data_reg = ACC_DATA_X_LSB;
			break;
		case GYROSCOPE:
			data_reg = GYR_DATA_X_LSB;
			break;
		case MAGNETOMETER:
			data_reg = MAG_DATA_X_LSB;
			break;
		case EULER:
			data_reg = EUL_DATA_HEADING_LSB;
			break;
		case LINEARACCEL:
			data_reg = LIA_DATA_X_LSB;
			break;
		case GRAVITY:
			data_reg = GRV_DATA_X_LSB;
			break;
		default:
		}

		HAL_I2C_Mem_Read(bno->hi2cx, bno->address, data_reg, 1, rxbuffer, 6, 10);

		int16_t x, y, z = 0;

		x  = ((int16_t) rxbuffer[0]) | (((int16_t) rxbuffer[1]) << 8);
		y  = ((int16_t) rxbuffer[2]) | (((int16_t) rxbuffer[3]) << 8);
		z  = ((int16_t) rxbuffer[4]) | (((int16_t) rxbuffer[5]) << 8);

		switch (type) {
		case ACCELEROMETER:
			bno->accel.x = ((double)x) / 100.0;
			bno->accel.y = ((double)y) / 100.0;
			bno->accel.z = ((double)z) / 100.0;
		case GYROSCOPE:
			bno->gyro.x = ((double)x) * M_PI / (16.0 * 180.0);
			bno->gyro.y = ((double)y) * M_PI / (16.0 * 180.0);
			bno->gyro.z = ((double)z) * M_PI / (16.0 * 180.0);
			break;
		case MAGNETOMETER:
			bno->mag.x = ((double)x) / 16.0;
			bno->mag.y = ((double)y) / 16.0;
			bno->mag.z = ((double)z) / 16.0;
			break;
		case EULER:
			bno->euler.yaw = ((double)x) * M_PI / (16.0 * 180.0);
			bno->euler.roll = ((double)y) * M_PI / (16.0 * 180.0);
			bno->euler.pitch = ((double)z) * M_PI / (16.0 * 180.0);
			break;
		case LINEARACCEL:
			bno->lin_acc.x = ((double)x) / 100.0;
			bno->lin_acc.y = ((double)y) / 100.0;
			bno->lin_acc.z = ((double)z) / 100.0;
		case GRAVITY:
			bno->grav.x = ((double)x) / 100.0;
			bno->grav.y = ((double)y) / 100.0;
			bno->grav.z = ((double)z) / 100.0;
		default:
		}
	} else {
		uint8_t rxbuffer[8];

		HAL_I2C_Mem_Read(bno->hi2cx, bno->address, QUA_DATA_X_LSB, 1, rxbuffer, 8, 10);

		int16_t x, y, z, w = 0;

		w  = ((int16_t) rxbuffer[0]) | (((int16_t) rxbuffer[1]) << 8);
		x  = ((int16_t) rxbuffer[2]) | (((int16_t) rxbuffer[3]) << 8);
		y  = ((int16_t) rxbuffer[4]) | (((int16_t) rxbuffer[5]) << 8);
		z  = ((int16_t) rxbuffer[6]) | (((int16_t) rxbuffer[7]) << 8);

		const double scale = (1.0 / (1 << 14));
		bno->quat.x = x * scale;
		bno->quat.y = y * scale;
		bno->quat.z = z * scale;
		bno->quat.w = w * scale;
	}

}

void BNO055_Read_DMA(BNO055_t *bno, uint8_t fast_mode)
{
	uint8_t read_mode;
	HAL_I2C_Mem_Read(bno->hi2cx, bno->address, OPR_MODE, 1, &read_mode, 1, 10);
	if (read_mode != bno->mode) {
		HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &bno->mode, 1, 10);
	}

	const double scale = (1.0 / (1 << 14));

	bno->accel.x = ((double) bno->DataBuffer[0].i16) / 100.0;
	bno->accel.y = ((double) bno->DataBuffer[1].i16) / 100.0;
	bno->accel.z = ((double) bno->DataBuffer[2].i16) / 100.0;

	bno->mag.x = ((double) bno->DataBuffer[3].i16) / 16.0;
	bno->mag.y = ((double) bno->DataBuffer[4].i16) / 16.0;
	bno->mag.z = ((double) bno->DataBuffer[5].i16) / 16.0;

	bno->gyro.x = ((double) bno->DataBuffer[6].i16) * M_PI / (16.0 * 180.0);
	bno->gyro.y = ((double) bno->DataBuffer[7].i16) * M_PI / (16.0 * 180.0);
	bno->gyro.z = ((double) bno->DataBuffer[8].i16) * M_PI / (16.0 * 180.0);

	bno->euler.yaw = ((double) bno->DataBuffer[9].i16) * M_PI / (16.0 * 180.0);
	bno->euler.roll = ((double) bno->DataBuffer[10].i16) * M_PI / (16.0 * 180.0);
	bno->euler.pitch = ((double) bno->DataBuffer[11].i16) * M_PI / (16.0 * 180.0);

	bno->quat.w = bno->DataBuffer[12].i16 * scale;
	bno->quat.x = bno->DataBuffer[13].i16 * scale;
	bno->quat.y = bno->DataBuffer[14].i16 * scale;
	bno->quat.z = bno->DataBuffer[15].i16 * scale;

	if (!fast_mode) {

		bno->lin_acc.x = ((double) bno->DataBuffer[16].i16) / 100.0;
		bno->lin_acc.y = ((double) bno->DataBuffer[17].i16) / 100.0;
		bno->lin_acc.z = ((double) bno->DataBuffer[18].i16) / 100.0;

		bno->grav.x = ((double) bno->DataBuffer[19].i16) / 100.0;
		bno->grav.y = ((double) bno->DataBuffer[20].i16) / 100.0;
		bno->grav.z = ((double) bno->DataBuffer[21].i16) / 100.0;

		HAL_I2C_Mem_Read_DMA(bno->hi2cx, bno->address, ACC_DATA_X_LSB, 1, bno->DataBuffer->u8, 44);
	} else {
		HAL_I2C_Mem_Read_DMA(bno->hi2cx, bno->address, ACC_DATA_X_LSB, 1, bno->DataBuffer->u8, 32);
	}
}

void BNO055_Calibrated(BNO055_t *bno, BNO055_Calibration_Status *calib_stat, BNO055_Offsets *bno_off)
{
	uint8_t read_mode;
	HAL_I2C_Mem_Read(bno->hi2cx, bno->address, OPR_MODE, 1, &read_mode, 1, 10);
	if (read_mode != bno->mode) {
		HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &bno->mode, 1, 10);
	}

	uint8_t txbuffer;
	uint8_t rxbuffer[22];

	HAL_StatusTypeDef done = HAL_BUSY;

	txbuffer = Page_ID_00;
	do {
		HAL_I2C_Mem_Write(bno->hi2cx, bno->address, PAGE_ID, 1, &txbuffer, 1, 10);

		HAL_Delay(10);
		HAL_I2C_Mem_Read(bno->hi2cx, bno->address, CALIB_STAT, 1, &rxbuffer[0], 1, 10);

		calib_stat->gyro_stat = (((rxbuffer[0] >> 4) & 0x03) == 3) ? HAL_OK : HAL_BUSY;
		calib_stat->accel_stat = (((rxbuffer[0] >> 2) & 0x03) == 3) ? HAL_OK : HAL_BUSY;
		calib_stat->mag_stat = (((rxbuffer[0]) & 0x03) == 3) ? HAL_OK : HAL_BUSY;

		switch(bno->mode){
		case ACCONLY:
				if(calib_stat->accel_stat == HAL_OK) done = HAL_OK;
				break;
			case MAGONLY:
				if(calib_stat->mag_stat == HAL_OK) done = HAL_OK;
				break;
			case GYROONLY:
				if(calib_stat->gyro_stat == HAL_OK) done = HAL_OK;
				break;
			case ACCMAG:
				if(calib_stat->accel_stat == HAL_OK && calib_stat->mag_stat == HAL_OK) done = HAL_OK;
				break;
			case ACCGYRO:
				if(calib_stat->accel_stat == HAL_OK && calib_stat->gyro_stat == HAL_OK) done = HAL_OK;
				break;
			case MAGGYRO:
				if(calib_stat->mag_stat == HAL_OK && calib_stat->gyro_stat == HAL_OK) done = HAL_OK;
				break;
			case AMG:
				if(calib_stat->accel_stat == HAL_OK && calib_stat->mag_stat == HAL_OK && calib_stat->gyro_stat == HAL_OK) done = HAL_OK;
				break;
			case IMU:
				if(calib_stat->accel_stat == HAL_OK && calib_stat->gyro_stat == HAL_OK) done = HAL_OK;
				break;
			case COMPASS:
				if(calib_stat->accel_stat == HAL_OK && calib_stat->mag_stat == HAL_OK) done = HAL_OK;
				break;
			case M4G:
				if(calib_stat->accel_stat == HAL_OK && calib_stat->mag_stat == HAL_OK) done = HAL_OK;
				break;
			case NDOF_FMC_OFF:
				if(calib_stat->accel_stat == HAL_OK && calib_stat->mag_stat == HAL_OK && calib_stat->gyro_stat == HAL_OK) done = HAL_OK;
			case NDOF:
				if(calib_stat->accel_stat == HAL_OK && calib_stat->mag_stat == HAL_OK && calib_stat->gyro_stat == HAL_OK) done = HAL_OK;
				break;
		}
	} while (done != HAL_OK);

	HAL_I2C_Mem_Read(bno->hi2cx, bno->address, OPR_MODE, 1, &read_mode, 1, 10);
	if (read_mode != CONFIGMODE) {
		uint8_t txbuffer = CONFIGMODE;
		HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &txbuffer, 1, 10);
		HAL_Delay(20);
	}

	HAL_I2C_Mem_Read(bno->hi2cx, bno->address, ACC_OFFSET_X_LSB, 1, rxbuffer, 22, 10);

	for (int i = 0; i < 2; i++)
	{
		bno_off->accel_offset_x.u8[i] = rxbuffer[0 + i];
		bno_off->accel_offset_y.u8[i] = rxbuffer[2 + i];
		bno_off->accel_offset_z.u8[i] = rxbuffer[4 + i];

		bno_off->mag_offset_x.u8[i] = rxbuffer[6 + i];
		bno_off->mag_offset_y.u8[i] = rxbuffer[8 + i];
		bno_off->mag_offset_z.u8[i] = rxbuffer[10 + i];

		bno_off->gyro_offset_x.u8[i] = rxbuffer[12 + i];
		bno_off->gyro_offset_y.u8[i] = rxbuffer[14 + i];
		bno_off->gyro_offset_z.u8[i] = rxbuffer[16 + i];

		bno_off->accel_radius.u8[i] = rxbuffer[18 + i];

		bno_off->mag_radius.u8[i] =rxbuffer[20 + i];
	}

	HAL_I2C_Mem_Read(bno->hi2cx, bno->address, OPR_MODE, 1, &read_mode, 1, 10);
	if (read_mode != bno->mode) {
		HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &bno->mode, 1, 10);
		HAL_Delay(20);
	}
}

void BNO055_SetOffsets(BNO055_t *bno, BNO055_Offsets *bno_offset)
{
	uint8_t read_mode;
	HAL_I2C_Mem_Read(bno->hi2cx, bno->address, OPR_MODE, 1, &read_mode, 1, 10);
	if (read_mode != CONFIGMODE) {
		uint8_t txbuffer = CONFIGMODE;
		HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &txbuffer, 1, 10);
		HAL_Delay(20);
	}

	uint8_t txbuffer[22];
	txbuffer[0] = bno_offset->accel_offset_x.u8[0];
	txbuffer[1] = bno_offset->accel_offset_x.u8[1];
	txbuffer[2] = bno_offset->accel_offset_y.u8[0];
	txbuffer[3] = bno_offset->accel_offset_y.u8[1];
	txbuffer[4] = bno_offset->accel_offset_z.u8[0];
	txbuffer[5] = bno_offset->accel_offset_z.u8[1];
	txbuffer[6] = bno_offset->mag_offset_x.u8[0];
	txbuffer[7] = bno_offset->mag_offset_x.u8[1];
	txbuffer[8] = bno_offset->mag_offset_y.u8[0];
	txbuffer[9] = bno_offset->mag_offset_y.u8[1];
	txbuffer[10] = bno_offset->mag_offset_z.u8[0];
	txbuffer[11] = bno_offset->mag_offset_z.u8[1];
	txbuffer[12] = bno_offset->gyro_offset_x.u8[0];
	txbuffer[13] = bno_offset->gyro_offset_x.u8[1];
	txbuffer[14] = bno_offset->gyro_offset_y.u8[0];
	txbuffer[15] = bno_offset->gyro_offset_y.u8[1];
	txbuffer[16] = bno_offset->gyro_offset_z.u8[0];
	txbuffer[17] = bno_offset->gyro_offset_z.u8[1];
	txbuffer[18] = bno_offset->accel_radius.u8[0];
	txbuffer[19] = bno_offset->accel_radius.u8[1];
	txbuffer[20] = bno_offset->mag_radius.u8[0];
	txbuffer[21] = bno_offset->mag_radius.u8[1];

	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, ACC_OFFSET_X_LSB, 1, txbuffer, 22, 10);

	HAL_I2C_Mem_Read(bno->hi2cx, bno->address, OPR_MODE, 1, &read_mode, 1, 10);
	if (read_mode != bno->mode) {
		HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &bno->mode, 1, 10);
		HAL_Delay(20);
	}
}

void BNO055_SetAxis(BNO055_t *bno, Remap_Config config, Remap_Sign sign)
{
	uint8_t read_mode;
	HAL_I2C_Mem_Read(bno->hi2cx, bno->address, OPR_MODE, 1, &read_mode, 1, 10);
	if (read_mode != CONFIGMODE) {
		uint8_t txbuffer = CONFIGMODE;
		HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &txbuffer, 1, 10);
		HAL_Delay(20);
	}

	uint8_t txbuffer;

	txbuffer = config;
	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, AXIS_MAP_CONFIG, 1, &txbuffer, 1, 10);
	HAL_Delay(20);

	txbuffer = sign;
	HAL_I2C_Mem_Write(bno->hi2cx, bno->address, AXIS_MAP_SIGN, 1, &txbuffer, 1, 10);
	HAL_Delay(20);

	HAL_I2C_Mem_Read(bno->hi2cx, bno->address, OPR_MODE, 1, &read_mode, 1, 10);
	if (read_mode != bno->mode) {
		HAL_I2C_Mem_Write(bno->hi2cx, bno->address, OPR_MODE, 1, &bno->mode, 1, 10);
		HAL_Delay(20);
	}
}

