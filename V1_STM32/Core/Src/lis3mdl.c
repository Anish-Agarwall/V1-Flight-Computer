/*
 * lis3mdl.c
 *
 *  Created on: Jan 27, 2026
 *      Author: gmais
 */


#include "lis3mdl.h"

#define MAG_I2C_ADDR (LIS3MDL_ADDR << 1)

HAL_StatusTypeDef LIS3MDL_Init(I2C_HandleTypeDef *hi2c) {

	uint8_t data;

	//whoami check
	if (HAL_I2C_Mem_Read(hi2c, MAG_I2C_ADDR, WHO_AM_I, 1, &data, 1, 10) != HAL_OK){
			return HAL_ERROR;
	}
	if (data != 0x3D) {
		return HAL_ERROR;
	}

	//0b11010000: temp sensor on, high-performance mode for X and Y axis, 10 Hz output data rate, self-test off
	data = 0xD0;
	HAL_I2C_Mem_Write(hi2c, MAG_I2C_ADDR, CTRL_REG1, 1, &data, 1, 10);
	HAL_Delay(10);

	//highest precision (+/-4 gauss)
	data = 0x00;
	HAL_I2C_Mem_Write(hi2c, MAG_I2C_ADDR, CTRL_REG2, 1, &data, 1, 10);
	HAL_Delay(10);


	//0b00001000: high-perforamce mode for Z axis, little endian mode
	data = 0x08;
	HAL_I2C_Mem_Write(hi2c, MAG_I2C_ADDR, CTRL_REG4, 1, &data, 1, 10);
	HAL_Delay(10);

	//fast read enabled, continuous update mode (not block update)
	data = 0x40;
	HAL_I2C_Mem_Write(hi2c, MAG_I2C_ADDR, CTRL_REG5, 1, &data, 1, 10);
	HAL_Delay(10);

	data = 0x00; //continuous conversion mode
	HAL_I2C_Mem_Write(hi2c, MAG_I2C_ADDR, CTRL_REG3, 1, &data, 1, 10);
	HAL_Delay(10);

	//consider adding self-test check, changing offset regs, and changing performance modes

	return HAL_OK;

}

void LIS3MDL_ReadData(I2C_HandleTypeDef *hi2c, LIS3MDL_Data_t *data) {

	uint8_t mag_raw[6];
	uint8_t temp_raw[2];
	HAL_StatusTypeDef status;

	//consider checking status register before reading values

	uint8_t status_reg = 0;

	// 1. Ask the sensor if new data is ready (STATUS_REG = 0x27)
	if (HAL_I2C_Mem_Read(hi2c, MAG_I2C_ADDR, 0x27, 1, &status_reg, 1, 10) != HAL_OK) {
		// If the bus is broken, print an obvious error value so you know it crashed
		data->mag_x = 999.0f;
		return;
	}

	if ((status_reg & 0x08) == 0x08) {

		status = HAL_I2C_Mem_Read(hi2c, MAG_I2C_ADDR, (OUT_X_L | 0x80), 1, mag_raw, 6, 20);

		if (status == HAL_OK) {
			int16_t raw_mx = (int16_t)(mag_raw[1] << 8 | mag_raw[0]);
			int16_t raw_my = (int16_t)(mag_raw[3] << 8 | mag_raw[2]);
			int16_t raw_mz = (int16_t)(mag_raw[5] << 8 | mag_raw[4]);

			data->mag_x = raw_mx / 6842.0f;
			data->mag_y = raw_my / 6842.0f;
			data->mag_z = raw_mz / 6842.0f;
		}

		status = HAL_I2C_Mem_Read(hi2c, MAG_I2C_ADDR, (TEMP_OUT_L | 0x80), 1, temp_raw, 2, 10);

		if (status == HAL_OK) {
			int16_t raw_t = (int16_t)(temp_raw[1] << 8 | temp_raw[0]);

			data->temp = 25.0f + (raw_t / 8.0f);
		}
	}


}
