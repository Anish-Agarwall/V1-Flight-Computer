/*
 * bmi088.c
 *
 *  Created on: Jan 25, 2026
 *      Author: gamin
 */


#include "bmi088.h"

#define ACC_I2C_ADDR  (BMI088_ACC_ALT_ADDRESS << 1)
#define GYRO_I2C_ADDR (BMI088_GYRO_ALT_ADDRESS << 1)

#define ACC_SCALE_G  (24.0f * 1.5f / 32768.0f)   // 0.001099 g/LSB

HAL_StatusTypeDef BMI088_Init(I2C_HandleTypeDef *hi2c){

	uint8_t data;

	// whoami check
	if (HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDR, BMI088_ACC_CHIP_ID , 1, &data, 1, 10) != HAL_OK){
		return HAL_ERROR;
	}
	if (data != 0x1E){
		return HAL_ERROR;
	}

	if (HAL_I2C_Mem_Read(hi2c, GYRO_I2C_ADDR, BMI088_GYRO_CHIP_ID, 1, &data, 1, 10) != HAL_OK) {
		return HAL_ERROR;
	}
	if (data != 0x0F) {
		return HAL_ERROR;
	}


	// wake up
	data = 0x00;
	HAL_I2C_Mem_Write(hi2c, ACC_I2C_ADDR, BMI088_ACC_PWR_CONF, 1, &data, 1, 10);	// active mode
	HAL_Delay(1);

	data = 0x04;
	HAL_I2C_Mem_Write(hi2c, ACC_I2C_ADDR, BMI088_ACC_PWR_CTRl, 1, &data, 1, 10);	// acc on
	HAL_Delay(1);


	// set range
	data = 0x03;
	HAL_I2C_Mem_Write(hi2c, ACC_I2C_ADDR, BMI088_ACC_RANGE, 1, &data, 1, 10);		// 24G Range
	HAL_Delay(10);

	data = 0x00;
	HAL_I2C_Mem_Write(hi2c, GYRO_I2C_ADDR, BMI088_GYRO_RANGE, 1, &data, 1, 10);		// 2000 degrees per second
	HAL_Delay(10);


	// odr config
	// acc odr (400Hz)
	/*
		0x08 = 100Hz
		0x09 = 200Hz
		0x0A = 400Hz
	*/
	data = 0xAA; // was 0xA
	HAL_I2C_Mem_Write(hi2c, ACC_I2C_ADDR, BMI088_ACC_CONF, 1, &data, 1, 10);

	// gyro odr (47Hz)
	data = 0x03;
	HAL_I2C_Mem_Write(hi2c, GYRO_I2C_ADDR, BMI088_GYRO_BAND_WIDTH, 1, &data, 1, 10);


	return HAL_OK;
}

HAL_StatusTypeDef BMI088_Verify_Config(I2C_HandleTypeDef *hi2c) {
    uint8_t val;

    // check ACC_RANGE (expect 0x03)
    if (HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDR, BMI088_ACC_RANGE, 1, &val, 1, 10) != HAL_OK) {
    	return HAL_ERROR;
    }
    if (val != 0x03) {
    	return HAL_ERROR;
    }

    // check ACC_CONF (expect 0x0A)
    if (HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDR, BMI088_ACC_CONF, 1, &val, 1, 10) != HAL_OK) {
    	return HAL_ERROR;
    }
    if (val != 0x0A) {
    	return HAL_ERROR;
    }

    // check ACC_PWR_CONF (expect 0x00 = active mode)
    if (HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDR, BMI088_ACC_PWR_CONF, 1, &val, 1, 10) != HAL_OK) {
    	return HAL_ERROR;
    }
    if (val != 0x00) {
    	return HAL_ERROR;
    }

    // check ACC_PWR_CTRL (expect 0x04 = acc on)
    if (HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDR, BMI088_ACC_PWR_CTRl, 1, &val, 1, 10) != HAL_OK) {
    	return HAL_ERROR;
    }
    if (val != 0x04) {
    	return HAL_ERROR;
    }

    // check GYRO_RANGE (expect 0x00 = 2000 dps)
    if (HAL_I2C_Mem_Read(hi2c, GYRO_I2C_ADDR, BMI088_GYRO_RANGE, 1, &val, 1, 10) != HAL_OK) {
    	return HAL_ERROR;
    }
    if (val != 0x00) {
    	return HAL_ERROR;
    }

    // check GYRO_BANDWIDTH (expect 0x03)
    if (HAL_I2C_Mem_Read(hi2c, GYRO_I2C_ADDR, BMI088_GYRO_BAND_WIDTH, 1, &val, 1, 10) != HAL_OK) {
    	return HAL_ERROR;
    }
    if (val != 0x03) {
    	return HAL_ERROR;
    }

    return HAL_OK;
}

void BMI088_ReadData(I2C_HandleTypeDef *hi2c, BMI088_Data_t *data){
	uint8_t accel_raw[7];
	uint8_t gyro_raw[6];
	uint8_t temp_raw[2];

	// getting accel data (24G)
	if (HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDR, BMI088_ACC_X_LSB, 1, accel_raw, 7, 20) == HAL_OK){
		// combindidngf bytes
		int16_t raw_ax = (int16_t)(accel_raw[1] << 8 | accel_raw[0]);
		int16_t raw_ay = (int16_t)(accel_raw[3] << 8 | accel_raw[2]);
		int16_t raw_az = (int16_t)(accel_raw[5] << 8 | accel_raw[4]);

//		data->acc_x = raw_ax * (24.0f / 32768.0f);
//		data->acc_y = raw_ay * (24.0f / 32768.0f);
//		data->acc_z = raw_az * (24.0f / 32768.0f);

		data->acc_x = raw_ax * ACC_SCALE_G;
		data->acc_y = raw_ay * ACC_SCALE_G;
		data->acc_z = raw_az * ACC_SCALE_G;
	}

	// getting gyro data (2000 DPS)
	if (HAL_I2C_Mem_Read(hi2c, GYRO_I2C_ADDR, BMI088_GYRO_RATE_X_LSB, 1, gyro_raw, 6, 20) == HAL_OK){
		// combindidngf bytes
		int16_t raw_gx = (int16_t)(gyro_raw[1] << 8 | gyro_raw[0]);
		int16_t raw_gy = (int16_t)(gyro_raw[3] << 8 | gyro_raw[2]);
		int16_t raw_gz = (int16_t)(gyro_raw[5] << 8 | gyro_raw[4]);

		data->gyro_x = raw_gx / 16.384f;
		data->gyro_y = raw_gy / 16.384f;
		data->gyro_z = raw_gz / 16.384f;
	}

	// getting temp
	if (HAL_I2C_Mem_Read(hi2c, ACC_I2C_ADDR, BMI088_ACC_TEMP_MSB, 1, temp_raw, 2, 10) == HAL_OK){
		int16_t raw_temp = (int16_t)((temp_raw[0] << 3) | (temp_raw[1] >> 5));
		if (raw_temp > 1023){
			raw_temp -=2048;
		}
		data->temp = (float)raw_temp * 0.125f + 23.0f;
	}

	// its a void
	//return HAL_OK;

}



