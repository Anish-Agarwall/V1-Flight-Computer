/*
 * bmi330.c
 *
 *  Created on: Mar 3, 2026
 *      Author: gamin
 */

#include "bmi330.h"

#define IMU_I2C_ADDR  (BMI330_ADDRESS << 1)

HAL_StatusTypeDef BMI330_Init(I2C_HandleTypeDef *hi2c) {

    uint8_t data;

    // Soft reset
    data = 0xB6;
    HAL_I2C_Mem_Write(hi2c, IMU_I2C_ADDR, BMI330_CMD, 1, &data, 1, 10);

    // need to wait for reset to complete
    HAL_Delay(10);

    // WHOA M I check (expect 0x23)
    if (HAL_I2C_Mem_Read(hi2c, IMU_I2C_ADDR, BMI330_CHIP_ID_REG, 1, &data, 1, 10) != HAL_OK) {
        return HAL_ERROR;
    }
    if (data != 0x23) {
        return HAL_ERROR;
    }


    // acc_odr  = 0x0A (400Hz)
    // acc_range = 0x03 (±16g)
    // acc_mode = 0 (normal, bit7 = 0)
    // reg value: (0x03 << 4) | 0x0A = 0x3A
    data = 0x3A;
    if (HAL_I2C_Mem_Write(hi2c, IMU_I2C_ADDR, BMI330_ACC_CONF, 1, &data, 1, 10) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(5);


    // gyr_odr  = 0x09 (200Hz)
    // gyr_range = 0x00 (±2000 dps)
    // gyr_mode = 0 (normal, bit7 = 0)
    // reg value: (0x00 << 4) | 0x09 = 0x09
    data = 0x09;
    if (HAL_I2C_Mem_Write(hi2c, IMU_I2C_ADDR, BMI330_GYR_CONF, 1, &data, 1, 10) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(5);

    return HAL_OK;
}

void BMI330_ReadData(I2C_HandleTypeDef *hi2c, BMI330_Data_t *data) {

	// acc XYZ (6 bytes) + gyro XYZ (6 bytes), contiguous from 0x03
    uint8_t raw[12];

    // read accel + gyro in one burst (0x03 to 0x0E)
    if (HAL_I2C_Mem_Read(hi2c, IMU_I2C_ADDR, BMI330_ACC_X_LSB, 1, raw, 12, 20) == HAL_OK) {

        // accelerometer data
    	// +/- 16g, 16-bit signed
        int16_t raw_ax = (int16_t)(raw[1]  << 8 | raw[0]);
        int16_t raw_ay = (int16_t)(raw[3]  << 8 | raw[2]);
        int16_t raw_az = (int16_t)(raw[5]  << 8 | raw[4]);

        // sensitivity: 32768 LSB / 16g = 2048 LSB/g
        data->acc_x = raw_ax / 2048.0f;
        data->acc_y = raw_ay / 2048.0f;
        data->acc_z = raw_az / 2048.0f;

        // gyroscope
        // +/-2000 dps, 16-bit signed
        int16_t raw_gx = (int16_t)(raw[7]  << 8 | raw[6]);
        int16_t raw_gy = (int16_t)(raw[9]  << 8 | raw[8]);
        int16_t raw_gz = (int16_t)(raw[11] << 8 | raw[10]);

        // sensitivity: 32768 LSB / 2000 dps = 16.384 LSB/(deg/s)
        data->gyro_x = raw_gx / 16.384f;
        data->gyro_y = raw_gy / 16.384f;
        data->gyro_z = raw_gz / 16.384f;
    }

    // temperature
    // 0x0F to 0x10, 16-bit signed
    // formula: T (clelsiucs) = raw_temp / 512 + 23
    uint8_t temp_raw[2];
    if (HAL_I2C_Mem_Read(hi2c, IMU_I2C_ADDR, BMI330_TEMP_LSB, 1, temp_raw, 2, 10) == HAL_OK) {
        int16_t raw_temp = (int16_t)(temp_raw[1] << 8 | temp_raw[0]);
        data->temp = (raw_temp / 512.0f) + 23.0f;
    }
}
