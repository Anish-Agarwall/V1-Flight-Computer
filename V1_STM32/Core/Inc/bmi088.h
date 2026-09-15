/*
 * bmi088.h
 *
 *  Created on: Jan 25, 2026
 *      Author: gamin
 */

#ifndef INC_BMI088_H_
#define INC_BMI088_H_

#include "stm32f4xx_hal.h"

// i2c addys
//#define BMI088_A_ADDRESS (0x19 << 1)
//#define BMI088_G_ADDRESS (0x69 << 1)

// registers
#define BMI088_ACC_ADDRESS          0x19
#define BMI088_ACC_ALT_ADDRESS      0x18 // WE USE ALT ADDRESS

#define BMI088_ACC_CHIP_ID          0x00 // default value 0x1E
#define BMI088_ACC_ERR_REG          0x02
#define BMI088_ACC_STATUS           0x03

#define BMI088_ACC_X_LSB            0x12
#define BMI088_ACC_X_MSB            0x13
#define BMI088_ACC_Y_LSB            0x14
#define BMI088_ACC_Y_MSB            0x15
#define BMI088_ACC_Z_LSB            0x16
#define BMI088_ACC_Z_MSB            0x17

#define BMI088_ACC_SENSOR_TIME_0    0x18
#define BMI088_ACC_SENSOR_TIME_1    0x19
#define BMI088_ACC_SENSOR_TIME_2    0x1A

#define BMI088_ACC_INT_STAT_1       0x1D

#define BMI088_ACC_TEMP_MSB         0x22
#define BMI088_ACC_TEMP_LSB         0x23

#define BMI088_ACC_CONF             0x40
#define BMI088_ACC_RANGE            0x41

#define BMI088_ACC_INT1_IO_CTRL     0x53
#define BMI088_ACC_INT2_IO_CTRL     0x54
#define BMI088_ACC_INT_MAP_DATA     0x58

#define BMI088_ACC_SELF_TEST        0x6D

#define BMI088_ACC_PWR_CONF         0x7C
#define BMI088_ACC_PWR_CTRl         0x7D

#define BMI088_ACC_SOFT_RESET       0x7E

#define BMI088_GYRO_ADDRESS         0x69
#define BMI088_GYRO_ALT_ADDRESS     0x68 // WE USE ALT ADDRESS


#define BMI088_GYRO_CHIP_ID             0x00 // default value 0x0F

#define BMI088_GYRO_RATE_X_LSB          0x02
#define BMI088_GYRO_RATE_X_MSB          0x03
#define BMI088_GYRO_RATE_Y_LSB          0x04
#define BMI088_GYRO_RATE_Y_MSB          0x05
#define BMI088_GYRO_RATE_Z_LSB          0x06
#define BMI088_GYRO_RATE_Z_MSB          0x07

#define BMI088_GYRO_INT_STAT_1          0x0A

#define BMI088_GYRO_RANGE               0x0F
#define BMI088_GYRO_BAND_WIDTH          0x10

#define BMI088_GYRO_LPM_1               0x11

#define BMI088_GYRO_SOFT_RESET          0x14

#define BMI088_GYRO_INT_CTRL            0x15
#define BMI088_GYRO_INT3_INT4_IO_CONF   0x16
#define BMI088_GYRO_INT3_INT4_IO_MAP    0x18

#define BMI088_GYRO_SELF_TEST           0x3C


typedef struct{

	float acc_x, acc_y, acc_z;
	float gyro_x, gyro_y, gyro_z;
	float temp;

} BMI088_Data_t;

// functions
HAL_StatusTypeDef BMI088_Init(I2C_HandleTypeDef *hi2c);
void BMI088_ReadData(I2C_HandleTypeDef *hi2c, BMI088_Data_t *data);
HAL_StatusTypeDef BMI088_Verify_Config(I2C_HandleTypeDef *hi2c);


#endif /* INC_BMI088_H_ */
