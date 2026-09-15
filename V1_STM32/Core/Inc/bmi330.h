/*
 * bmi330.h
 *
 *  Created on: Mar 3, 2026
 *      Author: gamin
 */

#ifndef INC_BMI330_H_
#define INC_BMI330_H_

#include "stm32f4xx_hal.h"

// we grounded SDO
#define BMI330_ADDRESS              0x68


// chip ID
#define BMI330_CHIP_ID_REG          0x00  // expected value: 0x23

// status / error
#define BMI330_ERR_REG              0x01
#define BMI330_STATUS               0x02

// accel data registers
#define BMI330_ACC_X_LSB            0x03
#define BMI330_ACC_X_MSB            0x04
#define BMI330_ACC_Y_LSB            0x05
#define BMI330_ACC_Y_MSB            0x06
#define BMI330_ACC_Z_LSB            0x07
#define BMI330_ACC_Z_MSB            0x08

// gyro data registers
#define BMI330_GYR_X_LSB            0x09
#define BMI330_GYR_X_MSB            0x0A
#define BMI330_GYR_Y_LSB            0x0B
#define BMI330_GYR_Y_MSB            0x0C
#define BMI330_GYR_Z_LSB            0x0D
#define BMI330_GYR_Z_MSB            0x0E

// temperature
#define BMI330_TEMP_LSB             0x0F
#define BMI330_TEMP_MSB             0x10

// sensor time
#define BMI330_SENSOR_TIME_0        0x11
#define BMI330_SENSOR_TIME_1        0x12
#define BMI330_SENSOR_TIME_2        0x13

// config registers
#define BMI330_ACC_CONF             0x20
#define BMI330_GYR_CONF             0x21

// feature / interrupt config
#define BMI330_INT_MAP1             0x3A
#define BMI330_INT_MAP2             0x3B
#define BMI330_INT_MAP3             0x3C

// power / command
// soft reset, write 0xDEAF
#define BMI330_CMD                  0x7E

typedef struct {
    float acc_x, acc_y, acc_z;   	// g
    float gyro_x, gyro_y, gyro_z; 	// degrees/s
    float temp;                   	// celsius
} BMI330_Data_t;




HAL_StatusTypeDef BMI330_Init(I2C_HandleTypeDef *hi2c);
void BMI330_ReadData(I2C_HandleTypeDef *hi2c, BMI330_Data_t *data);

#endif /* INC_BMI330_H_ */
