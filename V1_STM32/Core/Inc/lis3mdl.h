/*
 * lis3mdl.h
 *
 *  Created on: Jan 27, 2026
 *      Author: gsmin
 */

#ifndef INC_LIS3MDL_H_
#define INC_LIS3MDL_H_


#include "stm32f4xx_hal.h"

//registers

//0x00 - 0x04 reserved

//Hard-iron registers
#define OFFSET_X_REG_L_M 0x05
#define OFFSET_X_REG_H_M 0x06
#define OFFSET_Y_REG_L_M 0x07
#define OFFSET_Y_REG_H_M 0x08
#define OFFSET_Z_REG_L_M 0x09
#define OFFSET_Z_REG_H_M 0x0A

//0x0B - 0x0E reserved

//Dummy register
#define WHO_AM_I 0x0F // default value 0x3D

//0x10 - 0x1F reserved

//I2C addresses
#define LIS3MDL_ADDR 0x1C
#define LIS3MDL_ADDR_ALT 0x1E


#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

//0x25 - 0x26 reserved

#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define TEMP_OUT_L 0x2E
#define TEMP_OUT_H 0x2F
#define INT_CFG 0x30
#define INT_SRC 0x31
#define INT_THS_L 0x32
#define INT_THS_H 0x33


typedef struct {

	float mag_x, mag_y, mag_z;
	float temp;

} LIS3MDL_Data_t;


//functions
HAL_StatusTypeDef LIS3MDL_Init(I2C_HandleTypeDef *hi2c);
void LIS3MDL_ReadData(I2C_HandleTypeDef *hi2c, LIS3MDL_Data_t *data);


#endif /* INC_LIS3MDL_H_ */
