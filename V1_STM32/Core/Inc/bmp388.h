/*
 * bmp388.h
 *
 *  Created on: Jan 26, 2026
 *      Author: gamin
 */

#ifndef INC_BMP388_H_
#define INC_BMP388_H_


#include "stm32f4xx_hal.h"

#define BMP388_I2C_ADDR (0x76 << 1)

#define BMP388_CHIP_ID 0x00
#define BMP388_ERR_REG 0x02
#define BMP388_STATUS 0x03
#define BMP388_DATA_0 0x04
#define BMP388_DATA_1 0x05
#define BMP388_DATA_2 0x06
#define BMP388_DATA_3 0x07
#define BMP388_DATA_4 0x08
#define BMP388_DATA_5 0x09
#define BMP388_SENSORTIME_0 0x0C
#define BMP388_SENSORTIME_1 0x0D
#define BMP388_SENSORTIME_2 0x0E
#define BMP388_EVENT 0x10
#define BMP388_INT_STATUS 0x11
#define BMP388_FIFO_LENGTH_0 0x12
#define BMP388_FIFO_LENGTH_1 0x13
#define BMP388_FIFO_DATA 0x14
#define BMP388_FIFO_WTM_0 0x15
#define BMP388_FIFO_WTM_1 0x16
#define BMP388_FIFO_CONFIG_1 0x17
#define BMP388_FIFO_CONFIG_2 0x18
#define BMP388_INT_CTRL 0x19
#define BMP388_IF_CONF 0x1A
#define BMP388_PWR_CTRL 0x1B
#define BMP388_OSR 0x1C
#define BMP388_ODR 0x1D
#define BMP388_CONFIG 0x1F
#define BMP388_CMD 0x7E

#define BMP388_REG_CALIB_0   0x31

typedef struct{
	uint16_t T1, T2;
	int8_t T3;
	int16_t P1, P2;
	int8_t P3, P4;
	uint16_t P5, P6;
	int8_t P7, P8;
	int16_t P9;
	int8_t P10, P11;

	float temp;
	float pressure;
	float altitude;
} BMP388_Calib_t;

HAL_StatusTypeDef BMPP388_Init(I2C_HandleTypeDef *hi2c, BMP388_Calib_t *calib);
void BMP388_ReadData(I2C_HandleTypeDef *hi2c, BMP388_Calib_t *calib);





#endif /* INC_BMP388_H_ */
