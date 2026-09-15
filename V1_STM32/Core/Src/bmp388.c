/*
 * bmp388.c
 *
 *  Created on: Jan 26, 2026
 *      Author: gamin
 */


#include "bmp388.h"
#include <math.h>


static void read_coeffs(I2C_HandleTypeDef *hi2c, BMP388_Calib_t *calib){
	uint8_t reg_data[21];
	HAL_I2C_Mem_Read(hi2c, BMP388_I2C_ADDR, BMP388_REG_CALIB_0, 1, reg_data, 21, 100);

	calib->T1  = (reg_data[1] << 8) | reg_data[0];
	calib->T2  = (reg_data[3] << 8) | reg_data[2];
	calib->T3  = (int8_t)reg_data[4];
	calib->P1  = (int16_t)((reg_data[6] << 8) | reg_data[5]);
	calib->P2  = (int16_t)((reg_data[8] << 8) | reg_data[7]);
	calib->P3  = (int8_t)reg_data[9];
	calib->P4  = (int8_t)reg_data[10];
	calib->P5  = (reg_data[12] << 8) | reg_data[11];
	calib->P6  = (reg_data[14] << 8) | reg_data[13];
	calib->P7  = (int8_t)reg_data[15];
	calib->P8  = (int8_t)reg_data[16];
	calib->P9  = (int16_t)((reg_data[18] << 8) | reg_data[17]);
	calib->P10 = (int8_t)reg_data[19];
	calib->P11 = (int8_t)reg_data[20];
}


HAL_StatusTypeDef BMPP388_Init(I2C_HandleTypeDef *hi2c, BMP388_Calib_t *calib){
	uint8_t data = 0x00;

	HAL_Delay(20);

	// id check lol
	HAL_I2C_Mem_Read(hi2c, BMP388_I2C_ADDR, BMP388_CHIP_ID, 1, &data, 1, 10);
	if (data != 0x50){
		return HAL_ERROR;
	}

	// read calibs
	read_coeffs(hi2c, calib);

	// high res over sample
	/*
	 pressure is x8
	 temp is x1
	*/

	data = 0x03;
	HAL_I2C_Mem_Write(hi2c, BMP388_I2C_ADDR, BMP388_OSR, 1, &data, 1, 10);

	// odr (50Hz)
	data = 0x02;
	HAL_I2C_Mem_Write(hi2c, BMP388_I2C_ADDR, BMP388_ODR, 1, &data, 1, 10);

	// pwr contrls
	data = 0x33;
	HAL_I2C_Mem_Write(hi2c, BMP388_I2C_ADDR, BMP388_PWR_CTRL, 1, &data, 1, 10);


	return HAL_OK;
}

// comp math
static float comp_temp(BMP388_Calib_t *c, uint32_t uncomp_temp){

	/*
	float partial_d1 = (float)(uncomp_temp - c->T1);
	float partial_d2 = (float)(partial_d1 * c->T2);
	return partial_d2 + (partial_d1 * partial_d1) * c->T3;
	*/


	float par_t1 = (float)c->T1 * 256.0f;
	float par_t2 = (float)c->T2 / 1073741824.0f;
	float par_t3 = (float)c->T3 / 281474976710656.0f;

	float partial_d1 = (float)uncomp_temp - par_t1;
	float partial_d2 = partial_d1 * par_t2;

	// c degrees
	return partial_d2 + (partial_d1 * partial_d1) * par_t3;
}

static float comp_press(BMP388_Calib_t *c, uint32_t uncomp_press, float t_lin){

	/*
	float partial_d1, partial_d2, partial_d3, partial_d4, out1, out2;

	partial_d1 = (float)c->P6 * t_lin;
	partial_d2 = (float)c->P7 * powf(t_lin, 2);
	partial_d3 = (float)c->P8 * powf(t_lin, 3);
	out1 = (float)c->P5 + partial_d1 + partial_d2 + partial_d3;

	partial_d1 = (float)c->P2 * t_lin;
	partial_d2 = (float)c->P3 * powf(t_lin, 2);
	partial_d3 = (float)c->P4 * powf(t_lin, 3);
	out2 = (float)c->P5 + partial_d1 + partial_d2 + partial_d3;


	partial_d1 = powf((float)uncomp_press, 2);
	partial_d2 = (float)c->P9 + (float)c->P10 * t_lin;
	partial_d3 = partial_d1 * partial_d2;
	partial_d4 = partial_d3 + powf((float)uncomp_press, 3) * (float)c->P11;

	return out1 + out2 + partial_d4;

	*/


	float par_p1  = ((float)c->P1  - 16384.0f) / 1048576.0f;
	float par_p2  = ((float)c->P2  - 16384.0f) / 536870912.0f;
	float par_p3  = (float)c->P3  / 4294967296.0f;
	float par_p4  = (float)c->P4  / 137438953472.0f;
	float par_p5  = (float)c->P5  * 8.0f;
	float par_p6  = (float)c->P6  / 64.0f;
	float par_p7  = (float)c->P7  / 256.0f;
	float par_p8  = (float)c->P8  / 32768.0f;
	float par_p9  = (float)c->P9  / 281474976710656.0f;
	float par_p10 = (float)c->P10 / 281474976710656.0f;
	float par_p11 = (float)c->P11 / 36893488147419103232.0f;

	float partial_d1;
	float partial_d2;
	float partial_d3;
	float partial_d4;
	float partial_out1;
	float partial_out2;

	partial_d1 = par_p6 * t_lin;
	partial_d2 = par_p7 * (t_lin * t_lin);
	partial_d3 = par_p8 * (t_lin * t_lin * t_lin);
	partial_out1 = par_p5 + partial_d1 + partial_d2 + partial_d3;


	partial_d1 = par_p2 * t_lin;
	partial_d2 = par_p3 * (t_lin * t_lin);
	partial_d3 = par_p4 * (t_lin * t_lin * t_lin);
	partial_out2 = (float)uncomp_press * (par_p1 + partial_d1 + partial_d2 + partial_d3);

	partial_d1 = (float)uncomp_press * (float)uncomp_press;
	partial_d2 = par_p9 + par_p10 * t_lin;
	partial_d3 = partial_d1 * partial_d2;
	partial_d4 = partial_d3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * par_p11;

	return partial_out1 + partial_out2 + partial_d4;
}

void BMP388_ReadData(I2C_HandleTypeDef *hi2c, BMP388_Calib_t *calib){
	uint8_t raw[6]; //realized i didnt need all of the indiviusdla rsg in header fuckkkkkkkkkkkkkkkkkkkkkkkkkk
	HAL_I2C_Mem_Read(hi2c, BMP388_I2C_ADDR, BMP388_DATA_0, 1, raw, 6, 20);

	uint32_t raw_press = (uint32_t)raw[2] << 16 | (uint32_t)raw[1] << 8 | raw[0];
	uint32_t raw_temp = (uint32_t)raw[5] << 16 | (uint32_t)raw[4] << 8 | raw[3];

	float t_lin = comp_temp(calib, raw_temp);

	//celccusis
	//calib->temp = t_lin / 5120.0f;
	calib->temp = t_lin;

	// pressur
	calib->pressure = comp_press(calib, raw_press, t_lin);

	// altitude
	calib->altitude = 44330.0f * (1.0f - powf(calib->pressure / 101325.0f, 0.190259f));
}



