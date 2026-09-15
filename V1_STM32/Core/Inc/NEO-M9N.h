/*
 * NEO-M9N.h
 *
 *  Created on: Feb 24, 2026
 *      Author: GAN
 */

#ifndef INC_NEO_M9N_H_
#define INC_NEO_M9N_H_

#include "stm32f4xx_hal.h"


// UART interface config
#define CFG_UART1_BAUDRATE 0x40520001 // U4

#define CFG_UART1_STOPBITS 0x20520002 // E1

#define CFG_UART1_DATABITS 0x20520003 // E1

#define CFG_UART1_PARITY 0x20520004 // E1

#define CFG_UART1_ENABLED 0x10520005 // L


// input protocol config
#define CFG_UART1INPROT_UBX 0x10730001 // L

#define CFG_UART1INPROT_NMEA  0x10730002 // L

// output protocol config
#define CFG_UART1OUTPROT_UBX 0x10740001 // L
#define CFG_UART1OUTPROT_NMEA 0x10740002 // L


HAL_StatusTypeDef GPS_Interrupt_Init(UART_HandleTypeDef *huart1);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart1);
int GPS_Rx_FIFO_Put(void);
uint8_t GPS_Rx_FIFO_Get(void);
void GPS_Process_Stream(void);
uint32_t Convert_To_Unix_Time(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec);
void Parse_Valid_UBX_Message(void);
HAL_StatusTypeDef Send_UBX_Command(UART_HandleTypeDef *huart, uint8_t msgClass, uint8_t msgId, const uint8_t *payload, uint16_t length);
void calculate_ubx_checksum(uint8_t *msg, uint16_t len, uint8_t *ck_a, uint8_t *ck_b);
void GPS_Configure_Output(UART_HandleTypeDef *huart1);
HAL_StatusTypeDef NEO_Init(UART_HandleTypeDef *huart1);

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

void GPS_Enable_TimePulse(UART_HandleTypeDef *huart);

typedef struct{

	float longitude;
	float latitude;
	float altitude; // msl
	float speed_m_s;
	float heading_deg;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;

	uint8_t siv;
	uint32_t unix_timestamp;

} NEO_M9N_Data_t;


#endif /* INC_NEO_M9N_H_ */
