/*
 * E22_900MM22S.h
 *
 *  Created on: Mar 21, 2026
 *      Author: 17372
 */

#ifndef INC_E22_900MM22S_H_
#define INC_E22_900MM22S_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "Packet.h"


// Pins
#define E22_RXEN          GPIO_PIN_3
#define E22_TXEN          GPIO_PIN_2
#define E22_Telem_CS	  GPIO_PIN_0
#define E22_Telem_BUSY	  GPIO_PIN_1
#define E22_GPIO_Port     GPIOC



// Operational Mode Commands
#define SetSleep 0x84
#define SetStandby 0x80
#define SetTx 0x83
#define SetRx 0x82
#define Calibrate 0x89
#define SetPaConfig 0x95

// Register and Buffer Access Commands
#define WriteRegister 0x0D
#define ReadRegister 0x1D
#define WriteBuffer 0x0E
#define ReadBuffer 0x1E

// RF, Modulation, and Packet Commands
#define SetRfFrequency 0x86
#define SetPacketType 0x8A
#define GetPacketType 0x11
#define SetTxParams 0x8E
#define SetPacketParams 0x8C
#define SetModulationParams 0x8B

// Status Commands
#define GetStatus 0xC0
#define GetStats 0x10
#define ResetStats 0x00
#define GetDeviceErrors 0x14
#define ClearDeviceErrors 0x07
#define GetPacketStatus 0x14


// SX1262 IRQ Command Opcodes
#define GetIrqStatus   0x12
#define ClearIrqStatus 0x02


// Functions

HAL_StatusTypeDef E22_900MM22S_Wait_Busy(void);
HAL_StatusTypeDef E22_900MM22S_Send_Command(SPI_HandleTypeDef *hspi1, uint8_t command, uint8_t *params, uint16_t numParams);
HAL_StatusTypeDef E22_900MM22S_Sleep(SPI_HandleTypeDef *hspi1);
HAL_StatusTypeDef E22_900MM22S_Init_900MHz(SPI_HandleTypeDef *hspi1);
HAL_StatusTypeDef E22_900MM22S_Transmit(SPI_HandleTypeDef *hspi1, uint8_t *buffer, uint8_t length);








#endif /* INC_E22_900MM22S_H_ */
