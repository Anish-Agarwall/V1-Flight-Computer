/*
 * w25q128jvs.h
 *
 *  Created on: Feb 1, 2026
 *      Author: gamin
 */

#ifndef W25Q128JVS_H_
#define W25Q128JVS_H_

#include "stm32f4xx_hal.h"
#include "main.h"


// pins
#define FLASH_CS_Pin          GPIO_PIN_12
#define FLASH_CS_GPIO_Port    GPIOB


//instructions

//write enables
#define WRITE_ENABLE 0x06
#define VOLATILE_SR_WRITE_ENABLE 0x50
#define WRITE_DISABLE 0x04

//ID instructions
#define RELEASE_POWER_DOWN_ID 0xAB
#define MANUFACTURER_DEVICE_ID 0x90
#define JEDEC_ID 0x9F
#define READ_UNIQUE_ID 0x4B

//Status Register Reads
#define READ_STATUS_REGISTER_1 0x05
#define READ_STATUS_REGISTER_2 0x35
#define READ_STATUS_REGISTER_3 0x15

//Status Register Writes
#define WRITE_STATUS_REGISTER_1 0x01
#define WRITE_STATUS_REGISTER_2 0x31
#define WRITE_STATUS_REGISTER_3 0x11

//data reads
#define READ_DATA 0x03
#define FAST_READ 0x0B

//page program
#define PAGE_PROGRAM 0x02

//data erase
#define SECTOR_ERASE_4KB 0x20
#define BLOCK_ERASE_32KB 0x52
#define BLOCK_ERASE_64KB 0xD8
#define CHIP_ERASE 0xC7

//Security Register Instructions
#define READ_SFDP_REGISTER 0x5A
#define ERASE_SECURITY_REGISTER 0x44
#define PROGRAM_SECURITY_REGISTER 0x42
#define READ_SECURITY_REGISTER 0x48

//Block Lock Instructions
#define GLOBAL_BLOCK_LOCK 0x7E
#define GLOBAL_BLOCK_UNLOCK 0x98
#define READ_BLOCK_LOCK 0x3D
#define INDIVIDUAL_BLOCK_LOCK 0x36
#define INDIVIDUAL_BLOCK_UNLOCK 0x39


#define ERASE_PROGRAM_SUSPEND 0x75
#define ERASE_PROGRAM_RESUME 0x7A
#define POWER_DOWN 0xB9

#define ENABLE_RESET 0x66
#define RESET_DEVICE 0x99


#define DataTimeout 5;
#define PageTimeout 10;

//initializes flash IC
HAL_StatusTypeDef W25Q128JVS_Init(SPI_HandleTypeDef *hi2c);

//returns 1 if flash is busy, 0 otherwise
int8_t Flash_Busy(SPI_HandleTypeDef *hspi2);

/* Writes to flash after busy waiting using Flash_Busy() function
 * Parameters:
 * hspi2: HAL protocol
 * address: 24 bit address
 * flash_tx: values to be written into memory. 	[0] is reserved.
 * 											   	[1-3] are the address to be written to
 * 											   	[4] and after are the data to be written
 * size: number of bytes of data to be written (256 max)
 * Output: HAL_OK if write happened correctly, HAL_ERROR if it didn't
 *
 */
HAL_StatusTypeDef Flash_Write(SPI_HandleTypeDef *hspi2, uint32_t address, uint8_t *flash_tx, uint8_t size);




/* Reads from flash after busy waiting using Flash_Busy() function
 * Parameters:
 * hspi2: HAL protocol
 * address: 24 bit address of data
 * flash_rx: pointer to an array to be read from memory. 	[0] is reserved
 * 											   				[1-3] are the address to be written to
 * 											   				[4] is a reserved
 * 											   				[5] and after is space for data to be read
 * size: number of bytes of data (no max)
 * Output: HAL_OK if write happened correctly, HAL_ERROR if it didn't
 * Also, the rx pointer will have the data read once function has returned
 *
 */
HAL_StatusTypeDef Flash_Read(SPI_HandleTypeDef *hspi2, uint32_t address, uint8_t *rx, uint8_t size);


/* Erases all of memory
 * Parameters:
 * hspi2: HAL protocol
 * address: 24 bit address of data
 * Output: HAL_OK if command went through correctly, HAL_ERROR if not
 *
 */
HAL_StatusTypeDef Chip_Erase(SPI_HandleTypeDef *hspi2);


#endif /* W25Q128JVS_H_ */
