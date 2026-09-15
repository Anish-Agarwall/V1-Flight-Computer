/*
 * 1040310811.h
 *
 *  Created on: Feb 10, 2026
 *      Author: gomrin
 */

// https://academy.cba.mit.edu/classes/networking_communications/SD/SD.pdf     datasheet

#ifndef INC_1040310811_H_
#define INC_1040310811_H_

#include "stm32f4xx_hal.h"
#include "main.h"


// pins
#define SD_CS_Pin          GPIO_PIN_9
#define SD_CS_GPIO_Port    GPIOC





//commands

#define GO_IDLE_STATE 0 // reset SD Memory Card
#define SEND_OP_COND 1 // Sends Host Capacity Support info and activate's card initialization process

//CMD2 - CMD4 are not for SPI mode
//CMD5 is reserved for I/O mode

#define SWITCH_FUNC 6 // checks switchable function or switches card function

//CMD7 is not for SPI mode

#define SEND_IF_COND 8 // sends SD card interface condition (host supply voltage info) and asks card if it can operate in the supplied voltage range
#define SEND_CSD 9 // asks card to send its Card Specific Data (CSD)
#define SEND_CID 10 // asks card to send its Card Identification (CID)

//CMD11 is not for SPI mode

#define STOP_TRANSMISSION 12 // forces the card to stop transmission in Multiple Block Read Operation
#define SEND_STATUS 13 // asks the card to send its status register

//CMD14 is reserved
//CMD15 is not for SPI mode

// If SDSC card then block length is determined by this.
//If SDHC or SDXC card, block length of memory accesses is fixed to 512 bytes
//The length of LOCK_UNLOCK command is set by this command regardless of card capacity
#define SET_BLOCKLEN 16 // ^
#define READ_SINGLE_BLOCK 17 // reads a block of the size selected by SET_BLOCKLEN
#define READ_MULTIPLE_BLOCK 18 // continuously transfers data blocks from card until interrupted by STOP_TRANSMISSION

//CMD19 is reserved

//CMD20 is not for SPI mode

//CMD21 - CMD23 are reserved

#define WRITE_BLOCK 24 // writes a block of the size selected by SET_BLOCKLEN
#define WRITE_MULTIPLE_BLOCK 25 // continuously writes blocks of data until "Stop Tran" token is sent (instead of "Start Block")

//CMD26 is not for SPI mode

#define PROGRAM_CSD 27 // programs the programmable CSD bits

// write protection not available for SDHC and SDXC cards
#define SET_WRITE_PROT 28 // sets the write protection bit of the addressed group
#define CLR_WRITE_PROT 29 // clears the write protection bit of the addressed group
#define SEND_WRITE_PROT 30 // asks the card to send the status of the write protection bits

//CMD31 reserved

#define ERASE_WR_BLK_START_ADDR 32 // sets the address of the first write block to be erased
#define ERASE_WR_BLK_END_ADDR 33 // sets the address of the last write block to be erased

//CMD34 - CMD37 reserved for each command system, set by CMD6

#define ERASE 38 // erases all previously selected write blocks

//CMD39 and CMD40 are not for SPI mode
//CMD41 is reserved

#define LOCK_UNLOCK 42 // used to set/reset password or lock/unlock the card

//43 - 49, 51 reserved

//50 reserved for each command system, set by CMD6

//52 - 54 reserved for I/O mode

#define APP_CMD 55 // defines the next command as an application specific command rather than a standard command
#define GEN_CMD 56 // transfers data for general purpose / application specific commands

//57 reserved for each command system, set by CMD6

#define READ_OCR 58 // reads OCR register. CCS bit is assigned to OCR[30]
#define CRC_ON_OFF 59 // turns the CRC option on or off

//60 - 63 reserved for the manufacturer


uint8_t crc7(uint8_t *data, uint8_t len);
uint8_t SD_Send_Command(SPI_HandleTypeDef *hspi3, uint8_t command, uint32_t arg);
HAL_StatusTypeDef SD_Card_Init(SPI_HandleTypeDef *hspi);

//HAL_StatusTypeDef SD_Read_Data(SPI_HandleTypeDef *hspi3, uint32_t address, uint8_t *buffer);
//HAL_StatusTypeDef SD_Write_Data(SPI_HandleTypeDef *hspi3, uint32_t address, uint8_t *buffer);

//
HAL_StatusTypeDef SD_Read_Data(SPI_HandleTypeDef *hspi3, uint32_t address, uint8_t *buffer, uint32_t count);
//HAL_StatusTypeDef SD_Write_Data(SPI_HandleTypeDef *hspi, uint32_t sector, const uint8_t *buffer, uint32_t count);
HAL_StatusTypeDef SD_Write_Data(SPI_HandleTypeDef *hspi3, uint32_t address, const uint8_t *buffer, uint32_t count);


#endif /* INC_1040310811_H_ */
