/*
 * w25q128jvs.c
 *
 *  Created on: Feb 1, 2026
 *      Author: gamin
 */


#include "w25q128jvs.h"


// do more init stuff fr fr maybe?
HAL_StatusTypeDef W25Q128JVS_Init(SPI_HandleTypeDef *hspi2) {

	uint8_t flash_tx[6] = {JEDEC_ID, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t flash_rx[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	//whoami check
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET); 	// pull low
	if (HAL_SPI_TransmitReceive(hspi2, flash_tx, flash_rx, 4, 100) != HAL_OK) {		// data transfer
		return HAL_ERROR;
	}
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);		// pull high

	if (flash_rx[1] != 0xEF) {
		return HAL_ERROR;
	}
	//flash_rx[2] has memory type and flash_rx[3] has memory capacity

	flash_tx[0] = WRITE_ENABLE;
	flash_tx[1] = 0;
	flash_tx[2] = 0;
	flash_tx[3] = 0;

	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET); 	// pull low
	HAL_SPI_TransmitReceive(hspi2, flash_tx, flash_rx, 1, 100);			// data transfer
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);		// pull high

	return HAL_OK;

}

int8_t Flash_Busy(SPI_HandleTypeDef *hspi2) {
	uint8_t tx[2] = {READ_STATUS_REGISTER_1, 0x00};
	uint8_t rx[2] = {0x00, 0x00};

	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET); 	// pull low
	HAL_SPI_TransmitReceive(hspi2, tx, rx, 2, 5);							// data transfer
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);		// pull high

	return (rx[1] & 0x1);
}

HAL_StatusTypeDef Flash_Write(SPI_HandleTypeDef *hspi2, uint32_t address, uint8_t *tx, uint8_t size) {
	while (Flash_Busy(hspi2)) {}

	uint8_t we_cmd = WRITE_ENABLE;
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi2, &we_cmd, 1, 10);
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

	uint8_t flash_rx[size + 4];
	uint8_t flash_tx[size + 4];
	flash_tx[0] = PAGE_PROGRAM;


	flash_tx[1] = (address >> 16) & 0xFF;
	flash_tx[2] = (address >> 8)  & 0xFF;
	flash_tx[3] = (address)       & 0xFF;
	for (int i = 0; i < size; i++) {
		flash_tx[i + 4] = tx[i];
	}


	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET); 	// pull low
	HAL_StatusTypeDef retval = HAL_SPI_TransmitReceive(hspi2, flash_tx, flash_rx, size + 4, 10);			// data transfer
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);		// pull high


	return retval;

}

HAL_StatusTypeDef Flash_Read(SPI_HandleTypeDef *hspi2, uint32_t address, uint8_t *rx, uint8_t size) {
	while (Flash_Busy(hspi2)) {}

	uint8_t flash_rx[size + 5];
	uint8_t flash_tx[size + 5];
	flash_tx[0] = FAST_READ;
	flash_tx[1] = (address >> 16) & 0xFF;
	flash_tx[2] = (address >> 8)  & 0xFF;
	flash_tx[3] = (address)       & 0xFF;
	flash_tx[4] = 0x00; // dummy value

	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET); 	// pull low
	HAL_StatusTypeDef retval = HAL_SPI_TransmitReceive(hspi2, flash_tx, flash_rx, size + 5, 10);			// data transfer
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);		// pull high

	for (int i = 0; i < size; i++) {
		rx[i] = flash_rx[i + 5];
	}

	return retval;

}

HAL_StatusTypeDef Chip_Erase(SPI_HandleTypeDef *hspi2) {
	while (Flash_Busy(hspi2)) {}

	uint8_t flash_tx[1] = {CHIP_ERASE};
	uint8_t flash_rx[1] = {0x00};

	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET); 	// pull low
	HAL_StatusTypeDef retval = HAL_SPI_TransmitReceive(hspi2, flash_tx, flash_rx, 1, 5);			// data transfer
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);		// pull high


	while (Flash_Busy(hspi2)) {} // wait for chip erase to complete


	//reenable writes
	flash_tx[0] = WRITE_ENABLE;

	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET); 	// pull low
	HAL_SPI_TransmitReceive(hspi2, flash_tx, flash_rx, 1, 100);			// data transfer
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);		// pull high

	return retval;

}
