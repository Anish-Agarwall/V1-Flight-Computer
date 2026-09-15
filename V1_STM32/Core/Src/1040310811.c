/*
 * 1040310811.c
 *
 *  Created on: Feb 10, 2026
 *      Author: gongin
 */

// SD card specification:
// https://academy.cba.mit.edu/classes/networking_communications/SD/SD.pdf

#include "1040310811.h"

uint8_t crc7(uint8_t *data, uint8_t len) {
	uint8_t crc = 0;						// remainder is initialized to 0
	for (uint8_t i = 0; i < len; i++) {		// loop through the bytes of data
		uint8_t d = data[i];
		for (uint8_t j = 0; j < 8; j++) {	// loop through each bit
			crc = crc << 1;					// shift the remainder by 1
			if ((d ^ crc) & 0x80) {			// check the leftmost bit
				crc ^= 0x09;				// XOR with the polynomial (x^7 is already accounted for)
			}
			d = d << 1;						// shift data for the next bit
		}
	}
	return (crc & 0x7F);						// mask the data to 7 bits
}

uint8_t SD_Send_Command(SPI_HandleTypeDef *hspi3, uint8_t command, uint32_t arg) {

	uint8_t sd_tx[6];
	uint8_t response = 0xFF;
	uint8_t dummy = 0xFF;

	sd_tx[0] = (command | 0x40); // 0b01xxxxxx
	sd_tx[1] = (arg >> 24) & 0xFF;
	sd_tx[2] = (arg >> 16) & 0xFF;
	sd_tx[3] = (arg >> 8) & 0xFF;
	sd_tx[4] = arg & 0xFF;
	sd_tx[5] = ((crc7(sd_tx, 5) << 1) | 0x01); // 0bxxxxxx1

	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET); 	// pull low

	HAL_SPI_Transmit(hspi3, sd_tx, 6, 100); // data transfer

	// the card will send all 1's until it's ready to send the correct response
	for (uint8_t i = 0; i < 100; i++) {
		HAL_SPI_TransmitReceive(hspi3, &dummy, &response, 1, 10);
		if (response != 0xFF) {
			break;
		}
	}


	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);		// pull high

	return response;
}


HAL_StatusTypeDef SD_Read_Data(SPI_HandleTypeDef *hspi3, uint32_t address, uint8_t *buffer, uint32_t count) {

    for (uint32_t i = 0; i < count; i++) {
        uint8_t cmd_buf[6];
        uint8_t response = 0xFF;
        uint8_t dummy_tx = 0xFF;

        cmd_buf[0] = (READ_SINGLE_BLOCK | 0x40);
        cmd_buf[1] = (address >> 24) & 0xFF;
        cmd_buf[2] = (address >> 16) & 0xFF;
        cmd_buf[3] = (address >> 8)  & 0xFF;
        cmd_buf[4] =  address        & 0xFF;
        cmd_buf[5] = (crc7(cmd_buf, 5) << 1) | 0x01;

        // Sync
        HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
		for(uint8_t sync = 0; sync < 10; sync++) {
			HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &response, 1, 10);
			if (response == 0xFF) break;
		}


        HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

        HAL_SPI_Transmit(hspi3, cmd_buf, 6, 100);

        // Wait for R1 — MUST use TransmitReceive to drive the clock
        for (uint8_t j = 0; j < 100; j++) {
            HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &response, 1, 10);
            if (response != 0xFF) break;
        }

        if (response != 0x00) {
            HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
            return HAL_ERROR;
        }

        // Wait for data token 0xFE
        response = 0xFF;
        for (uint16_t j = 0; j < 5000; j++) {
            HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &response, 1, 10);
            if (response == 0xFE) break;
        }

        if (response != 0xFE) {
            HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
            return HAL_TIMEOUT;
        }

        // Read 512 bytes — HAL_SPI_Receive is OK here because we only care about RX
        // but TransmitReceive is safer; use a small tx buf trick via Receive
        // STM32 HAL Receive clocks out 0xFF automatically in master mode
        //HAL_SPI_Receive(hspi3, buffer, 512, 1000);

        // Read 512 bytes — SAFELY pumping 0xFF on MOSI
		for (uint16_t k = 0; k < 512; k++) {
			HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &buffer[k], 1, 10);
		}

		uint8_t crc[2];
		HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &crc[0], 1, 10);
		HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &crc[1], 1, 10);

		HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

		// 4. Send 8 dummy clocks to power down the SD card's read state machine
		HAL_SPI_Transmit(hspi3, &dummy_tx, 1, 10);



/*
        // Discard 2 CRC bytes
        uint8_t crc_discard[2];
        HAL_SPI_Receive(hspi3, crc_discard, 2, 100);



        // 1 trailing dummy byte
        uint8_t dummy_rx;
        HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &dummy_rx, 1, 10);

        HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
*/
        buffer  += 512;
        address++;
    }


    return HAL_OK;

}

/*
HAL_StatusTypeDef SD_Write_Data(SPI_HandleTypeDef *hspi3, uint32_t address, const uint8_t *buffer, uint32_t count) {

	for (uint32_t i = 0; i < count; i++) {

		uint8_t cmd_buf[8];
		uint8_t response = 0xFF;
		uint8_t dummy = 0xFF;

		// 1. Prepare the command
		cmd_buf[0] = WRITE_BLOCK | 0x40;
		cmd_buf[1] = (address >> 24) & 0xFF;
		cmd_buf[2] = (address >> 16) & 0xFF;
		cmd_buf[3] = (address >> 8) & 0xFF;
		cmd_buf[4] = address & 0xFF;
		cmd_buf[5] = (crc7(cmd_buf, 5) << 1) | 0x01;

		HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET); 	// pull low

		// 2. Send Command
		HAL_SPI_Transmit(hspi3, cmd_buf, 6, 100);

		// 3. Wait for R1 Response (Looking for 0x00 "ready")
		for (uint8_t i = 0; i < 100; i++) {
			HAL_SPI_Receive(hspi3, &response, 1, 10);
			if (response != 0xFF) break;
		}

		if (response != 0x00) {
			HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);		// pull high
			return HAL_ERROR;
		}

		// 4. Send a dummy byte (provide at least 8 clock cycles before the data token)
		HAL_SPI_Transmit(hspi3, &dummy, 1, 10);

		// 5. Send the Data Token (0xFE) to signal data transfer is about to begin
		uint8_t token = 0xFE;
		HAL_SPI_Transmit(hspi3, &token, 1, 10);

		// 6. Send the data
		HAL_SPI_Transmit(hspi3, buffer, 512, 1000);

		// 7. Send two dummy bytes for CRC16
		uint8_t crc_dummy[2] = {0xFF, 0xFF};
		HAL_SPI_Transmit(hspi3, crc_dummy, 2, 100);

		// 8. Read data response token
		HAL_SPI_Receive(hspi3, &response, 1, 10);
		if ((response & 0x1F) != 0x05) { // 0x05 means data was received correctly
			HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);		// pull high
			return HAL_ERROR;
		}

		// 9. Wait for card to finish burning data into memory
		response = 0x00;
		uint32_t timeout = HAL_GetTick() + 500; // 500ms timeout
		while (response == 0x00) {
			HAL_SPI_Receive(hspi3, &response, 1, 10);
			if (HAL_GetTick() > timeout) {
				HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);		// pull high
				return HAL_TIMEOUT; // card hung during write
			}
		}

		// 10. final dummy byte and release
		HAL_SPI_Transmit(hspi3, &dummy, 1, 10);
		HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);		// pull high

		buffer += 512;
		address++;

	}

	return HAL_OK;

}
*/

HAL_StatusTypeDef SD_Write_Data(SPI_HandleTypeDef *hspi3, uint32_t address, const uint8_t *buffer, uint32_t count) {

    for (uint32_t i = 0; i < count; i++) {

        uint8_t cmd_buf[6];
        uint8_t response = 0xFF;
        uint8_t dummy_tx = 0xFF;



        // ==========================================
		// THE BUS RESYNC FIX
		// Keep CS HIGH and pump clocks until the MISO line is completely clear
		// This forces the SD card to abort any stuck operations and realign.
		HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
		for(uint8_t sync = 0; sync < 10; sync++) {
			HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &response, 1, 10);
			if (response == 0xFF) break; // Bus is perfectly aligned and idle!
		}
		// ==========================================


        // 1. Prepare the command
        cmd_buf[0] = WRITE_BLOCK | 0x40;
        cmd_buf[1] = (address >> 24) & 0xFF;
        cmd_buf[2] = (address >> 16) & 0xFF;
        cmd_buf[3] = (address >> 8) & 0xFF;
        cmd_buf[4] = address & 0xFF;
        cmd_buf[5] = (crc7(cmd_buf, 5) << 1) | 0x01;

        HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

        // 2. Send Command
        HAL_SPI_Transmit(hspi3, cmd_buf, 6, 100);

        // 3. Wait for R1 Response (0x00 = ready) — use TransmitReceive to keep clock going
        for (uint8_t j = 0; j < 100; j++) {
            HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &response, 1, 10);
            if (response != 0xFF) break;
        }

        if (response != 0x00) {
            HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
            return HAL_ERROR;
        }

        // 4. At least 8 clocks before data token
        HAL_SPI_Transmit(hspi3, &dummy_tx, 1, 10);

        // 5. Send Data Token
        uint8_t token = 0xFE;
        HAL_SPI_Transmit(hspi3, &token, 1, 10);

        // 6. Send 512 bytes of data
        HAL_SPI_Transmit(hspi3, (uint8_t*)buffer, 512, 1000);

        // 7. Send dummy CRC16
        uint8_t crc_dummy[2] = {0xFF, 0xFF};
        HAL_SPI_Transmit(hspi3, crc_dummy, 2, 100);

        // 8. Read data response token

        for (uint8_t j = 0; j < 100; j++) {
        	HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &response, 1, 10);
			if (response != 0xFF) break; // Token arrived!
        }

        //HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &response, 1, 10);
        if ((response & 0x1F) != 0x05) {
            HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
            return HAL_ERROR;
        }

        // 9. Flush one byte — card pulls MISO low immediately after response
        HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &response, 1, 10);

        // 10. Busy-wait: card holds MISO=0x00 while programming
        response = 0x00;
        uint32_t timeout = HAL_GetTick() + 500;
        while (response == 0x00) {
            HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &response, 1, 10);
            if (HAL_GetTick() > timeout) {
                HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
                return HAL_TIMEOUT;
            }
        }

        // 11. Final dummy byte and release

        HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
		HAL_SPI_Transmit(hspi3, &dummy_tx, 1, 10);

//        HAL_SPI_Transmit(hspi3, &dummy_tx, 1, 10);
//        HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

        buffer += 512;
        address++;
    }

    return HAL_OK;
}




// do a lot more init stuff
HAL_StatusTypeDef SD_Card_Init(SPI_HandleTypeDef *hspi3) {
	// do something with the response value
//	SD_Send_Command(hspi3, GO_IDLE_STATE, 0);
//
//	SD_Send_Command(hspi3, SEND_IF_COND, 0x01);






	volatile uint8_t sd_init_step = 0;   // add at top of function
	volatile uint8_t sd_init_resp = 0;   // add at top of function







	uint8_t response;
	uint8_t dummy = 0xFF;
	uint8_t buf[4];




	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

	// 2. Send 10 bytes (80 clock cycles) to wake up the card's SPI logic
	for (uint8_t i = 0; i < 100; i++) {
		HAL_SPI_Transmit(hspi3, &dummy, 1, 10);
	}


	sd_init_step = 1;
	// ------------------------------------------------------------------
	// CMD0: Software reset — put card into SPI idle state
	// Expected R1 = 0x01 (in idle, waiting for init)
	// ------------------------------------------------------------------
	response = SD_Send_Command(hspi3, GO_IDLE_STATE, 0x00000000);
	sd_init_resp = response;
	if (response != 0x01) {
		return HAL_ERROR;
	}

	// ------------------------------------------------------------------
	// CMD8: Send interface condition
	// Arg: VHS = 0x01 (2.7–3.6V), check pattern = 0xAA → 0x000001AA
	// SDv2 card echoes back 0x01AA in the R7 response.
	// Must be sent manually because SD_Send_Command discards the 4 extra R7 bytes.
	// ------------------------------------------------------------------
	{
		sd_init_step = 2;

		uint8_t cmd8[6];
		cmd8[0] = SEND_IF_COND | 0x40;
		cmd8[1] = 0x00;
		cmd8[2] = 0x00;
		cmd8[3] = 0x01; // VHS = 1 (2.7–3.6V)
		cmd8[4] = 0xAA; // check pattern
		cmd8[5] = (crc7(cmd8, 5) << 1) | 0x01;

		HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(hspi3, cmd8, 6, 100);

		// wait for R1
		response = 0xFF;
		for (int i = 0; i < 10; i++) {
			HAL_SPI_TransmitReceive(hspi3, &dummy, &response, 1, 10);
			if (response != 0xFF) break;
		}

		if (response == 0x01) {
			// SDv2: read 4 more bytes of R7 response
			for (int i = 0; i < 4; i++) {
				HAL_SPI_TransmitReceive(hspi3, &dummy, &buf[i], 1, 10);
			}
			HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

			// buf[2] = voltage (must be 0x01), buf[3] = check pattern (must be 0xAA)
			if (buf[2] != 0x01 || buf[3] != 0xAA) {
				return HAL_ERROR; // card rejected our voltage range
			}
		} else if (response == 0x05) {
			// SDv1 card — doesn't understand CMD8, that's okay, we continue
			HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
		} else {
			// Unexpected response
			HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
			return HAL_ERROR;
		}
	}

	// ------------------------------------------------------------------
	// ACMD41: Activate card initialization
	// ACMD41 = CMD55 (app command prefix) + CMD41
	// Arg: 0x40000000 = HCS bit set, telling the card we support SDHC/SDXC
	// Loop until R1 = 0x00 (card leaves idle state and is ready)
	// ------------------------------------------------------------------
	{
		sd_init_step = 3;

		uint32_t deadline = HAL_GetTick() + 2000; // 2 second timeout
		do {
			SD_Send_Command(hspi3, APP_CMD, 0x00000000);  // CMD55 — prefix telling card next cmd is app-specific
			response = SD_Send_Command(hspi3, 41, 0x40000000); // ACMD41 — index 41, NOT SEND_OP_COND
			// SEND_OP_COND = 1 in the header, which is CMD1 (MMC init). SD cards need ACMD41 = index 41.

			if (HAL_GetTick() > deadline) {
				return HAL_TIMEOUT; // card never left idle
			}

			// 0x01 = still initializing, 0x00 = done
			if (response != 0x00 && response != 0x01) {
				return HAL_ERROR; // unexpected response
			}

			if (response != 0x00) {
				HAL_Delay(10); // small yield before retrying
			}
		} while (response != 0x00);
	}

	// ------------------------------------------------------------------
	// CMD58: Read OCR (Operating Conditions Register)
	// Check the CCS bit (bit 30 = byte 0 bit 6 of the 4-byte OCR) to find out
	// if the card uses block addressing (SDHC/SDXC) or byte addressing (SDSC).
	// Must be sent manually because we need the 4 OCR bytes after the R1.
	// ------------------------------------------------------------------
	uint8_t is_hc = 0;
	{
		sd_init_step = 4;

		uint8_t cmd58[6];
		cmd58[0] = READ_OCR | 0x40;
		cmd58[1] = 0x00;
		cmd58[2] = 0x00;
		cmd58[3] = 0x00;
		cmd58[4] = 0x00;
		cmd58[5] = (crc7(cmd58, 5) << 1) | 0x01;

		HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(hspi3, cmd58, 6, 100);

		// wait for R1
		response = 0xFF;
		for (int i = 0; i < 10; i++) {
			HAL_SPI_TransmitReceive(hspi3, &dummy, &response, 1, 10);
			if (response != 0xFF) break;
		}

		if (response == 0x00) {
			// read 4 bytes of OCR
			for (int i = 0; i < 4; i++) {
				HAL_SPI_TransmitReceive(hspi3, &dummy, &buf[i], 1, 10);
			}
			// CCS = OCR bit 30 = buf[0] bit 6
			is_hc = (buf[0] & 0x40) ? 1 : 0;
		} else {
			HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
			return HAL_ERROR;
		}
		HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
	}

	// ------------------------------------------------------------------
	// CMD16: Set block length to 512 bytes (SDSC / byte-addressed cards only)
	// SDHC and SDXC cards are permanently fixed at 512-byte blocks.
	// ------------------------------------------------------------------
	if (!is_hc) {
		response = SD_Send_Command(hspi3, SET_BLOCKLEN, 512);
		if (response != 0x00) {
			return HAL_ERROR;
		}
	}

	return HAL_OK;


}































///*
// * 1040310811.c
// *
// * SD card SPI driver for STM32F4 + FatFS
// * SD card specification: https://academy.cba.mit.edu/classes/networking_communications/SD/SD.pdf
// */
//
//#include "1040310811.h"
//
//// =============================================================================
//// GLOBAL DEBUG STATE — watch these in the debugger after f_mount fails
////
////   sd_init_step  tells you which command failed:
////     0 = never ran
////     1 = failed at CMD0  (SPI wiring/mode wrong, card not powered)
////     2 = failed at CMD8  (unexpected response, voltage range rejected)
////     3 = failed at ACMD41 (card never left idle — power issue or bad card)
////     4 = failed at CMD58 (OCR read failed)
////     5 = SUCCESS — SD_Card_Init returned HAL_OK
////
////   sd_init_resp  holds the last raw R1 byte from the failing command
////   sd_is_hc     1 = SDHC/SDXC (block-addressed), 0 = SDSC (byte-addressed)
//// =============================================================================
//volatile uint8_t sd_init_step = 0;
//volatile uint8_t sd_init_resp = 0xFF;
//volatile uint8_t sd_is_hc    = 0;
//
//
//// =============================================================================
//// CRC7 — standard SD SPI CRC
//// =============================================================================
//uint8_t crc7(uint8_t *data, uint8_t len) {
//    uint8_t crc = 0;
//    for (uint8_t i = 0; i < len; i++) {
//        uint8_t d = data[i];
//        for (uint8_t j = 0; j < 8; j++) {
//            crc = crc << 1;
//            if ((d ^ crc) & 0x80) {
//                crc ^= 0x09;
//            }
//            d = d << 1;
//        }
//    }
//    return (crc & 0x7F);
//}
//
//
//// =============================================================================
//// SD_Send_Command
//// Sends a 6-byte SPI command, polls for R1 response (up to 100 bytes).
//// CS is driven low before transmit and released after response.
//// Returns the R1 byte, or 0xFF on timeout.
//// =============================================================================
//uint8_t SD_Send_Command(SPI_HandleTypeDef *hspi3, uint8_t command, uint32_t arg) {
//    uint8_t sd_tx[6];
//    uint8_t response = 0xFF;
//    uint8_t dummy    = 0xFF;
//
//    sd_tx[0] = (command | 0x40);
//    sd_tx[1] = (arg >> 24) & 0xFF;
//    sd_tx[2] = (arg >> 16) & 0xFF;
//    sd_tx[3] = (arg >>  8) & 0xFF;
//    sd_tx[4] =  arg        & 0xFF;
//    sd_tx[5] = ((crc7(sd_tx, 5) << 1) | 0x01);
//
//    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
//
//    HAL_SPI_Transmit(hspi3, sd_tx, 6, 100);
//
//    // Card holds MISO=0xFF until it is ready — poll up to 100 times
//    for (uint8_t i = 0; i < 100; i++) {
//        HAL_SPI_TransmitReceive(hspi3, &dummy, &response, 1, 10);
//        if (response != 0xFF) break;
//    }
//
//    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
//
//    // One trailing dummy byte — required between commands
//    HAL_SPI_Transmit(hspi3, &dummy, 1, 10);
//
//    return response;
//}
//
//
//// =============================================================================
//// SD_Card_Init
//// Full SPI-mode initialisation sequence for SD/SDHC/SDXC cards.
//// Watch sd_init_step + sd_init_resp in debugger to diagnose failures.
//// =============================================================================
//HAL_StatusTypeDef SD_Card_Init(SPI_HandleTypeDef *hspi3) {
//    uint8_t response;
//    uint8_t dummy = 0xFF;
//    uint8_t buf[4];
//
//    sd_init_step = 0;
//    sd_init_resp = 0xFF;
//    sd_is_hc     = 0;
//
//    // -----------------------------------------------------------------
//    // 1. Power-up sequence
//    //    CS must be HIGH. Send ≥74 clock pulses (we send 160 = 20 bytes)
//    //    to let the card's internal logic initialise before CMD0.
//    // -----------------------------------------------------------------
//    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
//    HAL_Delay(10);  // extra settling time for power rail
//
//    for (uint8_t i = 0; i < 20; i++) {
//        HAL_SPI_Transmit(hspi3, &dummy, 1, 10);
//    }
//
//    // -----------------------------------------------------------------
//    // 2. CMD0 — Software reset
//    //    Expected R1 = 0x01 (card entered SPI idle state)
//    //    If you get 0xFF here: SPI wiring is wrong, card not powered,
//    //    or CPOL/CPHA is incorrect (must be Mode 0: CPOL=0, CPHA=0).
//    // -----------------------------------------------------------------
//    sd_init_step = 1;
//    response = SD_Send_Command(hspi3, GO_IDLE_STATE, 0x00000000);
//    sd_init_resp = response;
//
//    if (response != 0x01) {
//        // 0xFF = card not responding at all (check wiring + SPI mode)
//        // 0x00 = card skipped idle (rare, usually means it was already init'd)
//        return HAL_ERROR;
//    }
//
//    // -----------------------------------------------------------------
//    // 3. CMD8 — Send interface condition (SDv2 handshake)
//    //    Arg: VHS=0x01 (2.7-3.6V), check pattern=0xAA
//    //    SDv2 card echoes the lower 12 bits back in a 5-byte R7 response.
//    //    SDv1 card responds with 0x05 (illegal command) — that is OK.
//    //    Any other response means the card is faulty or incompatible.
//    //    We send this manually because SD_Send_Command discards the
//    //    4 extra R7 bytes.
//    // -----------------------------------------------------------------
//    sd_init_step = 2;
//    {
//        uint8_t cmd8[6];
//        cmd8[0] = SEND_IF_COND | 0x40;
//        cmd8[1] = 0x00;
//        cmd8[2] = 0x00;
//        cmd8[3] = 0x01;  // VHS: 2.7–3.6 V
//        cmd8[4] = 0xAA;  // check pattern
//        cmd8[5] = (crc7(cmd8, 5) << 1) | 0x01;
//
//        HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
//        HAL_SPI_Transmit(hspi3, cmd8, 6, 100);
//
//        // Poll for R1 — use 100 attempts like every other command
//        response = 0xFF;
//        for (int i = 0; i < 100; i++) {
//            HAL_SPI_TransmitReceive(hspi3, &dummy, &response, 1, 10);
//            if (response != 0xFF) break;
//        }
//        sd_init_resp = response;
//
//        if (response == 0x01) {
//            // SDv2 card: read the remaining 4 R7 bytes
//            for (int i = 0; i < 4; i++) {
//                HAL_SPI_TransmitReceive(hspi3, &dummy, &buf[i], 1, 10);
//            }
//            HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
//            HAL_SPI_Transmit(hspi3, &dummy, 1, 10);  // trailing byte
//
//            // buf[2]=voltage acceptance (must be 0x01), buf[3]=check pattern echo
//            if (buf[2] != 0x01 || buf[3] != 0xAA) {
//                return HAL_ERROR;  // card rejected our voltage range
//            }
//
//        } else if (response == 0x05) {
//            // SDv1 card — CMD8 is an illegal command for it, that is fine
//            HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
//            HAL_SPI_Transmit(hspi3, &dummy, 1, 10);
//
//        } else {
//            // Completely unexpected — bail
//            HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
//            HAL_SPI_Transmit(hspi3, &dummy, 1, 10);
//            return HAL_ERROR;
//        }
//    }
//
//    // -----------------------------------------------------------------
//    // 4. ACMD41 — Activate card initialization
//    //    ACMD41 = CMD55 (app prefix) followed by CMD41 (index 41).
//    //    HCS bit (0x40000000) tells the card we support SDHC/SDXC.
//    //    Card returns 0x01 while still initialising, 0x00 when done.
//    //    Loop with a 2-second timeout.
//    // -----------------------------------------------------------------
//    sd_init_step = 3;
//    {
//        uint32_t deadline = HAL_GetTick() + 2000;
//        do {
//            SD_Send_Command(hspi3, APP_CMD, 0x00000000);           // CMD55
//            response = SD_Send_Command(hspi3, 41, 0x40000000);     // ACMD41
//
//            if (HAL_GetTick() > deadline) {
//                sd_init_resp = response;
//                return HAL_TIMEOUT;
//            }
//
//            if (response != 0x00 && response != 0x01) {
//                sd_init_resp = response;
//                return HAL_ERROR;
//            }
//
//            if (response != 0x00) {
//                HAL_Delay(10);
//            }
//        } while (response != 0x00);
//
//        sd_init_resp = response;
//    }
//
//    // -----------------------------------------------------------------
//    // 5. CMD58 — Read OCR register
//    //    Check CCS bit (OCR bit 30 = buf[0] bit 6) to determine
//    //    addressing mode:  1 = SDHC/SDXC (block), 0 = SDSC (byte)
//    //    A 32GB card will have CCS=1.
//    // -----------------------------------------------------------------
//    sd_init_step = 4;
//    {
//        uint8_t cmd58[6];
//        cmd58[0] = READ_OCR | 0x40;
//        cmd58[1] = 0x00;
//        cmd58[2] = 0x00;
//        cmd58[3] = 0x00;
//        cmd58[4] = 0x00;
//        cmd58[5] = (crc7(cmd58, 5) << 1) | 0x01;
//
//        HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
//        HAL_SPI_Transmit(hspi3, cmd58, 6, 100);
//
//        // Poll for R1 with 100 attempts
//        response = 0xFF;
//        for (int i = 0; i < 100; i++) {
//            HAL_SPI_TransmitReceive(hspi3, &dummy, &response, 1, 10);
//            if (response != 0xFF) break;
//        }
//        sd_init_resp = response;
//
//        if (response == 0x00) {
//            for (int i = 0; i < 4; i++) {
//                HAL_SPI_TransmitReceive(hspi3, &dummy, &buf[i], 1, 10);
//            }
//            sd_is_hc = (buf[0] & 0x40) ? 1 : 0;
//        } else {
//            HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
//            HAL_SPI_Transmit(hspi3, &dummy, 1, 10);
//            return HAL_ERROR;
//        }
//
//        HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
//        HAL_SPI_Transmit(hspi3, &dummy, 1, 10);
//    }
//
//    // -----------------------------------------------------------------
//    // 6. CMD16 — Set block length to 512 bytes
//    //    Required only for SDSC (byte-addressed) cards.
//    //    SDHC/SDXC cards are permanently fixed at 512-byte blocks.
//    // -----------------------------------------------------------------
//    if (!sd_is_hc) {
//        response = SD_Send_Command(hspi3, SET_BLOCKLEN, 512);
//        if (response != 0x00) {
//            sd_init_resp = response;
//            return HAL_ERROR;
//        }
//    }
//
//    sd_init_step = 5;  // SUCCESS
//    return HAL_OK;
//}
//
//
//// =============================================================================
//// SD_Read_Data
//// Reads `count` 512-byte sectors starting at `address` (LBA for SDHC,
//// byte-addressed SDSC is handled by the block-length set in init).
//// FatFS calls this through USER_read in user_diskio.c.
//// =============================================================================
//HAL_StatusTypeDef SD_Read_Data(SPI_HandleTypeDef *hspi3, uint32_t address,
//                                uint8_t *buffer, uint32_t count) {
//    uint8_t response;
//    uint8_t dummy_tx = 0xFF;
//
//    for (uint32_t i = 0; i < count; i++) {
//        uint8_t cmd_buf[6];
//
//        cmd_buf[0] = (READ_SINGLE_BLOCK | 0x40);
//        cmd_buf[1] = (address >> 24) & 0xFF;
//        cmd_buf[2] = (address >> 16) & 0xFF;
//        cmd_buf[3] = (address >>  8) & 0xFF;
//        cmd_buf[4] =  address        & 0xFF;
//        cmd_buf[5] = (crc7(cmd_buf, 5) << 1) | 0x01;
//
//        HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
//        HAL_SPI_Transmit(hspi3, cmd_buf, 6, 100);
//
//        // Wait for R1
//        response = 0xFF;
//        for (uint8_t j = 0; j < 100; j++) {
//            HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &response, 1, 10);
//            if (response != 0xFF) break;
//        }
//
//        if (response != 0x00) {
//            HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
//            return HAL_ERROR;
//        }
//
//        // Wait for data start token 0xFE
//        response = 0xFF;
//        for (uint16_t j = 0; j < 5000; j++) {
//            HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &response, 1, 10);
//            if (response == 0xFE) break;
//        }
//
//        if (response != 0xFE) {
//            HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
//            return HAL_TIMEOUT;
//        }
//
//        // Read 512 bytes of data
//        HAL_SPI_Receive(hspi3, buffer, 512, 1000);
//
//        // Discard 2 CRC bytes
//        uint8_t crc_discard[2];
//        HAL_SPI_Receive(hspi3, crc_discard, 2, 100);
//
//        // One trailing dummy byte
//        uint8_t dummy_rx;
//        HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &dummy_rx, 1, 10);
//
//        HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
//
//        buffer  += 512;
//        address++;
//    }
//
//    return HAL_OK;
//}
//
//
//// =============================================================================
//// SD_Write_Data
//// Writes `count` 512-byte sectors starting at `address`.
//// FatFS calls this through USER_write in user_diskio.c.
//// =============================================================================
//HAL_StatusTypeDef SD_Write_Data(SPI_HandleTypeDef *hspi3, uint32_t address,
//                                 const uint8_t *buffer, uint32_t count) {
//    uint8_t response;
//    uint8_t dummy_tx = 0xFF;
//
//    for (uint32_t i = 0; i < count; i++) {
//        uint8_t cmd_buf[6];
//
//        cmd_buf[0] = WRITE_BLOCK | 0x40;
//        cmd_buf[1] = (address >> 24) & 0xFF;
//        cmd_buf[2] = (address >> 16) & 0xFF;
//        cmd_buf[3] = (address >>  8) & 0xFF;
//        cmd_buf[4] =  address        & 0xFF;
//        cmd_buf[5] = (crc7(cmd_buf, 5) << 1) | 0x01;
//
//        HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
//        HAL_SPI_Transmit(hspi3, cmd_buf, 6, 100);
//
//        // Wait for R1 = 0x00 (ready)
//        response = 0xFF;
//        for (uint8_t j = 0; j < 100; j++) {
//            HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &response, 1, 10);
//            if (response != 0xFF) break;
//        }
//
//        if (response != 0x00) {
//            HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
//            return HAL_ERROR;
//        }
//
//        // At least 8 clock cycles before data token
//        HAL_SPI_Transmit(hspi3, &dummy_tx, 1, 10);
//
//        // Send data start token
//        uint8_t token = 0xFE;
//        HAL_SPI_Transmit(hspi3, &token, 1, 10);
//
//        // Send 512 bytes
//        HAL_SPI_Transmit(hspi3, (uint8_t*)buffer, 512, 1000);
//
//        // Send dummy CRC16
//        uint8_t crc_dummy[2] = {0xFF, 0xFF};
//        HAL_SPI_Transmit(hspi3, crc_dummy, 2, 100);
//
//        // Read data response token — lower 5 bits: 0x05 = accepted
//        HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &response, 1, 10);
//        if ((response & 0x1F) != 0x05) {
//            HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
//            return HAL_ERROR;
//        }
//
//        // Flush: card drives MISO low immediately after the response token
//        HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &response, 1, 10);
//
//        // Busy-wait: card holds MISO=0x00 while programming flash internally
//        response = 0x00;
//        uint32_t timeout = HAL_GetTick() + 500;
//        while (response == 0x00) {
//            HAL_SPI_TransmitReceive(hspi3, &dummy_tx, &response, 1, 10);
//            if (HAL_GetTick() > timeout) {
//                HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
//                return HAL_TIMEOUT;
//            }
//        }
//
//        // Final dummy byte + release CS
//        HAL_SPI_Transmit(hspi3, &dummy_tx, 1, 10);
//        HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
//
//        buffer  += 512;
//        address++;
//    }
//
//    return HAL_OK;
//}
