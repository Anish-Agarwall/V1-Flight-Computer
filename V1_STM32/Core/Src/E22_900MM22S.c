/*
 * E22_900MM22S.c
 *
 *  Created on: Mar 21, 2026
 *      Author: 17372
 */

#include "E22_900MM22S.h"


// datasheets:
// E22-900MM22S: Linked on KiCad
// SX1262 (inside of the E22): https://semtech.my.salesforce.com/sfc/p/#E0000000JelG/a/RQ000008nKCH/hp2iKwMDKWl34g1D3LBf_zC7TGBRIo2ff5LMnS8r19s



// MAYBE ADD A CALIBRATE FUNCTION??????


/**
 * command: opcode for the command
 * params: arguments for the command
 * numParams: length of the params array
 *
 */

#define frequency 908400000

HAL_StatusTypeDef E22_900MM22S_Wait_Busy(void) {
	uint32_t timeout = HAL_GetTick() + 1000; // 1 second timeout

	// wait while busy pin is high
	while (HAL_GPIO_ReadPin(E22_GPIO_Port, E22_Telem_BUSY) == GPIO_PIN_SET) {
		if (HAL_GetTick() > timeout) {
			return HAL_TIMEOUT;
		}
	}
	return HAL_OK;
}

HAL_StatusTypeDef E22_900MM22S_Send_Command(SPI_HandleTypeDef *hspi1, uint8_t command, uint8_t *params, uint16_t numParams) {
	if (E22_900MM22S_Wait_Busy() != HAL_OK) return HAL_TIMEOUT;

	HAL_GPIO_WritePin(E22_GPIO_Port, E22_Telem_CS, GPIO_PIN_RESET);		// pull low

	HAL_SPI_Transmit(hspi1, &command, 1, 100);							// send opcode

	if (numParams > 0) {
		HAL_SPI_Transmit(hspi1, params, numParams, 100);				// send arguments
	}

	HAL_GPIO_WritePin(E22_GPIO_Port, E22_Telem_CS, GPIO_PIN_SET);		// pull high

	return HAL_OK;
}


HAL_StatusTypeDef E22_900MM22S_Sleep(SPI_HandleTypeDef *hspi1) {
	// Turn off RF switch to save max power
	HAL_GPIO_WritePin(E22_GPIO_Port, E22_RXEN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(E22_GPIO_Port, E22_TXEN, GPIO_PIN_RESET);

	// Send sleep command
	uint8_t sleepConfig = 0x00;
	return E22_900MM22S_Send_Command(hspi1, SetSleep, &sleepConfig, 1);

}

uint32_t FrequencyToPLL(long rfFreq) {
	/* Datasheet Says:
	   *    rfFreq = (pllFreq * xtalFreq) / 2^25
	   * Rewrite to solve for pllFreq
	   *    pllFreq = (2^25 * rfFreq)/xtalFreq
	   *
	   *  In our case, xtalFreq is 32mhz
	   *  pllFreq = (2^25 * rfFreq) / 32000000
	   */

	  //Basically, we need to do "return ((1 << 25) * rfFreq) / 32000000L"
	  //It's very important to perform this without losing precision or integer overflow.
	  //If arduino supported 64-bit varibales (which it doesn't), we could just do this:
	  //    uint64_t firstPart = (1 << 25) * (uint64_t)rfFreq;
	  //    return (uint32_t)(firstPart / 32000000L);
	  //
	  //Instead, we need to break this up mathimatically to avoid integer overflow
	  //First, we'll simplify the equation by dividing both parts by 2048 (2^11)
	  //    ((1 << 25) * rfFreq) / 32000000L      -->      (16384 * rfFreq) / 15625;
	  //
	  // Now, we'll divide first, then multiply (multiplying first would cause integer overflow)
	  // Because we're dividing, we need to keep track of the remainder to avoid losing precision
	  uint32_t q = rfFreq / 15625UL;  //Gives us the result (quotient), rounded down to the nearest integer
	  uint32_t r = rfFreq % 15625UL;  //Everything that isn't divisible, aka "the part that hasn't been divided yet"

	  //Multiply by 16384 to satisfy the equation above
	  q *= 16384UL;
	  r *= 16384UL; //Don't forget, this part still needs to be divided because it was too small to divide before

	  return q + (r / 15625UL);  //Finally divide the the remainder part before adding it back in with the quotient
}

HAL_StatusTypeDef E22_900MM22S_Init_900MHz(SPI_HandleTypeDef *hspi1) {

	// Set RF switch to off (not currently transmitting or receiving)
	HAL_GPIO_WritePin(E22_GPIO_Port, E22_RXEN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(E22_GPIO_Port, E22_TXEN, GPIO_PIN_RESET);

	HAL_StatusTypeDef status;

	// Wake up the chip by pulling CS low then high
	HAL_GPIO_WritePin(E22_GPIO_Port, E22_Telem_CS, GPIO_PIN_RESET);		// pull low

	HAL_Delay(1);

	HAL_GPIO_WritePin(E22_GPIO_Port, E22_Telem_CS, GPIO_PIN_SET);		// pull high

	if (E22_900MM22S_Wait_Busy() != HAL_OK) return HAL_TIMEOUT;

	// Set to standby mode
	uint8_t stdbyConfig = 0x00;
	status = E22_900MM22S_Send_Command(hspi1, SetStandby, &stdbyConfig, 1);
	if (status != HAL_OK) return status;

	// Set packet type to LoRa (0x01)
	uint8_t packetType = 0x01;
	E22_900MM22S_Send_Command(hspi1, SetPacketType, &packetType, 1);

	// Set RF frequency to 900MHz (0x38400000)
	//uint8_t freqParams[] = {0x38, 0x40, 0x00, 0x00};

	uint32_t pllFreq = FrequencyToPLL(frequency);

	uint8_t freqParams[4];
	freqParams[0] = (pllFreq >> 24) & 0xFF;
	freqParams[1] = (pllFreq >> 16) & 0xFF;
	freqParams[2] = (pllFreq >> 8) & 0xFF;
	freqParams[3] = (pllFreq >> 0) & 0xFF;
	E22_900MM22S_Send_Command(hspi1, SetRfFrequency, freqParams, 4);

	// Configure PA for +22dBm (Hardware specific to E22-900MM22S)
	// paDutyCycle = 0x04, hpMax = 0x07, deviceSel = 0x00; paLut = 0x01
	uint8_t paParams[] = {0x04, 0x07, 0x00, 0x01};
	E22_900MM22S_Send_Command(hspi1, SetPaConfig, paParams, 4);

	// Set TX params: +22dBm (0x16) power, 200us ramp time (0x04)
	uint8_t txParams[] = {0x16, 0x04};
	E22_900MM22S_Send_Command(hspi1, SetTxParams, txParams, 2);


	// set sync word params
	//uint8_t syncWordParams[4] = {0x07, 0x40, 0x14, 0x24}; // private default
	uint8_t syncWordParams[4] = {0x07, 0x40, 0x34, 0x44}; // public default
	E22_900MM22S_Send_Command(hspi1, 0x0D, syncWordParams, 4);


	// Set Packet Params based on Michael's packets: 8 byte preamble, 74 byte packet, CRC on, IQ Standard
	uint8_t pktParams[] = {0x00, 0x08, 0x00, 0xF0, 0x01, 0x00};
	E22_900MM22S_Send_Command(hspi1, SetPacketParams, pktParams, 6);

	// Set mod params:  SF9, 500kHz BW, CR 4/5, LDRO Off
	// Consider switchting to SF8 or SF7 for faster transmission
	//uint8_t modParams[] = {0x09, 0x06, 0x01, 0x00};

	// SF10, 125kHz BW, CR 4/5, LDRO On
	uint8_t modParams[] = {0x07, 0x04, 0x01, 0x00};
	E22_900MM22S_Send_Command(hspi1, SetModulationParams, modParams, 4);



	uint8_t bufBase[] = {0x00, 0x00}; // TX base = 0x00, RX base = 0x00
	E22_900MM22S_Send_Command(hspi1, 0x8F, bufBase, 2); // SetBufferBaseAddress

	return HAL_OK;

}


HAL_StatusTypeDef E22_900MM22S_Transmit(SPI_HandleTypeDef *hspi1, uint8_t *buffer, uint8_t length) {
    if (E22_900MM22S_Wait_Busy() != HAL_OK) return HAL_TIMEOUT;

    // Optional but recommended: Update packet params with the exact 'length' being sent
    uint8_t pktParams[] = {0x00, 0x08, 0x00, length, 0x01, 0x00};
    E22_900MM22S_Send_Command(hspi1, SetPacketParams, pktParams, 6);

    // Write data to the internal SX1262 buffer
    uint8_t writeCmd[2] = {WriteBuffer, 0x00};
    HAL_GPIO_WritePin(E22_GPIO_Port, E22_Telem_CS, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi1, writeCmd, 2, 100);
    HAL_SPI_Transmit(hspi1, buffer, length, 1000);
    HAL_GPIO_WritePin(E22_GPIO_Port, E22_Telem_CS, GPIO_PIN_SET);

    // Turn on power amplifier (TX_EN High, RX_EN Low)
    HAL_GPIO_WritePin(E22_GPIO_Port, E22_RXEN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(E22_GPIO_Port, E22_TXEN, GPIO_PIN_SET);

    // Tell the SX1262 to fire the radio (No timeout)
    uint8_t txTimeout[] = {0x00, 0x00, 0x00};
    E22_900MM22S_Send_Command(hspi1, SetTx, txTimeout, 3);

    // Wait for the TX to complete by polling the IRQ register
    uint8_t irqStatus[3] = {0}; // [0] = status, [1] = IRQ High, [2] = IRQ Low
    uint8_t getIrqCmd = GetIrqStatus;
    uint32_t startTick = HAL_GetTick();

    while(1) {
        if (HAL_GetTick() - startTick > 1000) { // 1 second absolute timeout
            break;
        }

        HAL_GPIO_WritePin(E22_GPIO_Port, E22_Telem_CS, GPIO_PIN_RESET);
        HAL_SPI_Transmit(hspi1, &getIrqCmd, 1, 100);
        HAL_SPI_Receive(hspi1, irqStatus, 3, 100);
        HAL_GPIO_WritePin(E22_GPIO_Port, E22_Telem_CS, GPIO_PIN_SET);

        // Check if TxDone flag (Bit 0 of IRQ High byte) is set
        if (irqStatus[1] & 0x01) {
            break;
        }
        HAL_Delay(1); // Give the bus a brief rest
    }

    // CRITICAL: Turn off the RF Switch after transmission!
    HAL_GPIO_WritePin(E22_GPIO_Port, E22_TXEN, GPIO_PIN_RESET);

    // Clear the IRQ flags so it's ready for the next transmission
    uint8_t clearIrqParams[2] = {0x03, 0xFF};
    E22_900MM22S_Send_Command(hspi1, ClearIrqStatus, clearIrqParams, 2);

    return HAL_OK;
}


HAL_StatusTypeDef E22_900MM22S_Transmit_Start(SPI_HandleTypeDef *hspi1, uint8_t *buffer, uint8_t length) {
	if (E22_900MM22S_Wait_Busy() != HAL_OK) return HAL_TIMEOUT;

	uint8_t pktParams[] = {0x00, 0x08, 0x00, length, 0x01, 0x00};
	E22_900MM22S_Send_Command(hspi1, SetPacketParams, pktParams, 6);

	// Write data to the internal SX1262 buffer
	uint8_t writeCmd[2] = {WriteBuffer, 0x00};
	HAL_GPIO_WritePin(E22_GPIO_Port, E22_Telem_CS, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi1, writeCmd, 2, 100);
	HAL_SPI_Transmit(hspi1, buffer, length, 1000);
	HAL_GPIO_WritePin(E22_GPIO_Port, E22_Telem_CS, GPIO_PIN_SET);

	// Turn on power amplifier (TX_EN High, RX_EN Low)
	HAL_GPIO_WritePin(E22_GPIO_Port, E22_RXEN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(E22_GPIO_Port, E22_TXEN, GPIO_PIN_SET);

	// Tell the SX1262 to fire the radio (No timeout)
	uint8_t txTimeout[] = {0x00, 0x00, 0x00};
	E22_900MM22S_Send_Command(hspi1, SetTx, txTimeout, 3);

	// return immediately, radio transmits in the background while we do other things.
	return HAL_OK;
}

HAL_StatusTypeDef E22_900MM22S_Transmit_Finish(SPI_HandleTypeDef *hspi1) {
	// check if still busy. If so, try again later
	if (HAL_GPIO_ReadPin(E22_GPIO_Port, E22_Telem_BUSY) == GPIO_PIN_SET) {
		return HAL_BUSY;
	}

	// Turn off RF Switch after transmission
	HAL_GPIO_WritePin(E22_GPIO_Port, E22_TXEN, GPIO_PIN_RESET);

	// Clear IRQ flags so it's ready for the next transmission
	uint8_t clearIrqParams[2] = {0x03, 0xFF};
	E22_900MM22S_Send_Command(hspi1, ClearIrqStatus, clearIrqParams, 2);

	return HAL_OK;
}





/*
HAL_StatusTypeDef E22_900MM22S_Transmit(SPI_HandleTypeDef *hspi1, uint8_t *buffer, uint8_t length) {
	if (E22_900MM22S_Wait_Busy() != HAL_OK) return HAL_TIMEOUT;

	// write data to the internal SX1262 buffer
	uint8_t writeCmd[2] = {WriteBuffer, 0x00};
	HAL_GPIO_WritePin(E22_GPIO_Port, E22_Telem_CS, GPIO_PIN_RESET);		// pull low
	HAL_SPI_Transmit(hspi1, writeCmd, 2, 100);							// send command + offset
	HAL_SPI_Transmit(hspi1, buffer, length, 1000);						// send actual payload
	HAL_GPIO_WritePin(E22_GPIO_Port, E22_Telem_CS, GPIO_PIN_SET);		// pull high

	// Turn on power amplifier (TX_EN)
	HAL_GPIO_WritePin(E22_GPIO_Port, E22_RXEN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(E22_GPIO_Port, E22_TXEN, GPIO_PIN_SET);

	// Tell the SX1262 to actually fire the the radio
	uint8_t txTimeout[] = {0x00, 0x00, 0x00};
	return E22_900MM22S_Send_Command(hspi1, SetTx, txTimeout, 3);

}
*/

