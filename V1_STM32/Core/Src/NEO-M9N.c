/*
 * NEO-M9N.c
 *
 *  Created on: Feb 24, 2026
 *      Author: gamin
 */

#include <string.h>
#include "NEO-M9N.h"

// global struct
NEO_M9N_Data_t gps_data;
static UART_HandleTypeDef *gps_huart_ptr;


// INTERRUPT STUFF
#define RX_BUFFER_SIZE 1024 // might need to be bigger
volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;
uint8_t rx_byte;	// temporary variable to hold the incoming byte

HAL_StatusTypeDef GPS_Interrupt_Init(UART_HandleTypeDef *huart1) {
	// start listening for 1 byte on the rx line
	// when the byte arrives, HAL_UART_RxCpltCallback will be triggered

	gps_huart_ptr = huart1; // Store it here

	if (HAL_UART_Receive_IT(gps_huart_ptr, &rx_byte, 1) != HAL_OK) {
		return HAL_ERROR;
	}
	return HAL_OK;
}


// This overrides the weak HAL callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart1) {
	if (huart1->Instance == USART6) {		// check if it's the GPS UART
		GPS_Rx_FIFO_Put();	// if status is 0 then put failed
	}
}

int GPS_Rx_FIFO_Put(void) {
	if (((rx_head + 1) % RX_BUFFER_SIZE) == rx_tail) {
		// re-arm the interrupt so interrupts never stop triggering
		HAL_UART_Receive_IT(gps_huart_ptr, &rx_byte, 1);
		return 0;		// fail, buffer full, increase buffer size
	}
	// Save the incoming byte to the circular buffer
	rx_buffer[rx_head] = rx_byte;

	// increment the head pointer
	rx_head = (rx_head + 1) % RX_BUFFER_SIZE;

	// re-arm the interrupt to catch the next byte
	HAL_UART_Receive_IT(gps_huart_ptr, &rx_byte, 1);

	return 1;
}

// returns data if not empty, else returns -1
uint8_t GPS_Rx_FIFO_Get(void) {
	if (rx_head == rx_tail) {
		return -1;		// fail, buffer empty
	}

	uint8_t data = rx_buffer[rx_tail];
	rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
	return data;

}


// INTERPRET BUFFER
typedef enum {
	STATE_SYNC1,
	STATE_SYNC2,
	STATE_CLASS,
	STATE_ID,
	STATE_LEN1,
	STATE_LEN2,
	STATE_PAYLOAD,
	STATE_CK_A,
	STATE_CK_B
} UBX_State_t;

UBX_State_t ubx_state = STATE_SYNC1;
uint8_t current_class, current_id;
uint16_t current_length;
uint16_t payload_idx = 0;
uint8_t payload_buffer[100];
uint8_t calc_ck_a, calc_ck_b;


void GPS_Process_Stream(void) {
	// process all available bytes immediately
	while (rx_head != rx_tail) {
		uint8_t data = GPS_Rx_FIFO_Get();

		switch (ubx_state) {
				case STATE_SYNC1:
					if (data == 0xB5) ubx_state = STATE_SYNC2;
					break;

				case STATE_SYNC2:
					if (data == 0x62) {
						ubx_state = STATE_CLASS;
					} else {
						ubx_state = STATE_SYNC1; // lost sync, start over
					}
					break;

				case STATE_CLASS:
					current_class = data;
					calc_ck_a = data;
					calc_ck_b = data;
					ubx_state = STATE_ID;
					break;

				case STATE_ID:
					current_id = data;
					calc_ck_a += data;
					calc_ck_b += calc_ck_a;
					ubx_state = STATE_LEN1;
					break;

				case STATE_LEN1:
					current_length = data;	// LSB (little endian)
					calc_ck_a += data;
					calc_ck_b += calc_ck_a;
					ubx_state = STATE_LEN2;
					break;

				case STATE_LEN2:
					current_length |= (data << 8);	// MSB
					calc_ck_a += data;
					calc_ck_b += calc_ck_a;
					payload_idx = 0;


					if (current_length > sizeof(payload_buffer)) { // protect against overflow
						ubx_state = STATE_SYNC1;
					} else {
						ubx_state = STATE_PAYLOAD;
					}
					break;

				case STATE_PAYLOAD:
					payload_buffer[payload_idx] = data;
					payload_idx++;
					calc_ck_a += data;
					calc_ck_b += calc_ck_a;

					if (payload_idx >= current_length) {
						ubx_state = STATE_CK_A;
					}
					break;

				case STATE_CK_A:
					if (data == calc_ck_a) {
						ubx_state = STATE_CK_B;
					} else {
						ubx_state = STATE_SYNC1;	// checksum failed
					}
					break;

				case STATE_CK_B:
					if (data == calc_ck_b) {
						Parse_Valid_UBX_Message();	// success, parse the message
					}
					ubx_state = STATE_SYNC1;		// reset for next packet
					break;
			}

	}
}

/*
void GPS_Process_Stream(void) {
	uint8_t data = GPS_Rx_FIFO_Get();

	// if data = -1, the buffer is empty, so return
	if (data == -1) return;


	switch (ubx_state) {
		case STATE_SYNC1:
			if (data == 0xB5) ubx_state = STATE_SYNC2;
			break;

		case STATE_SYNC2:
			if (data == 0x62) {
				ubx_state = STATE_CLASS;
			} else {
				ubx_state = STATE_SYNC1; // lost sync, start over
			}
			break;

		case STATE_CLASS:
			current_class = data;
			calc_ck_a = data;
			calc_ck_b = data;
			ubx_state = STATE_ID;
			break;

		case STATE_ID:
			current_id = data;
			calc_ck_a += data;
			calc_ck_b += calc_ck_a;
			ubx_state = STATE_LEN1;
			break;

		case STATE_LEN1:
			current_length = data;	// LSB (little endian)
			calc_ck_a += data;
			calc_ck_b += calc_ck_a;
			ubx_state = STATE_LEN2;
			break;

		case STATE_LEN2:
			current_length |= (data << 8);	// MSB
			calc_ck_a += data;
			calc_ck_b += calc_ck_a;
			payload_idx = 0;


			if (current_length > sizeof(payload_buffer)) { // protect against overflow
				ubx_state = STATE_SYNC1;
			} else {
				ubx_state = STATE_PAYLOAD;
			}
			break;

		case STATE_PAYLOAD:
			payload_buffer[payload_idx] = data;
			payload_idx++;
			calc_ck_a += data;
			calc_ck_b += calc_ck_a;

			if (payload_idx >= current_length) {
				ubx_state = STATE_CK_A;
			}
			break;

		case STATE_CK_A:
			if (data == calc_ck_a) {
				ubx_state = STATE_CK_B;
			} else {
				ubx_state = STATE_SYNC1;	// checksum failed
			}
			break;

		case STATE_CK_B:
			if (data == calc_ck_b) {
				Parse_Valid_UBX_Message();	// success, parse the message
			}
			ubx_state = STATE_SYNC1;		// reset for next packet
			break;
	}
}
*/


uint32_t Convert_To_Unix_Time(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
	// array of cumulative days per month (non-leap year)
	const uint16_t days_in_month[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

	// calculate total number of days since January 1, 1970 (including leap days)
	uint32_t days = 365 * (year - 1970) + (year - 1969) / 4 - (year - 1901) / 100 + (year - 1601) / 400;

	days += days_in_month[month - 1];

	// add an extra day if we are past february in a leap year
	if (month > 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0))) {
		days++;
	}

	days += day - 1;

	// convert days into seconds, add hours, mins, and secs
	return (days * 86400) + (hour * 3600) + (min * 60) + sec;
}



void Parse_Valid_UBX_Message(void) {
	// make sure current class and id match
	if ((current_class == 0x01) && (current_id == 0x07)) {

		uint8_t fix_type = payload_buffer[20];

		// read time
		gps_data.year = *(uint16_t*)(&payload_buffer[4]);
		gps_data.month = payload_buffer[6];
		gps_data.day = payload_buffer[7];
		gps_data.hour = payload_buffer[8];
		gps_data.min = payload_buffer[9];
		gps_data.sec = payload_buffer[10];



		// added SIV
		gps_data.siv = payload_buffer[23];

		// added unix_timestamp
		gps_data.unix_timestamp = Convert_To_Unix_Time(
				gps_data.year, gps_data.month, gps_data.day,
				gps_data.hour, gps_data.min, gps_data.sec
		);


		// we can only trust the position and velocity if we have a 2D or 3D fix
		if ((fix_type == 2) || (fix_type == 3)) {

			// position
			int32_t lon_raw = *(int32_t*)(&payload_buffer[24]);
			int32_t lat_raw = *(int32_t*)(&payload_buffer[28]);
			int32_t alt_raw = *(int32_t*)(&payload_buffer[36]);

			gps_data.longitude = lon_raw / 10000000.0f; // 1e-7 degrees
			gps_data.latitude = lat_raw / 10000000.0f; // 1e-7 degrees
			gps_data.altitude = alt_raw / 1000.0f; // mm

			// velocity
			int32_t speed_raw = *(int32_t*)(&payload_buffer[60]);
			int32_t heading_raw = *(int32_t*)(&payload_buffer[64]);
			gps_data.speed_m_s = speed_raw / 1000.0f;
			gps_data.heading_deg = heading_raw / 100000.0f;

		}

	}
}


// UART STUFF
HAL_StatusTypeDef Send_UBX_Command(UART_HandleTypeDef *huart, uint8_t msgClass, uint8_t msgId, const uint8_t *payload, uint16_t length) {

	// 1. prepare the 6 byte header
	uint8_t header[6] = {
			0xB5,	// sync char
			0x62,	// sync char
			msgClass,
			msgId,
			(uint8_t)(length & 0xFF),
			(uint8_t)(length >> 8)
	};

	// 2. calculate the CheckSum (Fletcher-8 algorithm)
	// checksum is calculated over the class, id, length, and payload fields
	uint8_t ckA = 0;
	uint8_t ckB = 0;

	// checksum over header (skipping the sync chars)
	for (int i = 2; i < 6; i++) {
		ckA += header[i];
		ckB += ckA;
	}

	// checksum over payload
	for (uint16_t i = 0; i < length; i++) {
		ckA += payload[i];
		ckB += ckA;
	}

	uint8_t checksum[2] = {ckA, ckB};

	// 3. transmit the data using HAL (in chunks to save RAM)
	HAL_StatusTypeDef status;

	// Send header
	status = HAL_UART_Transmit(huart, header, 6, 100);
	if (status != HAL_OK) return status;

	// send payload (if there is one)
	if (length > 0 && payload != NULL) {
		status = HAL_UART_Transmit(huart, payload, length, 100);
		if (status != HAL_OK) return status;
	}

	// send checksum
	status = HAL_UART_Transmit(huart, checksum, 2, 100);

	return status;
}

// Function to calculate the UBX checksum (8-bit Fletcher Algorithm)
void calculate_ubx_checksum(uint8_t *msg, uint16_t len, uint8_t *ck_a, uint8_t *ck_b) {
    *ck_a = 0;
    *ck_b = 0;
    // Checksum is calculated over Class, ID, Length, and Payload (starts at index 2)
    for (uint16_t i = 2; i < len - 2; i++) {
        *ck_a = *ck_a + msg[i];
        *ck_b = *ck_b + *ck_a;
    }
}

void GPS_Configure_Output(UART_HandleTypeDef *huart1) {
	// UBX-CFG-VALSET to enable UBX-NAV-PVT (position, velocity, time) on UART1
	uint8_t cfg_valset[17] = {
			0xB5, 0x62,				// Sync chars 1 and 2
			0x06, 0x8A,				// Class: CFG (0x06), ID: VALSET: (0x8A)
			0x09, 0x00,				// Length: 9 bytes of payload
			0x00,					// Payload: message version
			0x01,					// Payload: Layer (0x01 = RAM layer)
			0x00, 0x00, 			// Payload: Transactioin info and reserved
			0x07, 0x00, 0x91, 0x20,	// Payload: Key ID for CFG-MSGOUT-UBX_NAV_PVT_UART1 (Little Endian: 0x20910007)
			0x01,					// Payload: Value (1 = output every 1 navigation epoch)
			0x00, 0x00				// Checksum (CK_A, CK_B) - to be calculated
	};

	// Calculate and append checksum
	calculate_ubx_checksum(cfg_valset, sizeof(cfg_valset), &cfg_valset[15], &cfg_valset[16]);

	// Send the command to the GPS over UART
	HAL_UART_Transmit(huart1, cfg_valset, sizeof(cfg_valset), 100);
}


// sets up UART communication for GPS
// ENABLES UART INTERRUPTS FOR STM32
HAL_StatusTypeDef NEO_Init(UART_HandleTypeDef *huart1) {


	HAL_Delay(500);

	memset(&gps_data, 0, sizeof(gps_data));

	// was giving syntax error
	//gps_data = {0,0,0,0,0,0,0,0,0,0};

	// create a new UART_HandleTypeDef that has baud rate of 38400
	// pass it into the baud rate init instruction, then use the huart1 param for the rest

//	UART_HandleTypeDef tempUart;
//	memset(&tempUart, 0, sizeof(tempUart));
//
//	tempUart.Instance = USART1;
//
//	tempUart.Init.BaudRate = 38400;
//
//	tempUart.Init.WordLength = UART_WORDLENGTH_8B;
//
//	tempUart.Init.StopBits = UART_STOPBITS_1;
//
//	tempUart.Init.Parity = UART_PARITY_NONE;
//	tempUart.Init.Mode = UART_MODE_TX_RX;
//	tempUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//	tempUart.Init.OverSampling = UART_OVERSAMPLING_16;
//	if (HAL_UART_Init(&tempUart) != HAL_OK)
//	{
//		Error_Handler();
//	}


	huart1->Init.BaudRate = 38400;
	if (HAL_UART_Init(huart1) != HAL_OK) {
		return HAL_ERROR;
	}

	HAL_StatusTypeDef status;

	// Little Endian
	uint8_t payload_baud[] = {
			0x00,						// Version (always 0)
			0x01,						// Layers (0x01 = RAM only. Safer for testing so you don't brick it)
			0x00, 0x00,					// Reserved
			0x01, 0x00, 0x52, 0x40,		// Key ID for CFG-UART1-BAUDRATE
			0x00, 0xC2, 0x01, 0x00		// New Baud Rate Value: 115200 (0x0001C200)
	};

	status = Send_UBX_Command(huart1, 0x06, 0x8A, payload_baud, sizeof(payload_baud));

	if (status != HAL_OK) {

		// this is very dumb, caused error loops for some reasom
		//Error_Handler();

		return HAL_ERROR;

	}

	HAL_Delay(100);

	// switches STM32 back to 115200
	huart1->Init.BaudRate = 115200;
	if (HAL_UART_Init(huart1) != HAL_OK) {
		return HAL_ERROR;
	}

	// force hal state machine back to ready
	huart1->gState = HAL_UART_STATE_READY;
	huart1->RxState = HAL_UART_STATE_READY;

	// clear all error flags
	__HAL_UART_CLEAR_OREFLAG(huart1);
	__HAL_UART_CLEAR_NEFLAG(huart1);
	__HAL_UART_CLEAR_FEFLAG(huart1);


	// more init in the works

	GPS_Configure_Output(huart1);
	HAL_Delay(50);
	GPS_Enable_TimePulse(huart1);

	// 4. Clear any overrun errors that happened during the 100ms delay
	// If the GPS blasted data while we were waiting, the hardware throws an ORE flag.
	// If we don't clear it, HAL_UART_Receive_IT will instantly fail.
	__HAL_UART_CLEAR_OREFLAG(huart1);
	__HAL_UART_CLEAR_NEFLAG(huart1);
	__HAL_UART_CLEAR_FEFLAG(huart1);

	return GPS_Interrupt_Init(huart1);

	//return HAL_OK; // had to add this missing return

}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART6) {
		// clear the Overrun Error Flag
		__HAL_UART_CLEAR_OREFLAG(huart);

		// clear any other common error flags just in case
		__HAL_UART_CLEAR_FEFLAG(huart);
		__HAL_UART_CLEAR_NEFLAG(huart);

		// restart interrupt reception so it doesn't stay dead forever
		extern uint8_t rx_byte;
		HAL_UART_Receive_IT(huart, &rx_byte, 1);
	}
}

void GPS_Enable_TimePulse(UART_HandleTypeDef *huart) {
    // UBX-CFG-VALSET payload to enable TIMEPULSE 1
    uint8_t payload_tp[] = {
        0x00,                   // Version (always 0)
        0x01,                   // Layer: 0x01 = RAM. (Use 0x07 to save to RAM + BBR + Flash so it persists after reboot)
        0x00, 0x00,             // Transaction info and reserved

        // --- KEY 1: CONFIGURATION ITEM ---
        0x07, 0x00, 0x05, 0x10, // Key ID: CFG-TP-TP1_ENA (0x10050007 in Little Endian)
        0x01,                    // Value: 1 (Boolean True)

		// --- KEY 2: CFG-TP-PERIOD_TP1 (0x40050001) ---
		0x01, 0x00, 0x05, 0x40,
		0x40, 0x42, 0x0F, 0x00, // Value: 1,000,000 microseconds = 1 Hz (Little Endian)

		// --- KEY 3: CFG-TP-LEN_TP1 (0x40050002) ---
		0x02, 0x00, 0x05, 0x40,
		0xA0, 0x86, 0x01, 0x00  // Value: 100,000 microseconds = 100ms pulse (Little Endian)
    };

    // Send the command using your existing helper function
    Send_UBX_Command(huart, 0x06, 0x8A, payload_tp, sizeof(payload_tp));
}
