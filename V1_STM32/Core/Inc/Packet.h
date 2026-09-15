/*
 * Packet.h
 *
 *  Created on: Apr 17, 2026
 *      Author: gorgan
 */

#ifndef SRC_PACKET_H_
#define SRC_PACKET_H_

#include "stm32f4xx_hal.h"
#include "main.h"

#define PACKET_DATA_SIZE_BYTES 72
#define FULL_PACKET_SIZE_BYTES (PACKET_DATA_SIZE_BYTES + 2)

typedef enum {
    PACKET_OK = 0,
    PACKET_ERR = 1
} Packet_Status_t;


/**
 * @brief Packet configuration
 *
 */
typedef struct {

    // gps
    double gps_longitude, gps_latitude, gps_altitude, gps_heading;
    int gps_unix_timestamp;
    short gps_SIV;

    // accelerometer
    float imu_ax, imu_ay, imu_az, imu_pitch, imu_yaw, imu_roll;

    // barometer
    int barometric_pressure;

    // state
    short state;
} Packet_t;


/**
 * @brief Ensure string buffer is at least 74 bytes
 *
 */
void packet_to_string(Packet_t* pkt, char* string_buffer);

Packet_Status_t string_to_packet(char* string_buffer, Packet_t* pkt);



#endif /* SRC_PACKET_H_ */
