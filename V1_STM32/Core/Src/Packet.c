/*
 * Packet.c
 *
 *  Created on: Apr 17, 2026
 *      Author: gameon
 */


#include "Packet.h"
#include <string.h>

void packet_to_string(Packet_t *pkt, char *string_buffer) {
    string_buffer[0] = '{';
    memcpy(string_buffer + 1, pkt, sizeof(Packet_t));
    string_buffer[1 + sizeof(Packet_t)] = '}';
}

Packet_Status_t string_to_packet(char *string_buffer, Packet_t *pkt) {
    if (string_buffer[0] != '{' || string_buffer[1 + sizeof(Packet_t)] != '}') {
        return PACKET_ERR;
    }
    memcpy(pkt, string_buffer + 1, sizeof(Packet_t));
    return PACKET_OK;
}
