/*
 * circular_buffer.h
 *
 *  Created on: Mar 20, 2026
 *      Author: gamin
 */

#ifndef INC_CIRCULAR_BUFFER_H_
#define INC_CIRCULAR_BUFFER_H_

#include <stdint.h>
#include <string.h>

// flight stage
typedef enum{
	NOT_LAUNCHED = 0,
	LAUNCHED,
	DROGUE_F,
	MAIN_F,
	POST_APOGEE,
	LANDED
} FlightStage_t;




// 100 sample analysis circular buffer
// keep most recent 100 samples to help decide logic
// get(0) will get newest; get (n) will get n samples ago.
#define CIRC_BUF_SIZE 100

typedef struct{
	float data[CIRC_BUF_SIZE];
	uint16_t head;
	uint16_t count;
} CircBuf_t;

// just leatned what inline does :o
// elimnates the fucntion call overhead, good for buffer fuinctions

// init func
static inline void CB_Init(CircBuf_t *b){
	b->head = 0;
	b->count = 0;
}

// push func
// writes at head, wraps atound, and wiull overwrite data as it goes
static inline void CB_Push(CircBuf_t *b, float d){
	b->head = (b->head + 1) % CIRC_BUF_SIZE;
	b->data[b->head] = d;
	if (b->count < CIRC_BUF_SIZE){
		b->count++;
	}
}

// get stuff from n away from head
static inline float CB_Get(CircBuf_t *b, uint16_t n){
    return b->data[(b->head + CIRC_BUF_SIZE - (n % CIRC_BUF_SIZE)) % CIRC_BUF_SIZE];
}


// just count
static inline uint16_t CB_Length(CircBuf_t *b){
	return b->count;
}

// 1 for full and 0 for not
static inline uint8_t CB_Full(CircBuf_t *b){
	return b->count == CIRC_BUF_SIZE;
}


// AI told me to add raw functionality as well as a debugging tool and specialized occasions
static inline void CB_SetRaw(CircBuf_t *b, uint16_t i, float d){
	b->data[i % CIRC_BUF_SIZE] = d;
}
static inline float CB_GetRaw(CircBuf_t *b, uint16_t i){
	return b->data[i % CIRC_BUF_SIZE];
}





// 1000 sample FIFO buffer used for logging and such :p
// push is to add new; pop is for removing oldest

#define LOG_BUF_SIZE 1000

// for floats
typedef struct{
	float data[LOG_BUF_SIZE];
	uint16_t head;
	uint16_t tail;
	uint16_t count;
} LogFloatBuf_t;

// init func
static inline void LFB_Init(LogFloatBuf_t *b){
	b->head = 0;
	b->tail = 0;
	b->count = 0;
}

// push func
// writes to head, wraps atound with mod, and overwrites oldest if full or sumn
static inline void LFB_Push(LogFloatBuf_t *b, float d){
    b->data[b->head] = d;
    b->head = (b->head + 1) % LOG_BUF_SIZE;
    if (b->count < LOG_BUF_SIZE) {
        b->count++;
    } else {
    	// overwrite oldest
        b->tail = (b->tail + 1) % LOG_BUF_SIZE;
    }
}


// pop func
// get oldest, move tail upsies, and upadate count
static inline float LFB_Pop(LogFloatBuf_t *b){
    if (b->count == 0){
    	return 0.0f;
    }
    float d  = b->data[b->tail];
    b->tail  = (b->tail + 1) % LOG_BUF_SIZE;
    b->count--;
    return d;
}


// 1 if can pop or 0 is emptuyy
static inline uint8_t LFB_Available(LogFloatBuf_t *b){
	return b->count > 0;
}





// uint32 for timestamps
typedef struct {
    uint32_t data[LOG_BUF_SIZE];
    uint16_t head;
	uint16_t tail;
	uint16_t count;
} LogU32Buf_t;

// init
static inline void LU32_Init(LogU32Buf_t *b){
	b->head = b->tail = b->count = 0;
}

// push like prev
static inline void LU32_Push(LogU32Buf_t *b, uint32_t d){
    b->data[b->head] = d;
    b->head = (b->head + 1) % LOG_BUF_SIZE;
    if (b->count < LOG_BUF_SIZE) {
    	b->count++;
    }
    else {
    	b->tail = (b->tail + 1) % LOG_BUF_SIZE;
    }
}

// pop like prev
static inline uint32_t LU32_Pop(LogU32Buf_t *b){
    if (b->count == 0){
    	return 0;
    }
    uint32_t d = b->data[b->tail];
    b->tail = (b->tail + 1) % LOG_BUF_SIZE;
    b->count--;
    return d;
}

// 1 is can pop or 0 if emprty
static inline uint8_t LU32_Available(LogU32Buf_t *b){
	return b->count > 0;
}





// flight stage fifo
typedef struct {
    FlightStage_t data[LOG_BUF_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} LogStageBuf_t;

// init
static inline void LSB_Init(LogStageBuf_t *b){
	b->head = 0;
	b->tail = 0;
	b->count = 0;
}

// push like prev
static inline void LSB_Push(LogStageBuf_t *b, FlightStage_t d) {
    b->data[b->head] = d;
    b->head = (b->head + 1) % LOG_BUF_SIZE;
    if (b->count < LOG_BUF_SIZE){
    	b->count++;
    }
    else {
    	b->tail = (b->tail + 1) % LOG_BUF_SIZE;
    }
}

// pop like prev
static inline FlightStage_t LSB_Pop(LogStageBuf_t *b) {
    if (b->count == 0){
    	return NOT_LAUNCHED;
    }
    FlightStage_t v = b->data[b->tail];
    b->tail = (b->tail + 1) % LOG_BUF_SIZE;
    b->count--;
    return v;
}

// 1 is can pop or 0 if emprty
static inline uint8_t LSB_Available(LogStageBuf_t *b) {
	return b->count > 0;
}






#endif /* INC_CIRCULAR_BUFFER_H_ */
