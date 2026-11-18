#ifndef CAN_H
#define CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "bms_config.h"

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} can_frame_t;

/* Intel (little-endian) signal packing, DBC convention: start_bit = LSB position. */
void     can_pack_le(uint8_t data[8], uint8_t start_bit, uint8_t length, uint32_t raw);
uint32_t can_unpack_le(const uint8_t data[8], uint8_t start_bit, uint8_t length);
int32_t  can_sign_extend(uint32_t raw, uint8_t length);

/* CRC-8 SAE J1850: poly 0x1D, init 0xFF, xorout 0xFF. */
uint8_t  crc8_j1850(const uint8_t *data, uint32_t len);

/* Single-producer / single-consumer ring buffer for the CAN TX path.
 * Free-running indices + power-of-two size: all slots are usable. */
typedef struct {
    can_frame_t      buf[CAN_TX_QUEUE_SIZE];
    volatile uint16_t head;   /* written by producer */
    volatile uint16_t tail;   /* written by consumer */
} can_queue_t;

void     can_queue_init(can_queue_t *q);
bool     can_queue_push(can_queue_t *q, const can_frame_t *f);
bool     can_queue_pop(can_queue_t *q, can_frame_t *out);
uint16_t can_queue_count(const can_queue_t *q);

#endif /* CAN_H */
