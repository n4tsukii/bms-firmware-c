#include "can.h"

#include "bms_config.h"

#define QUEUE_LEN ((uint16_t)CAN_TX_QUEUE_SIZE)

/* Compile-time check (C99-compatible static assert) */
typedef char assert_queue_size_power_of_two[((QUEUE_LEN & (QUEUE_LEN - 1u)) == 0u) ? 1 : -1];

void can_pack_le(uint8_t data[8], uint8_t start_bit, uint8_t length, uint32_t raw)
{
    uint8_t i;
    for (i = 0u; i < length; i++) {
        uint8_t bit = (uint8_t)(start_bit + i);
        uint8_t byte = (uint8_t)(bit >> 3);
        uint8_t mask = (uint8_t)(1u << (bit & 7u));
        if (byte >= 8u) {
            return;
        }
        if ((raw >> i) & 1u) {
            data[byte] = (uint8_t)(data[byte] | mask);
        } else {
            data[byte] = (uint8_t)(data[byte] & (uint8_t)~mask);
        }
    }
}

uint32_t can_unpack_le(const uint8_t data[8], uint8_t start_bit, uint8_t length)
{
    uint32_t raw = 0u;
    uint8_t i;
    for (i = 0u; i < length; i++) {
        uint8_t bit = (uint8_t)(start_bit + i);
        if ((bit >> 3) >= 8u) {
            break;
        }
        if (((uint32_t)data[bit >> 3] >> (bit & 7u)) & 1u) {
            raw |= (uint32_t)(1u << i);
        }
    }
    return raw;
}

int32_t can_sign_extend(uint32_t raw, uint8_t length)
{
    uint32_t sign = (uint32_t)1u << (length - 1u);
    return (int32_t)((raw ^ sign) - sign);
}

uint8_t crc8_j1850(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0xFFu;
    uint32_t i;
    uint8_t b;
    for (i = 0u; i < len; i++) {
        crc = (uint8_t)(crc ^ data[i]);
        for (b = 0u; b < 8u; b++) {
            crc = (uint8_t)((crc & 0x80u) ? (((unsigned)crc << 1) ^ 0x1Du) : ((unsigned)crc << 1));
        }
    }
    return (uint8_t)(crc ^ 0xFFu);
}

void can_queue_init(can_queue_t *q)
{
    q->head = 0u;
    q->tail = 0u;
}

uint16_t can_queue_count(const can_queue_t *q)
{
    return (uint16_t)(q->head - q->tail);
}

bool can_queue_push(can_queue_t *q, const can_frame_t *f)
{
    uint16_t head = q->head;
    if ((uint16_t)(head - q->tail) >= QUEUE_LEN) {
        return false;                                /* full: caller counts the overrun */
    }
    q->buf[head & (QUEUE_LEN - 1u)] = *f;
    /* On a real MCU a memory barrier (e.g. __DMB()) goes here before publishing head. */
    q->head = (uint16_t)(head + 1u);
    return true;
}

bool can_queue_pop(can_queue_t *q, can_frame_t *out)
{
    uint16_t tail = q->tail;
    if (tail == q->head) {
        return false;
    }
    *out = q->buf[tail & (QUEUE_LEN - 1u)];
    q->tail = (uint16_t)(tail + 1u);
    return true;
}
