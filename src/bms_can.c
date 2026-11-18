#include "bms_can.h"

static uint32_t clamp_u(int32_t v, uint32_t max)
{
    if (v < 0) { return 0u; }
    if ((uint32_t)v > max) { return max; }
    return (uint32_t)v;
}

void bms_status_from_bms(const bms_t *bms, uint8_t alive_counter, bms_status_msg_t *msg)
{
    msg->soc_dpct      = bms->soc_dpct;
    msg->pack_mv       = bms->pack_mv;
    msg->current_ma    = bms->current_ma;
    msg->max_temp_dc   = bms->max_temp_dc;
    msg->state         = bms->state;
    msg->alive_counter = (uint8_t)(alive_counter & 0x0Fu);
}

void bms_status_encode(const bms_status_msg_t *msg, can_frame_t *frame)
{
    int32_t current_raw = msg->current_ma / 100;                 /* 0.1 A */
    int32_t temp_raw    = msg->max_temp_dc / 10 + 40;            /* 1 degC, offset -40 */
    uint8_t i;

    if (current_raw > 32767) { current_raw = 32767; }
    if (current_raw < -32768) { current_raw = -32768; }

    frame->id  = BMS_CAN_ID_STATUS;
    frame->dlc = 8u;
    for (i = 0u; i < 8u; i++) {
        frame->data[i] = 0u;
    }
    can_pack_le(frame->data, 0u, 8u, clamp_u((int32_t)(msg->soc_dpct / 5u), 200u));
    can_pack_le(frame->data, 8u, 16u, clamp_u((int32_t)(msg->pack_mv / 100u), 0xFFFFu));
    can_pack_le(frame->data, 24u, 16u, (uint32_t)current_raw & 0xFFFFu);
    can_pack_le(frame->data, 40u, 8u, clamp_u(temp_raw, 0xFFu));
    can_pack_le(frame->data, 48u, 4u, (uint32_t)msg->state);
    can_pack_le(frame->data, 52u, 4u, msg->alive_counter);
    frame->data[7] = crc8_j1850(frame->data, 7u);
}

bool bms_status_decode(const can_frame_t *frame, bms_status_msg_t *msg)
{
    if (frame->id != BMS_CAN_ID_STATUS || frame->dlc != 8u ||
        crc8_j1850(frame->data, 7u) != frame->data[7]) {
        return false;
    }
    msg->soc_dpct      = (uint16_t)(can_unpack_le(frame->data, 0u, 8u) * 5u);
    msg->pack_mv       = can_unpack_le(frame->data, 8u, 16u) * 100u;
    msg->current_ma    = can_sign_extend(can_unpack_le(frame->data, 24u, 16u), 16u) * 100;
    msg->max_temp_dc   = (int16_t)(((int32_t)can_unpack_le(frame->data, 40u, 8u) - 40) * 10);
    msg->state         = (bms_state_t)can_unpack_le(frame->data, 48u, 4u);
    msg->alive_counter = (uint8_t)can_unpack_le(frame->data, 52u, 4u);
    return true;
}
