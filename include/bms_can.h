#ifndef BMS_CAN_H
#define BMS_CAN_H

#include "bms.h"
#include "can.h"

/* Physical values of the BMS_Status frame (CAN ID 0x100), same layout as ev_network.dbc:
 *   SOC          0|8   0.5 %/bit         PackVoltage   8|16  0.1 V/bit
 *   PackCurrent 24|16  0.1 A/bit signed  MaxCellTemp  40|8   1 degC, offset -40
 *   BMS_State   48|4                     AliveCounter 52|4   Checksum 56|8 (CRC8 J1850, bytes 0..6) */
typedef struct {
    uint16_t    soc_dpct;
    uint32_t    pack_mv;
    int32_t     current_ma;
    int16_t     max_temp_dc;
    bms_state_t state;
    uint8_t     alive_counter;
} bms_status_msg_t;

void bms_status_from_bms(const bms_t *bms, uint8_t alive_counter, bms_status_msg_t *msg);
void bms_status_encode(const bms_status_msg_t *msg, can_frame_t *frame);
bool bms_status_decode(const can_frame_t *frame, bms_status_msg_t *msg);  /* false if CRC/ID/DLC bad */

#endif /* BMS_CAN_H */
