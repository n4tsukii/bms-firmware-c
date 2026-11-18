#ifndef BMS_H
#define BMS_H

#include <stdbool.h>
#include <stdint.h>

#include "bms_config.h"

/* Values match the BMS_State value table in the DBC. */
typedef enum {
    BMS_STATE_INIT     = 0,
    BMS_STATE_STANDBY  = 1,
    BMS_STATE_DRIVE    = 2,
    BMS_STATE_CHARGING = 3,
    BMS_STATE_FAULT    = 4
} bms_state_t;

typedef enum {
    BMS_FAULT_IDX_CELL_OV = 0,
    BMS_FAULT_IDX_CELL_UV,
    BMS_FAULT_IDX_OVERTEMP,
    BMS_FAULT_IDX_OVERCURRENT,
    BMS_FAULT_IDX_SENSOR,
    BMS_FAULT_COUNT
} bms_fault_idx_t;

#define BMS_FAULT_BIT(idx) ((uint8_t)(1u << (idx)))

/* Raw measurements, as delivered by the HAL every tick. */
typedef struct {
    uint16_t cell_mv[BMS_NUM_CELLS];
    int16_t  temp_dc[BMS_NUM_TEMPS];   /* deci-degC                          */
    int32_t  current_ma;               /* + discharge, - charge/regen        */
    bool     ignition;
    bool     charger_connected;
    bool     fault_reset;              /* request from VCU / tester          */
} bms_inputs_t;

typedef struct {
    uint8_t set_cnt;
    uint8_t clear_cnt;
} bms_debounce_t;

typedef struct {
    bms_state_t    state;
    uint8_t        faults;             /* debounced, currently active        */
    uint8_t        fault_latch;        /* every fault seen since last reset  */
    bms_debounce_t debounce[BMS_FAULT_COUNT];

    /* Derived values */
    uint16_t min_cell_mv;
    uint16_t max_cell_mv;
    uint32_t pack_mv;
    int16_t  max_temp_dc;
    int32_t  current_ma;
    bool     contactor_closed;

    /* State of charge (coulomb counting) */
    int64_t  remaining_uas;            /* micro-ampere-seconds               */
    uint16_t soc_dpct;                 /* 0.1 % units, 0..1000               */

    uint32_t tick;
} bms_t;

void        bms_init(bms_t *bms, const bms_inputs_t *in);
void        bms_step(bms_t *bms, const bms_inputs_t *in);
uint16_t    bms_soc_from_ocv(uint16_t cell_mv);
const char *bms_state_name(bms_state_t s);

#endif /* BMS_H */
