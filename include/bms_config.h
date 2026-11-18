#ifndef BMS_CONFIG_H
#define BMS_CONFIG_H

/* ---- Pack configuration ---- */
#define BMS_NUM_CELLS            96u      /* 96s pack, ~400 V nominal            */
#define BMS_NUM_TEMPS            8u       /* temperature sensors                 */
#define BMS_CAPACITY_MAH         100000u  /* 100 Ah                              */

/* ---- Scheduling ---- */
#define BMS_TICK_MS              10u      /* bms_step() period                   */
#define BMS_STATUS_PERIOD_MS     100u     /* BMS_Status CAN frame period         */

/* ---- Protection thresholds (with hysteresis) ---- */
#define BMS_CELL_OV_SET_MV       4250u
#define BMS_CELL_OV_CLEAR_MV     4150u
#define BMS_CELL_UV_SET_MV       2800u
#define BMS_CELL_UV_CLEAR_MV     3000u
#define BMS_OT_SET_DC            600      /* 60.0 degC (deci-degC)               */
#define BMS_OT_CLEAR_DC          550      /* 55.0 degC                           */
#define BMS_OC_SET_MA            300000L  /* 300 A                               */
#define BMS_OC_CLEAR_MA          250000L

/* Sensor plausibility window */
#define BMS_CELL_MIN_PLAUSIBLE_MV 1000u
#define BMS_CELL_MAX_PLAUSIBLE_MV 5000u
#define BMS_TEMP_MIN_PLAUSIBLE_DC (-400)
#define BMS_TEMP_MAX_PLAUSIBLE_DC 1250

/* ---- Debounce (in ticks) ---- */
#define BMS_DEBOUNCE_SET         3u       /* 30 ms of consecutive violation      */
#define BMS_DEBOUNCE_CLEAR       10u      /* 100 ms of consecutive recovery      */

/* ---- CAN ---- */
#define BMS_CAN_ID_STATUS        0x100u
#define CAN_TX_QUEUE_SIZE        16u      /* must be a power of two              */

#endif /* BMS_CONFIG_H */
