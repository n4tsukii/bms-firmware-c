#include "bms.h"

#include <stddef.h>

#define CAPACITY_UAS ((int64_t)BMS_CAPACITY_MAH * 3600LL * 1000LL)

/* Open-circuit-voltage -> SOC lookup table (simplified NMC curve). */
static const uint16_t OCV_MV[]   = {3000u, 3450u, 3600u, 3700u, 3800u, 3950u, 4100u, 4200u};
static const uint16_t OCV_DPCT[] = {   0u,   50u,  150u,  350u,  550u,  750u,  920u, 1000u};
#define OCV_POINTS (sizeof(OCV_MV) / sizeof(OCV_MV[0]))

uint16_t bms_soc_from_ocv(uint16_t cell_mv)
{
    size_t i;
    if (cell_mv <= OCV_MV[0]) {
        return OCV_DPCT[0];
    }
    if (cell_mv >= OCV_MV[OCV_POINTS - 1u]) {
        return OCV_DPCT[OCV_POINTS - 1u];
    }
    for (i = 1u; i < OCV_POINTS; i++) {
        if (cell_mv <= OCV_MV[i]) {
            uint32_t dv   = (uint32_t)cell_mv - OCV_MV[i - 1u];
            uint32_t span = (uint32_t)OCV_MV[i] - OCV_MV[i - 1u];
            uint32_t dsoc = (uint32_t)OCV_DPCT[i] - OCV_DPCT[i - 1u];
            return (uint16_t)(OCV_DPCT[i - 1u] + (dv * dsoc) / span);
        }
    }
    return OCV_DPCT[OCV_POINTS - 1u]; /* unreachable */
}

const char *bms_state_name(bms_state_t s)
{
    switch (s) {
    case BMS_STATE_INIT:     return "INIT";
    case BMS_STATE_STANDBY:  return "STANDBY";
    case BMS_STATE_DRIVE:    return "DRIVE";
    case BMS_STATE_CHARGING: return "CHARGING";
    case BMS_STATE_FAULT:    return "FAULT";
    default:                 return "?";
    }
}

static void aggregate(bms_t *bms, const bms_inputs_t *in, bool *sensor_bad)
{
    uint32_t sum = 0u;
    uint16_t mn = 0xFFFFu, mx = 0u;
    int16_t tmax = INT16_MIN;
    size_t i;

    *sensor_bad = false;
    for (i = 0u; i < BMS_NUM_CELLS; i++) {
        uint16_t v = in->cell_mv[i];
        if (v < BMS_CELL_MIN_PLAUSIBLE_MV || v > BMS_CELL_MAX_PLAUSIBLE_MV) {
            *sensor_bad = true;
        }
        sum += v;
        if (v < mn) { mn = v; }
        if (v > mx) { mx = v; }
    }
    for (i = 0u; i < BMS_NUM_TEMPS; i++) {
        int16_t t = in->temp_dc[i];
        if (t < BMS_TEMP_MIN_PLAUSIBLE_DC || t > BMS_TEMP_MAX_PLAUSIBLE_DC) {
            *sensor_bad = true;
        }
        if (t > tmax) { tmax = t; }
    }
    bms->min_cell_mv = mn;
    bms->max_cell_mv = mx;
    bms->pack_mv     = sum;
    bms->max_temp_dc = tmax;
    bms->current_ma  = in->current_ma;
}

/* Debounce one fault: `violated` uses the set threshold when inactive and the
 * clear threshold when active, giving hysteresis. Returns the new active flag. */
static bool debounce(bms_debounce_t *d, bool active, bool violated)
{
    if (violated) {
        d->clear_cnt = 0u;
        if (d->set_cnt < BMS_DEBOUNCE_SET) { d->set_cnt++; }
        if (d->set_cnt >= BMS_DEBOUNCE_SET) { active = true; }
    } else {
        d->set_cnt = 0u;
        if (d->clear_cnt < BMS_DEBOUNCE_CLEAR) { d->clear_cnt++; }
        if (d->clear_cnt >= BMS_DEBOUNCE_CLEAR) { active = false; }
    }
    return active;
}

static void evaluate_faults(bms_t *bms, bool sensor_bad)
{
    bool viol[BMS_FAULT_COUNT];
    int32_t abs_i = (bms->current_ma < 0) ? -bms->current_ma : bms->current_ma;
    size_t k;

#define ACTIVE(idx) ((bms->faults & BMS_FAULT_BIT(idx)) != 0u)
    viol[BMS_FAULT_IDX_CELL_OV] = ACTIVE(BMS_FAULT_IDX_CELL_OV)
        ? (bms->max_cell_mv > BMS_CELL_OV_CLEAR_MV) : (bms->max_cell_mv >= BMS_CELL_OV_SET_MV);
    viol[BMS_FAULT_IDX_CELL_UV] = ACTIVE(BMS_FAULT_IDX_CELL_UV)
        ? (bms->min_cell_mv < BMS_CELL_UV_CLEAR_MV) : (bms->min_cell_mv <= BMS_CELL_UV_SET_MV);
    viol[BMS_FAULT_IDX_OVERTEMP] = ACTIVE(BMS_FAULT_IDX_OVERTEMP)
        ? (bms->max_temp_dc > BMS_OT_CLEAR_DC) : (bms->max_temp_dc >= BMS_OT_SET_DC);
    viol[BMS_FAULT_IDX_OVERCURRENT] = ACTIVE(BMS_FAULT_IDX_OVERCURRENT)
        ? (abs_i > BMS_OC_CLEAR_MA) : (abs_i >= BMS_OC_SET_MA);
    viol[BMS_FAULT_IDX_SENSOR] = sensor_bad;

    for (k = 0u; k < (size_t)BMS_FAULT_COUNT; k++) {
        uint8_t bit = BMS_FAULT_BIT(k);
        bool now = debounce(&bms->debounce[k], (bms->faults & bit) != 0u, viol[k]);
        bms->faults = now ? (uint8_t)(bms->faults | bit) : (uint8_t)(bms->faults & (uint8_t)~bit);
    }
#undef ACTIVE
    /* Sensor faults are safety-relevant from the first sample. */
    if (sensor_bad) {
        bms->faults |= BMS_FAULT_BIT(BMS_FAULT_IDX_SENSOR);
    }
    bms->fault_latch |= bms->faults;
}

static void update_soc(bms_t *bms)
{
    /* mA * ms = uAs */
    bms->remaining_uas -= (int64_t)bms->current_ma * (int64_t)BMS_TICK_MS;
    if (bms->remaining_uas < 0) { bms->remaining_uas = 0; }
    if (bms->remaining_uas > CAPACITY_UAS) { bms->remaining_uas = CAPACITY_UAS; }
    bms->soc_dpct = (uint16_t)((bms->remaining_uas * 1000LL) / CAPACITY_UAS);
}

static void run_state_machine(bms_t *bms, const bms_inputs_t *in)
{
    if (bms->faults != 0u) {
        bms->state = BMS_STATE_FAULT;   /* from any state */
    }

    switch (bms->state) {
    case BMS_STATE_INIT:
        bms->state = BMS_STATE_STANDBY;  /* self-check passed */
        break;
    case BMS_STATE_STANDBY:
        if (in->charger_connected) {
            bms->state = BMS_STATE_CHARGING;
        } else if (in->ignition) {
            bms->state = BMS_STATE_DRIVE;
        }
        break;
    case BMS_STATE_DRIVE:
        if (!in->ignition) {
            bms->state = BMS_STATE_STANDBY;
        }
        break;
    case BMS_STATE_CHARGING:
        if (!in->charger_connected || bms->soc_dpct >= 1000u) {
            bms->state = BMS_STATE_STANDBY;
        }
        break;
    case BMS_STATE_FAULT:
        /* Latched: leave only when all faults are gone, ignition is off and a reset is requested. */
        if (bms->faults == 0u && !in->ignition && in->fault_reset) {
            bms->fault_latch = 0u;
            bms->state = BMS_STATE_STANDBY;
        }
        break;
    default:
        bms->state = BMS_STATE_FAULT;    /* defensive: corrupted state */
        break;
    }

    bms->contactor_closed = (bms->state == BMS_STATE_DRIVE) || (bms->state == BMS_STATE_CHARGING);
}

void bms_init(bms_t *bms, const bms_inputs_t *in)
{
    bool sensor_bad;
    size_t k;

    bms->state = BMS_STATE_INIT;
    bms->faults = 0u;
    bms->fault_latch = 0u;
    for (k = 0u; k < (size_t)BMS_FAULT_COUNT; k++) {
        bms->debounce[k].set_cnt = 0u;
        bms->debounce[k].clear_cnt = 0u;
    }
    bms->contactor_closed = false;
    bms->tick = 0u;

    aggregate(bms, in, &sensor_bad);
    /* Initial SOC from the open-circuit voltage of the weakest cell. */
    bms->soc_dpct = sensor_bad ? 0u : bms_soc_from_ocv(bms->min_cell_mv);
    bms->remaining_uas = (CAPACITY_UAS * bms->soc_dpct) / 1000LL;
}

void bms_step(bms_t *bms, const bms_inputs_t *in)
{
    bool sensor_bad;

    bms->tick++;
    aggregate(bms, in, &sensor_bad);
    evaluate_faults(bms, sensor_bad);
    update_soc(bms);
    run_state_machine(bms, in);
}
