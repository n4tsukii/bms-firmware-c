/* PC simulation of the battery pack ("plant model"). Only this file uses floating point;
 * the firmware in src/ is integer-only. CAN frames are written in candump log format. */
#include "hal.h"

#include <stdio.h>

#define DT_S 0.010

static hal_scenario_t g_scn;
static FILE *g_log;
static uint32_t g_ms;
static bool g_contactor;
static double g_soc;        /* 0..1   */
static double g_temp;       /* degC   */
static double g_current;    /* A      */
static double g_cell42_drift_mv;

static const double OCV_MV[]  = {3000, 3450, 3600, 3700, 3800, 3950, 4100, 4200};
static const double OCV_SOC[] = {0.00, 0.05, 0.15, 0.35, 0.55, 0.75, 0.92, 1.00};

static double ocv_from_soc(double soc)
{
    unsigned i;
    if (soc <= 0.0) { return OCV_MV[0]; }
    for (i = 1u; i < 8u; i++) {
        if (soc <= OCV_SOC[i]) {
            double f = (soc - OCV_SOC[i - 1u]) / (OCV_SOC[i] - OCV_SOC[i - 1u]);
            return OCV_MV[i - 1u] + f * (OCV_MV[i] - OCV_MV[i - 1u]);
        }
    }
    return OCV_MV[7];
}

static double t_s(void) { return (double)g_ms / 1000.0; }

void hal_init(hal_scenario_t scenario, const char *can_log_path)
{
    g_scn = scenario;
    g_ms = 0u;
    g_contactor = false;
    g_soc = (scenario == HAL_SCENARIO_OVERVOLTAGE) ? 0.90 : 0.80;
    g_temp = 30.0;
    g_current = 0.0;
    g_cell42_drift_mv = 0.0;
    g_log = can_log_path ? fopen(can_log_path, "w") : NULL;
}

void hal_deinit(void)
{
    if (g_log) { fclose(g_log); g_log = NULL; }
}

static void plant_update(void)
{
    double t = t_s();

    /* Current demand */
    if (!g_contactor) {
        g_current = 0.0;
    } else if (g_scn == HAL_SCENARIO_OVERVOLTAGE) {
        g_current = -100.0;                              /* charging at 100 A */
    } else {
        g_current = (t >= 2.0 && t < 4.0) ? 150.0 : 80.0;
    }

    g_soc -= g_current * DT_S / (100.0 * 3600.0);

    /* Thermal model: I^2R heating + cooling to 25 degC ambient */
    g_temp += (g_current * g_current * 3e-6 - (g_temp - 25.0) * 0.02) * DT_S;
    if (g_scn == HAL_SCENARIO_OVERTEMP && t >= 2.0) {
        g_temp += (g_contactor ? 6.0 : -3.0) * DT_S;     /* cooling pump failure, recovers once open */
        if (g_temp < 30.0) { g_temp = 30.0; }
    }

    /* One weak cell drifts up while charging */
    if (g_scn == HAL_SCENARIO_OVERVOLTAGE && g_contactor) {
        g_cell42_drift_mv += 40.0 * DT_S;
    }
}

void hal_read_inputs(bms_inputs_t *in)
{
    double t = t_s();
    double v_cell = ocv_from_soc(g_soc) - g_current * 0.4;   /* 0.4 mOhm per cell */
    unsigned i;

    plant_update();

    for (i = 0u; i < BMS_NUM_CELLS; i++) {
        double v = v_cell + (double)((int)(i % 7u) - 3);      /* small cell-to-cell spread */
        if (i == 42u) { v += g_cell42_drift_mv; }
        in->cell_mv[i] = (uint16_t)(v + 0.5);
    }
    for (i = 0u; i < BMS_NUM_TEMPS; i++) {
        in->temp_dc[i] = (int16_t)((g_temp - (double)i * 0.3) * 10.0);
    }
    if (g_scn == HAL_SCENARIO_SENSOR_FAIL && t >= 3.0) {
        in->temp_dc[5] = -500;                               /* NTC open circuit */
    }
    in->current_ma = (int32_t)(g_current * 1000.0);

    in->charger_connected = (g_scn == HAL_SCENARIO_OVERVOLTAGE) && t >= 0.5;
    in->ignition = (g_scn != HAL_SCENARIO_OVERVOLTAGE) && t >= 0.5 && t < 12.0;
    in->fault_reset = (t >= 13.0 && t < 13.05);
}

void hal_set_contactor(bool closed) { g_contactor = closed; }

bool hal_can_send(const can_frame_t *f)
{
    uint8_t i;
    if (g_log) {
        fprintf(g_log, "(%.6f) vcan0 %03X#", t_s(), (unsigned)f->id);
        for (i = 0u; i < f->dlc; i++) {
            fprintf(g_log, "%02X", f->data[i]);
        }
        fputc('\n', g_log);
    }
    return true;
}

uint32_t hal_millis(void) { return g_ms; }

void hal_wait_tick(uint32_t ms) { g_ms += ms; }
