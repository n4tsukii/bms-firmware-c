/* Super-loop scheduler: 10 ms BMS task, 100 ms CAN status task, TX queue drained every tick.
 *
 *   ./bms_sim [normal|overtemp|overvoltage|sensor] [duration_s] [can_log_path]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bms.h"
#include "bms_can.h"
#include "can.h"
#include "hal.h"

static const char *const FAULT_NAMES[BMS_FAULT_COUNT] = {"CELL_OV", "CELL_UV", "OVERTEMP", "OVERCURRENT", "SENSOR"};

static void print_faults(uint8_t faults)
{
    unsigned k;
    if (faults == 0u) {
        printf("none");
        return;
    }
    for (k = 0u; k < (unsigned)BMS_FAULT_COUNT; k++) {
        if (faults & BMS_FAULT_BIT(k)) {
            printf("%s ", FAULT_NAMES[k]);
        }
    }
}

static hal_scenario_t parse_scenario(const char *s)
{
    if (strcmp(s, "overtemp") == 0)    { return HAL_SCENARIO_OVERTEMP; }
    if (strcmp(s, "overvoltage") == 0) { return HAL_SCENARIO_OVERVOLTAGE; }
    if (strcmp(s, "sensor") == 0)      { return HAL_SCENARIO_SENSOR_FAIL; }
    return HAL_SCENARIO_NORMAL;
}

int main(int argc, char **argv)
{
    static bms_t bms;            /* static: no heap, no large stack frames */
    static bms_inputs_t in;
    static can_queue_t txq;
    const char *scn_name = (argc > 1) ? argv[1] : "normal";
    uint32_t duration_ms = (argc > 2) ? (uint32_t)(atof(argv[2]) * 1000.0) : 15000u;
    const char *log_path = (argc > 3) ? argv[3] : "bms_can.log";
    uint8_t alive = 0u;
    uint32_t overruns = 0u, frames = 0u;
    bms_state_t prev_state;

    hal_init(parse_scenario(scn_name), log_path);
    can_queue_init(&txq);
    hal_read_inputs(&in);
    bms_init(&bms, &in);
    prev_state = bms.state;
    printf("BMS sim | scenario=%s | initial SOC %u.%u %%\n", scn_name, bms.soc_dpct / 10u, bms.soc_dpct % 10u);

    while (hal_millis() < duration_ms) {
        /* 10 ms task */
        hal_read_inputs(&in);
        bms_step(&bms, &in);
        hal_set_contactor(bms.contactor_closed);

        if (bms.state != prev_state) {
            printf("[%7.3fs] %-8s -> %-8s  faults: ", (double)hal_millis() / 1000.0,
                   bms_state_name(prev_state), bms_state_name(bms.state));
            print_faults(bms.faults);
            printf(" | Tmax %.1f C, Vcell %u..%u mV, I %.1f A, SOC %u.%u %%\n",
                   (double)bms.max_temp_dc / 10.0, bms.min_cell_mv, bms.max_cell_mv,
                   (double)bms.current_ma / 1000.0, bms.soc_dpct / 10u, bms.soc_dpct % 10u);
            prev_state = bms.state;
        }

        /* 100 ms task */
        if ((hal_millis() % BMS_STATUS_PERIOD_MS) == 0u) {
            bms_status_msg_t msg;
            can_frame_t frame;
            bms_status_from_bms(&bms, alive, &msg);
            bms_status_encode(&msg, &frame);
            alive = (uint8_t)((alive + 1u) & 0x0Fu);
            if (!can_queue_push(&txq, &frame)) {
                overruns++;
            }
        }

        /* CAN TX: drain queue (on an MCU this runs from the TX-complete interrupt) */
        {
            can_frame_t out;
            while (can_queue_pop(&txq, &out)) {
                if (hal_can_send(&out)) {
                    frames++;
                }
            }
        }

        hal_wait_tick(BMS_TICK_MS);
    }

    printf("Done: %lu CAN frames sent, %lu queue overruns, final state %s, fault history: ",
           (unsigned long)frames, (unsigned long)overruns, bms_state_name(bms.state));
    print_faults(bms.fault_latch);
    printf("\nCAN log: %s (candump format)\n", log_path);
    hal_deinit();
    return 0;
}
