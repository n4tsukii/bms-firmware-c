#ifndef HAL_H
#define HAL_H

/* Hardware abstraction layer. The application only talks to hardware through these calls.
 * hal/hal_sim.c implements them on a PC. An MCU port would implement them with ADC/AFE
 * drivers (cell voltages), an NTC table (temperatures), a GPIO (contactor) and the CAN peripheral. */

#include <stdbool.h>
#include <stdint.h>

#include "bms.h"
#include "can.h"

typedef enum {
    HAL_SCENARIO_NORMAL = 0,
    HAL_SCENARIO_OVERTEMP,
    HAL_SCENARIO_OVERVOLTAGE,
    HAL_SCENARIO_SENSOR_FAIL
} hal_scenario_t;

void     hal_init(hal_scenario_t scenario, const char *can_log_path);
void     hal_deinit(void);
void     hal_read_inputs(bms_inputs_t *in);
void     hal_set_contactor(bool closed);
bool     hal_can_send(const can_frame_t *frame);
uint32_t hal_millis(void);
void     hal_wait_tick(uint32_t ms);   /* MCU: wait for SysTick flag. Sim: advance virtual time. */

#endif /* HAL_H */
