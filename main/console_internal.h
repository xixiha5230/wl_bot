#pragma once

/*
 * Command groups registered by console.c.
 *
 * The serial console is split by domain so each file only needs the headers for
 * the hardware it drives: servo bus, FOC motors, robot control/tuning, or
 * sensor/rate diagnostics. console.c owns the REPL and the OTA command.
 */

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t console_servo_register(void);
esp_err_t console_motor_register(void);
esp_err_t console_control_register(void);
esp_err_t console_diag_register(void);

#ifdef __cplusplus
}
#endif
