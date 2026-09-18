#pragma once

/*
 * Private interface between the board translation units.
 *
 * board.c owns the I2C buses and the servo UART; battery.c owns the battery
 * ADC and indicator LED. board_init() calls battery_init() at the right point
 * in the bring-up order.
 */

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t battery_init(void);

#ifdef __cplusplus
}
#endif
