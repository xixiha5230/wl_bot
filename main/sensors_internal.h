#pragma once

/*
 * Private interface between the sensors translation units.
 *
 * sensors.c only orchestrates init; the MPU6050 (sensors_imu.c) and the two
 * AS5600 encoders (sensors_enc.c) each own their device handles and state, and
 * share the thin I2C register helpers in sensors_i2c.c.
 */

#include "driver/i2c_master.h"
#include "esp_err.h"

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Runtime reads must not stall the real-time tasks; the buses only run at
 * 400 kHz, so a few milliseconds is already far more than one transaction. */
#define SENSORS_I2C_TIMEOUT_MS      20
#define SENSORS_I2C_TIMEOUT_FAST_MS 10

esp_err_t sensors_add_device(i2c_master_bus_handle_t bus, uint8_t address,
                             i2c_master_dev_handle_t *device);
esp_err_t sensors_write_register(i2c_master_dev_handle_t device, uint8_t reg, uint8_t value);
esp_err_t sensors_read_registers(i2c_master_dev_handle_t device, uint8_t reg,
                                 uint8_t *data, size_t length, int timeout_ms);

/* Called by sensors_init(). */
esp_err_t sensors_imu_init(void);
esp_err_t sensors_enc_init(void);

#ifdef __cplusplus
}
#endif
