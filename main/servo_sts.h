#pragma once

#include "esp_err.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t position;
    int16_t speed;
    int16_t load;
    int16_t current;
    uint8_t voltage;
    uint8_t temperature;
    uint8_t status;
} servo_sts_feedback_t;

typedef struct {
    int16_t minimum;
    int16_t maximum;
} servo_sts_travel_t;

esp_err_t servo_sts_init(void);
esp_err_t servo_sts_read_feedback(uint8_t id, servo_sts_feedback_t *feedback);
esp_err_t servo_sts_sync_write_position(const uint8_t *ids, const int16_t *positions,
                                        size_t count, uint16_t speed, uint8_t acceleration);
esp_err_t servo_sts_set_torque(uint8_t id, bool enable);
esp_err_t servo_sts_calibrate_travel(uint8_t id, servo_sts_travel_t *travel);

#ifdef __cplusplus
}
#endif
