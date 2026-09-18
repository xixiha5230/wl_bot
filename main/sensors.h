#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t raw_angle;
    float angle_degrees;
    float continuous_angle_degrees;
    float velocity_degrees_per_second;
} as5600_sample_t;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float angle_acc_x;
    float angle_acc_y;
    float angle_x;
    float angle_y;
} mpu6050_sample_t;

esp_err_t sensors_init(void);
/* Read both AS5600 encoders plus their continuous angle and velocity. This is a
 * diagnostics path (the console 'enc' command): the control loop uses the
 * SimpleFOC shaft angle/velocity for the balance and roll loops. */
esp_err_t sensors_read_encoders(as5600_sample_t *left, as5600_sample_t *right);
esp_err_t sensors_read_imu(mpu6050_sample_t *imu);

/* Re-measure the gyro zero-rate offsets. The robot must be perfectly still
 * while this runs (it samples for ~1 s). Used by the 'gcal' command. */
esp_err_t sensors_calibrate_gyro(void);

/* Nudge the gyro Z zero-rate offset by `residual` dps, for the slow runtime
 * auto-trim that cancels the MPU6050 temperature drift. Clamped internally. */
void sensors_trim_gyro_z(float residual);

/* Current gyro Z zero-rate offset (dps), for telemetry. */
float sensors_gyro_offset_z(void);

/* Read one AS5600 raw angle (index 0 = left bus, 1 = right bus). Used by the
 * FOC loop, hence the short I2C timeout. */
esp_err_t sensors_read_encoder(uint8_t index, uint16_t *raw_angle);

#ifdef __cplusplus
}
#endif
