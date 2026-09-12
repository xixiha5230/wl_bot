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
    float angle_z;
} mpu6050_sample_t;

esp_err_t sensors_init(void);
esp_err_t sensors_read_encoders(as5600_sample_t *left, as5600_sample_t *right);
esp_err_t sensors_read_imu(mpu6050_sample_t *imu);

/* Read one AS5600 raw angle (index 0 = left bus, 1 = right bus). Used by the
 * FOC loop, hence the short I2C timeout. */
esp_err_t sensors_read_encoder(uint8_t index, uint16_t *raw_angle);

#ifdef __cplusplus
}
#endif
