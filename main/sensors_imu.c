#include "sensors.h"
#include "sensors_internal.h"

#include "board.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

#define MPU6050_ADDRESS         0x68
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A

static const char *TAG = "sensors";
static i2c_master_dev_handle_t imu;
static float gyro_offset_x;
static float gyro_offset_y;
static float gyro_offset_z;
static float angle_x;
static float angle_y;
static int64_t previous_update_us;

static esp_err_t read_mpu6050(mpu6050_sample_t *sample)
{
    uint8_t data[14];
    esp_err_t err = sensors_read_registers(imu, MPU6050_ACCEL_XOUT_H, data, sizeof(data),
                                           SENSORS_I2C_TIMEOUT_FAST_MS);
    if (err != ESP_OK) {
        static uint32_t fail_count;
        if (fail_count++ % 200 == 0) {
            ESP_LOGW(TAG, "MPU6050 read failed: %s (count %u)",
                     esp_err_to_name(err), (unsigned)fail_count);
        }
        return err;
    }
    sample->accel_x = (int16_t)((data[0] << 8) | data[1]);
    sample->accel_y = (int16_t)((data[2] << 8) | data[3]);
    sample->accel_z = (int16_t)((data[4] << 8) | data[5]);
    sample->gyro_x = (int16_t)((data[8] << 8) | data[9]);
    sample->gyro_y = (int16_t)((data[10] << 8) | data[11]);
    sample->gyro_z = (int16_t)((data[12] << 8) | data[13]);
    sample->accel_x_g = sample->accel_x / 16384.0f;
    sample->accel_y_g = sample->accel_y / 16384.0f;
    sample->accel_z_g = sample->accel_z / 16384.0f;
    sample->gyro_x_dps = sample->gyro_x / 65.5f;
    sample->gyro_y_dps = sample->gyro_y / 65.5f;
    sample->gyro_z_dps = sample->gyro_z / 65.5f;

    sample->gyro_x_dps -= gyro_offset_x;
    sample->gyro_y_dps -= gyro_offset_y;
    sample->gyro_z_dps -= gyro_offset_z;
    sample->angle_acc_x = atan2f(sample->accel_y_g,
                                  sample->accel_z_g + fabsf(sample->accel_x_g)) * 180.0f / (float)M_PI;
    sample->angle_acc_y = atan2f(sample->accel_x_g,
                                  sample->accel_z_g + fabsf(sample->accel_y_g)) * -180.0f / (float)M_PI;

    int64_t now_us = esp_timer_get_time();
    float interval = previous_update_us == 0 ? 0.0f :
                     (float)(now_us - previous_update_us) * 0.000001f;
    previous_update_us = now_us;
    angle_x = 0.98f * (angle_x + sample->gyro_x_dps * interval) + 0.02f * sample->angle_acc_x;
    angle_y = 0.98f * (angle_y + sample->gyro_y_dps * interval) + 0.02f * sample->angle_acc_y;
    sample->angle_x = angle_x;
    sample->angle_y = angle_y;
    return ESP_OK;
}

esp_err_t sensors_calibrate_gyro(void)
{
    uint8_t data[6];
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    for (int i = 0; i < 3000; ++i) {
        ESP_RETURN_ON_ERROR(sensors_read_registers(imu, MPU6050_GYRO_XOUT_H, data, sizeof(data),
                                                   SENSORS_I2C_TIMEOUT_FAST_MS),
                            TAG, "gyro calibration read failed");
        sum_x += (int16_t)((data[0] << 8) | data[1]) / 65.5f;
        sum_y += (int16_t)((data[2] << 8) | data[3]) / 65.5f;
        sum_z += (int16_t)((data[4] << 8) | data[5]) / 65.5f;
    }
    gyro_offset_x = sum_x / 3000.0f;
    gyro_offset_y = sum_y / 3000.0f;
    gyro_offset_z = sum_z / 3000.0f;
    ESP_LOGI(TAG, "gyro offsets: x=%.3f y=%.3f z=%.3f dps",
             gyro_offset_x, gyro_offset_y, gyro_offset_z);
    return ESP_OK;
}

void sensors_trim_gyro_z(float residual)
{
    if (!(residual > -10.0f && residual < 10.0f)) {
        return;   /* a wild value means the robot is moving, not a bias */
    }
    gyro_offset_z += residual;
    if (gyro_offset_z > 50.0f) {
        gyro_offset_z = 50.0f;
    } else if (gyro_offset_z < -50.0f) {
        gyro_offset_z = -50.0f;
    }
}

float sensors_gyro_offset_z(void)
{
    return gyro_offset_z;
}

static esp_err_t configure_mpu6050(void)
{
    ESP_RETURN_ON_ERROR(sensors_write_register(imu, MPU6050_PWR_MGMT_1, 0x01),
                        TAG, "MPU6050 wake failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(sensors_write_register(imu, MPU6050_SMPLRT_DIV, 0x00),
                        TAG, "MPU6050 sample rate config failed");
    ESP_RETURN_ON_ERROR(sensors_write_register(imu, MPU6050_CONFIG, 0x00),
                        TAG, "MPU6050 filter config failed");
    ESP_RETURN_ON_ERROR(sensors_write_register(imu, MPU6050_ACCEL_CONFIG, 0x00),
                        TAG, "MPU6050 accel config failed");
    ESP_RETURN_ON_ERROR(sensors_write_register(imu, MPU6050_GYRO_CONFIG, 0x08),
                        TAG, "MPU6050 gyro config failed");
    return ESP_OK;
}

esp_err_t sensors_imu_init(void)
{
    if (imu == NULL) {
        ESP_RETURN_ON_ERROR(sensors_add_device(board_imu_i2c_bus, MPU6050_ADDRESS, &imu),
                            TAG, "MPU6050 init failed");
    }

    /* The MPU6050 may not ACK immediately after power-up, and a soft reset can
     * leave the shared I2C bus busy. Retry the configuration with delays. */
    esp_err_t mpu_status = ESP_FAIL;
    for (int attempt = 0; attempt < 5 && mpu_status != ESP_OK; ++attempt) {
        vTaskDelay(pdMS_TO_TICKS(50));
        mpu_status = configure_mpu6050();
        if (mpu_status != ESP_OK) {
            ESP_LOGW(TAG, "MPU6050 config attempt %d failed: %s",
                     attempt + 1, esp_err_to_name(mpu_status));
        }
    }
    ESP_RETURN_ON_ERROR(mpu_status, TAG, "MPU6050 configuration failed");

    mpu6050_sample_t initial_sample = {0};
    ESP_RETURN_ON_ERROR(read_mpu6050(&initial_sample), TAG, "MPU6050 initial read failed");
    angle_x = initial_sample.angle_acc_x;
    angle_y = initial_sample.angle_acc_y;
    ESP_RETURN_ON_ERROR(sensors_calibrate_gyro(), TAG, "MPU6050 gyro calibration failed");
    previous_update_us = 0;
    return ESP_OK;
}

esp_err_t sensors_read_imu(mpu6050_sample_t *sample)
{
    if (sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return read_mpu6050(sample);
}
