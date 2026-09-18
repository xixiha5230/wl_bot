#include "sensors.h"
#include "sensors_internal.h"

#include "esp_check.h"
#include "esp_log.h"

/*
 * Sensor init orchestration. The MPU6050 driver lives in sensors_imu.c and the
 * AS5600 drivers in sensors_enc.c; this file only brings them up in order.
 */

static const char *TAG = "sensors";

esp_err_t sensors_init(void)
{
    ESP_RETURN_ON_ERROR(sensors_enc_init(), TAG, "encoder init failed");
    ESP_RETURN_ON_ERROR(sensors_imu_init(), TAG, "IMU init failed");
    ESP_LOGI(TAG, "AS5600 x2 and MPU6050 initialized");
    return ESP_OK;
}
