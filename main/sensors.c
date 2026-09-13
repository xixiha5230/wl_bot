#include "sensors.h"

#include "board.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#define AS5600_ADDRESS          0x36
#define AS5600_RAW_ANGLE_REG   0x0C
#define MPU6050_ADDRESS         0x68
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A

/* Runtime reads must not stall the real-time tasks; the buses only run at
 * 400 kHz, so a few milliseconds is already far more than one transaction. */
#define SENSORS_I2C_TIMEOUT_MS      20
#define SENSORS_I2C_TIMEOUT_FAST_MS 10

static const char *TAG = "sensors";
static i2c_master_dev_handle_t left_encoder;
static i2c_master_dev_handle_t right_encoder;
static i2c_master_dev_handle_t imu;
static float gyro_offset_x;
static float gyro_offset_y;
static float gyro_offset_z;
static float angle_gyro_z;
static float angle_x;
static float angle_y;
static int64_t previous_update_us;
static uint16_t previous_left_angle;
static uint16_t previous_right_angle;
static float left_continuous_angle;
static float right_continuous_angle;
static int64_t previous_encoder_update_us;
static bool encoder_history_valid;

static esp_err_t add_device(i2c_master_bus_handle_t bus, uint8_t address,
                            i2c_master_dev_handle_t *device)
{
    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 400000,
    };
    return i2c_master_bus_add_device(bus, &config, device);
}

static esp_err_t write_register(i2c_master_dev_handle_t device, uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    return i2c_master_transmit(device, data, sizeof(data), 100);
}

static esp_err_t read_registers(i2c_master_dev_handle_t device, uint8_t reg,
                                uint8_t *data, size_t length, int timeout_ms)
{
    return i2c_master_transmit_receive(device, &reg, 1, data, length, timeout_ms);
}

static esp_err_t read_as5600(i2c_master_dev_handle_t device, as5600_sample_t *sample)
{
    uint8_t data[2];
    ESP_RETURN_ON_ERROR(read_registers(device, AS5600_RAW_ANGLE_REG, data, sizeof(data),
                                       SENSORS_I2C_TIMEOUT_MS),
                        TAG, "AS5600 read failed");
    sample->raw_angle = ((uint16_t)data[0] << 8 | data[1]) & 0x0FFF;
    sample->angle_degrees = sample->raw_angle * (360.0f / 4096.0f);
    return ESP_OK;
}

static esp_err_t read_mpu6050(mpu6050_sample_t *sample)
{
    uint8_t data[14];
    esp_err_t err = read_registers(imu, MPU6050_ACCEL_XOUT_H, data, sizeof(data),
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
    angle_gyro_z += sample->gyro_z_dps * interval;
    angle_x = 0.98f * (angle_x + sample->gyro_x_dps * interval) + 0.02f * sample->angle_acc_x;
    angle_y = 0.98f * (angle_y + sample->gyro_y_dps * interval) + 0.02f * sample->angle_acc_y;
    sample->angle_x = angle_x;
    sample->angle_y = angle_y;
    sample->angle_z = angle_gyro_z;
    return ESP_OK;
}

esp_err_t sensors_calibrate_gyro(void)
{
    uint8_t data[6];
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    for (int i = 0; i < 3000; ++i) {
        ESP_RETURN_ON_ERROR(read_registers(imu, MPU6050_GYRO_XOUT_H, data, sizeof(data),
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

static esp_err_t configure_mpu6050(void)
{
    ESP_RETURN_ON_ERROR(write_register(imu, MPU6050_PWR_MGMT_1, 0x01),
                        TAG, "MPU6050 wake failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(write_register(imu, MPU6050_SMPLRT_DIV, 0x00),
                        TAG, "MPU6050 sample rate config failed");
    ESP_RETURN_ON_ERROR(write_register(imu, MPU6050_CONFIG, 0x00),
                        TAG, "MPU6050 filter config failed");
    ESP_RETURN_ON_ERROR(write_register(imu, MPU6050_ACCEL_CONFIG, 0x00),
                        TAG, "MPU6050 accel config failed");
    ESP_RETURN_ON_ERROR(write_register(imu, MPU6050_GYRO_CONFIG, 0x08),
                        TAG, "MPU6050 gyro config failed");
    return ESP_OK;
}

static void update_encoder_continuous(as5600_sample_t *sample, uint16_t *previous,
                                      float *continuous, float *velocity, int64_t now_us)
{
    if (!encoder_history_valid) {
        *previous = sample->raw_angle;
        *velocity = 0.0f;
        return;
    }
    int delta = (int)sample->raw_angle - (int)*previous;
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    *continuous += delta * (360.0f / 4096.0f);
    float dt = previous_encoder_update_us == 0 ? 0.0f :
               (float)(now_us - previous_encoder_update_us) * 0.000001f;
    *velocity = dt > 0.0f ? delta * (360.0f / 4096.0f) / dt : 0.0f;
    *previous = sample->raw_angle;
}

esp_err_t sensors_init(void)
{
    if (left_encoder == NULL) {
        ESP_RETURN_ON_ERROR(add_device(board_encoder_i2c_bus, AS5600_ADDRESS, &left_encoder),
                            TAG, "left AS5600 init failed");
    }
    if (right_encoder == NULL) {
        ESP_RETURN_ON_ERROR(add_device(board_imu_i2c_bus, AS5600_ADDRESS, &right_encoder),
                            TAG, "right AS5600 init failed");
    }
    if (imu == NULL) {
        ESP_RETURN_ON_ERROR(add_device(board_imu_i2c_bus, MPU6050_ADDRESS, &imu),
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
    angle_gyro_z = 0.0f;
    ESP_RETURN_ON_ERROR(sensors_calibrate_gyro(), TAG, "MPU6050 gyro calibration failed");
    previous_update_us = 0;
    previous_encoder_update_us = 0;
    encoder_history_valid = false;
    ESP_LOGI(TAG, "AS5600 x2 and MPU6050 initialized");
    return ESP_OK;
}

esp_err_t sensors_read_encoder(uint8_t index, uint16_t *raw_angle)
{
    if (raw_angle == NULL || index > 1) {
        return ESP_ERR_INVALID_ARG;
    }
    i2c_master_dev_handle_t device = index == 0 ? left_encoder : right_encoder;
    if (device == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t reg = AS5600_RAW_ANGLE_REG;
    uint8_t data[2];
    esp_err_t err = i2c_master_transmit_receive(device, &reg, 1, data, sizeof(data),
                                                SENSORS_I2C_TIMEOUT_FAST_MS);
    if (err != ESP_OK) {
        static uint32_t fail_count;
        if (fail_count++ % 200 == 0) {
            ESP_LOGW(TAG, "encoder %u read failed: %s (count %u)",
                     index, esp_err_to_name(err), (unsigned)fail_count);
        }
        return err;
    }
    *raw_angle = ((uint16_t)data[0] << 8 | data[1]) & 0x0FFF;
    return ESP_OK;
}

esp_err_t sensors_read_encoders(as5600_sample_t *left, as5600_sample_t *right)
{
    ESP_RETURN_ON_ERROR(read_as5600(left_encoder, left), TAG, "left AS5600 sample failed");
    ESP_RETURN_ON_ERROR(read_as5600(right_encoder, right), TAG, "right AS5600 sample failed");
    int64_t now_us = esp_timer_get_time();
    update_encoder_continuous(left, &previous_left_angle, &left_continuous_angle,
                              &left->velocity_degrees_per_second, now_us);
    update_encoder_continuous(right, &previous_right_angle, &right_continuous_angle,
                              &right->velocity_degrees_per_second, now_us);
    left->continuous_angle_degrees = left_continuous_angle;
    right->continuous_angle_degrees = right_continuous_angle;
    previous_encoder_update_us = now_us;
    encoder_history_valid = true;
    return ESP_OK;
}

esp_err_t sensors_read_imu(mpu6050_sample_t *sample)
{
    if (sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return read_mpu6050(sample);
}
