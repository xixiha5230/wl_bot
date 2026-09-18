#include "sensors.h"
#include "sensors_internal.h"

#include "board.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "robot_math.h"

#include <stdbool.h>

#define AS5600_ADDRESS        0x36
#define AS5600_RAW_ANGLE_REG  0x0C

static const char *TAG = "sensors";
static i2c_master_dev_handle_t left_encoder;
static i2c_master_dev_handle_t right_encoder;
static uint16_t previous_left_angle;
static uint16_t previous_right_angle;
static float left_continuous_angle;
static float right_continuous_angle;
static int64_t previous_encoder_update_us;
static bool encoder_history_valid;

static esp_err_t read_as5600(i2c_master_dev_handle_t device, as5600_sample_t *sample)
{
    uint8_t data[2];
    ESP_RETURN_ON_ERROR(sensors_read_registers(device, AS5600_RAW_ANGLE_REG, data, sizeof(data),
                                               SENSORS_I2C_TIMEOUT_MS),
                        TAG, "AS5600 read failed");
    sample->raw_angle = ((uint16_t)data[0] << 8 | data[1]) & 0x0FFF;
    sample->angle_degrees = sample->raw_angle * (360.0f / 4096.0f);
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
    int delta = robot_encoder_delta(*previous, sample->raw_angle);
    *continuous += delta * (360.0f / 4096.0f);
    float dt = previous_encoder_update_us == 0 ? 0.0f :
               (float)(now_us - previous_encoder_update_us) * 0.000001f;
    *velocity = dt > 0.0f ? delta * (360.0f / 4096.0f) / dt : 0.0f;
    *previous = sample->raw_angle;
}

esp_err_t sensors_enc_init(void)
{
    if (left_encoder == NULL) {
        ESP_RETURN_ON_ERROR(sensors_add_device(board_encoder_i2c_bus, AS5600_ADDRESS,
                                               &left_encoder),
                            TAG, "left AS5600 init failed");
    }
    if (right_encoder == NULL) {
        ESP_RETURN_ON_ERROR(sensors_add_device(board_imu_i2c_bus, AS5600_ADDRESS,
                                               &right_encoder),
                            TAG, "right AS5600 init failed");
    }
    previous_encoder_update_us = 0;
    encoder_history_valid = false;
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
