#include "robot_control_internal.h"

#include "esp_log.h"
#include "nvs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * Non-volatile persistence for the robot_control module.
 *
 * The only setting worth surviving a reboot is the roll level calibration: it
 * is measured against the physical mounting, so re-deriving it on every boot
 * would make the robot stand at a different angle each time. Everything else is
 * deliberately ephemeral and re-initialised from robot_config.h defaults.
 *
 * These functions run on the console / HTTP tasks, never on the control task.
 */

#define NVS_NS             "wlbot"
#define NVS_KEY_ROLL_BIAS  "roll_bias"

void roll_bias_save(void)
{
    if (!shared_ready()) return;
    nvs_handle_t handle;
    if (nvs_open(NVS_NS, NVS_READWRITE, &handle) == ESP_OK) {
        shared_lock();
        float bias = shared.roll_bias;
        shared_unlock();
        nvs_set_blob(handle, NVS_KEY_ROLL_BIAS, &bias, sizeof(bias));
        nvs_commit(handle);
        nvs_close(handle);
    }
}

void roll_bias_load(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NS, NVS_READONLY, &handle) != ESP_OK) {
        return;
    }
    float value = 0.0f;
    size_t length = sizeof(value);
    if (nvs_get_blob(handle, NVS_KEY_ROLL_BIAS, &value, &length) == ESP_OK &&
        length == sizeof(value) && value > -10.0f && value < 10.0f) {
        shared_lock();
        shared.roll_bias = value;
        shared_unlock();
    }
    nvs_close(handle);
}

float robot_control_calibrate_level(void)
{
    if (!shared_ready()) {
        return 0.0f;
    }
    /* Suspend roll correction so both legs sit at their symmetric nominal pose,
     * then average the IMU roll (published by the control task) to measure the
     * mounting bias. Runs on the calling task, so it must not touch control
     * state directly. */
    shared_lock();
    const int previous_mode = shared.roll_mode;
    shared.roll_mode = 0;
    shared_unlock();

    vTaskDelay(pdMS_TO_TICKS(900));
    float sum = 0.0f;
    int samples = 0;
    for (int i = 0; i < 100; ++i) {
        telemetry_lock();
        sum += telemetry.roll_angle;
        telemetry_unlock();
        samples++;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    float bias = samples > 0 ? -sum / (float)samples : 0.0f;
    shared_lock();
    if (bias > -10.0f && bias < 10.0f) {
        shared.roll_bias = bias;
    } else {
        ESP_LOGW("robot_control", "level calibration rejected (rb=%.1f)", bias);
    }
    shared.roll_mode = (previous_mode == 0) ? 1 : previous_mode;
    float result = shared.roll_bias;
    shared_unlock();
    roll_bias_save();

    ESP_LOGI("robot_control", "level calibrated: rb=%.2f", result);
    return result;
}
