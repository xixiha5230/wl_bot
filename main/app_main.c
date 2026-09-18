#include "board.h"
#include "console.h"
#include "esp_log.h"
#include "http_server.h"
#include "motor_foc.h"
#include "nvs_store.h"
#include "robot_control.h"
#include "robot_state.h"
#include "sensors.h"
#include "servo_sts.h"
#include "wifi_net.h"
#include "ws_server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app";

/* All application tasks share core 1, leaving core 0 to the Wi-Fi/system stack. */
#define APP_TASK_CORE 1

static void servo_task(void *arg)
{
    (void)arg;
    servo_sts_feedback_t feedback = {0};

    while (true) {
        for (uint8_t id = 1; id <= 2; ++id) {
            esp_err_t status = servo_sts_read_feedback(id, &feedback);
            if (status == ESP_OK) {
                ESP_LOGD(TAG, "servo id=%u position=%d speed=%d temp=%u",
                         id, feedback.position, feedback.speed, feedback.temperature);
            } else {
                ESP_LOGW(TAG, "servo id=%u read failed: %s",
                         id, esp_err_to_name(status));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void battery_task(void *arg)
{
    (void)arg;
    bool led_on = false;
    while (true) {
        float voltage = board_battery_voltage();
        if (led_on) {
            if (voltage < BOARD_BATTERY_LED_THRESHOLD - BOARD_BATTERY_LED_HYSTERESIS) {
                led_on = false;
            }
        } else if (voltage > BOARD_BATTERY_LED_THRESHOLD + BOARD_BATTERY_LED_HYSTERESIS) {
            led_on = true;
        }
        board_battery_led_set(led_on);
        ESP_LOGI(TAG, "battery %.2f V (LED %s)", voltage, led_on ? "on" : "off");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    robot_state_init();
    ESP_ERROR_CHECK(nvs_store_init());
    ESP_ERROR_CHECK(board_init());
    ESP_ERROR_CHECK(servo_sts_init());

    esp_err_t sensor_status = sensors_init();
    for (int attempt = 0; sensor_status != ESP_OK && attempt < 3; ++attempt) {
        ESP_LOGW(TAG, "sensors_init failed (%s), retry %d", esp_err_to_name(sensor_status),
                 attempt + 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        sensor_status = sensors_init();
    }

    if (sensor_status == ESP_OK) {
        if (motor_foc_init() != ESP_OK) {
            ESP_LOGE(TAG, "motor FOC initialization failed");
            robot_state_set(ROBOT_STATE_FAULT);
        } else {
            robot_state_set(ROBOT_STATE_CALIBRATING);
            esp_err_t align_status = ESP_FAIL;
            for (int attempt = 0; attempt < 3 && align_status != ESP_OK; ++attempt) {
                align_status = motor_foc_align();
                if (align_status != ESP_OK) {
                    ESP_LOGW(TAG, "motor alignment failed, retry %d", attempt + 1);
                }
            }
            if (align_status != ESP_OK) {
                ESP_LOGE(TAG, "motor alignment failed after retries; run 'm_align'");
                robot_state_set(ROBOT_STATE_FAULT);
            } else {
                robot_control_start();
            }
        }
    } else {
        ESP_LOGE(TAG, "sensor initialization failed: %s; continuing in SAFE state",
                 esp_err_to_name(sensor_status));
        robot_state_set(ROBOT_STATE_FAULT);
    }

    xTaskCreatePinnedToCore(servo_task, "servo_task", 4096, NULL, 3, NULL, APP_TASK_CORE);
    xTaskCreatePinnedToCore(battery_task, "battery_task", 3072, NULL, 2, NULL, APP_TASK_CORE);
    ESP_ERROR_CHECK(wifi_net_start());
    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(http_server_start(&server));
    ESP_ERROR_CHECK(ws_server_start(server));

    ESP_LOGI(TAG, "WLROBOT ESP-IDF firmware started");
    ESP_ERROR_CHECK(console_start());
}
