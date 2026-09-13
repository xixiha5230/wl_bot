#include "ota.h"

#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "robot_control.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "ota";

static void ota_task(void *arg)
{
    char *url = (char *)arg;
    ESP_LOGW(TAG, "OTA start: %s", url);

    robot_control_set_go(false);
    vTaskDelay(pdMS_TO_TICKS(100));

    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (update == NULL) {
        ESP_LOGE(TAG, "no OTA partition available");
        goto done;
    }
    ESP_LOGI(TAG, "writing to %s @ 0x%08" PRIx32, update->label, update->address);

    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = 10000,
        .keep_alive_enable = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "http client init failed");
        goto done;
    }

    if (esp_http_client_open(client, 0) != ESP_OK) {
        ESP_LOGE(TAG, "http open failed");
        goto done_client;
    }
    int content_length = esp_http_client_fetch_headers(client);
    if (content_length <= 0) {
        ESP_LOGE(TAG, "bad content length: %d", content_length);
        goto done_client;
    }
    ESP_LOGI(TAG, "image size %d bytes", content_length);

    esp_ota_handle_t handle = 0;
    if (esp_ota_begin(update, OTA_SIZE_UNKNOWN, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed");
        goto done_client;
    }

    char buffer[1024];
    int total = 0;
    int read = 0;
    bool ok = true;
    while ((read = esp_http_client_read(client, buffer, sizeof(buffer))) > 0) {
        if (esp_ota_write(handle, buffer, read) != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed at %d", total);
            ok = false;
            break;
        }
        total += read;
    }
    if (read < 0) {
        ESP_LOGE(TAG, "http read error at %d", total);
        ok = false;
    }

    if (!ok) {
        ESP_LOGE(TAG, "OTA failed (read %d bytes); keeping current firmware", total);
        esp_ota_abort(handle);
        goto done_client;
    }
    if (esp_ota_end(handle) != ESP_OK) {
        ESP_LOGE(TAG, "image validation failed");
        goto done_client;
    }
    if (esp_ota_set_boot_partition(update) != ESP_OK) {
        ESP_LOGE(TAG, "failed to select boot partition");
        goto done_client;
    }
    ESP_LOGI(TAG, "OTA complete (%d bytes), rebooting", total);
    esp_restart();

done_client:
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
done:
    free(url);
    vTaskDelete(NULL);
}

esp_err_t ota_start_from_url(const char *url)
{
    if (url == NULL || url[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    char *copy = strdup(url);
    if (copy == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(ota_task, "ota", 8192, copy, 5, NULL) != pdPASS) {
        free(copy);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
