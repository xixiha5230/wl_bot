#include "nvs_store.h"

#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "nvs";
static bool initialized;

esp_err_t nvs_store_init(void)
{
    if (initialized) {
        return ESP_OK;
    }
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs a format (%s), erasing", esp_err_to_name(err));
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "NVS erase failed");
        err = nvs_flash_init();
    }
    if (err == ESP_OK) {
        initialized = true;
    }
    return err;
}