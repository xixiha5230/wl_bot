#include "wifi_ap.h"

#include "esp_event.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include <string.h>

static const char *TAG = "wifi";

/* Access point configuration, matching the reference firmware. */
#define WIFI_AP_SSID        "WLROBOT"
#define WIFI_AP_PASSWORD    "12345678"
#define WIFI_AP_CHANNEL     1
#define WIFI_AP_MAX_CLIENTS 2
#define WIFI_AP_ADDRESS     ESP_IP4TOADDR(192, 168, 1, 11)
#define WIFI_AP_NETMASK     ESP_IP4TOADDR(255, 255, 255, 0)

esp_err_t wifi_ap_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "NVS erase failed");
        ESP_RETURN_ON_ERROR(nvs_flash_init(), TAG, "NVS reinit failed");
    } else {
        ESP_RETURN_ON_ERROR(err, TAG, "NVS init failed");
    }

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop failed");
    esp_netif_t *netif = esp_netif_create_default_wifi_ap();
    if (netif == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_netif_ip_info_t ip_info = {
        .ip = {.addr = WIFI_AP_ADDRESS},
        .gw = {.addr = WIFI_AP_ADDRESS},
        .netmask = {.addr = WIFI_AP_NETMASK},
    };
    ESP_RETURN_ON_ERROR(esp_netif_dhcps_stop(netif), TAG, "DHCP server stop failed");
    ESP_RETURN_ON_ERROR(esp_netif_set_ip_info(netif, &ip_info), TAG, "AP IP config failed");
    ESP_RETURN_ON_ERROR(esp_netif_dhcps_start(netif), TAG, "DHCP server start failed");

    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&config), TAG, "Wi-Fi init failed");
    wifi_config_t ap_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASSWORD,
            .max_connection = WIFI_AP_MAX_CLIENTS,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "Wi-Fi mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap_config), TAG, "AP config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Wi-Fi start failed");

    ESP_LOGI(TAG, "AP started: %s, password: %s", WIFI_AP_SSID, WIFI_AP_PASSWORD);
    return ESP_OK;
}
