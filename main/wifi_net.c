#include "wifi_net.h"

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/ip4_addr.h"
#include "mdns.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include <string.h>

static const char *TAG = "wifi";

/* Wi-Fi credentials. Supply them at build time through the environment so no
 * secrets live in the repository:
 *   WLROBOT_WIFI_SSID / WLROBOT_WIFI_PASSWORD   station (home router)
 *   WLROBOT_AP_SSID   / WLROBOT_AP_PASSWORD     fallback access point
 * The fallbacks below are only defaults for a fresh checkout. */
#ifndef WIFI_STA_SSID
#define WIFI_STA_SSID       "wlrobot-setup"
#endif
#ifndef WIFI_STA_PASSWORD
#define WIFI_STA_PASSWORD   ""
#endif

/* Fallback access point, kept on 192.168.4.x so it cannot collide with the
 * common 192.168.1.x home networks. */
#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID        "WLROBOT"
#endif
#ifndef WIFI_AP_PASSWORD
#define WIFI_AP_PASSWORD    "12345678"
#endif
#define WIFI_AP_CHANNEL     1
#define WIFI_AP_MAX_CLIENTS 2
#define WIFI_AP_ADDRESS     ESP_IP4TOADDR(192, 168, 4, 1)
#define WIFI_AP_NETMASK     ESP_IP4TOADDR(255, 255, 255, 0)

#define WIFI_HOSTNAME       "wlrobot"
#define WIFI_CONNECTED_BIT  BIT0

static EventGroupHandle_t s_wifi_events;

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "STA disconnected, retrying");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "STA got IP " IPSTR " (http://%s.local/)",
                 IP2STR(&event->ip_info.ip), WIFI_HOSTNAME);
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "NVS erase failed");
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t wifi_net_start(void)
{
    ESP_RETURN_ON_ERROR(init_nvs(), TAG, "NVS init failed");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop failed");

    s_wifi_events = xEventGroupCreate();
    if (s_wifi_events == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();
    if (ap_netif == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_netif_ip_info_t ip_info = {
        .ip = {.addr = WIFI_AP_ADDRESS},
        .gw = {.addr = WIFI_AP_ADDRESS},
        .netmask = {.addr = WIFI_AP_NETMASK},
    };
    ESP_RETURN_ON_ERROR(esp_netif_dhcps_stop(ap_netif), TAG, "DHCP server stop failed");
    ESP_RETURN_ON_ERROR(esp_netif_set_ip_info(ap_netif, &ip_info), TAG, "AP IP config failed");
    ESP_RETURN_ON_ERROR(esp_netif_dhcps_start(ap_netif), TAG, "DHCP server start failed");

    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                   &wifi_event_handler, NULL),
                        TAG, "wifi event handler failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                   &wifi_event_handler, NULL),
                        TAG, "ip event handler failed");

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
    wifi_config_t sta_config = {
        .sta = {
            .ssid = WIFI_STA_SSID,
            .password = WIFI_STA_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_APSTA), TAG, "Wi-Fi mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap_config), TAG, "AP config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &sta_config), TAG, "STA config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Wi-Fi start failed");

    ESP_LOGI(TAG, "connecting to '%s'; fallback AP '%s' at 192.168.4.1",
             WIFI_STA_SSID, WIFI_AP_SSID);

    /* Give the STA some time; the AP is up regardless, so this is not fatal. */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_events, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdTRUE, pdMS_TO_TICKS(15000));
    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGW(TAG, "STA not connected yet; AP fallback is available");
    }

    ESP_RETURN_ON_ERROR(mdns_init(), TAG, "mDNS init failed");
    ESP_RETURN_ON_ERROR(mdns_hostname_set(WIFI_HOSTNAME), TAG, "mDNS hostname failed");
    mdns_instance_name_set("WLROBOT");
    return ESP_OK;
}
