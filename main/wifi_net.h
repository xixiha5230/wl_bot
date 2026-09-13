#pragma once

#include "esp_err.h"

/* Start Wi-Fi in AP+STA: connect to the home router (STA) and keep a fallback
 * access point on a separate subnet. Also starts mDNS (hostname "wlrobot"). */
esp_err_t wifi_net_start(void);
