#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Download a firmware image from `url` over HTTP(S) into the next OTA
 * partition and reboot on success. Runs in its own task, so it returns
 * immediately after the transfer has been scheduled. */
esp_err_t ota_start_from_url(const char *url);

#ifdef __cplusplus
}
#endif
