#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* WebSocket server for the web remote control, on port 81 (matches the page). */
esp_err_t ws_server_start(void);

#ifdef __cplusplus
}
#endif
