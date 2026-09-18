#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Register the WebSocket control handler (/ws) on an already-running httpd
 * (see http_server_start). The reference web page connects to
 * ws://<host>/ws. */
esp_err_t ws_server_start(httpd_handle_t server);

#ifdef __cplusplus
}
#endif