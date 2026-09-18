#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start the single HTTP server: the JSON API, the OTA endpoint and the OTA
 * client all live here, and the WebSocket handler is registered on the same
 * server by ws_server_start(). Returns the handle through `out_server` so other
 * modules can add URI handlers. */
esp_err_t http_server_start(httpd_handle_t *out_server);

#ifdef __cplusplus
}
#endif