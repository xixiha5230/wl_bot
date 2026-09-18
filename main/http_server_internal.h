#pragma once

/*
 * Handlers shared between the http_server translation units.
 *
 * http_server.c owns the server socket and the simple endpoints (root, OTA,
 * servo probe, CORS preflight); the telemetry and tuning endpoints live in
 * http_status.c and http_set.c and are registered from http_server_start().
 */

#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CORS headers for the host-served control UI. */
void http_set_cors(httpd_req_t *request);

/* GET /api/status: the full JSON telemetry snapshot. */
esp_err_t http_status_handler(httpd_req_t *request);

/* GET /api/set: table-driven runtime tuning. */
esp_err_t http_set_handler(httpd_req_t *request);

#ifdef __cplusplus
}
#endif
