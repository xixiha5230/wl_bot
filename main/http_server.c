#include "http_server.h"
#include "http_server_internal.h"

#include "esp_http_server.h"
#include "esp_check.h"
#include "esp_log.h"
#include "ota.h"
#include "servo_sts.h"

#include <stdio.h>

static const char *TAG = "http";

/* The user interface lives on the host (tools/web); the firmware only exposes a
 * small JSON API. Cross-origin headers let that host-served page call us. */
void http_set_cors(httpd_req_t *request)
{
    httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(request, "Access-Control-Allow-Headers", "Content-Type");
}

static esp_err_t root_handler(httpd_req_t *request)
{
    static const char page[] =
        "<!doctype html><meta charset=utf-8><title>WLROBOT</title>"
        "<h3>WLROBOT firmware</h3>"
        "<p>The control UI is served from the host (tools/web), not the robot.</p>"
        "<ul>"
        "<li>GET <code>/api/status</code></li>"
        "<li>GET <code>/api/set?zero=&amp;pid=&amp;lpf=&amp;yaw=&amp;go=</code></li>"
        "<li>POST <code>/api/ota</code> (body = firmware URL)</li>"
        "<li>WebSocket <code>ws://&lt;host&gt;/ws</code></li>"
        "</ul>";
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    return httpd_resp_send(request, page, HTTPD_RESP_USE_STRLEN);
}


/* Firmware over Wi-Fi: POST /api/ota with the image URL in the body. */
static esp_err_t ota_handler(httpd_req_t *request)
{
    char url[192];
    int received = 0;
    while (received < (int)sizeof(url) - 1) {
        int chunk = httpd_req_recv(request, url + received, sizeof(url) - 1 - received);
        if (chunk == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (chunk <= 0) {
            break;
        }
        received += chunk;
    }
    url[received] = '\0';
    while (received > 0 && (url[received - 1] == '\n' || url[received - 1] == '\r' ||
                            url[received - 1] == ' ')) {
        url[--received] = '\0';
    }

    esp_err_t status = ota_start_from_url(url);
    http_set_cors(request);
    if (status != ESP_OK) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "invalid url");
        return ESP_OK;
    }
    httpd_resp_set_type(request, "text/plain; charset=utf-8");
    return httpd_resp_send(request, "ota started\n", HTTPD_RESP_USE_STRLEN);
}

/* Live leg-servo positions, for jump tuning. Reads the STS bus on demand. */
static esp_err_t servo_handler(httpd_req_t *request)
{
    char response[192];
    servo_sts_feedback_t f1, f2;
    esp_err_t e1 = servo_sts_read_feedback(1, &f1);
    esp_err_t e2 = servo_sts_read_feedback(2, &f2);
    snprintf(response, sizeof(response),
             "{\"s1\":%d,\"s2\":%d,\"ld1\":%d,\"ld2\":%d,\"cu1\":%d,\"cu2\":%d,\"e1\":%d,\"e2\":%d}",
             e1 == ESP_OK ? f1.position : 0,
             e2 == ESP_OK ? f2.position : 0,
             e1 == ESP_OK ? f1.load : 0,
             e2 == ESP_OK ? f2.load : 0,
             e1 == ESP_OK ? f1.current : 0,
             e2 == ESP_OK ? f2.current : 0,
             (int)e1, (int)e2);
    http_set_cors(request);
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_send(request, response, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t options_handler(httpd_req_t *request)
{
    http_set_cors(request);
    httpd_resp_set_status(request, "204 No Content");
    return httpd_resp_send(request, NULL, 0);
}

esp_err_t http_server_start(httpd_handle_t *out_server)
{
    if (out_server == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_server = NULL;

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.core_id = 0;   /* keep the real-time control loop (core 1) undisturbed */
    config.max_open_sockets = 7;
    /* Do NOT enable lru_purge: the long-lived /ws control socket looks idle
     * between commands and would be the first thing purged, dropping the
     * control link while status polling keeps opening connections. */
    httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_handler};
    httpd_uri_t status = {.uri = "/api/status", .method = HTTP_GET, .handler = http_status_handler};
    httpd_uri_t set = {.uri = "/api/set", .method = HTTP_GET, .handler = http_set_handler};
    httpd_uri_t servo = {.uri = "/api/servo", .method = HTTP_GET, .handler = servo_handler};
    httpd_uri_t ota = {.uri = "/api/ota", .method = HTTP_POST, .handler = ota_handler};
    httpd_uri_t options = {.uri = "/api/*", .method = HTTP_OPTIONS, .handler = options_handler};

    ESP_RETURN_ON_ERROR(httpd_start(&server, &config), TAG, "HTTP server start failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &root), TAG, "root handler failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &status), TAG, "status handler failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &set), TAG, "set handler failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &servo), TAG, "servo handler failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &ota), TAG, "ota handler failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &options), TAG, "options handler failed");
    *out_server = server;
    ESP_LOGI(TAG, "HTTP server listening on port %d", config.server_port);
    return ESP_OK;
}
