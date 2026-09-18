#include "http_server_internal.h"
#include "http_set_internal.h"

#include "esp_http_server.h"
#include "esp_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "http";

/*
 * GET /api/set: wireless tuning, so the robot can be adjusted while running
 * untethered. The key handlers live in http_set_tune.c and http_set_motion.c;
 * this file owns the request loop and the "applied" note buffer.
 */

void http_set_note(char *applied, size_t cap, const char *fmt, ...)
{
    size_t used = strlen(applied);
    if (used >= cap) {
        return;
    }
    va_list args;
    va_start(args, fmt);
    vsnprintf(applied + used, cap - used, fmt, args);
    va_end(args);
}

esp_err_t http_set_handler(httpd_req_t *request)
{
    char query[320];
    if (httpd_req_get_url_query_str(request, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "missing or oversized query");
        return ESP_OK;
    }

    const http_set_entry_t *groups[2];
    size_t counts[2];
    groups[0] = http_set_tune_entries(&counts[0]);
    groups[1] = http_set_motion_entries(&counts[1]);

    char value[256];
    char applied[256] = {0};
    for (size_t g = 0; g < 2; ++g) {
        for (size_t i = 0; i < counts[g]; ++i) {
            if (httpd_query_key_value(query, groups[g][i].key, value, sizeof(value)) == ESP_OK) {
                groups[g][i].apply(query, value, applied, sizeof(applied));
            }
        }
    }

    if (applied[0] == '\0') {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "nothing applied");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "tune: %s", applied);
    http_set_cors(request);
    httpd_resp_set_type(request, "text/plain; charset=utf-8");
    return httpd_resp_send(request, applied, HTTPD_RESP_USE_STRLEN);
}
