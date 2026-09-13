#include "http_server.h"

#include "basic_web.h"
#include "board.h"
#include "esp_http_server.h"
#include "esp_check.h"
#include "esp_log.h"
#include "robot_control.h"
#include "robot_state.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "http";

static esp_err_t root_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    return httpd_resp_send(request, basic_web_page(), HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *request)
{
    robot_command_t cmd = robot_control_get_command();
    char response[320];
    snprintf(response, sizeof(response),
             "{\"state\":\"%s\",\"battery\":%.2f,\"go\":%d,\"height\":%d,"
             "\"lqr_angle\":%.2f,\"lqr_u\":%.3f,\"fault\":%d,"
             "\"joy_x\":%d,\"joy_y\":%d,\"dir\":%d,"
             "\"zero\":%.2f,\"yaw_mode\":%d}",
             robot_state_name(robot_state_get()), board_battery_voltage(),
             cmd.go ? 1 : 0, cmd.height, robot_control_lqr_angle(),
             robot_control_lqr_u(), robot_control_faulted() ? 1 : 0,
             cmd.joy_x, cmd.joy_y, cmd.dir,
             robot_control_get_balance_zero(), robot_control_get_yaw_mode());
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_send(request, response, HTTPD_RESP_USE_STRLEN);
}

/* Wireless tuning, so the robot can be adjusted while running untethered:
 *   /api/set?zero=3.0
 *   /api/set?pid=angle&p=1.2&i=0
 *   /api/set?lpf=roll&tf=0.5
 *   /api/set?yaw=-1
 */
static esp_err_t set_handler(httpd_req_t *request)
{
    char query[192];
    if (httpd_req_get_url_query_str(request, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "missing query");
        return ESP_OK;
    }

    char value[32];
    char applied[192] = {0};

    if (httpd_query_key_value(query, "zero", value, sizeof(value)) == ESP_OK) {
        float v = strtof(value, NULL);
        robot_control_set_angle_zeropoint(v);
        snprintf(applied + strlen(applied), sizeof(applied) - strlen(applied), "zero=%.2f ", v);
    }
    if (httpd_query_key_value(query, "yaw", value, sizeof(value)) == ESP_OK) {
        int v = atoi(value);
        robot_control_set_yaw_mode(v);
        snprintf(applied + strlen(applied), sizeof(applied) - strlen(applied), "yaw=%d ", v);
    }

    char name[24];
    if (httpd_query_key_value(query, "pid", name, sizeof(name)) == ESP_OK) {
        int which = -1;
        for (int idx = 0; idx < robot_control_pid_count(); ++idx) {
            if (strcmp(name, robot_control_pid_name(idx)) == 0) {
                which = idx;
                break;
            }
        }
        if (which >= 0 && httpd_query_key_value(query, "p", value, sizeof(value)) == ESP_OK) {
            float p = strtof(value, NULL);
            float i = -1.0f;
            float d = -1.0f;
            float limit = -1.0f;
            if (httpd_query_key_value(query, "i", value, sizeof(value)) == ESP_OK) i = strtof(value, NULL);
            if (httpd_query_key_value(query, "d", value, sizeof(value)) == ESP_OK) d = strtof(value, NULL);
            if (httpd_query_key_value(query, "limit", value, sizeof(value)) == ESP_OK) limit = strtof(value, NULL);
            robot_control_set_pid(which, p, i, d, limit);
            snprintf(applied + strlen(applied), sizeof(applied) - strlen(applied),
                     "pid %s P=%.4g I=%.4g ", name, p, i);
        }
    }

    if (httpd_query_key_value(query, "lpf", name, sizeof(name)) == ESP_OK &&
        httpd_query_key_value(query, "tf", value, sizeof(value)) == ESP_OK) {
        static const char *const lpf_names[] = {"joyy", "zeropoint", "roll"};
        float tf = strtof(value, NULL);
        for (int idx = 0; idx < 3; ++idx) {
            if (strcmp(name, lpf_names[idx]) == 0) {
                robot_control_set_lpf(idx, tf);
                snprintf(applied + strlen(applied), sizeof(applied) - strlen(applied),
                         "lpf %s=%.4g ", name, tf);
                break;
            }
        }
    }

    if (applied[0] == '\0') {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "nothing applied");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "tune: %s", applied);
    httpd_resp_set_type(request, "text/plain; charset=utf-8");
    return httpd_resp_send(request, applied, HTTPD_RESP_USE_STRLEN);
}

esp_err_t http_server_start(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_handler};
    httpd_uri_t status = {.uri = "/api/status", .method = HTTP_GET, .handler = status_handler};
    httpd_uri_t set = {.uri = "/api/set", .method = HTTP_GET, .handler = set_handler};

    ESP_RETURN_ON_ERROR(httpd_start(&server, &config), TAG, "HTTP server start failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &root), TAG, "root handler failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &status), TAG, "status handler failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &set), TAG, "set handler failed");
    ESP_LOGI(TAG, "HTTP server listening on port %d", config.server_port);
    return ESP_OK;
}
