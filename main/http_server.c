#include "http_server.h"

#include "basic_web.h"
#include "board.h"
#include "esp_http_server.h"
#include "esp_check.h"
#include "esp_log.h"
#include "robot_control.h"
#include "robot_state.h"

#include <stdio.h>

static const char *TAG = "http";

static esp_err_t root_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    return httpd_resp_send(request, basic_web_page(), HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *request)
{
    robot_command_t cmd = robot_control_get_command();
    char response[256];
    snprintf(response, sizeof(response),
             "{\"state\":\"%s\",\"battery\":%.2f,\"go\":%d,\"height\":%d,"
             "\"lqr_angle\":%.2f,\"lqr_u\":%.3f,\"fault\":%d,"
             "\"joy_x\":%d,\"joy_y\":%d,\"dir\":%d}",
             robot_state_name(robot_state_get()), board_battery_voltage(),
             cmd.go ? 1 : 0, cmd.height, robot_control_lqr_angle(),
             robot_control_lqr_u(), robot_control_faulted() ? 1 : 0,
             cmd.joy_x, cmd.joy_y, cmd.dir);
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_send(request, response, HTTPD_RESP_USE_STRLEN);
}

esp_err_t http_server_start(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_handler};
    httpd_uri_t status = {.uri = "/api/status", .method = HTTP_GET, .handler = status_handler};

    ESP_RETURN_ON_ERROR(httpd_start(&server, &config), TAG, "HTTP server start failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &root), TAG, "root handler failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &status), TAG, "status handler failed");
    ESP_LOGI(TAG, "HTTP server listening on port %d", config.server_port);
    return ESP_OK;
}
