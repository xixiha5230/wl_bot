#include "ws_server.h"

#include "cJSON.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "robot_config.h"
#include "robot_control.h"

#include <stdlib.h>
#include <string.h>

static const char *TAG = "ws";

static int dir_from_string(const char *dir)
{
    if (strcmp(dir, "forward") == 0) {
        return ROBOT_FORWARD;
    }
    if (strcmp(dir, "back") == 0) {
        return ROBOT_BACK;
    }
    if (strcmp(dir, "right") == 0) {
        return ROBOT_RIGHT;
    }
    if (strcmp(dir, "left") == 0) {
        return ROBOT_LEFT;
    }
    if (strcmp(dir, "jump") == 0) {
        return ROBOT_JUMP;
    }
    return ROBOT_STOP;
}

/* ArduinoJson (used by the reference firmware) converts numeric strings, while
 * cJSON keeps them as strings. The original web page sends the joystick values
 * through Number.toFixed(), i.e. as strings, so accept both forms. */
static int json_to_int(const cJSON *item, int fallback)
{
    if (cJSON_IsNumber(item)) {
        return item->valueint;
    }
    if (cJSON_IsString(item) && item->valuestring != NULL) {
        return atoi(item->valuestring);
    }
    return fallback;
}

/* Mirrors RobotProtocol::parseBasic from the reference firmware. */
static void handle_basic_json(const char *text)
{
    cJSON *doc = cJSON_Parse(text);
    if (doc == NULL) {
        ESP_LOGW(TAG, "JSON parse failed");
        return;
    }

    const cJSON *mode = cJSON_GetObjectItemCaseSensitive(doc, "mode");
    if (cJSON_IsString(mode) && strcmp(mode->valuestring, "basic") == 0) {
        const cJSON *item;

        item = cJSON_GetObjectItemCaseSensitive(doc, "dir");
        if (cJSON_IsString(item)) {
            robot_control_set_dir(dir_from_string(item->valuestring));
        }

        item = cJSON_GetObjectItemCaseSensitive(doc, "height");
        if (item != NULL) {
            int height = json_to_int(item, LEG_HEIGHT_DEFAULT);
            if (height < LEG_HEIGHT_MIN) {
                height = LEG_HEIGHT_MIN;
            } else if (height > LEG_HEIGHT_MAX) {
                height = LEG_HEIGHT_MAX;
            }
            robot_control_set_height(height);
        }
        item = cJSON_GetObjectItemCaseSensitive(doc, "roll");
        if (item != NULL) {
            robot_control_set_roll(json_to_int(item, 0));
        }
        item = cJSON_GetObjectItemCaseSensitive(doc, "linear");
        if (item != NULL) {
            robot_control_set_linear(json_to_int(item, 0));
        }
        item = cJSON_GetObjectItemCaseSensitive(doc, "angular");
        if (item != NULL) {
            robot_control_set_angular(json_to_int(item, 0));
        }
        item = cJSON_GetObjectItemCaseSensitive(doc, "stable");
        if (item != NULL) {
            robot_control_set_go(json_to_int(item, 0) != 0);
        }
        int joy_x = json_to_int(cJSON_GetObjectItemCaseSensitive(doc, "joy_x"), 0);
        int joy_y = json_to_int(cJSON_GetObjectItemCaseSensitive(doc, "joy_y"), 0);
        robot_control_set_joy(joy_x, joy_y);
    }

    cJSON_Delete(doc);
}

static esp_err_t ws_handler(httpd_req_t *request)
{
    if (request->method == HTTP_GET) {
        ESP_LOGI(TAG, "client connected");
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    frame.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(request, &frame, 0);
    if (ret != ESP_OK || frame.len == 0) {
        return ret;
    }

    char *payload = calloc(1, frame.len + 1);
    if (payload == NULL) {
        return ESP_ERR_NO_MEM;
    }
    frame.payload = (uint8_t *)payload;

    ret = httpd_ws_recv_frame(request, &frame, frame.len);
    if (ret == ESP_OK && frame.type == HTTPD_WS_TYPE_TEXT) {
        handle_basic_json(payload);
    }

    free(payload);
    return ret;
}

esp_err_t ws_server_start(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 81;
    /* The control port must differ from the HTTP server's default 32768. */
    config.ctrl_port = 32769;
    config.max_open_sockets = 4;

    httpd_uri_t ws = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = ws_handler,
        .is_websocket = true,
    };

    ESP_RETURN_ON_ERROR(httpd_start(&server, &config), TAG, "WS server start failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &ws), TAG, "WS handler failed");
    ESP_LOGI(TAG, "WebSocket server listening on port %d", config.server_port);
    return ESP_OK;
}
