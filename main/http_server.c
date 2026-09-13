#include "http_server.h"

#include "board.h"
#include "esp_http_server.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "motor_foc.h"
#include "ota.h"
#include "robot_control.h"
#include "robot_state.h"
#include "sensors.h"
#include "servo_sts.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "http";

/* The user interface lives on the host (tools/web); the firmware only exposes a
 * small JSON API. Cross-origin headers let that host-served page call us. */
static void set_cors(httpd_req_t *request)
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
        "<li>WebSocket <code>ws://&lt;host&gt;:81/</code></li>"
        "</ul>";
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    return httpd_resp_send(request, page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *request)
{
    robot_command_t cmd = robot_control_get_command();
    float ta, tg, td, ts;
    robot_control_get_terms(&ta, &tg, &td, &ts);
    char response[768];
    int jh, jl, js, ja, jt;
    int jc, jct;
    int16_t leg1, leg2;
    int jf;
    float ax, ay, az;
    robot_control_get_jump_profile(&jh, &jl, &js, &ja, &jt);
    robot_control_get_jump_crouch(&jc, &jct);
    robot_control_get_leg_diag(&leg1, &leg2, &jf);
    robot_control_get_accel(&ax, &ay, &az);
    snprintf(response, sizeof(response),
             "{\"state\":\"%s\",\"battery\":%.2f,\"go\":%d,\"height\":%d,"
             "\"lqr_angle\":%.2f,\"lqr_u\":%.3f,\"fault\":%d,"
             "\"joy_x\":%d,\"joy_y\":%d,\"dir\":%d,"
             "\"zero\":%.2f,\"yaw_mode\":%d,\"roll_mode\":%d,\"rb\":%.2f,\"faultdeg\":%.1f,"
             "\"amag\":%.2f,\"air\":%d,\"airth\":%.2f,\"airscale\":%.2f,"
             "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
             "\"mt\":%d,\"mmode\":%d,\"malign\":%d,\"gstate\":%d,"
             "\"angle_pp\":%.2f,\"ta\":%.2f,\"tg\":%.2f,\"td\":%.2f,\"ts\":%.2f,"
             "\"leg_add\":%.1f,\"yaw\":%.1f,\"yaw_out\":%.2f,\"roll\":%.2f,"
             "\"vl\":%.2f,\"vr\":%.2f,\"gz\":%.2f,\"uptime\":%d,"
             "\"jh\":%d,\"jl\":%d,\"js\":%d,\"jacc\":%d,\"jlt\":%d,"
             "\"jc\":%d,\"jct\":%d,\"lt1\":%d,\"lt2\":%d,\"jf\":%d}",
             robot_state_name(robot_state_get()), board_battery_voltage(),
             cmd.go ? 1 : 0, cmd.height, robot_control_lqr_angle(),
             robot_control_lqr_u(), robot_control_faulted() ? 1 : 0,
             cmd.joy_x, cmd.joy_y, cmd.dir,
             robot_control_get_balance_zero(), robot_control_get_yaw_mode(),
             robot_control_get_roll_mode(), robot_control_get_roll_bias(),
             robot_control_get_fault_deg(),
             robot_control_accel_mag(), robot_control_airborne(),
             robot_control_get_air_thresh(), robot_control_get_air_scale(),
             ax, ay, az,
             robot_control_manual_ticks(), (int)motor_foc_get_mode(),
             motor_foc_is_aligned() ? 1 : 0, robot_control_getup_state(),
             robot_control_angle_pp(), ta, tg, td, ts,
             robot_control_leg_add(), robot_control_yaw_total(),
             robot_control_yaw_output(), robot_control_roll_angle(),
             robot_control_left_velocity(), robot_control_right_velocity(),
             robot_control_gyro_z(), (int)(esp_timer_get_time() / 1000000),
             jh, jl, js, ja, jt, jc, jct, leg1, leg2, jf);
    set_cors(request);
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
    if (httpd_query_key_value(query, "rollmode", value, sizeof(value)) == ESP_OK) {
        int v = atoi(value);
        robot_control_set_roll_mode(v);
        snprintf(applied + strlen(applied), sizeof(applied) - strlen(applied), "rollmode=%d ", v);
    }
    if (httpd_query_key_value(query, "rb", value, sizeof(value)) == ESP_OK) {
        float v = strtof(value, NULL);
        robot_control_set_roll_bias(v);
        snprintf(applied + strlen(applied), sizeof(applied) - strlen(applied), "rb=%.2f ", v);
    }
    if (httpd_query_key_value(query, "rblevel", value, sizeof(value)) == ESP_OK) {
        float v = robot_control_calibrate_level();
        snprintf(applied + strlen(applied), sizeof(applied) - strlen(applied),
                 "rblevel=%.2f ", v);
    }
    if (httpd_query_key_value(query, "faultdeg", value, sizeof(value)) == ESP_OK) {
        float v = strtof(value, NULL);
        robot_control_set_fault_deg(v);
        snprintf(applied + strlen(applied), sizeof(applied) - strlen(applied),
                 "faultdeg=%.1f ", robot_control_get_fault_deg());
    }
    if (httpd_query_key_value(query, "airth", value, sizeof(value)) == ESP_OK) {
        float th = strtof(value, NULL);
        robot_control_set_air(th, robot_control_get_air_scale());
        snprintf(applied + strlen(applied), sizeof(applied) - strlen(applied),
                 "airth=%.2f ", robot_control_get_air_thresh());
    }
    if (httpd_query_key_value(query, "airscale", value, sizeof(value)) == ESP_OK) {
        float sc = strtof(value, NULL);
        robot_control_set_air(robot_control_get_air_thresh(), sc);
        snprintf(applied + strlen(applied), sizeof(applied) - strlen(applied),
                 "airscale=%.2f ", robot_control_get_air_scale());
    }
    /* Research: /api/set?wdrive=<torque>&wms=<ms> drives both wheels directly. */
    if (httpd_query_key_value(query, "wdrive", value, sizeof(value)) == ESP_OK) {
        float target = strtof(value, NULL);
        int ms = 300;
        if (httpd_query_key_value(query, "wms", value, sizeof(value)) == ESP_OK) {
            ms = atoi(value);
        }
        robot_control_manual_drive(target, ms);
        snprintf(applied + strlen(applied), sizeof(applied) - strlen(applied),
                 "wdrive=%.2f for %dms ", target, ms);
    }
    /* Self-right: ?getup=1 starts, plus gtorque / grelease / gsign params. */
    if (httpd_query_key_value(query, "getup", value, sizeof(value)) == ESP_OK) {
        robot_control_set_getup(atoi(value) != 0);
        snprintf(applied + strlen(applied), sizeof(applied) - strlen(applied),
                 "getup=%d ", atoi(value));
    }
    {
        bool any = false;
        float torque = robot_control_getup_torque();
        float release = robot_control_getup_release();
        int sign = robot_control_getup_sign();
        if (httpd_query_key_value(query, "gtorque", value, sizeof(value)) == ESP_OK) {
            torque = strtof(value, NULL);
            any = true;
        }
        if (httpd_query_key_value(query, "grelease", value, sizeof(value)) == ESP_OK) {
            release = strtof(value, NULL);
            any = true;
        }
        if (httpd_query_key_value(query, "gsign", value, sizeof(value)) == ESP_OK) {
            sign = atoi(value);
            any = true;
        }
        if (any) {
            robot_control_set_getup_params(torque, release, sign);
            snprintf(applied + strlen(applied), sizeof(applied) - strlen(applied),
                     "getup t=%.1f r=%.0f s=%d ",
                     robot_control_getup_torque(), release, sign);
        }
    }
    if (httpd_query_key_value(query, "go", value, sizeof(value)) == ESP_OK) {
        int v = atoi(value);
        robot_control_set_go(v != 0);
        snprintf(applied + strlen(applied), sizeof(applied) - strlen(applied), "go=%d ", v);
    }
    if (httpd_query_key_value(query, "gcal", value, sizeof(value)) == ESP_OK) {
        esp_err_t status = sensors_calibrate_gyro();
        snprintf(applied + strlen(applied), sizeof(applied) - strlen(applied),
                 "gcal=%s ", status == ESP_OK ? "ok" : "fail");
    }

    /* Jump profile tuning: jh=height jl=land_height js=speed jacc=acc jlt=land_ticks
     * plus the crouch phase: jc=crouch_height jct=crouch_ticks */
    int jparams[5];
    bool any_jump = false;
    for (int idx = 0; idx < 5; ++idx) {
        jparams[idx] = -1;   /* -1 = key absent, so explicit 0 values apply */
    }
    const char *const jump_keys[] = {"jh", "jl", "js", "jacc", "jlt"};
    for (int idx = 0; idx < 5; ++idx) {
        if (httpd_query_key_value(query, jump_keys[idx], value, sizeof(value)) == ESP_OK) {
            jparams[idx] = atoi(value);
            any_jump = true;
        }
    }
    int jc = -1, jct = -1;
    if (httpd_query_key_value(query, "jc", value, sizeof(value)) == ESP_OK) {
        jc = atoi(value);
        any_jump = true;
    }
    if (httpd_query_key_value(query, "jct", value, sizeof(value)) == ESP_OK) {
        jct = atoi(value);
        any_jump = true;
    }
    if (any_jump) {
        int h, l, s, a, t;
        robot_control_get_jump_profile(&h, &l, &s, &a, &t);
        if (jparams[0] >= 0) h = jparams[0];
        if (jparams[1] >= 0) l = jparams[1];
        if (jparams[2] >= 0) s = jparams[2];
        if (jparams[3] >= 0) a = jparams[3];
        if (jparams[4] >= 0) t = jparams[4];
        robot_control_set_jump_profile(h, l, s, a, t);
        robot_control_set_jump_crouch(jc, jct);
        robot_control_get_jump_crouch(&jc, &jct);
        snprintf(applied + strlen(applied), sizeof(applied) - strlen(applied),
                 "jump h=%d l=%d s=%d a=%d t=%d c=%d ct=%d ", h, l, s, a, t, jc, jct);
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
    set_cors(request);
    httpd_resp_set_type(request, "text/plain; charset=utf-8");
    return httpd_resp_send(request, applied, HTTPD_RESP_USE_STRLEN);
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
    set_cors(request);
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
    char response[128];
    servo_sts_feedback_t f1, f2;
    esp_err_t e1 = servo_sts_read_feedback(1, &f1);
    esp_err_t e2 = servo_sts_read_feedback(2, &f2);
    snprintf(response, sizeof(response),
             "{\"s1\":%d,\"s2\":%d,\"e1\":%d,\"e2\":%d}",
             e1 == ESP_OK ? f1.position : 0,
             e2 == ESP_OK ? f2.position : 0, (int)e1, (int)e2);
    set_cors(request);
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_send(request, response, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t options_handler(httpd_req_t *request)
{
    set_cors(request);
    httpd_resp_set_status(request, "204 No Content");
    return httpd_resp_send(request, NULL, 0);
}

esp_err_t http_server_start(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.core_id = 0;   /* keep the real-time control loop (core 1) undisturbed */
    httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_handler};
    httpd_uri_t status = {.uri = "/api/status", .method = HTTP_GET, .handler = status_handler};
    httpd_uri_t set = {.uri = "/api/set", .method = HTTP_GET, .handler = set_handler};
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
    ESP_LOGI(TAG, "HTTP server listening on port %d", config.server_port);
    return ESP_OK;
}
