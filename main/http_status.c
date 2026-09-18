#include "http_server_internal.h"

#include "board.h"
#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "motor_foc.h"
#include "robot_control.h"
#include "robot_state.h"
#include "sensors.h"

/*
 * GET /api/status: one JSON object with the whole telemetry snapshot.
 *
 * The control-task fields come from a single robot_control_get_telemetry() copy
 * so a response cannot mix two control ticks; the tunables are read separately
 * (they only change when the operator tunes, so a mixed frame is harmless).
 * Built with cJSON rather than a 60-argument snprintf: a single misordered
 * argument in the old format string was undetectable at compile time.
 */

esp_err_t http_status_handler(httpd_req_t *request)
{
    robot_command_t cmd = robot_control_get_command();
    robot_telemetry_t t;
    robot_control_get_telemetry(&t);

    int jh, jl, js, ja, jt, jc, jct;
    int bamp, bms, bspd, bacc;
    float yws, ywc;
    int lim1min, lim1max, lim2min, lim2max;
    robot_control_get_leg_limits(&lim1min, &lim1max, &lim2min, &lim2max);
    robot_control_get_jump_profile(&jh, &jl, &js, &ja, &jt);
    robot_control_get_jump_crouch(&jc, &jct);
    robot_control_get_bump_params(&bamp, &bms, &bspd, &bacc);
    robot_control_get_yaw_wheel(&yws, &ywc);

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
        return ESP_OK;
    }
    cJSON_AddStringToObject(root, "state", robot_state_name(robot_state_get()));
    cJSON_AddNumberToObject(root, "battery", board_battery_voltage());
    cJSON_AddNumberToObject(root, "go", cmd.go ? 1 : 0);
    cJSON_AddNumberToObject(root, "height", cmd.height);
    cJSON_AddNumberToObject(root, "lqr_angle", t.lqr_angle);
    cJSON_AddNumberToObject(root, "lqr_u", t.lqr_u);
    cJSON_AddNumberToObject(root, "fault", t.fault_reason != 0 ? 1 : 0);
    cJSON_AddNumberToObject(root, "joy_x", cmd.joy_x);
    cJSON_AddNumberToObject(root, "joy_y", cmd.joy_y);
    cJSON_AddNumberToObject(root, "dir", cmd.dir);
    cJSON_AddNumberToObject(root, "balance_zero", t.balance_zero);
    cJSON_AddNumberToObject(root, "yaw_mode", robot_control_get_yaw_mode());
    cJSON_AddNumberToObject(root, "roll_mode", robot_control_get_roll_mode());
    cJSON_AddNumberToObject(root, "roll_bias", robot_control_get_roll_bias());
    cJSON_AddNumberToObject(root, "fault_deg", robot_control_get_fault_deg());
    cJSON_AddNumberToObject(root, "accel_mag", t.accel_mag);
    cJSON_AddNumberToObject(root, "air", t.airborne);
    cJSON_AddNumberToObject(root, "air_thresh", robot_control_get_air_thresh());
    cJSON_AddNumberToObject(root, "air_scale", robot_control_get_air_scale());
    cJSON_AddNumberToObject(root, "ax", t.accel_x);
    cJSON_AddNumberToObject(root, "ay", t.accel_y);
    cJSON_AddNumberToObject(root, "az", t.accel_z);
    cJSON_AddNumberToObject(root, "manual_ms", t.manual_ms);
    cJSON_AddNumberToObject(root, "motor_mode", (int)motor_foc_get_mode());
    cJSON_AddNumberToObject(root, "motor_aligned", motor_foc_is_aligned() ? 1 : 0);
    cJSON_AddNumberToObject(root, "getup_state", t.getup_state);
    cJSON_AddNumberToObject(root, "fault_reason", t.fault_reason);
    cJSON_AddNumberToObject(root, "p1min", lim1min);
    cJSON_AddNumberToObject(root, "p1max", lim1max);
    cJSON_AddNumberToObject(root, "p2min", lim2min);
    cJSON_AddNumberToObject(root, "p2max", lim2max);
    cJSON_AddNumberToObject(root, "angle_pp", t.angle_pp);
    cJSON_AddNumberToObject(root, "term_angle", t.angle_term);
    cJSON_AddNumberToObject(root, "term_gyro", t.gyro_term);
    cJSON_AddNumberToObject(root, "term_distance", t.distance_term);
    cJSON_AddNumberToObject(root, "term_speed", t.speed_term);
    cJSON_AddNumberToObject(root, "leg_add", t.leg_add);
    cJSON_AddNumberToObject(root, "yaw_total", t.yaw_total);
    cJSON_AddNumberToObject(root, "yaw_out", t.yaw_output);
    cJSON_AddNumberToObject(root, "roll_angle", t.roll_angle);
    cJSON_AddNumberToObject(root, "velocity_left", t.left_velocity);
    cJSON_AddNumberToObject(root, "velocity_right", t.right_velocity);
    cJSON_AddNumberToObject(root, "gyro_z", t.gyro_z);
    cJSON_AddNumberToObject(root, "uptime", (int)(esp_timer_get_time() / 1000000));
    cJSON_AddNumberToObject(root, "jump_height", jh);
    cJSON_AddNumberToObject(root, "jump_land_height", jl);
    cJSON_AddNumberToObject(root, "jump_speed", js);
    cJSON_AddNumberToObject(root, "jump_acc", ja);
    cJSON_AddNumberToObject(root, "jump_land_ms", jt);
    cJSON_AddNumberToObject(root, "jump_crouch", jc);
    cJSON_AddNumberToObject(root, "jump_crouch_ms", jct);
    cJSON_AddNumberToObject(root, "leg1_target", t.leg_target1);
    cJSON_AddNumberToObject(root, "leg2_target", t.leg_target2);
    cJSON_AddNumberToObject(root, "jump_state", t.jump_state);
    cJSON_AddNumberToObject(root, "manual_legs", t.manual_legs);
    cJSON_AddNumberToObject(root, "bump_leg", t.bump_state);
    cJSON_AddNumberToObject(root, "bump_amp", bamp);
    cJSON_AddNumberToObject(root, "bump_ms", bms);
    cJSON_AddNumberToObject(root, "bump_speed", bspd);
    cJSON_AddNumberToObject(root, "bump_acc", bacc);
    cJSON_AddNumberToObject(root, "yaw_fused", t.yaw_fused);
    cJSON_AddNumberToObject(root, "yaw_wheel_rate", t.yaw_wheel_rate);
    cJSON_AddNumberToObject(root, "yaw_wheel_scale", yws);
    cJSON_AddNumberToObject(root, "yaw_wheel_corr", ywc);
    cJSON_AddNumberToObject(root, "gyro_z_offset", sensors_gyro_offset_z());
    cJSON_AddNumberToObject(root, "zauto", t.zero_auto);
    cJSON_AddNumberToObject(root, "zero_trim_rate", robot_control_get_zero_trim());

    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (body == NULL) {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
        return ESP_OK;
    }
    http_set_cors(request);
    httpd_resp_set_type(request, "application/json");
    esp_err_t err = httpd_resp_send(request, body, HTTPD_RESP_USE_STRLEN);
    cJSON_free(body);
    return err;
}
