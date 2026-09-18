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
 * Built with cJSON rather than a 60-argument snprintf: a single misordered
 * argument in the old format string was undetectable at compile time.
 */

esp_err_t http_status_handler(httpd_req_t *request)
{
    robot_command_t cmd = robot_control_get_command();
    float ta, tg, td, ts;
    robot_control_get_terms(&ta, &tg, &td, &ts);
    int jh, jl, js, ja, jt, jc, jct, jf;
    int16_t leg1, leg2;
    int bamp, bms, bspd, bacc;
    float ax, ay, az, yws, ywc;
    int lim1min, lim1max, lim2min, lim2max;
    robot_control_get_leg_limits(&lim1min, &lim1max, &lim2min, &lim2max);
    robot_control_get_jump_profile(&jh, &jl, &js, &ja, &jt);
    robot_control_get_jump_crouch(&jc, &jct);
    robot_control_get_bump_params(&bamp, &bms, &bspd, &bacc);
    robot_control_get_yaw_wheel(&yws, &ywc);
    robot_control_get_leg_diag(&leg1, &leg2, &jf);
    robot_control_get_accel(&ax, &ay, &az);

    /* Build the JSON with cJSON rather than a 60-argument snprintf: a single
     * misordered argument in the old format was undetectable at compile time. */
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
        return ESP_OK;
    }
    cJSON_AddStringToObject(root, "state", robot_state_name(robot_state_get()));
    cJSON_AddNumberToObject(root, "battery", board_battery_voltage());
    cJSON_AddNumberToObject(root, "go", cmd.go ? 1 : 0);
    cJSON_AddNumberToObject(root, "height", cmd.height);
    cJSON_AddNumberToObject(root, "lqr_angle", robot_control_lqr_angle());
    cJSON_AddNumberToObject(root, "lqr_u", robot_control_lqr_u());
    cJSON_AddNumberToObject(root, "fault", robot_control_faulted() ? 1 : 0);
    cJSON_AddNumberToObject(root, "joy_x", cmd.joy_x);
    cJSON_AddNumberToObject(root, "joy_y", cmd.joy_y);
    cJSON_AddNumberToObject(root, "dir", cmd.dir);
    cJSON_AddNumberToObject(root, "balance_zero", robot_control_get_balance_zero());
    cJSON_AddNumberToObject(root, "yaw_mode", robot_control_get_yaw_mode());
    cJSON_AddNumberToObject(root, "roll_mode", robot_control_get_roll_mode());
    cJSON_AddNumberToObject(root, "roll_bias", robot_control_get_roll_bias());
    cJSON_AddNumberToObject(root, "fault_deg", robot_control_get_fault_deg());
    cJSON_AddNumberToObject(root, "accel_mag", robot_control_accel_mag());
    cJSON_AddNumberToObject(root, "air", robot_control_airborne());
    cJSON_AddNumberToObject(root, "air_thresh", robot_control_get_air_thresh());
    cJSON_AddNumberToObject(root, "air_scale", robot_control_get_air_scale());
    cJSON_AddNumberToObject(root, "ax", ax);
    cJSON_AddNumberToObject(root, "ay", ay);
    cJSON_AddNumberToObject(root, "az", az);
    cJSON_AddNumberToObject(root, "manual_ms", robot_control_manual_remaining_ms());
    cJSON_AddNumberToObject(root, "motor_mode", (int)motor_foc_get_mode());
    cJSON_AddNumberToObject(root, "motor_aligned", motor_foc_is_aligned() ? 1 : 0);
    cJSON_AddNumberToObject(root, "getup_state", robot_control_getup_state());
    cJSON_AddNumberToObject(root, "fault_reason", robot_control_fault_reason());
    cJSON_AddNumberToObject(root, "p1min", lim1min);
    cJSON_AddNumberToObject(root, "p1max", lim1max);
    cJSON_AddNumberToObject(root, "p2min", lim2min);
    cJSON_AddNumberToObject(root, "p2max", lim2max);
    cJSON_AddNumberToObject(root, "angle_pp", robot_control_angle_pp());
    cJSON_AddNumberToObject(root, "term_angle", ta);
    cJSON_AddNumberToObject(root, "term_gyro", tg);
    cJSON_AddNumberToObject(root, "term_distance", td);
    cJSON_AddNumberToObject(root, "term_speed", ts);
    cJSON_AddNumberToObject(root, "leg_add", robot_control_leg_add());
    cJSON_AddNumberToObject(root, "yaw_total", robot_control_yaw_total());
    cJSON_AddNumberToObject(root, "yaw_out", robot_control_yaw_output());
    cJSON_AddNumberToObject(root, "roll_angle", robot_control_roll_angle());
    cJSON_AddNumberToObject(root, "velocity_left", robot_control_left_velocity());
    cJSON_AddNumberToObject(root, "velocity_right", robot_control_right_velocity());
    cJSON_AddNumberToObject(root, "gyro_z", robot_control_gyro_z());
    cJSON_AddNumberToObject(root, "uptime", (int)(esp_timer_get_time() / 1000000));
    cJSON_AddNumberToObject(root, "jump_height", jh);
    cJSON_AddNumberToObject(root, "jump_land_height", jl);
    cJSON_AddNumberToObject(root, "jump_speed", js);
    cJSON_AddNumberToObject(root, "jump_acc", ja);
    cJSON_AddNumberToObject(root, "jump_land_ms", jt);
    cJSON_AddNumberToObject(root, "jump_crouch", jc);
    cJSON_AddNumberToObject(root, "jump_crouch_ms", jct);
    cJSON_AddNumberToObject(root, "leg1_target", leg1);
    cJSON_AddNumberToObject(root, "leg2_target", leg2);
    cJSON_AddNumberToObject(root, "jump_state", jf);
    cJSON_AddNumberToObject(root, "manual_legs", robot_control_manual_legs_active());
    cJSON_AddNumberToObject(root, "bump_leg", robot_control_bump_state());
    cJSON_AddNumberToObject(root, "bump_amp", bamp);
    cJSON_AddNumberToObject(root, "bump_ms", bms);
    cJSON_AddNumberToObject(root, "bump_speed", bspd);
    cJSON_AddNumberToObject(root, "bump_acc", bacc);
    cJSON_AddNumberToObject(root, "yaw_fused", robot_control_yaw_fused());
    cJSON_AddNumberToObject(root, "yaw_wheel_rate", robot_control_yaw_wheel_rate());
    cJSON_AddNumberToObject(root, "yaw_wheel_scale", yws);
    cJSON_AddNumberToObject(root, "yaw_wheel_corr", ywc);
    cJSON_AddNumberToObject(root, "gyro_z_offset", sensors_gyro_offset_z());
    cJSON_AddNumberToObject(root, "zauto", robot_control_get_zero_auto());
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
