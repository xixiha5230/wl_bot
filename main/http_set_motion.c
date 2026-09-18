#include "http_set_internal.h"

#include "esp_http_server.h"
#include "robot_config.h"
#include "robot_control.h"
#include "sensors.h"

#include <stdlib.h>
#include <string.h>

/*
 * Motion keys: height, leg travel/hold, bump, reset, wheel sequence, get-up,
 * go and the jump profile. Each handler is one entry in the table at the
 * bottom; it reads its own value (and any neighbour keys) from the query.
 */

static void set_wdrive(const char *query, const char *value, char *applied, size_t cap)
{
    float target = strtof(value, NULL);
    int ms = 300;
    char v[32];
    if (httpd_query_key_value(query, "wms", v, sizeof(v)) == ESP_OK) {
        ms = atoi(v);
    }
    robot_control_manual_drive(target, ms);
    http_set_note(applied, cap, "wdrive=%.2f for %dms ", target, ms);
}

static void set_height(const char *query, const char *value, char *applied, size_t cap)
{
    (void)query;
    robot_control_set_height(atoi(value));
    http_set_note(applied, cap, "h=%d ", robot_control_get_command().height);
}

static void set_leg_limits(const char *query, const char *value, char *applied, size_t cap)
{
    (void)value;
    char v[32];
    int limits[4];
    robot_control_get_leg_limits(&limits[0], &limits[1], &limits[2], &limits[3]);
    bool any = false;
    static const char *const names[] = {"p1min", "p1max", "p2min", "p2max"};
    for (int i = 0; i < 4; ++i) {
        if (httpd_query_key_value(query, names[i], v, sizeof(v)) == ESP_OK) {
            limits[i] = atoi(v);
            any = true;
        }
    }
    if (any) {
        robot_control_set_leg_limits(limits[0], limits[1], limits[2], limits[3]);
        robot_control_get_leg_limits(&limits[0], &limits[1], &limits[2], &limits[3]);
        http_set_note(applied, cap, "leg p1[%d..%d] p2[%d..%d] ",
             limits[0], limits[1], limits[2], limits[3]);
    }
}

/* lp1=<pos1>/lp2=<pos2> hold both servos; a leg that is not named keeps its
 * last commanded target, so sending only lp1 no longer slams the other leg to
 * the servo centre. */
static void set_manual_legs(const char *query, const char *value, char *applied, size_t cap)
{
    (void)value;
    char v[32];
    bool has_lp1 = httpd_query_key_value(query, "lp1", v, sizeof(v)) == ESP_OK;
    int p1 = has_lp1 ? atoi(v) : 0;
    bool has_lp2 = httpd_query_key_value(query, "lp2", v, sizeof(v)) == ESP_OK;
    int p2 = has_lp2 ? atoi(v) : 0;
    if (!has_lp1 && !has_lp2) {
        return;
    }
    int16_t last1, last2;
    int jump_state;
    robot_control_get_leg_diag(&last1, &last2, &jump_state);
    if (!has_lp1) p1 = last1;
    if (!has_lp2) p2 = last2;
    robot_control_manual_legs(1, p1, p2);
    http_set_note(applied, cap, "lp1=%d lp2=%d ", p1, p2);
}

static void set_manual_legs_off(const char *query, const char *value, char *applied, size_t cap)
{
    (void)query;
    if (atoi(value) == 0) {
        robot_control_manual_legs(0, 0, 0);
        http_set_note(applied, cap, "lp=off ");
    }
}

static void set_bump_params(const char *query, const char *value, char *applied, size_t cap)
{
    (void)value;
    char v[32];
    int amp, ticks, speed, acc;
    robot_control_get_bump_params(&amp, &ticks, &speed, &acc);
    bool any = false;
    if (httpd_query_key_value(query, "bumpamp", v, sizeof(v)) == ESP_OK) { amp = atoi(v); any = true; }
    if (httpd_query_key_value(query, "bumpms", v, sizeof(v)) == ESP_OK) { ticks = atoi(v); any = true; }
    if (httpd_query_key_value(query, "bumpspeed", v, sizeof(v)) == ESP_OK) { speed = atoi(v); any = true; }
    if (httpd_query_key_value(query, "bumpacc", v, sizeof(v)) == ESP_OK) { acc = atoi(v); any = true; }
    if (any) {
        robot_control_set_bump_params(amp, ticks, speed, acc);
        robot_control_get_bump_params(&amp, &ticks, &speed, &acc);
        http_set_note(applied, cap, "bump amp=%d ms=%d speed=%d acc=%d ", amp, ticks, speed, acc);
    }
}

static void set_bump(const char *query, const char *value, char *applied, size_t cap)
{
    (void)query;
    int leg = atoi(value);
    robot_control_set_bump(leg);
    http_set_note(applied, cap, "bump=%d ", leg);
}

/* Return to the default standing pose: release manual legs, drop height and
 * roll to the defaults, stop, and clear the roll/yaw integrators so the legs
 * snap back symmetric and the robot holds its current heading. */
static void set_reset(const char *query, const char *value, char *applied, size_t cap)
{
    (void)query;
    if (atoi(value) != 0) {
        robot_control_manual_legs(0, 0, 0);
        robot_control_set_height(LEG_HEIGHT_DEFAULT);
        robot_control_set_roll(0);
        robot_control_set_dir(ROBOT_STOP);
        robot_control_reset_attitude();
        http_set_note(applied, cap, "reset(h=%d) ", LEG_HEIGHT_DEFAULT);
    }
}

/* seq=<t:ms,t:ms,...>[&arm=1]: multi-phase wheel torque sequence. */
static void set_seq(const char *query, const char *value, char *applied, size_t cap)
{
    char seq[160];
    snprintf(seq, sizeof(seq), "%s", value);
    float targets[8];
    int durs[8];
    int n = 0;
    char *p = seq;
    while (p != NULL && *p != '\0' && n < 8) {
        char *comma = strchr(p, ',');
        if (comma != NULL) {
            *comma = '\0';
        }
        char *colon = strchr(p, ':');
        if (colon != NULL) {
            *colon = '\0';
            targets[n] = strtof(p, NULL);
            durs[n] = atoi(colon + 1);
            n++;
        }
        if (comma == NULL) {
            break;
        }
        p = comma + 1;
    }
    if (n > 0) {
        robot_control_wheel_sequence(targets, durs, n);
        http_set_note(applied, cap, "seq=%d phases ", n);
    }
    char v[32];
    if (httpd_query_key_value(query, "arm", v, sizeof(v)) == ESP_OK) {
        robot_control_wheel_sequence_arm(atoi(v) != 0);
        http_set_note(applied, cap, "arm=%d ", atoi(v));
    }
}

/* Self-right: getup=1 starts, getup=0 cancels. */
static void set_getup(const char *query, const char *value, char *applied, size_t cap)
{
    (void)query;
    int v = atoi(value);
    robot_control_set_getup(v != 0);
    http_set_note(applied, cap, "getup=%d ", v);
}

static void set_getup_params(const char *query, const char *value, char *applied, size_t cap)
{
    (void)value;
    char v[32];
    float torque = robot_control_getup_torque();
    float release = robot_control_getup_release();
    int sign = robot_control_getup_sign();
    bool any = false;
    if (httpd_query_key_value(query, "gtorque", v, sizeof(v)) == ESP_OK) {
        torque = strtof(v, NULL);
        any = true;
    }
    if (httpd_query_key_value(query, "grelease", v, sizeof(v)) == ESP_OK) {
        release = strtof(v, NULL);
        any = true;
    }
    if (httpd_query_key_value(query, "gsign", v, sizeof(v)) == ESP_OK) {
        sign = atoi(v);
        any = true;
    }
    if (any) {
        robot_control_set_getup_params(torque, release, sign);
        http_set_note(applied, cap, "getup t=%.1f r=%.0f s=%d ",
             robot_control_getup_torque(), robot_control_getup_release(),
             robot_control_getup_sign());
    }
}

static void set_go(const char *query, const char *value, char *applied, size_t cap)
{
    (void)query;
    int v = atoi(value);
    robot_control_set_go(v != 0);
    http_set_note(applied, cap, "go=%d ", v);
}

static void set_gcal(const char *query, const char *value, char *applied, size_t cap)
{
    (void)query;
    (void)value;
    esp_err_t status = sensors_calibrate_gyro();
    http_set_note(applied, cap, "gcal=%s ", status == ESP_OK ? "ok" : "fail");
}

/* Jump profile and crouch: jh/jl/js/jacc/jlt and jc/jct, read-modified
 * together so a partial update keeps the values that were not sent. */
static void set_jump(const char *query, const char *value, char *applied, size_t cap)
{
    (void)value;
    char v[32];
    int params[5];
    bool any = false;
    static const char *const keys[] = {"jh", "jl", "js", "jacc", "jlt"};
    for (int i = 0; i < 5; ++i) {
        params[i] = -1;   /* -1 = absent, so explicit 0 values still apply */
        if (httpd_query_key_value(query, keys[i], v, sizeof(v)) == ESP_OK) {
            params[i] = atoi(v);
            any = true;
        }
    }
    int crouch = -1, crouch_ticks = -1;
    if (httpd_query_key_value(query, "jc", v, sizeof(v)) == ESP_OK) {
        crouch = atoi(v);
        any = true;
    }
    if (httpd_query_key_value(query, "jct", v, sizeof(v)) == ESP_OK) {
        crouch_ticks = atoi(v);
        any = true;
    }
    if (!any) {
        return;
    }
    int h, l, s, a, t;
    robot_control_get_jump_profile(&h, &l, &s, &a, &t);
    if (params[0] >= 0) h = params[0];
    if (params[1] >= 0) l = params[1];
    if (params[2] >= 0) s = params[2];
    if (params[3] >= 0) a = params[3];
    if (params[4] >= 0) t = params[4];
    robot_control_set_jump_profile(h, l, s, a, t);
    robot_control_set_jump_crouch(crouch, crouch_ticks);
    robot_control_get_jump_crouch(&crouch, &crouch_ticks);
    http_set_note(applied, cap, "jump h=%d l=%d s=%d a=%d t=%d c=%d ct=%d ",
         h, l, s, a, t, crouch, crouch_ticks);
}

static const http_set_entry_t motion_entries[] = {
    {"wdrive", set_wdrive},
    {"h", set_height},
    {"p1min", set_leg_limits},
    {"p1max", set_leg_limits},
    {"p2min", set_leg_limits},
    {"p2max", set_leg_limits},
    {"lp1", set_manual_legs},
    {"lp2", set_manual_legs},
    {"lp", set_manual_legs_off},
    {"bumpamp", set_bump_params},
    {"bumpms", set_bump_params},
    {"bumpspeed", set_bump_params},
    {"bumpacc", set_bump_params},
    {"bump", set_bump},
    {"reset", set_reset},
    {"seq", set_seq},
    {"getup", set_getup},
    {"gtorque", set_getup_params},
    {"grelease", set_getup_params},
    {"gsign", set_getup_params},
    {"go", set_go},
    {"gcal", set_gcal},
    {"jh", set_jump},
    {"jl", set_jump},
    {"js", set_jump},
    {"jacc", set_jump},
    {"jlt", set_jump},
    {"jc", set_jump},
    {"jct", set_jump},
};

const http_set_entry_t *http_set_motion_entries(size_t *count)
{
    *count = sizeof(motion_entries) / sizeof(motion_entries[0]);
    return motion_entries;
}
