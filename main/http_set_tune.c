#include "http_set_internal.h"

#include "esp_http_server.h"
#include "robot_control.h"

#include <stdlib.h>
#include <string.h>

/*
 * Balance / yaw / filter tuning keys. Each handler is one entry in the table at
 * the bottom; it reads its own value (and any neighbour keys) from the query
 * and appends what it applied to the note buffer.
 */

static void set_zero(const char *query, const char *value, char *applied, size_t cap)
{
    (void)query;
    float v = strtof(value, NULL);
    robot_control_set_angle_zeropoint(v);
    http_set_note(applied, cap, "zero=%.2f ", v);
}

/* Balance-zero self-calibration: zadapt=<deg/s> (0 = off), zauto=0 reset. */
static void set_zadapt(const char *query, const char *value, char *applied, size_t cap)
{
    (void)query;
    robot_control_set_zero_trim(strtof(value, NULL));
    http_set_note(applied, cap, "zadapt=%.3f ", robot_control_get_zero_trim());
}

static void set_zauto(const char *query, const char *value, char *applied, size_t cap)
{
    (void)query;
    if (atoi(value) == 0) {
        robot_control_reset_zero_auto();
        http_set_note(applied, cap, "zauto=0 ");
    }
}

static void set_yaw(const char *query, const char *value, char *applied, size_t cap)
{
    (void)query;
    int v = atoi(value);
    robot_control_set_yaw_mode(v);
    http_set_note(applied, cap, "yaw=%d ", v);
}

/* yawscale=<deg/s per rad/s>, yawcorr=<1/s>; both are read-modified together. */
static void set_yaw_wheel(const char *query, const char *value, char *applied, size_t cap)
{
    (void)value;
    char v[32];
    float scale, corr;
    robot_control_get_yaw_wheel(&scale, &corr);
    bool any = false;
    if (httpd_query_key_value(query, "yawscale", v, sizeof(v)) == ESP_OK) {
        scale = strtof(v, NULL);
        any = true;
    }
    if (httpd_query_key_value(query, "yawcorr", v, sizeof(v)) == ESP_OK) {
        corr = strtof(v, NULL);
        any = true;
    }
    if (any) {
        robot_control_set_yaw_wheel(scale, corr);
        robot_control_get_yaw_wheel(&scale, &corr);
        http_set_note(applied, cap, "yawscale=%.2f yawcorr=%.2f ", scale, corr);
    }
}

static void set_rollmode(const char *query, const char *value, char *applied, size_t cap)
{
    (void)query;
    int v = atoi(value);
    robot_control_set_roll_mode(v);
    http_set_note(applied, cap, "rollmode=%d ", v);
}

static void set_rb(const char *query, const char *value, char *applied, size_t cap)
{
    (void)query;
    robot_control_set_roll_bias(strtof(value, NULL));
    http_set_note(applied, cap, "rb=%.2f ", robot_control_get_roll_bias());
}

/* Presence alone re-levels the chassis (the value is ignored). */
static void set_rblevel(const char *query, const char *value, char *applied, size_t cap)
{
    (void)query;
    (void)value;
    http_set_note(applied, cap, "rblevel=%.2f ", robot_control_calibrate_level());
}

static void set_faultdeg(const char *query, const char *value, char *applied, size_t cap)
{
    (void)query;
    robot_control_set_fault_deg(strtof(value, NULL));
    http_set_note(applied, cap, "faultdeg=%.1f ", robot_control_get_fault_deg());
}

static void set_air(const char *query, const char *value, char *applied, size_t cap)
{
    (void)value;
    char v[32];
    float th = robot_control_get_air_thresh();
    float sc = robot_control_get_air_scale();
    if (httpd_query_key_value(query, "airth", v, sizeof(v)) == ESP_OK) {
        th = strtof(v, NULL);
    }
    if (httpd_query_key_value(query, "airscale", v, sizeof(v)) == ESP_OK) {
        sc = strtof(v, NULL);
    }
    robot_control_set_air(th, sc);
    http_set_note(applied, cap, "airth=%.2f airscale=%.2f ",
         robot_control_get_air_thresh(), robot_control_get_air_scale());
}
/* pid=<name>&p=..[&i=..][&d=..][&limit=..] */
static void set_pid(const char *query, const char *value, char *applied, size_t cap)
{
    (void)value;
    char name[24];
    char v[32];
    if (httpd_query_key_value(query, "pid", name, sizeof(name)) != ESP_OK) {
        return;
    }
    int which = -1;
    for (int i = 0; i < robot_control_pid_count(); ++i) {
        if (strcmp(name, robot_control_pid_name(i)) == 0) {
            which = i;
            break;
        }
    }
    if (which < 0 || httpd_query_key_value(query, "p", v, sizeof(v)) != ESP_OK) {
        return;
    }
    float p = strtof(v, NULL);
    float i = -1.0f;
    float d = -1.0f;
    float limit = -1.0f;
    if (httpd_query_key_value(query, "i", v, sizeof(v)) == ESP_OK) i = strtof(v, NULL);
    if (httpd_query_key_value(query, "d", v, sizeof(v)) == ESP_OK) d = strtof(v, NULL);
    if (httpd_query_key_value(query, "limit", v, sizeof(v)) == ESP_OK) limit = strtof(v, NULL);
    robot_control_set_pid(which, p, i, d, limit);
    http_set_note(applied, cap, "pid %s P=%.4g I=%.4g ", name, p, i);
}

/* lpf=<joyy|roll>&tf=.. */
static void set_lpf(const char *query, const char *value, char *applied, size_t cap)
{
    (void)value;
    char name[24];
    char v[32];
    if (httpd_query_key_value(query, "lpf", name, sizeof(name)) != ESP_OK ||
        httpd_query_key_value(query, "tf", v, sizeof(v)) != ESP_OK) {
        return;
    }
    float tf = strtof(v, NULL);
    for (int i = 0; i < robot_control_lpf_count(); ++i) {
        if (strcmp(name, robot_control_lpf_name(i)) == 0) {
            robot_control_set_lpf(i, tf);
            http_set_note(applied, cap, "lpf %s=%.4g ", name, tf);
            break;
        }
    }
}

static const http_set_entry_t tune_entries[] = {
    {"zero", set_zero},
    {"zadapt", set_zadapt},
    {"zauto", set_zauto},
    {"yaw", set_yaw},
    {"yawscale", set_yaw_wheel},
    {"yawcorr", set_yaw_wheel},
    {"rollmode", set_rollmode},
    {"rb", set_rb},
    {"rblevel", set_rblevel},
    {"faultdeg", set_faultdeg},
    {"airth", set_air},
    {"airscale", set_air},
    {"pid", set_pid},
    {"lpf", set_lpf},
};

const http_set_entry_t *http_set_tune_entries(size_t *count)
{
    *count = sizeof(tune_entries) / sizeof(tune_entries[0]);
    return tune_entries;
}
