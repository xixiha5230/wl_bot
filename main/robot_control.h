#pragma once

#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Robot motion direction, mirrors QR_State_t in the original firmware. */
typedef enum {
    ROBOT_FORWARD = 0,
    ROBOT_BACK,
    ROBOT_RIGHT,
    ROBOT_LEFT,
    ROBOT_STOP,
    ROBOT_JUMP,
} robot_dir_t;

/* Remote-control command, mirrors Wrobot in the original firmware.
 * Written by the console/HTTP/WebSocket tasks and consumed by the control task,
 * so all access goes through the thread-safe helpers below. */
typedef struct {
    int height;
    int roll;
    int linear;
    int angular;
    int dir;
    int joy_x;
    int joy_y;
    bool go;
} robot_command_t;

/* Start the balance/yaw/leg control plus leg-output tasks.
 * Call after the motors are aligned. */
esp_err_t robot_control_start(void);

/* Thread-safe command accessors (any task). */
void robot_control_set_go(bool go);
void robot_control_set_height(int height);
void robot_control_set_dir(int dir);
void robot_control_set_joy(int joy_x, int joy_y);
void robot_control_set_roll(int roll);
void robot_control_set_linear(int linear);
void robot_control_set_angular(int angular);
robot_command_t robot_control_get_command(void);

/* Yaw correction mode for diagnostics: 1 = normal, -1 = inverted, 0 = disabled. */
void robot_control_set_yaw_mode(int mode);
int robot_control_get_yaw_mode(void);

/* Runtime tuning, mirroring the reference firmware's SimpleFOC Commander.
 * Names match the original A-L mappings (angle, gyro, distance, speed,
 * yaw_angle, yaw_gyro, lqr_u, zeropoint, roll_angle, joyy, zeropoint_lpf, roll). */
typedef enum {
    ROBOT_PID_ANGLE = 0,
    ROBOT_PID_GYRO,
    ROBOT_PID_DISTANCE,
    ROBOT_PID_SPEED,
    ROBOT_PID_YAW_ANGLE,
    ROBOT_PID_YAW_GYRO,
    ROBOT_PID_LQR_U,
    ROBOT_PID_ZEROPOINT,
    ROBOT_PID_ROLL_ANGLE,
    ROBOT_PID_COUNT,
} robot_pid_t;

int robot_control_pid_count(void);
const char *robot_control_pid_name(int which);
void robot_control_get_pid(int which, float *p, float *i, float *d, float *limit);
/* Pass a negative value to leave I/D/limit unchanged. */
void robot_control_set_pid(int which, float p, float i, float d, float limit);

/* Low-pass filters: 0 = joyy, 1 = zeropoint, 2 = roll. */
void robot_control_get_lpf(int which, float *tf);
void robot_control_set_lpf(int which, float tf);

/* Balance zero point in degrees (the original 'I'/'angle_zeropoint'). */
void robot_control_set_angle_zeropoint(float degrees);
float robot_control_get_angle_zeropoint(void);
/* Effective balance zero after the height compensation (for display). */
float robot_control_get_balance_zero(void);

float robot_control_lqr_angle(void);
float robot_control_lqr_u(void);
float robot_control_yaw_output(void);
float robot_control_yaw_total(void);
/* Individual LQR term outputs, for tuning. */
void robot_control_get_terms(float *angle, float *gyro, float *distance, float *speed);
/* Leg roll compensation and the last IMU roll angle, for tuning. */
float robot_control_leg_add(void);
float robot_control_roll_angle(void);
bool robot_control_faulted(void);

/* Number of control-loop iterations since boot (for rate diagnostics). */
uint32_t robot_control_loop_count(void);

#ifdef __cplusplus
}
#endif
