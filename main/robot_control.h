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

/* Roll correction sign, same convention (flip live to A/B test the sense). */
void robot_control_set_roll_mode(int mode);
int robot_control_get_roll_mode(void);

/* IMU roll mounting bias in degrees (raw angle_x at visually level). */
void robot_control_set_roll_bias(float bias);
float robot_control_get_roll_bias(void);
/* Pitch threshold (deg) that latches an attitude fault. */
void robot_control_set_fault_deg(float degrees);
float robot_control_get_fault_deg(void);

/* Airborne detection: specific-force threshold (g) and output scale while
 * airborne. Also exposes the current magnitude and flag for telemetry. */
void robot_control_set_air(float thresh_g, float scale);
float robot_control_get_air_thresh(void);
float robot_control_get_air_scale(void);
float robot_control_accel_mag(void);
void robot_control_get_accel(float *x, float *y, float *z);
int robot_control_airborne(void);

/* Research: drive both wheels at `target` (LQR_u units, clamped to +/-12) for
 * `ms` milliseconds, bypassing balance and fault handling. */
void robot_control_manual_drive(float target, int ms);
bool robot_control_manual_active(void);
int robot_control_manual_ticks(void);

/* Self-right: drive the wheels toward upright while attitude-faulted, then
 * hand over to the balance loop. [on]=1 starts, 0 cancels; params are torque
 * (wheel units), release angle (deg) and direction sign. */
void robot_control_set_getup(int on);
void robot_control_set_getup_params(float torque, float release_deg, int sign);
float robot_control_getup_torque(void);
float robot_control_getup_release(void);
int robot_control_getup_sign(void);
int robot_control_getup_state(void);
int robot_control_fault_reason(void);
/* One-shot level calibration: disables roll correction so both legs sit at
 * their symmetric nominal pose (the mechanical horizontal reference), averages
 * the IMU roll, stores it as the new bias in NVS, then restores the loop.
 * Takes ~1.4 s and must be called with the robot standing on flat ground. */
float robot_control_calibrate_level(void);

/* Jump profile, tunable at runtime (defaults from LEG_JUMP_* in robot_config.h):
 *   height/land_height in mm, speed 0..2000 (0 = max), acc 0..100,
 *   land_ticks = control ticks (~1/500 s) after launch before the land command. */
void robot_control_set_jump_profile(int height, int land_height, int speed,
                                    int acc, int land_ticks);
void robot_control_get_jump_profile(int *height, int *land_height, int *speed,
                                    int *acc, int *land_ticks);
/* Crouch phase of the jump gait: height and hold time in control ticks. */
void robot_control_set_jump_crouch(int height, int ticks);
void robot_control_get_jump_crouch(int *height, int *ticks);
/* Diagnostic: last leg positions sent to servos, and jump state (0/idle). */
void robot_control_get_leg_diag(int16_t *target1, int16_t *target2, int *jump_state);

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
/* Peak-to-peak of lqr_angle over the last ~1 s window (jitter metric). */
float robot_control_angle_pp(void);

float robot_control_lqr_angle(void);
float robot_control_lqr_u(void);
float robot_control_yaw_output(void);
float robot_control_yaw_total(void);
float robot_control_left_velocity(void);
float robot_control_right_velocity(void);
float robot_control_gyro_z(void);
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
