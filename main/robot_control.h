#pragma once

#include "esp_err.h"

#include <stdbool.h>

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

float robot_control_lqr_angle(void);
float robot_control_lqr_u(void);
bool robot_control_faulted(void);

#ifdef __cplusplus
}
#endif
