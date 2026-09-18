#include "robot_control_internal.h"

/*
 * Telemetry readback. The control task publishes a coherent snapshot under
 * telemetry_mutex; these getters copy a single field at a time so the HTTP and
 * console tasks never see a half-updated control variable.
 */

/* ------------------------------------------------------------------------- */
/* Telemetry access                                                          */
/* ------------------------------------------------------------------------- */

void robot_control_get_telemetry(robot_telemetry_t *out)
{
    if (out == NULL) {
        return;
    }
    telemetry_lock();
    *out = telemetry;
    telemetry_unlock();
}

float robot_control_lqr_angle(void)
{
    telemetry_lock();
    float value = telemetry.lqr_angle;
    telemetry_unlock();
    return value;
}

float robot_control_lqr_u(void)
{
    telemetry_lock();
    float value = telemetry.lqr_u;
    telemetry_unlock();
    return value;
}

float robot_control_yaw_output(void)
{
    telemetry_lock();
    float value = telemetry.yaw_output;
    telemetry_unlock();
    return value;
}

float robot_control_yaw_total(void)
{
    telemetry_lock();
    float value = telemetry.yaw_total;
    telemetry_unlock();
    return value;
}

float robot_control_yaw_fused(void)
{
    telemetry_lock();
    float value = telemetry.yaw_fused;
    telemetry_unlock();
    return value;
}

float robot_control_yaw_wheel_rate(void)
{
    telemetry_lock();
    float value = telemetry.yaw_wheel_rate;
    telemetry_unlock();
    return value;
}

float robot_control_yaw_wheel_heading(void)
{
    telemetry_lock();
    float value = telemetry.yaw_wheel_heading;
    telemetry_unlock();
    return value;
}

float robot_control_left_velocity(void)
{
    telemetry_lock();
    float value = telemetry.left_velocity;
    telemetry_unlock();
    return value;
}

float robot_control_right_velocity(void)
{
    telemetry_lock();
    float value = telemetry.right_velocity;
    telemetry_unlock();
    return value;
}

float robot_control_gyro_z(void)
{
    telemetry_lock();
    float value = telemetry.gyro_z;
    telemetry_unlock();
    return value;
}

float robot_control_accel_mag(void)
{
    telemetry_lock();
    float value = telemetry.accel_mag;
    telemetry_unlock();
    return value;
}

void robot_control_get_accel(float *x, float *y, float *z)
{
    telemetry_lock();
    if (x) *x = telemetry.accel_x;
    if (y) *y = telemetry.accel_y;
    if (z) *z = telemetry.accel_z;
    telemetry_unlock();
}

int robot_control_airborne(void)
{
    telemetry_lock();
    int value = telemetry.airborne;
    telemetry_unlock();
    return value;
}

void robot_control_get_terms(float *angle, float *gyro, float *distance, float *speed)
{
    telemetry_lock();
    if (angle) *angle = telemetry.angle_term;
    if (gyro) *gyro = telemetry.gyro_term;
    if (distance) *distance = telemetry.distance_term;
    if (speed) *speed = telemetry.speed_term;
    telemetry_unlock();
}

float robot_control_leg_add(void)
{
    telemetry_lock();
    float value = telemetry.leg_add;
    telemetry_unlock();
    return value;
}

float robot_control_roll_angle(void)
{
    telemetry_lock();
    float value = telemetry.roll_angle;
    telemetry_unlock();
    return value;
}

float robot_control_get_balance_zero(void)
{
    telemetry_lock();
    float value = telemetry.balance_zero;
    telemetry_unlock();
    return value;
}

float robot_control_angle_pp(void)
{
    telemetry_lock();
    float value = telemetry.angle_pp;
    telemetry_unlock();
    return value;
}

bool robot_control_faulted(void)
{
    telemetry_lock();
    bool value = telemetry.fault_reason != FAULT_NONE;
    telemetry_unlock();
    return value;
}

void robot_control_get_leg_diag(int16_t *target1, int16_t *target2, int *jump_state)
{
    telemetry_lock();
    if (target1) *target1 = telemetry.leg_target1;
    if (target2) *target2 = telemetry.leg_target2;
    if (jump_state) *jump_state = telemetry.jump_state;
    telemetry_unlock();
}
