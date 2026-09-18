#pragma once

#include <stdint.h>

/*
 * A coherent snapshot of the control task's telemetry.
 *
 * The control task publishes the whole struct under one lock, so a caller that
 * reads it through robot_control_get_telemetry() sees a consistent frame
 * instead of fields from two different control ticks. The individual
 * robot_control_*() getters remain for one-off reads.
 */
typedef struct {
    float lqr_angle, lqr_u;
    float angle_term, gyro_term, distance_term, speed_term;
    float balance_zero;
    float yaw_total, yaw_output, yaw_fused, yaw_wheel_rate, yaw_wheel_heading;
    float left_velocity, right_velocity, gyro_z;
    float leg_add, roll_angle;
    float angle_pp;
    float zero_auto;
    float accel_mag, accel_x, accel_y, accel_z;
    int airborne;
    int fault_reason;
    int getup_state;
    int bump_state;        /* 0 idle, 1 left, 2 right */
    int manual_legs;       /* 0/1 */
    int manual_ms;
    int jump_state;
    int16_t leg_target1, leg_target2;
} robot_telemetry_t;
