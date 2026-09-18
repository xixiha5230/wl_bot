#pragma once

/*
 * Pure control logic shared by the firmware and the host test suite.
 *
 * Like robot_math.h this file touches no ESP-IDF, FreeRTOS or hardware, so the
 * state machines and predicates that decide how the robot behaves can be
 * exercised with a plain C compiler. The control task keeps the state and the
 * I/O; only the decision logic lives here.
 *
 * Keep behaviour identical to what the control loops need; the point is that
 * the firmware and the tests run the *same* code, not a copy.
 */

#include "robot_config.h"

#include <math.h>
#include <stdbool.h>

/* Fault latch reason, shared with the control task. */
typedef enum {
    FAULT_NONE = 0,
    FAULT_ATTITUDE,
    FAULT_BATTERY,
} fault_reason_t;

/*
 * Yaw heading: fuse the gyro (fast, no slip) with the wheel odometry (bias-free
 * but slip-prone), accumulate the operator's setpoint, and give up the
 * accumulated heading when the body is rotated uncommanded.
 */
typedef struct {
    float wheel;        /* integrated wheel-odometry heading (deg) */
    float fused;        /* integrated fused heading (deg) */
    float fused_last;   /* fused heading at the previous step (deg) */
    float setpoint;     /* accumulated heading setpoint (deg) */
    float wheel_rate;   /* wheel-derived yaw rate from the last step (deg/s) */
    bool  gave_up;      /* this step re-referenced the setpoint */
} robot_yaw_t;

static inline void robot_yaw_step(robot_yaw_t *yaw, float gyro_z, float wheel_rate,
                                  float corr, int joy_x, float dt)
{
    yaw->wheel_rate = wheel_rate;
    yaw->wheel += wheel_rate * dt;
    yaw->fused += gyro_z * dt;
    yaw->fused += corr * (yaw->wheel - yaw->fused) * dt;

    yaw->setpoint += yaw->fused - yaw->fused_last;
    yaw->fused_last = yaw->fused;

    /* The operator's yaw stick ramps the heading setpoint. */
    yaw->setpoint += (float)joy_x * YAW_STICK_RATE_DPS * dt;

    /* Give-up: re-reference instead of winding back a heading the operator did
     * not ask for. Fires once the accumulated error is large, or as soon as the
     * body is rotated hard (picked up / shoved). */
    yaw->gave_up = false;
    if (joy_x == 0 &&
        (fabsf(yaw->setpoint) > YAW_GIVEUP_DEG || fabsf(gyro_z) > YAW_GIVEUP_RATE)) {
        yaw->setpoint = 0.0f;
        yaw->gave_up = true;
    }
}

/* Re-reference the setpoint to the current heading (on arm / attitude reset),
 * so the accumulated heading while disarmed is not unwound. */
static inline void robot_yaw_hold(robot_yaw_t *yaw)
{
    yaw->setpoint = 0.0f;
    yaw->fused_last = yaw->fused;
}

/*
 * Residual (deg/s) for the balancing gyro-Z trim, from the wheel-derived yaw
 * rate. The gyro and the wheels measure the same body rate, so their difference
 * is the residual zero-rate bias - but only while both are small: a fast or
 * forced rotation makes the wheels slip, and then the reference is wrong.
 * Returns 0 when either rate is outside the trusted window.
 */
static inline float robot_yaw_bias_residual(float gyro_z, float wheel_rate, float max_rate)
{
    if (fabsf(gyro_z) > max_rate || fabsf(wheel_rate) > max_rate) {
        return 0.0f;
    }
    return gyro_z - wheel_rate;
}

/*
 * Airborne / drop detection: the accelerometer's specific-force magnitude drops
 * toward zero when both wheels leave the ground. The flag is held for AIR_HOLD_S
 * after the magnitude recovers, so a single noisy sample cannot toggle it.
 */
typedef struct {
    float hold_s;
    int airborne;
} robot_airborne_t;

static inline void robot_airborne_step(robot_airborne_t *air, float accel_mag_g,
                                       float thresh_g, float dt)
{
    if (accel_mag_g < thresh_g) {
        air->hold_s += dt;
        if (air->hold_s > AIR_HOLD_S) {
            air->hold_s = AIR_HOLD_S;
        }
    } else {
        air->hold_s -= dt;
        if (air->hold_s < 0.0f) {
            air->hold_s = 0.0f;
        }
    }
    air->airborne = (air->hold_s > 0.0f) ? 1 : 0;
}

/* Move `current` toward `target` by at most `max_step` (a slew-rate limit). */
static inline float robot_slew(float current, float target, float max_step)
{
    if (target > current) {
        return current + fminf(max_step, target - current);
    }
    return current - fminf(max_step, current - target);
}

/*
 * Jump gait phase: 1 crouch, 2 slam, 3 land, 0 idle. Returns the phase after
 * `elapsed_s`, or the same phase if it has not advanced yet.
 */
static inline int robot_jump_phase(int phase, float elapsed_s,
                                   int crouch_ticks, int land_ticks)
{
    const float slam_s = (float)(crouch_ticks + 2) * 0.001f;
    const float land_s = slam_s + (float)land_ticks * 0.001f;
    if (phase == 1 && elapsed_s >= slam_s) {
        return 2;
    }
    if (phase == 2 && elapsed_s >= land_s) {
        return 3;
    }
    if (phase == 3 && elapsed_s >= land_s + JUMP_SETTLE_S) {
        return 0;
    }
    return phase;
}

/* Leg-bump pulse phase from the pre-increment elapsed time: 1 extend, 2
 * retract, 0 finished. */
static inline int robot_bump_phase(float elapsed_s, int ticks)
{
    const float extend_s = (float)ticks * 0.001f;
    if (elapsed_s >= 2.0f * extend_s) {
        return 0;
    }
    return (elapsed_s < extend_s) ? 1 : 2;
}

/* Attitude fault: |pitch| past the threshold. */
static inline bool robot_attitude_faulted(float pitch_deg, float threshold_deg)
{
    return fabsf(pitch_deg) > threshold_deg;
}

/* Whether a latched fault's recovery condition currently holds. */
static inline bool robot_fault_recover_condition(fault_reason_t reason, float pitch_deg,
                                                 float battery_v, float battery_recover_v)
{
    if (reason == FAULT_BATTERY) {
        return battery_v > battery_recover_v;
    }
    return fabsf(pitch_deg) < ATTITUDE_RECOVER_DEG;
}

/* Advance a hold timer; true once `condition_ok` has held for `need_s`. */
static inline bool robot_hold_elapsed(float *hold_s, bool condition_ok, float dt, float need_s)
{
    if (!condition_ok) {
        *hold_s = 0.0f;
        return false;
    }
    *hold_s += dt;
    if (*hold_s >= need_s) {
        *hold_s = 0.0f;
        return true;
    }
    return false;
}

/* One step of the self-calibrating balance zero: walk it toward the measured
 * pitch error, bounded, and only while the error is small (a bigger error is a
 * push, not a bias). */
static inline float robot_zero_trim(float zero_auto, float err, float rate, float dt)
{
    if (fabsf(err) >= LEG_BALANCE_ZERO_TRIM_BAND) {
        return zero_auto;
    }
    zero_auto += rate * err * dt;
    if (zero_auto > LEG_BALANCE_ZERO_TRIM_MAX) {
        zero_auto = LEG_BALANCE_ZERO_TRIM_MAX;
    } else if (zero_auto < -LEG_BALANCE_ZERO_TRIM_MAX) {
        zero_auto = -LEG_BALANCE_ZERO_TRIM_MAX;
    }
    return zero_auto;
}
