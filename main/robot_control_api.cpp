#include "robot_control_internal.h"

#include <stddef.h>

/*
 * Public setter/getter surface for commands, tunables, tuning and telemetry.
 *
 * These run on the console / HTTP / WebSocket tasks (core 0). They only touch
 * the `shared` snapshot and the `telemetry` snapshot through the locks declared
 * in robot_control_internal.h; the real-time control task lives in
 * robot_control.cpp.
 */

/* ------------------------------------------------------------------------- */
/* Command / tunable access (thread-safe)                                    */
/* ------------------------------------------------------------------------- */

robot_command_t robot_control_get_command(void)
{
    robot_command_t snapshot = {};
    shared_lock();
    snapshot = shared.command;
    shared_unlock();
    return snapshot;
}

void robot_control_set_go(bool go)
{
    if (!shared_ready()) {
        return;
    }
    shared_lock();
    shared.command.go = go;
    shared_unlock();

    if (!go) {
        /* An explicit stop cancels a pending auto-recovery. The fault latch
         * itself is owned by the control task, so deliver the intent as a
         * request instead of writing it here. */
        robot_request_t request = {};
        request.kind = REQ_GO_OFF;
        post_request(&request);
    }
}

void robot_control_set_height(int height)
{
    /* Any height command means "drive the legs normally": drop a stale manual
     * hold so the operator can always recover control over Wi-Fi. */
    if (height < LEG_HEIGHT_MIN) {
        height = LEG_HEIGHT_MIN;
    } else if (height > LEG_HEIGHT_MAX) {
        height = LEG_HEIGHT_MAX;
    }
    if (!shared_ready()) {
        return;
    }
    shared_lock();
    shared.manual_leg_enable = false;
    shared.command.height = height;
    shared_unlock();
}

void robot_control_set_dir(int dir)
{
    if (!shared_ready()) return;
    shared_lock();
    shared.command.dir = dir;
    shared_unlock();
}

void robot_control_set_joy(int joy_x, int joy_y)
{
    if (!shared_ready()) return;
    shared_lock();
    shared.command.joy_x = joy_x;
    shared.command.joy_y = joy_y;
    shared_unlock();
}

void robot_control_set_roll(int roll)
{
    if (!shared_ready()) return;
    shared_lock();
    shared.command.roll = roll;
    shared_unlock();
}

void robot_control_set_linear(int linear)
{
    if (!shared_ready()) return;
    shared_lock();
    shared.command.linear = linear;
    shared_unlock();
}

void robot_control_set_angular(int angular)
{
    if (!shared_ready()) return;
    shared_lock();
    shared.command.angular = angular;
    shared_unlock();
}

void robot_control_set_yaw_mode(int mode)
{
    if (!shared_ready()) return;
    shared_lock();
    shared.yaw_mode = (mode > 0) ? 1 : (mode < 0 ? -1 : 0);
    shared_unlock();
}

int robot_control_get_yaw_mode(void)
{
    if (!shared_ready()) return 1;
    shared_lock();
    int mode = shared.yaw_mode;
    shared_unlock();
    return mode;
}

void robot_control_set_roll_mode(int mode)
{
    if (!shared_ready()) return;
    shared_lock();
    shared.roll_mode = (mode > 0) ? 1 : (mode < 0 ? -1 : 0);
    shared_unlock();
}

int robot_control_get_roll_mode(void)
{
    if (!shared_ready()) return 1;
    shared_lock();
    int mode = shared.roll_mode;
    shared_unlock();
    return mode;
}

void robot_control_set_roll_bias(float bias)
{
    if (!(bias > -10.0f && bias < 10.0f) || !shared_ready()) {
        return;
    }
    shared_lock();
    shared.roll_bias = bias;
    shared_unlock();
    roll_bias_save();   /* keep a manual rb across reboots */
}

float robot_control_get_roll_bias(void)
{
    if (!shared_ready()) return 0.0f;
    shared_lock();
    float bias = shared.roll_bias;
    shared_unlock();
    return bias;
}

void robot_control_set_fault_deg(float degrees)
{
    if (!(degrees >= 15.0f && degrees <= 80.0f) || !shared_ready()) {
        return;
    }
    shared_lock();
    shared.fault_deg = degrees;
    shared_unlock();
}

float robot_control_get_fault_deg(void)
{
    if (!shared_ready()) return ATTITUDE_FAULT_DEG;
    shared_lock();
    float degrees = shared.fault_deg;
    shared_unlock();
    return degrees;
}

void robot_control_set_air(float thresh_g, float scale)
{
    if (!shared_ready()) return;
    shared_lock();
    if (thresh_g > 0.05f && thresh_g < 1.5f) {
        shared.air_thresh_g = thresh_g;
    }
    if (scale >= 0.0f && scale <= 1.0f) {
        shared.air_scale = scale;
    }
    shared_unlock();
}

float robot_control_get_air_thresh(void)
{
    if (!shared_ready()) return AIR_THRESH_G;
    shared_lock();
    float thresh = shared.air_thresh_g;
    shared_unlock();
    return thresh;
}

float robot_control_get_air_scale(void)
{
    if (!shared_ready()) return AIR_SCALE;
    shared_lock();
    float scale = shared.air_scale;
    shared_unlock();
    return scale;
}

void robot_control_set_leg_limits(int pos1_min, int pos1_max, int pos2_min, int pos2_max)
{
    if (!shared_ready()) return;
    shared_lock();
    if (pos1_min > 0 && pos1_min < pos1_max) {
        shared.leg1_min = pos1_min;
        shared.leg1_max = pos1_max;
    }
    if (pos2_min > 0 && pos2_min < pos2_max) {
        shared.leg2_min = pos2_min;
        shared.leg2_max = pos2_max;
    }
    shared_unlock();
}

void robot_control_get_leg_limits(int *pos1_min, int *pos1_max, int *pos2_min, int *pos2_max)
{
    if (!shared_ready()) return;
    shared_lock();
    if (pos1_min) *pos1_min = shared.leg1_min;
    if (pos1_max) *pos1_max = shared.leg1_max;
    if (pos2_min) *pos2_min = shared.leg2_min;
    if (pos2_max) *pos2_max = shared.leg2_max;
    shared_unlock();
}

void robot_control_manual_legs(int enable, int pos1, int pos2)
{
    if (!shared_ready()) return;
    shared_lock();
    if (enable) {
        shared.manual_leg1 = pos1;
        shared.manual_leg2 = pos2;
    }
    shared.manual_leg_enable = enable ? true : false;
    shared_unlock();
}

int robot_control_manual_legs_active(void)
{
    if (!shared_ready()) return 0;
    shared_lock();
    int active = shared.manual_leg_enable ? 1 : 0;
    shared_unlock();
    return active;
}

void robot_control_set_bump(int leg)
{
    robot_request_t request = {};
    request.kind = REQ_BUMP;
    request.u.i[0] = (leg == 1 || leg == 2) ? leg : 0;
    post_request(&request);
}

void robot_control_set_bump_params(int amp, int ticks, int speed, int acc)
{
    if (!shared_ready()) return;
    shared_lock();
    if (amp > 0 && amp <= 400) shared.bump_amp = amp;
    if (ticks > 0 && ticks <= 2000) shared.bump_ticks = ticks;
    if (speed >= 0 && speed <= 3400) shared.bump_speed = speed;
    if (acc >= 0 && acc <= 255) shared.bump_acc = acc;
    shared_unlock();
}

void robot_control_get_bump_params(int *amp, int *ticks, int *speed, int *acc)
{
    if (!shared_ready()) return;
    shared_lock();
    if (amp) *amp = shared.bump_amp;
    if (ticks) *ticks = shared.bump_ticks;
    if (speed) *speed = shared.bump_speed;
    if (acc) *acc = shared.bump_acc;
    shared_unlock();
}

int robot_control_bump_state(void)
{
    telemetry_lock();
    int state = telemetry.bump_state;
    telemetry_unlock();
    return state;
}

void robot_control_reset_attitude(void)
{
    robot_request_t request = {};
    request.kind = REQ_ATTITUDE_RESET;
    post_request(&request);
}

void robot_control_manual_drive(float target, int ms)
{
    if (target < -30.0f) target = -30.0f;
    if (target > 30.0f) target = 30.0f;
    if (ms < 0) ms = 0;
    if (ms > 3000) ms = 3000;
    robot_request_t request = {};
    request.kind = REQ_MANUAL_DRIVE;
    request.u.f[0] = target;
    request.u.i[0] = ms;
    post_request(&request);
}

int robot_control_manual_remaining_ms(void)
{
    telemetry_lock();
    int remaining_ms = telemetry.manual_ms;
    telemetry_unlock();
    return remaining_ms;
}

void robot_control_wheel_sequence(const float *targets, const int *durations_ms, int count)
{
    if (count <= 0 || targets == NULL || durations_ms == NULL) {
        return;
    }
    if (count > WHEEL_SEQ_MAX) {
        count = WHEEL_SEQ_MAX;
    }
    robot_request_t request = {};
    request.kind = REQ_WHEEL_SEQ;
    request.u.seq.count = count;
    for (int i = 0; i < count; ++i) {
        float target = targets[i];
        if (target < -30.0f) target = -30.0f;
        if (target > 30.0f) target = 30.0f;
        int ms = durations_ms[i];
        if (ms < 0) ms = 0;
        if (ms > 3000) ms = 3000;
        request.u.seq.targets[i] = target;
        request.u.seq.durations_ms[i] = ms;
    }
    post_request(&request);
}

void robot_control_wheel_sequence_arm(bool arm)
{
    if (!shared_ready()) return;
    shared_lock();
    shared.wheel_seq_arm = arm;
    shared_unlock();
}

void robot_control_set_getup(int on)
{
    robot_request_t request = {};
    request.kind = REQ_GETUP;
    request.u.i[0] = on ? 1 : 0;
    post_request(&request);
}

void robot_control_set_getup_params(float torque, float release_deg, int sign)
{
    if (!shared_ready()) return;
    shared_lock();
    if (torque >= 0.0f && torque <= 30.0f) {
        shared.getup_torque = torque;
    }
    if (release_deg >= 2.0f && release_deg <= 45.0f) {
        shared.getup_release_deg = release_deg;
    }
    shared.getup_sign = (sign < 0) ? -1 : 1;
    shared_unlock();
}

void robot_control_set_getup_height(int height)
{
    if (!(height >= LEG_HEIGHT_MIN && height <= LEG_HEIGHT_MAX) || !shared_ready()) {
        return;
    }
    shared_lock();
    shared.getup_final_height = height;
    shared_unlock();
}

int robot_control_getup_height(void)
{
    if (!shared_ready()) return LEG_HEIGHT_DEFAULT;
    shared_lock();
    int height = shared.getup_final_height;
    shared_unlock();
    return height;
}

float robot_control_getup_torque(void)
{
    if (!shared_ready()) return GETUP_TORQUE;
    shared_lock();
    float value = shared.getup_torque;
    shared_unlock();
    return value;
}

float robot_control_getup_release(void)
{
    if (!shared_ready()) return GETUP_RELEASE_DEG;
    shared_lock();
    float value = shared.getup_release_deg;
    shared_unlock();
    return value;
}

int robot_control_getup_sign(void)
{
    if (!shared_ready()) return 1;
    shared_lock();
    int value = shared.getup_sign;
    shared_unlock();
    return value;
}

int robot_control_getup_state(void)
{
    telemetry_lock();
    int state = telemetry.getup_state;
    telemetry_unlock();
    return state;
}

int robot_control_fault_reason(void)
{
    telemetry_lock();
    int reason = telemetry.fault_reason;
    telemetry_unlock();
    return reason;
}

void robot_control_set_jump_profile(int height, int land_height, int speed,
                                    int acc, int land_ticks)
{
    if (!shared_ready()) return;
    shared_lock();
    if (height >= LEG_HEIGHT_MIN && height <= LEG_HEIGHT_MAX) {
        shared.jump_height = height;
    }
    if (land_height >= LEG_HEIGHT_MIN && land_height <= LEG_HEIGHT_MAX) {
        shared.jump_land_height = land_height;
    }
    if (speed >= 0 && speed <= 2000) {
        shared.jump_speed = speed;
    }
    if (acc >= 0 && acc <= 100) {
        shared.jump_acc = acc;
    }
    if (land_ticks > 5 && land_ticks < 200) {
        shared.jump_land_ticks = land_ticks;
    }
    shared_unlock();
}

void robot_control_get_jump_profile(int *height, int *land_height, int *speed,
                                    int *acc, int *land_ticks)
{
    if (!shared_ready()) return;
    shared_lock();
    if (height) *height = shared.jump_height;
    if (land_height) *land_height = shared.jump_land_height;
    if (speed) *speed = shared.jump_speed;
    if (acc) *acc = shared.jump_acc;
    if (land_ticks) *land_ticks = shared.jump_land_ticks;
    shared_unlock();
}

void robot_control_set_jump_crouch(int height, int ticks)
{
    if (!shared_ready()) return;
    shared_lock();
    if (height >= LEG_HEIGHT_MIN && height <= LEG_HEIGHT_MAX) {
        shared.jump_crouch = height;
    }
    if (ticks > 10 && ticks < 400) {
        shared.jump_crouch_ticks = ticks;
    }
    shared_unlock();
}

void robot_control_get_jump_crouch(int *height, int *ticks)
{
    if (!shared_ready()) return;
    shared_lock();
    if (height) *height = shared.jump_crouch;
    if (ticks) *ticks = shared.jump_crouch_ticks;
    shared_unlock();
}

void robot_control_set_angle_zeropoint(float degrees)
{
    if (!shared_ready()) return;
    shared_lock();
    shared.angle_zeropoint = degrees;
    shared_unlock();
}

float robot_control_get_angle_zeropoint(void)
{
    if (!shared_ready()) return LEG_BALANCE_ZERO_DEFAULT;
    shared_lock();
    float value = shared.angle_zeropoint;
    shared_unlock();
    return value;
}

void robot_control_set_zero_trim(float rate)
{
    if (!shared_ready()) return;
    shared_lock();
    if (rate >= 0.0f && rate < 20.0f) {
        shared.zero_trim = rate;
    }
    shared_unlock();
}

float robot_control_get_zero_trim(void)
{
    if (!shared_ready()) return 0.0f;
    shared_lock();
    float value = shared.zero_trim;
    shared_unlock();
    return value;
}

float robot_control_get_zero_auto(void)
{
    telemetry_lock();
    float value = telemetry.zero_auto;
    telemetry_unlock();
    return value;
}

void robot_control_reset_zero_auto(void)
{
    robot_request_t request = {};
    request.kind = REQ_ZERO_AUTO_RESET;
    post_request(&request);
}

void robot_control_set_yaw_wheel(float scale, float corr)
{
    if (!shared_ready()) return;
    shared_lock();
    if (scale > -100.0f && scale < 100.0f) {
        shared.yaw_wheel_scale = scale;
    }
    if (corr >= 0.0f && corr < 50.0f) {
        shared.yaw_wheel_corr = corr;
    }
    shared_unlock();
}

void robot_control_get_yaw_wheel(float *scale, float *corr)
{
    if (!shared_ready()) return;
    shared_lock();
    if (scale) *scale = shared.yaw_wheel_scale;
    if (corr) *corr = shared.yaw_wheel_corr;
    shared_unlock();
}
