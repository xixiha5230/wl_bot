#include "robot_control_internal.h"

/*
 * PID and low-pass-filter tuning surface. Only the shared settings snapshot is
 * touched here; the control task copies the gains into its live PID/filter
 * objects once per loop, so tuning never races the real-time state.
 */

/* ------------------------------------------------------------------------- */
/* PID / LPF tuning                                                          */
/* ------------------------------------------------------------------------- */

static const char *const pid_names[ROBOT_PID_COUNT] = {
    "angle", "gyro", "distance", "speed", "yaw_angle",
    "yaw_gyro", "lqr_u", "roll_angle",
};

static const char *const lpf_names[ROBOT_LPF_COUNT] = {
    "joyy", "roll",
};

int robot_control_pid_count(void)
{
    return ROBOT_PID_COUNT;
}

const char *robot_control_pid_name(int which)
{
    return (which >= 0 && which < ROBOT_PID_COUNT) ? pid_names[which] : "";
}

void robot_control_get_pid(int which, float *p, float *i, float *d, float *limit)
{
    if (which < 0 || which >= ROBOT_PID_COUNT || !shared_ready()) {
        return;
    }
    shared_lock();
    if (p) *p = shared.pid[which].p;
    if (i) *i = shared.pid[which].i;
    if (d) *d = shared.pid[which].d;
    if (limit) *limit = shared.pid[which].limit;
    shared_unlock();
}

void robot_control_set_pid(int which, float p, float i, float d, float limit)
{
    if (which < 0 || which >= ROBOT_PID_COUNT || !shared_ready()) {
        return;
    }
    shared_lock();
    shared.pid[which].p = p;
    if (i >= 0.0f) shared.pid[which].i = i;
    if (d >= 0.0f) shared.pid[which].d = d;
    if (limit > 0.0f) shared.pid[which].limit = limit;
    shared_unlock();
}

int robot_control_lpf_count(void)
{
    return ROBOT_LPF_COUNT;
}

const char *robot_control_lpf_name(int which)
{
    return (which >= 0 && which < ROBOT_LPF_COUNT) ? lpf_names[which] : "";
}

void robot_control_get_lpf(int which, float *tf)
{
    if (which < 0 || which >= ROBOT_LPF_COUNT || !shared_ready()) {
        return;
    }
    shared_lock();
    if (tf) *tf = shared.lpf_tf[which];
    shared_unlock();
}

void robot_control_set_lpf(int which, float tf)
{
    if (which < 0 || which >= ROBOT_LPF_COUNT || !shared_ready()) {
        return;
    }
    shared_lock();
    shared.lpf_tf[which] = tf;
    shared_unlock();
}
