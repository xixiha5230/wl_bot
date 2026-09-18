#include "robot_control.h"

#include "board.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lowpass_filter.h"
#include "motor_foc.h"
#include "nvs_store.h"
#include "pid.h"
#include "robot_config.h"
#include "robot_control_internal.h"
#include "robot_math.h"
#include "robot_state.h"
#include "sensors.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <math.h>
#include <string.h>

/* Port of the balance / yaw / leg loops from wl_pro_robot.ino.
 * PID gains, filter time constants and the control structure follow the
 * original firmware. Angles are in degrees and gyro rates in deg/s, matching
 * MPU6050_tockn / our complementary filter.
 *
 * Concurrency model
 * -----------------
 * The control task is the only writer of the runtime state (LQR/yaw/leg
 * integrators, fault latch, get-up/bump state machines, PIDs and filters).
 * Every other task talks to it through exactly two mechanisms:
 *
 *   - `shared`: continuous commands and tunables. The control task takes a
 *     snapshot at the top of each loop, so the newest value always wins. All
 *     access is guarded by `shared_mutex`.
 *   - `request_queue`: discrete, ordered actions (a bump, a get-up, a wheel
 *     pulse) that must not be dropped, delivered once and consumed by the
 *     control task.
 *
 * Telemetry is published by the control task into `telemetry` under
 * `telemetry_mutex`, so the HTTP/console tasks never read a half-updated
 * control variable. */

/* All rates, thresholds and durations live in robot_config.h. */

/* PIDController::reset() clears the integral accumulator as well as the error
 * history; the original firmware's clear_error() (error_prev only) left
 * pid_lqr_u able to stay wound up after a fault/fall, so every "re-reference"
 * below uses reset() instead. */

/* Control-task-owned PID and filter objects. Their gains are refreshed from the
 * shared settings snapshot every loop, so tuning never touches them directly. */
static PIDController pid_angle(1.1f, 0.0f, 0.0f, 100000.0f, 8.0f);
static PIDController pid_gyro(0.09f, 0.0f, 0.0f, 100000.0f, 8.0f);
static PIDController pid_distance(0.4f, 0.0f, 0.0f, 100000.0f, 8.0f);
static PIDController pid_speed(0.4f, 0.0f, 0.0f, 100000.0f, 8.0f);
static PIDController pid_yaw_angle(1.0f, 0.0f, 0.0f, 100000.0f, 8.0f);
static PIDController pid_yaw_gyro(0.04f, 0.0f, 0.0f, 100000.0f, 8.0f);
static PIDController pid_lqr_u(1.0f, 15.0f, 0.0f, 100000.0f, 8.0f);
/* P=4, I=10: the proportional gain has to stay low because the leg servo +
 * LPF lag leaves little phase margin (P=10 rang at ~1.2 Hz). The integral then
 * does the real work: it levels a slope in ~1.5 s where I=1 took tens of
 * seconds. I=20 was rejected: on a large roll error (a fall) the integral
 * grew at ~600/s and slammed the legs to the clamps within a second, which
 * kicks the chassis; I=10 keeps the slope response while staying gentle.
 * Output limit 500 sits above the maximum differential the leg travel clamps
 * allow; the anti-windup keeps the integral inside the same limit. */
static PIDController pid_roll_angle(4.0f, 10.0f, 0.0f, 100000.0f, 500.0f);

static LowPassFilter lpf_joy_y(0.2f);
static LowPassFilter lpf_roll(0.3f);

/* ------------------------------------------------------------------------- */
/* Shared state (mutex)                                                      */
/* ------------------------------------------------------------------------- */

robot_shared_t shared;
SemaphoreHandle_t shared_mutex;

robot_telemetry_t telemetry;
SemaphoreHandle_t telemetry_mutex;

QueueHandle_t request_queue;

/* ------------------------------------------------------------------------- */
/* Control-task-private runtime state                                        */
/* ------------------------------------------------------------------------- */

/* LQR balance state. */
static float loop_dt = CONTROL_DT_DEFAULT;   /* measured control-loop period (s) */
static int64_t last_loop_us;
static float LQR_angle;
static float LQR_u;
static float LQR_gyro;
static float LQR_speed;
static float LQR_distance;
static float last_left_velocity;
static float last_right_velocity;
static float last_gyro_z;
static float angle_control;
static float gyro_control;
static float speed_control;
static float distance_control;
/* Balance zero: the base value plus the height model and the self-calibrated
 * `zero_auto` produce the effective zero each loop. */
static float distance_zeropoint = LQR_DISTANCE_SENTINEL;
static float zero_auto;
static bool zero_auto_reset;

/* Yaw state: fused heading (gyro + wheel odometry), see yaw_loop(). */
static float YAW_angle;          /* fused heading, deg (telemetry) */
static float YAW_angle_total;    /* accumulated heading setpoint, deg */
static float YAW_output;
static float yaw_wheel;          /* wheel-odometry heading, deg */
static float yaw_fused;          /* complementary fused heading, deg */
static float yaw_fused_last;
static float yaw_wheel_rate;     /* wheel-derived yaw rate, deg/s */

/* Previous command snapshot (for edge detection). */
static int prev_dir = ROBOT_STOP;
static int prev_joy_x;
static int prev_joy_y;
static bool prev_go;

/* Leg / motion state. */
static float robot_speed;
static int odometry_just_stopped;
static int jump_flag;
static float leg_position_add;
static float leg_height_cmd;
static float last_roll_angle;
static float last_balance_zero;
static float angle_pp;
static float gyro_trim_still_s;    /* time the gyro-Z trim has been at rest (s) */

/* Fault latch. */

static fault_reason_t fault_reason = FAULT_NONE;
static float recover_s;             /* time the recovery condition held (s) */
/* Latched at attitude-fault entry when the operator had go on: the robot
 * re-arms by itself once upright again, without a manual go command. */
static bool go_auto_latch;

/* Airborne / drop detection. While the wheels are off the ground the balance
 * loop's drive just spins them up; the leftover wheel speed then makes the
 * robot lunge forward on landing. Below `air_thresh_g` of specific force the
 * robot is treated as airborne and the balance output is scaled by
 * `air_scale` until it lands. */
static float air_hold_s;
static float accel_mag_g;
static int airborne;
static float accel_x_last;
static float accel_y_last;
static float accel_z_last;

/* Research: manual wheel torque / sequence. Durations are held in seconds and
 * decremented by the measured dt so the pulse lasts the requested wall time. */
static float manual_target;
static float manual_s_left;
static bool manual_armed;
static struct {
    float target;
    float seconds;
} wheel_seq[WHEEL_SEQ_MAX];
static int wheel_seq_len;
static int wheel_seq_idx;
static float wheel_seq_left_s;

/* Self-right state machine (research). */
static int getup_request;
static int getup_state;             /* 0 idle, 1 lowering legs */
static int getup_state_pub;         /* published copy for telemetry */
static float getup_s;               /* time spent lowering the legs (s) */
static int getup_low_height = GETUP_LOW_HEIGHT;

/* One-shot leg bump state machine. */
static int bump_flag;               /* runtime state machine, 0 idle */
static int bump_leg;                /* 1 left, 2 right */
static float bump_elapsed_s;        /* time since the pulse started (s) */

/* Jump gait timing (s) and phase (0 idle, 1 crouch, 2 slam, 3 land). */
static float jump_elapsed_s;

/* One-shot requests handled at the top of leg_loop(). */
static bool attitude_reset_request;
static bool arm_request;            /* reset distance zero/PIDs when go turns on */

static bool control_started;
static uint32_t control_loop_count;

/* ------------------------------------------------------------------------- */
/* Shared-state helpers                                                      */
/* ------------------------------------------------------------------------- */

bool shared_ready(void)
{
    return shared_mutex != NULL;
}

void shared_lock(void)
{
    if (shared_mutex != NULL) {
        xSemaphoreTake(shared_mutex, portMAX_DELAY);
    }
}

void shared_unlock(void)
{
    if (shared_mutex != NULL) {
        xSemaphoreGive(shared_mutex);
    }
}

void telemetry_lock(void)
{
    if (telemetry_mutex != NULL) {
        xSemaphoreTake(telemetry_mutex, portMAX_DELAY);
    }
}

void telemetry_unlock(void)
{
    if (telemetry_mutex != NULL) {
        xSemaphoreGive(telemetry_mutex);
    }
}

void post_request(const robot_request_t *request)
{
    if (request_queue == NULL) {
        return;
    }
    if (xQueueSend(request_queue, request, 0) != pdTRUE) {
        ESP_LOGW("robot_control", "request queue full, dropping kind %d",
                 (int)request->kind);
    }
}

/* Defaults for a fresh boot: these replace the old file-scope initialisers. */
static void shared_load_defaults(void)
{
    memset(&shared, 0, sizeof(shared));
    shared.command.height = LEG_HEIGHT_DEFAULT;
    shared.command.dir = ROBOT_STOP;

    shared.pid[ROBOT_PID_ANGLE]      = pid_gains_t{1.1f, 0.0f, 0.0f, 100000.0f};
    shared.pid[ROBOT_PID_GYRO]       = pid_gains_t{0.09f, 0.0f, 0.0f, 100000.0f};
    shared.pid[ROBOT_PID_DISTANCE]   = pid_gains_t{0.4f, 0.0f, 0.0f, 100000.0f};
    shared.pid[ROBOT_PID_SPEED]      = pid_gains_t{0.4f, 0.0f, 0.0f, 100000.0f};
    shared.pid[ROBOT_PID_YAW_ANGLE]  = pid_gains_t{1.0f, 0.0f, 0.0f, 100000.0f};
    shared.pid[ROBOT_PID_YAW_GYRO]   = pid_gains_t{0.04f, 0.0f, 0.0f, 100000.0f};
    shared.pid[ROBOT_PID_LQR_U]      = pid_gains_t{1.0f, 15.0f, 0.0f, 100000.0f};
    shared.pid[ROBOT_PID_ROLL_ANGLE] = pid_gains_t{4.0f, 10.0f, 0.0f, 500.0f};

    shared.lpf_tf[0] = 0.2f;   /* joyy */
    shared.lpf_tf[1] = 0.3f;   /* roll */

    shared.leg1_min = LEG_POS1_MIN;
    shared.leg1_max = LEG_POS1_MAX;
    shared.leg2_min = LEG_POS2_MIN;
    shared.leg2_max = LEG_POS2_MAX;

    shared.jump_height = LEG_JUMP_HEIGHT;
    shared.jump_land_height = LEG_JUMP_LAND_HEIGHT;
    shared.jump_speed = LEG_JUMP_SPEED;
    shared.jump_acc = LEG_JUMP_ACC;
    shared.jump_land_ticks = 32;
    shared.jump_crouch = 34;      /* chassis clearance; h32 scrapes */
    shared.jump_crouch_ticks = 150;

    shared.bump_amp = LEG_BUMP_AMP;
    shared.bump_ticks = LEG_BUMP_TICKS;
    shared.bump_speed = LEG_BUMP_SPEED;
    shared.bump_acc = LEG_BUMP_ACC;

    shared.getup_torque = GETUP_TORQUE;
    shared.getup_release_deg = GETUP_RELEASE_DEG;
    shared.getup_sign = 1;
    shared.getup_final_height = LEG_HEIGHT_DEFAULT;

    shared.yaw_mode = 1;
    shared.roll_mode = 1;
    shared.fault_deg = ATTITUDE_FAULT_DEG;
    shared.air_thresh_g = AIR_THRESH_G;
    shared.air_scale = AIR_SCALE;
    shared.zero_trim = LEG_BALANCE_ZERO_TRIM_RATE;
    shared.yaw_wheel_scale = YAW_WHEEL_SCALE;
    shared.yaw_wheel_corr = YAW_WHEEL_CORR;
    shared.roll_bias = 0.0f;
    shared.angle_zeropoint = LEG_BALANCE_ZERO_DEFAULT;

    leg_height_cmd = (float)LEG_HEIGHT_DEFAULT;
}

static PIDController *pid_at(int which)
{
    switch (which) {
    case ROBOT_PID_ANGLE:      return &pid_angle;
    case ROBOT_PID_GYRO:       return &pid_gyro;
    case ROBOT_PID_DISTANCE:   return &pid_distance;
    case ROBOT_PID_SPEED:      return &pid_speed;
    case ROBOT_PID_YAW_ANGLE:  return &pid_yaw_angle;
    case ROBOT_PID_YAW_GYRO:   return &pid_yaw_gyro;
    case ROBOT_PID_LQR_U:      return &pid_lqr_u;
    case ROBOT_PID_ROLL_ANGLE: return &pid_roll_angle;
    default:                   return NULL;
    }
}

static LowPassFilter *lpf_at(int which)
{
    switch (which) {
    case ROBOT_LPF_JOY_Y: return &lpf_joy_y;
    case ROBOT_LPF_ROLL:  return &lpf_roll;
    default:              return NULL;
    }
}


uint32_t robot_control_loop_count(void)
{
    return __atomic_load_n(&control_loop_count, __ATOMIC_RELAXED);
}

/* ------------------------------------------------------------------------- */
/* Control loops                                                             */
/* ------------------------------------------------------------------------- */

static void reset_pids(void)
{
    pid_angle.reset();
    pid_gyro.reset();
    pid_distance.reset();
    pid_speed.reset();
    pid_yaw_angle.reset();
    pid_yaw_gyro.reset();
    pid_lqr_u.reset();
    pid_roll_angle.reset();
}

/* Yaw: fuse the gyro (fast, no slip) with the wheel odometry (no zero-rate
 * offset) so a gyro-Z bias can no longer be turned into a real spin, and give
 * up the accumulated heading instead of fighting an uncommanded rotation. */
static void yaw_loop(const mpu6050_sample_t *imu, const robot_command_t *cmd,
                     const robot_shared_t *s)
{
    const float gz = imu->gyro_z_dps;

    /* Bias-free but slip-prone body yaw rate from the wheel differential. */
    const float wz = s->yaw_wheel_scale * (last_right_velocity - last_left_velocity);
    yaw_wheel_rate = wz;

    yaw_wheel += wz * loop_dt;
    yaw_fused += gz * loop_dt;
    yaw_fused += s->yaw_wheel_corr * (yaw_wheel - yaw_fused) * loop_dt;

    YAW_angle = yaw_fused;
    YAW_angle_total += yaw_fused - yaw_fused_last;
    yaw_fused_last = yaw_fused;

    /* The operator's yaw stick ramps the heading setpoint. */
    YAW_angle_total += (float)cmd->joy_x * YAW_STICK_RATE_DPS * loop_dt;

    /* Give-up: re-reference instead of winding back a heading the operator did
     * not ask for (mirrors the distance loop's distance_zeropoint reset). Fire
     * once the accumulated error is large, or as soon as the body is rotated
     * hard (picked up / shoved), so the wheels do not fight a manual turn. */
    if (cmd->joy_x == 0 &&
        (fabsf(YAW_angle_total) > YAW_GIVEUP_DEG || fabsf(gz) > YAW_GIVEUP_RATE)) {
        YAW_angle_total = 0.0f;
        pid_yaw_angle.reset();
    }

    float yaw_angle_control = pid_yaw_angle(YAW_angle_total);
    float yaw_gyro_control = pid_yaw_gyro(gz);
    YAW_output = yaw_angle_control + yaw_gyro_control;
}

static void lqr_balance_loop(const motor_feedback_t *left, const motor_feedback_t *right,
                             const mpu6050_sample_t *imu, const robot_command_t *cmd,
                             const robot_shared_t *s)
{
    LQR_distance = (-0.5f) * (left->angle + right->angle);
    LQR_speed = (-0.5f) * (left->velocity + right->velocity);
    last_left_velocity = left->velocity;
    last_right_velocity = right->velocity;
    last_gyro_z = imu->gyro_z_dps;
    LQR_angle = imu->angle_y;
    LQR_gyro = imu->gyro_y_dps;

    if (arm_request) {
        /* Start from the current wheel position instead of the -256 sentinel,
         * so enabling balance does not produce a large distance kick. */
        distance_zeropoint = LQR_distance;
        reset_pids();
        /* Hold the heading at the moment GO is enabled: the yaw accumulator
         * keeps integrating while disarmed (a fall, the robot being carried),
         * and unwinding it makes the robot spin on start-up. */
        YAW_angle_total = 0.0f;
        yaw_fused_last = yaw_fused;
        arm_request = false;
    }

    /* Balance zero compensates for the height-dependent CoM position, plus the
     * self-calibrated offset learned while standing still. */
    float zero = robot_balance_zero(s->angle_zeropoint, leg_height_cmd, zero_auto);
    last_balance_zero = zero;
    angle_control = pid_angle(LQR_angle - zero);
    gyro_control = pid_gyro(LQR_gyro);

    /* The distance loop integrates the wheel angle, so the odometry is only a
     * trustworthy position reference while the robot actually rolls on the
     * ground. While that is not true, hold the reference at the current
     * reading and drop the LQR_u bias instead of letting the loop wind up.
     *
     * "Not true" is: the operator is commanding motion, the wheels are slipping
     * (a per-loop speed step), spinning fast (a launch / airborne), or a jump
     * is in progress. A separate slower threshold re-references the position
     * while merely rolling fast. */
    const bool command_active = (cmd->joy_y != 0);
    const bool wheels_slipping =
        fabsf(LQR_speed - robot_speed) > LQR_ODOMETRY_SLIP_STEP ||
        fabsf(LQR_speed) > LQR_ODOMETRY_SLIP_SPEED ||
        jump_flag != 0;

    if (command_active || wheels_slipping) {
        distance_zeropoint = LQR_distance;
        pid_lqr_u.reset();
    }

    /* Latch "just stopped" on the joystick release and re-reference once the
     * wheels really are still. */
    if ((prev_joy_x != 0 && cmd->joy_x == 0) || (prev_joy_y != 0 && cmd->joy_y == 0)) {
        odometry_just_stopped = 1;
    }
    if (odometry_just_stopped != 0 && fabsf(LQR_speed) < LQR_ODOMETRY_STOP_SPEED) {
        distance_zeropoint = LQR_distance;
        odometry_just_stopped = 0;
    }

    if (fabsf(LQR_speed) > LQR_ODOMETRY_FAST_SPEED) {
        distance_zeropoint = LQR_distance;
    }

    distance_control = pid_distance(LQR_distance - distance_zeropoint);
    speed_control = pid_speed(LQR_speed - LQR_SPEED_JOY_GAIN * lpf_joy_y((float)cmd->joy_y));

    robot_speed = LQR_speed;
    if (wheels_slipping) {
        LQR_u = angle_control + gyro_control;
    } else {
        LQR_u = angle_control + gyro_control + distance_control + speed_control;
    }

    if (fabsf(LQR_u) < LQR_U_TRIM_BAND && cmd->joy_y == 0 &&
        fabsf(distance_control) < LQR_U_DISTANCE_BAND && jump_flag == 0) {
        LQR_u = pid_lqr_u(LQR_u);
    } else {
        pid_lqr_u.reset();
    }

    /* Airborne detection: the accelerometer's specific-force magnitude drops
     * toward zero when both wheels leave the ground (a step, a drop, a bump).
     * Scaling the drive down then stops the wheels winding up, so the robot
     * does not lunge as they regain traction. */
    accel_mag_g = sqrtf(imu->accel_x_g * imu->accel_x_g +
                        imu->accel_y_g * imu->accel_y_g +
                        imu->accel_z_g * imu->accel_z_g);
    accel_x_last = imu->accel_x_g;
    accel_y_last = imu->accel_y_g;
    accel_z_last = imu->accel_z_g;
    if (accel_mag_g < s->air_thresh_g) {
        air_hold_s += loop_dt;
        if (air_hold_s > AIR_HOLD_S) {
            air_hold_s = AIR_HOLD_S;
        }
    } else {
        air_hold_s -= loop_dt;
        if (air_hold_s < 0.0f) {
            air_hold_s = 0.0f;
        }
    }
    airborne = (air_hold_s > 0.0f) ? 1 : 0;
    if (airborne) {
        LQR_u *= s->air_scale;
        pid_lqr_u.reset();
    }

    /* Self-calibrate the balance zero: while balancing straight and slow, walk
     * the target toward the pitch the robot actually rests at, so the model's
     * height slope does not have to be exact and the loop stops fighting. */
    if (s->zero_trim > 0.0f && cmd->go && fault_reason == FAULT_NONE &&
        fabsf((float)cmd->joy_x) < 5.0f && fabsf((float)cmd->joy_y) < 5.0f &&
        fabsf(LQR_speed) < 3.0f && jump_flag == 0 && bump_flag == 0 && !airborne) {
        const float err = LQR_angle - zero;
        if (fabsf(err) < LEG_BALANCE_ZERO_TRIM_BAND) {   /* bigger = a push, not a bias */
            zero_auto += s->zero_trim * err * loop_dt;
            if (zero_auto > LEG_BALANCE_ZERO_TRIM_MAX) {
                zero_auto = LEG_BALANCE_ZERO_TRIM_MAX;
            } else if (zero_auto < -LEG_BALANCE_ZERO_TRIM_MAX) {
                zero_auto = -LEG_BALANCE_ZERO_TRIM_MAX;
            }
        }
    }
}

static void jump_legs_to(int height, int speed, int acc, const robot_shared_t *s)
{
    int16_t p1, p2;
    robot_leg_positions((float)height, 0.0f,
                        s->leg1_min, s->leg1_max, s->leg2_min, s->leg2_max, &p1, &p2);
    robot_control_leg_output(p1, p2, speed, acc);
}

/* Three-phase jump gait, timed with the measured dt:
 *   1. crouch to jump_crouch at max speed, hold jump_crouch_ticks to settle,
 *   2. slam up to jump_height (launch),
 *   3. after jump_land_ticks retract to jump_land_height to catch the landing.
 * jump_flag doubles as the phase (1 crouch, 2 slam, 3 land) and as the
 * "jumping" flag the balance loop checks. */
static void jump_loop(const robot_command_t *cmd, const robot_shared_t *s)
{
    if (prev_dir == ROBOT_JUMP && cmd->dir == ROBOT_STOP && jump_flag == 0) {
        jump_flag = 1;
        jump_elapsed_s = 0.0f;
        jump_legs_to(s->jump_crouch, 0, 0, s);
    }
    if (jump_flag == 0) {
        return;
    }
    jump_elapsed_s += loop_dt;

    const float slam_s = (float)(s->jump_crouch_ticks + 2) * 0.001f;
    const float land_s = slam_s + (float)s->jump_land_ticks * 0.001f;
    if (jump_flag == 1 && jump_elapsed_s >= slam_s) {
        jump_legs_to(s->jump_height, s->jump_speed, s->jump_acc, s);
        jump_flag = 2;
    } else if (jump_flag == 2 && jump_elapsed_s >= land_s) {
        jump_legs_to(s->jump_land_height, s->jump_speed, s->jump_acc, s);
        jump_flag = 3;
    } else if (jump_flag == 3 && jump_elapsed_s >= land_s + JUMP_SETTLE_S) {
        jump_flag = 0;
    }
}

/* Timed two-phase one-leg extension: extend for bump_ticks, then snap back to
 * the normal pose for another bump_ticks at the same high speed (instead of the
 * slow normal slew), then hand control back. Roll correction is skipped for the
 * pulse. Returns true while the pulse owns the leg output. */
static bool bump_loop(const mpu6050_sample_t *imu, const robot_shared_t *s)
{
    if (bump_flag == 0) {
        return false;
    }
    last_roll_angle = imu->angle_x;
    const float extend_s = (float)s->bump_ticks * 0.001f;
    if (bump_elapsed_s >= 2.0f * extend_s) {
        bump_flag = 0;
        return false;   /* pulse finished, resume normal control this tick */
    }
    const bool extending = bump_elapsed_s < extend_s;
    bump_elapsed_s += loop_dt;

    int16_t position1, position2;
    robot_leg_positions(leg_height_cmd, 0.0f,
                        s->leg1_min, s->leg1_max, s->leg2_min, s->leg2_max,
                        &position1, &position2);
    if (extending) {
        if (bump_leg == 1) {
            position1 = robot_clamp_servo((float)position1 + s->bump_amp,
                                          s->leg1_min, s->leg1_max);
        } else {
            position2 = robot_clamp_servo((float)position2 - s->bump_amp,
                                          s->leg2_min, s->leg2_max);
        }
    }
    robot_control_leg_output(position1, position2,
               (uint16_t)s->bump_speed, (uint8_t)s->bump_acc);
    return true;
}

static void leg_loop(const mpu6050_sample_t *imu, const robot_command_t *cmd,
                     const robot_shared_t *s)
{
    if (attitude_reset_request) {
        attitude_reset_request = false;
        pid_roll_angle.reset();
        pid_yaw_angle.reset();
        pid_yaw_gyro.reset();
        leg_position_add = 0.0f;
        /* Hold the heading the robot has right now instead of unwinding
         * whatever accumulated while it was disarmed or being carried. */
        YAW_angle_total = 0.0f;
        yaw_fused_last = yaw_fused;
        zero_auto = 0.0f;
    }
    if (zero_auto_reset) {
        zero_auto_reset = false;
        zero_auto = 0.0f;
    }
    /* Jumps and bumps are operator actions, so never run them while faulted;
     * the get-up path still calls this to move the legs. */
    if (fault_reason == FAULT_NONE) {
        jump_loop(cmd, s);
        if (jump_flag != 0) {
            return;
        }
        if (bump_loop(imu, s)) {
            return;
        }
    }

    if (s->manual_leg_enable) {
        last_roll_angle = imu->angle_x;
        robot_control_leg_output(robot_clamp_servo((float)s->manual_leg1, s->leg1_min, s->leg1_max),
                   robot_clamp_servo((float)s->manual_leg2, s->leg2_min, s->leg2_max),
                   LEG_MOVE_SPEED, LEG_MOVE_ACC);
        return;
    }

    float roll_angle = imu->angle_x + s->roll_bias - (float)cmd->roll;
    last_roll_angle = imu->angle_x;
    /* Roll correction is disabled while faulted to avoid winding up on a huge
     * roll error (the robot is usually on its side). It stays active during a
     * get-up/rock so the chassis does not roll over while it comes up. */
    const bool roll_active = (fault_reason == FAULT_NONE) ||
                             (wheel_seq_len > 0) || (getup_state == 1);
    if (!roll_active) {
        pid_roll_angle.reset();
        leg_position_add = 0.0f;
    } else {
        float roll_out = pid_roll_angle(lpf_roll(roll_angle));
        leg_position_add = (s->roll_mode == 0) ? 0.0f : (float)s->roll_mode * roll_out;
    }

    /* Slew the commanded height so a slider jump does not kick the chassis. */
    float target_height = (float)cmd->height;
    const float max_step = LEG_HEIGHT_SLEW_RATE * loop_dt;
    if (target_height > leg_height_cmd) {
        leg_height_cmd += fminf(max_step, target_height - leg_height_cmd);
    } else {
        leg_height_cmd -= fminf(max_step, leg_height_cmd - target_height);
    }

    int16_t p1, p2;
    robot_leg_positions(leg_height_cmd, leg_position_add,
                        s->leg1_min, s->leg1_max, s->leg2_min, s->leg2_max, &p1, &p2);
    robot_control_leg_output(p1, p2, LEG_MOVE_SPEED, LEG_MOVE_ACC);
}

static void apply_motor_targets(const robot_command_t *cmd, const robot_shared_t *s)
{
    if (cmd->go == 0) {
        motor_foc_set_target(MOTOR_LEFT, 0.0f);
        motor_foc_set_target(MOTOR_RIGHT, 0.0f);
        leg_position_add = 0.0f;
        return;
    }
    float yaw = (s->yaw_mode == 0) ? 0.0f : ((s->yaw_mode > 0) ? YAW_output : -YAW_output);
    motor_foc_set_target(MOTOR_LEFT, (-0.5f) * (LQR_u + yaw));
    motor_foc_set_target(MOTOR_RIGHT, (-0.5f) * (LQR_u - yaw));
}

/* ------------------------------------------------------------------------- */
/* Fault handling                                                            */
/* ------------------------------------------------------------------------- */

/* One ADC sample every BATTERY_SAMPLE_S; the flag tells the debouncer below
 * when a fresh reading is available. */
static bool battery_sample_fresh;
static float battery_sample_s;
static float battery_cached_v = -1.0f;

static float battery_voltage_throttled(void)
{
    battery_sample_s += loop_dt;
    if (battery_cached_v < 0.0f || battery_sample_s >= BATTERY_SAMPLE_S) {
        battery_sample_s = 0.0f;
        battery_cached_v = board_battery_voltage();
        battery_sample_fresh = true;
    }
    return battery_cached_v;
}

/* The battery sense input is noisy on some builds, so only cut the motors after
 * the reading stays low for BATTERY_LOW_HOLD_S, and ignore implausibly low
 * values (a powered 2S pack cannot read below ~4 V; that means a bad
 * connection). */
static bool battery_is_low(void)
{
    static float low_s;
    float voltage = battery_voltage_throttled();

    if (battery_sample_fresh) {
        battery_sample_fresh = false;
        if (voltage >= BATTERY_MIN_PLAUSIBLE && voltage < BOARD_BATTERY_LOW_VOLTAGE) {
            low_s += BATTERY_SAMPLE_S;
        } else {
            low_s = 0.0f;
        }
    }
    return low_s >= BATTERY_LOW_HOLD_S;
}

static void enter_fault(fault_reason_t reason, bool operator_go)
{
    if (fault_reason != FAULT_NONE) {
        return;
    }
    fault_reason = reason;
    recover_s = 0.0f;
    jump_flag = 0;   /* abort any gait that was running when the fault hit */
    bump_flag = 0;
    motor_foc_stop();
    reset_pids();
    /* Auto-recovery only for attitude faults while the operator wanted to run;
     * battery faults always need a manual restart. */
    go_auto_latch = (reason == FAULT_ATTITUDE) && operator_go;
    /* Take the operator's go down without going through robot_control_set_go(),
     * which would cancel the latch we just set. */
    shared_lock();
    shared.command.go = false;
    shared_unlock();
    robot_state_set(ROBOT_STATE_FAULT);
    ESP_LOGE("robot_control", "fault (%s), motors disabled%s",
             reason == FAULT_BATTERY ? "battery low" : "attitude",
             go_auto_latch ? "; will auto-recover when upright" : "; set go=1 when resolved");
}

static void fault_recover(const robot_command_t *cmd, const mpu6050_sample_t *imu)
{
    LQR_angle = imu->angle_y;

    bool condition_ok = (fault_reason == FAULT_BATTERY)
        ? (battery_voltage_throttled() > BOARD_BATTERY_RECOVER_VOLTAGE)
        : (fabsf(LQR_angle) < ATTITUDE_RECOVER_DEG);

    if (condition_ok && (cmd->go || go_auto_latch)) {
        recover_s += loop_dt;
        if (recover_s >= RECOVER_HOLD_S) {
            recover_s = 0.0f;
            reset_pids();
            if (motor_foc_enable_torque() == ESP_OK) {
                fault_reason = FAULT_NONE;
                go_auto_latch = false;
                if (!cmd->go) {
                    robot_control_set_go(true);   /* resume the latched run */
                }
                robot_state_set(ROBOT_STATE_READY);
                ESP_LOGI("robot_control", "recovered from fault%s",
                         cmd->go ? "" : " (auto)");
            } else {
                robot_state_set(ROBOT_STATE_FAULT);
            }
        }
    } else {
        recover_s = 0.0f;
    }
}

/* ------------------------------------------------------------------------- */
/* Control task                                                              */
/* ------------------------------------------------------------------------- */

/* Gyro Z auto-trim: the MPU6050 zero-rate offset drifts with temperature, which
 * the yaw loop turns into a slow spin. While the robot is disarmed and not
 * rotating the gyro Z reading *is* that offset, so walk the offset to null it.
 * Runs on every idle/rest period, so a bad boot calibration self-heals. */
static void gyro_trim_loop(const mpu6050_sample_t *imu, const robot_command_t *cmd)
{
    /* The window must exceed a plausible bad boot offset (MPU6050 ZRO is
     * +/-20 dps but a moving boot calibration can leave more), otherwise the
     * trim can never reach it. Real rotations are far larger and excluded. */
    if (!cmd->go && fabsf(imu->gyro_z_dps) < GYRO_TRIM_WINDOW_DPS) {
        gyro_trim_still_s += loop_dt;
        if (gyro_trim_still_s > GYRO_TRIM_WINDOW_S) {
            gyro_trim_still_s = GYRO_TRIM_WINDOW_S;
        }
    } else {
        gyro_trim_still_s = 0.0f;
    }
    if (gyro_trim_still_s >= GYRO_TRIM_REST_S) {
        sensors_trim_gyro_z(imu->gyro_z_dps * loop_dt / GYRO_TRIM_TAU_S);
    }
}

/* Apply one queued discrete action. Runs on the control task. */
static void apply_request(const robot_request_t *request)
{
    switch (request->kind) {
    case REQ_ATTITUDE_RESET:
        attitude_reset_request = true;
        break;
    case REQ_ZERO_AUTO_RESET:
        zero_auto_reset = true;
        break;
    case REQ_GO_OFF:
        go_auto_latch = false;
        break;
    case REQ_BUMP:
        if (request->u.i[0] == 1 || request->u.i[0] == 2) {
            bump_leg = request->u.i[0];
            bump_flag = 1;
            bump_elapsed_s = 0.0f;
            shared_lock();
            shared.manual_leg_enable = false;   /* a bump always returns to normal control */
            shared_unlock();
        } else {
            bump_flag = 0;
        }
        break;
    case REQ_GETUP:
        if (request->u.i[0]) {
            getup_request = 1;
        } else {
            getup_request = 0;
            getup_state = 0;
            getup_state_pub = 0;
        }
        break;
    case REQ_MANUAL_DRIVE:
        manual_target = request->u.f[0];
        manual_s_left = (float)request->u.i[0] * 0.001f;
        break;
    case REQ_WHEEL_SEQ:
        wheel_seq_len = request->u.seq.count;
        wheel_seq_idx = 0;
        wheel_seq_left_s = wheel_seq_len > 0
            ? (float)request->u.seq.durations_ms[0] * 0.001f
            : 0.0f;
        for (int i = 0; i < wheel_seq_len; ++i) {
            wheel_seq[i].target = request->u.seq.targets[i];
            wheel_seq[i].seconds = (float)request->u.seq.durations_ms[i] * 0.001f;
        }
        break;
    }
}

static void process_requests(void)
{
    robot_request_t request;
    while (request_queue != NULL && xQueueReceive(request_queue, &request, 0) == pdTRUE) {
        apply_request(&request);
    }
}

/* Refresh the control-task PIDs/filters from the shared tuning snapshot. */
static void apply_tuning(const robot_shared_t *s)
{
    for (int i = 0; i < ROBOT_PID_COUNT; ++i) {
        PIDController *pid = pid_at(i);
        if (pid != NULL) {
            pid->P = s->pid[i].p;
            pid->I = s->pid[i].i;
            pid->D = s->pid[i].d;
            pid->limit = s->pid[i].limit;
        }
    }
    for (int i = 0; i < ROBOT_LPF_COUNT; ++i) {
        LowPassFilter *filter = lpf_at(i);
        if (filter != NULL) {
            filter->Tf = s->lpf_tf[i];
        }
    }
}

static void publish_telemetry(void)
{
    shared_lock();
    bool manual_legs = shared.manual_leg_enable;
    shared_unlock();

    robot_telemetry_t t;
    t.lqr_angle = LQR_angle;
    t.lqr_u = LQR_u;
    t.angle_term = angle_control;
    t.gyro_term = gyro_control;
    t.distance_term = distance_control;
    t.speed_term = speed_control;
    t.balance_zero = last_balance_zero;
    t.yaw_total = YAW_angle_total;
    t.yaw_output = YAW_output;
    t.yaw_fused = YAW_angle;
    t.yaw_wheel_rate = yaw_wheel_rate;
    t.yaw_wheel_heading = yaw_wheel;
    t.left_velocity = last_left_velocity;
    t.right_velocity = last_right_velocity;
    t.gyro_z = last_gyro_z;
    t.leg_add = leg_position_add;
    t.roll_angle = last_roll_angle;
    t.angle_pp = angle_pp;
    t.zero_auto = zero_auto;
    t.accel_mag = accel_mag_g;
    t.accel_x = accel_x_last;
    t.accel_y = accel_y_last;
    t.accel_z = accel_z_last;
    t.airborne = airborne;
    t.fault_reason = (int)fault_reason;
    t.getup_state = getup_state_pub;
    t.bump_state = bump_flag ? bump_leg : 0;
    t.manual_legs = manual_legs ? 1 : 0;
    t.manual_ms = (int)(manual_s_left * 1000.0f);
    t.jump_state = jump_flag;
    robot_control_leg_targets(&t.leg_target1, &t.leg_target2);

    telemetry_lock();
    telemetry = t;
    telemetry_unlock();
}

static void control_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    motor_feedback_t left = {};
    motor_feedback_t right = {};
    mpu6050_sample_t imu = {};
    float a_min = 1e9f;
    float a_max = -1e9f;
    float pp_s = 0.0f;

    while (true) {
        /* Measure the real loop period so every rate/integrator below is
         * correct even when an I2C read or a preemption makes a loop late. */
        const int64_t now_us = esp_timer_get_time();
        if (last_loop_us != 0) {
            float dt = (float)(now_us - last_loop_us) * 0.000001f;
            if (dt > 0.0f && dt < 0.1f) {
                loop_dt = dt;
            }
        }
        last_loop_us = now_us;

        process_requests();

        robot_shared_t s;
        shared_lock();
        s = shared;
        shared_unlock();
        apply_tuning(&s);

        __atomic_fetch_add(&control_loop_count, 1, __ATOMIC_RELAXED);

        /* Latched direction buttons drive like a held joystick (the reference
         * firmware left ROBOT_FORWARD/BACK/LEFT/RIGHT unwired; only the jump
         * edge used cmd.dir). Magnitudes match the tested joystick range. */
        robot_command_t cmd = s.command;
        if (cmd.dir == ROBOT_FORWARD) {
            cmd.joy_y = 60;
        } else if (cmd.dir == ROBOT_BACK) {
            cmd.joy_y = -60;
        } else if (cmd.dir == ROBOT_LEFT) {
            cmd.joy_x = -60;
        } else if (cmd.dir == ROBOT_RIGHT) {
            cmd.joy_x = 60;
        }

        if (sensors_read_imu(&imu) == ESP_OK) {
            if (cmd.go && !prev_go) {
                arm_request = true;
            }
            prev_go = cmd.go;

            /* Cancel the gyro Z zero-rate drift while disarmed and at rest. */
            gyro_trim_loop(&imu, &cmd);

            /* Research mode: direct wheel torque (sequence or single pulse),
             * bypasses balancing and fault handling. */
            if (wheel_seq_len > 0 || manual_s_left > 0.0f) {
                if (!manual_armed) {
                    motor_foc_enable_torque();
                    manual_armed = true;
                }
                LQR_angle = imu.angle_y;
                const bool release_now = s.wheel_seq_arm &&
                    fabsf(LQR_angle) < s.getup_release_deg;
                float target;
                if (release_now) {
                    target = 0.0f;
                    wheel_seq_len = 0;
                    manual_s_left = 0.0f;
                    shared_lock();
                    shared.wheel_seq_arm = false;
                    shared_unlock();
                    motor_foc_enable_torque();
                    manual_armed = false;
                    reset_pids();
                    arm_request = true;
                    go_auto_latch = false;
                    fault_reason = FAULT_NONE;
                    robot_control_set_go(true);
                    /* Stand back up at the configured get-up height. */
                    robot_control_set_height(s.getup_final_height);
                    robot_state_set(ROBOT_STATE_RUNNING);
                    ESP_LOGI("robot_control", "rock get-up handed over at %.1f deg",
                             LQR_angle);
                } else if (wheel_seq_len > 0) {
                    target = wheel_seq[wheel_seq_idx].target;
                    wheel_seq_left_s -= loop_dt;
                    if (wheel_seq_left_s <= 0.0f) {
                        int next = wheel_seq_idx + 1;
                        if (next >= wheel_seq_len) {
                            wheel_seq_len = 0;
                            motor_foc_stop();
                            manual_armed = false;
                        } else {
                            wheel_seq_idx = next;
                            wheel_seq_left_s = wheel_seq[next].seconds;
                        }
                    }
                } else {
                    target = manual_target;
                    manual_s_left -= loop_dt;
                    if (manual_s_left <= 0.0f) {
                        manual_s_left = 0.0f;
                        motor_foc_stop();
                        manual_armed = false;
                    }
                }
                motor_foc_set_target(MOTOR_LEFT, target);
                motor_foc_set_target(MOTOR_RIGHT, target);
                leg_loop(&imu, &cmd, &s);
                prev_dir = cmd.dir;
                prev_joy_x = cmd.joy_x;
                prev_joy_y = cmd.joy_y;
                publish_telemetry();
                motor_foc_step();
                vTaskDelayUntil(&last_wake, 1);
                continue;
            }

            if (fault_reason != FAULT_NONE) {
                LQR_angle = imu.angle_y;
                if (getup_request && fault_reason == FAULT_ATTITUDE) {
                    getup_request = 0;
                    getup_state = 1;   /* prepare: lower the legs */
                    getup_s = 0.0f;
                    robot_control_set_height(getup_low_height);
                    ESP_LOGI("robot_control", "get-up: lowering legs to h=%d",
                             getup_low_height);
                }
                if (getup_state == 1) {
                    leg_loop(&imu, &cmd, &s);
                    getup_s += loop_dt;
                    if (getup_s >= GETUP_LOWER_S) {   /* let the legs reach the low pose */
                        float tg[2];
                        int du[2];
                        const float amp = s.getup_torque;
                        const float direction = (s.getup_sign < 0) ? -1.0f : 1.0f;
                        if (LQR_angle < 0.0f) {   /* fell backwards */
                            tg[0] = -amp;
                            tg[1] = amp;
                        } else {                  /* fell forwards */
                            tg[0] = amp;
                            tg[1] = -amp;
                        }
                        tg[0] *= direction;
                        tg[1] *= direction;
                        du[0] = GETUP_ROCK_BACK_MS;
                        du[1] = GETUP_ROCK_FWD_MS;
                        robot_control_wheel_sequence(tg, du, 2);
                        robot_control_wheel_sequence_arm(true);
                        getup_state = 0;
                        ESP_LOGI("robot_control", "get-up: rocking at %.1f deg",
                                 LQR_angle);
                    }
                } else {
                    fault_recover(&cmd, &imu);
                    leg_loop(&imu, &cmd, &s);
                }
                getup_state_pub = getup_state;
            } else {
                motor_foc_get_feedback(MOTOR_LEFT, &left);
                motor_foc_get_feedback(MOTOR_RIGHT, &right);

                lqr_balance_loop(&left, &right, &imu, &cmd, &s);
                yaw_loop(&imu, &cmd, &s);
                leg_loop(&imu, &cmd, &s);

                if (fabsf(LQR_angle) > s.fault_deg) {
                    enter_fault(FAULT_ATTITUDE, cmd.go);
                } else if (cmd.go && battery_is_low()) {
                    enter_fault(FAULT_BATTERY, cmd.go);
                } else {
                    apply_motor_targets(&cmd, &s);
                    robot_state_set(cmd.go ? ROBOT_STATE_RUNNING : ROBOT_STATE_READY);
                }
            }

            prev_dir = cmd.dir;
            prev_joy_x = cmd.joy_x;
            prev_joy_y = cmd.joy_y;
        }

        if (LQR_angle < a_min) a_min = LQR_angle;
        if (LQR_angle > a_max) a_max = LQR_angle;
        pp_s += loop_dt;
        if (pp_s >= LQR_ANGLE_PP_WINDOW_S) {
            angle_pp = a_max - a_min;
            a_min = 1e9f;
            a_max = -1e9f;
            pp_s = 0.0f;
        }

        publish_telemetry();
        motor_foc_step();
        vTaskDelayUntil(&last_wake, 1);
    }
}

esp_err_t robot_control_start(void)
{
    if (control_started) {
        return ESP_OK;
    }

    shared_mutex = xSemaphoreCreateMutex();
    telemetry_mutex = xSemaphoreCreateMutex();
    request_queue = xQueueCreate(REQUEST_QUEUE_LEN, sizeof(robot_request_t));
    if (shared_mutex == NULL || telemetry_mutex == NULL || request_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    shared_load_defaults();

    /* NVS is initialised once in app_main; the call here is a safe no-op and
     * keeps the saved level calibration loadable if control is started some
     * other way. */
    nvs_store_init();
    roll_bias_load();

    ESP_RETURN_ON_ERROR(robot_control_leg_io_start(), "robot_control",
                        "leg I/O task start failed");
    if (xTaskCreatePinnedToCore(control_task, "control_task", 6144, NULL, CONTROL_TASK_PRIO,
                                NULL, ROBOT_TASK_CORE) != pdPASS) {
        return ESP_FAIL;
    }

    control_started = true;
    robot_state_set(ROBOT_STATE_READY);
    ESP_LOGI("robot_control", "control + leg tasks started (LQR + yaw + leg)");
    return ESP_OK;
}