#include "robot_control.h"

#include "board.h"
#include "esp_log.h"
#include "lowpass_filter.h"
#include "motor_foc.h"
#include "pid.h"
#include "robot_config.h"
#include "robot_state.h"
#include "sensors.h"
#include "servo_sts.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <math.h>

/* Port of the balance / yaw / leg loops from wl_pro_robot.ino.
 * PID gains, filter time constants and the control structure follow the
 * original firmware. Angles are in degrees and gyro rates in deg/s, matching
 * MPU6050_tockn / our complementary filter. */

/* All app tasks share core 1, leaving core 0 to the Wi-Fi/system stack, like the
 * single-core Arduino reference where the control loop never competes with Wi-Fi. */
#define ROBOT_TASK_CORE      1
#define CONTROL_TASK_PRIO    7
#define LEG_TASK_PRIO        4

/* Leg poses are decoupled from the 1 kHz control loop: the control task only
 * publishes the newest pose (overwrite mailbox) and a low-rate task writes it
 * to the servo bus, so a busy/locked UART never stalls balancing. */
#define LEG_OUTPUT_PERIOD_MS 10

/* PIDController keeps error_prev protected in arduino-foc 2.4, but the original
 * firmware clears it in several places to avoid integral wind-up. */
struct StabPID : public PIDController {
    using PIDController::PIDController;
    void clear_error(void)
    {
        error_prev = 0.0f;
    }
};

static StabPID pid_angle(1.1f, 0.0f, 0.0f, 100000.0f, 8.0f);
static StabPID pid_gyro(0.09f, 0.0f, 0.0f, 100000.0f, 8.0f);
static StabPID pid_distance(0.4f, 0.0f, 0.0f, 100000.0f, 8.0f);
static StabPID pid_speed(0.4f, 0.0f, 0.0f, 100000.0f, 8.0f);
static StabPID pid_yaw_angle(1.0f, 0.0f, 0.0f, 100000.0f, 8.0f);
static StabPID pid_yaw_gyro(0.04f, 0.0f, 0.0f, 100000.0f, 8.0f);
static StabPID pid_lqr_u(1.0f, 15.0f, 0.0f, 100000.0f, 8.0f);
/* Disabled by default: the base + height feed-forward already track the measured
 * stationary lean well, and this slow loop otherwise wanders ('pid zeropoint'
 * re-enables it if ever needed). */
static StabPID pid_zeropoint(0.0f, 0.0f, 0.0f, 100000.0f, 4.0f);
/* P=4, I=10: the proportional gain has to stay low because the leg servo +
 * LPF lag leaves little phase margin (P=10 rang at ~1.2 Hz). The integral then
 * does the real work: it levels a slope in ~1.5 s where I=1 took tens of
 * seconds. I=20 was rejected: on a large roll error (a fall) the integral
 * grew at ~600/s and slammed the legs to the clamps within a second, which
 * kicks the chassis; I=10 keeps the slope response while staying gentle.
 * Output limit 500 sits above the maximum differential the leg travel clamps
 * allow; the anti-windup keeps the integral inside the same limit. */
static StabPID pid_roll_angle(4.0f, 10.0f, 0.0f, 100000.0f, 500.0f);

static LowPassFilter lpf_joy_y(0.2f);
static LowPassFilter lpf_zeropoint(0.1f);
static LowPassFilter lpf_roll(0.3f);

/* Shared remote-control command, guarded by cmd_mux. */
static portMUX_TYPE cmd_mux = portMUX_INITIALIZER_UNLOCKED;
static robot_command_t command = {
    .height = LEG_HEIGHT_DEFAULT,
    .roll = 0,
    .linear = 0,
    .angular = 0,
    .dir = ROBOT_STOP,
    .joy_x = 0,
    .joy_y = 0,
    .go = false,
};

/* LQR balance state (only touched by the control task, volatile for telemetry). */
static volatile float LQR_angle;
static volatile float LQR_u;
static float LQR_gyro;
static float LQR_speed;
static float LQR_distance;
/* Raw wheel velocities, exposed for diagnostics (a pure yaw spin shows up as
 * left and right velocities that are equal and opposite). */
static float last_left_velocity;
static float last_right_velocity;
static float last_gyro_z;
static float angle_control;
static float gyro_control;
static float speed_control;
static float distance_control;
/* Balance zero point: base value adjusted by the height-dependent offset. The
 * reference -2.25 assumes a different sensor orientation; this is the value
 * measured on this build (see robot_config.h). 'zero' changes the base. */
static float angle_zeropoint = LEG_BALANCE_ZERO_DEFAULT;
static float angle_zeropoint_base = LEG_BALANCE_ZERO_DEFAULT;
static float distance_zeropoint = -256.0f;

/* Yaw state */
static float YAW_gyro;
static float YAW_angle;
static float YAW_angle_last;
static float YAW_angle_total;
static float YAW_angle_zero_point = -10.0f;
static float YAW_output;

/* Control-task-local previous command snapshot (for edge detection). */
static int prev_dir = ROBOT_STOP;
static int prev_joy_x;
static int prev_joy_y;
static bool prev_go;

/* Leg / motion flags */
static float robot_speed;
static float robot_speed_last;
static int wrobot_move_stop_flag;
static int jump_flag;
static float leg_position_add;
static float leg_height_cmd = (float)LEG_HEIGHT_DEFAULT;
static float last_roll_angle;
static float last_balance_zero;
static volatile float angle_pp;
static int yaw_mode = 1;   /* 1 = normal, -1 = inverted, 0 = disabled */
static int roll_mode = 1;  /* roll correction sign, same convention */
/* IMU mounting bias: raw angle_x reading when the chassis is visually level.
 * Exposed as `rb` for live calibration (falls can shift the sensor board). */
static float roll_bias = 2.0f;
/* Pitch threshold that latches an attitude fault and cuts the motors. Raised
 * from 25 to 35 deg so a transient while driving over an obstacle (a wheel
 * dropping off a plank) does not abort the run; tunable as `faultdeg`. */
static float attitude_fault_deg = 35.0f;
/* Airborne / drop detection. While the wheels are off the ground the balance
 * loop's drive just spins them up; the leftover wheel speed then makes the
 * robot lunge forward on landing. Below `air_thresh_g` of specific force the
 * robot is treated as airborne and the balance output is scaled by
 * `air_scale` until it lands. Tumable as `airth` / `airscale`. */
static float air_thresh_g = 0.60f;
static float air_scale = 0.25f;
static int air_hold_ticks = 20;   /* debounce, control ticks (~1 kHz) */
static int air_count;
static volatile float accel_mag_g;
static volatile int airborne;
static volatile float accel_x_last;
static volatile float accel_y_last;
static volatile float accel_z_last;
/* Research: direct wheel-torque override for self-righting experiments. While
 * `manual_ticks` > 0 the control loop drives both wheels at `manual_target`
 * (same units as LQR_u), bypassing balancing and fault handling. */
static volatile float manual_target;
static volatile int manual_ticks;
static volatile bool manual_armed;
/* Self-right state machine (research). When triggered while attitude-faulted,
 * drives the wheels toward the side that reduces |pitch| and hands over to the
 * balance loop once the chassis is near upright. */
static volatile int getup_request;
static int getup_state;             /* 0 idle, 1 driving to upright */
static volatile int getup_state_pub;
static float getup_torque = 18.0f;
static float getup_release_deg = 20.0f;
static int getup_sign = 1;
static int getup_ticks;
static bool arm_request;   /* reset distance zero/PIDs when go turns on */

/* Runtime-tunable jump profile (defaults mirror LEG_JUMP_* in robot_config.h).
 * Written over HTTP/console, read by the control task. */
static int jump_height = LEG_JUMP_HEIGHT;
static int jump_land_height = LEG_JUMP_LAND_HEIGHT;
static int jump_speed = LEG_JUMP_SPEED;
static int jump_acc = LEG_JUMP_ACC;
static int jump_land_ticks = 32;
/* Gait: crouch first to load the legs, then slam up, then retract to catch.
 * h34 keeps chassis-to-wheel clearance (h32 scrapes) and jumps reliably. */
static int jump_crouch = 34;
static int jump_crouch_ticks = 150;

/* Attitude/battery fault latch: motors are disabled and stay off until the
 * operator commands go=1 again once the fault condition has cleared. */
typedef enum {
    FAULT_NONE = 0,
    FAULT_ATTITUDE,
    FAULT_BATTERY,
} fault_reason_t;

static volatile fault_reason_t fault_reason = FAULT_NONE;
static int recover_ticks;
/* Latched at attitude-fault entry when the operator had go on: the robot
 * re-arms by itself once upright again, without a manual go command. */
static bool go_auto_latch;

static bool control_started;
static QueueHandle_t leg_queue;
static uint32_t control_loop_count;
static int16_t last_leg_target1;
static int16_t last_leg_target2;

typedef struct {
    int16_t position1;
    int16_t position2;
    uint16_t speed;
    uint8_t acceleration;
} leg_pose_t;

/* ------------------------------------------------------------------------- */
/* Command access                                                            */
/* ------------------------------------------------------------------------- */

robot_command_t robot_control_get_command(void)
{
    portENTER_CRITICAL(&cmd_mux);
    robot_command_t snapshot = command;
    portEXIT_CRITICAL(&cmd_mux);
    return snapshot;
}

static const char *const pid_names[ROBOT_PID_COUNT] = {
    "angle", "gyro", "distance", "speed", "yaw_angle",
    "yaw_gyro", "lqr_u", "zeropoint", "roll_angle",
};

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
    case ROBOT_PID_ZEROPOINT:  return &pid_zeropoint;
    case ROBOT_PID_ROLL_ANGLE: return &pid_roll_angle;
    default:                   return NULL;
    }
}

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
    PIDController *pid = pid_at(which);
    if (pid == NULL) {
        return;
    }
    if (p) *p = pid->P;
    if (i) *i = pid->I;
    if (d) *d = pid->D;
    if (limit) *limit = pid->limit;
}

void robot_control_set_pid(int which, float p, float i, float d, float limit)
{
    PIDController *pid = pid_at(which);
    if (pid == NULL) {
        return;
    }
    pid->P = p;
    if (i >= 0.0f) pid->I = i;
    if (d >= 0.0f) pid->D = d;
    if (limit > 0.0f) pid->limit = limit;
}

static LowPassFilter *lpf_at(int which)
{
    switch (which) {
    case 0: return &lpf_joy_y;
    case 1: return &lpf_zeropoint;
    case 2: return &lpf_roll;
    default: return NULL;
    }
}

void robot_control_get_lpf(int which, float *tf)
{
    LowPassFilter *filter = lpf_at(which);
    if (filter != NULL && tf != NULL) {
        *tf = filter->Tf;
    }
}

void robot_control_set_lpf(int which, float tf)
{
    LowPassFilter *filter = lpf_at(which);
    if (filter != NULL) {
        filter->Tf = tf;
    }
}

void robot_control_set_angle_zeropoint(float degrees)
{
    angle_zeropoint = degrees;
    angle_zeropoint_base = degrees;
}

float robot_control_get_angle_zeropoint(void)
{
    return angle_zeropoint;
}

float robot_control_get_balance_zero(void)
{
    return last_balance_zero;
}

float robot_control_angle_pp(void)
{
    return angle_pp;
}

void robot_control_set_go(bool go)
{
    portENTER_CRITICAL(&cmd_mux);
    command.go = go;
    portEXIT_CRITICAL(&cmd_mux);
    if (!go) {
        go_auto_latch = false;   /* explicit stop cancels pending auto-recovery */
    }
}

void robot_control_set_height(int height)
{
    portENTER_CRITICAL(&cmd_mux);
    command.height = height;
    portEXIT_CRITICAL(&cmd_mux);
}

void robot_control_set_dir(int dir)
{
    portENTER_CRITICAL(&cmd_mux);
    command.dir = dir;
    portEXIT_CRITICAL(&cmd_mux);
}

void robot_control_set_joy(int joy_x, int joy_y)
{
    portENTER_CRITICAL(&cmd_mux);
    command.joy_x = joy_x;
    command.joy_y = joy_y;
    portEXIT_CRITICAL(&cmd_mux);
}

void robot_control_set_roll(int roll)
{
    portENTER_CRITICAL(&cmd_mux);
    command.roll = roll;
    portEXIT_CRITICAL(&cmd_mux);
}

void robot_control_set_linear(int linear)
{
    portENTER_CRITICAL(&cmd_mux);
    command.linear = linear;
    portEXIT_CRITICAL(&cmd_mux);
}

void robot_control_set_angular(int angular)
{
    portENTER_CRITICAL(&cmd_mux);
    command.angular = angular;
    portEXIT_CRITICAL(&cmd_mux);
}

void robot_control_set_yaw_mode(int mode)
{
    yaw_mode = (mode > 0) ? 1 : (mode < 0 ? -1 : 0);
}

int robot_control_get_yaw_mode(void)
{
    return yaw_mode;
}

void robot_control_set_roll_mode(int mode)
{
    roll_mode = (mode > 0) ? 1 : (mode < 0 ? -1 : 0);
}

int robot_control_get_roll_mode(void)
{
    return roll_mode;
}

void robot_control_set_roll_bias(float bias)
{
    if (bias > -10.0f && bias < 10.0f) {
        roll_bias = bias;
    }
}

float robot_control_get_roll_bias(void)
{
    return roll_bias;
}

void robot_control_set_fault_deg(float degrees)
{
    if (degrees >= 15.0f && degrees <= 80.0f) {
        attitude_fault_deg = degrees;
    }
}

float robot_control_get_fault_deg(void)
{
    return attitude_fault_deg;
}

void robot_control_set_air(float thresh_g, float scale)
{
    if (thresh_g > 0.05f && thresh_g < 1.5f) {
        air_thresh_g = thresh_g;
    }
    if (scale >= 0.0f && scale <= 1.0f) {
        air_scale = scale;
    }
}

float robot_control_get_air_thresh(void)
{
    return air_thresh_g;
}

float robot_control_get_air_scale(void)
{
    return air_scale;
}

float robot_control_accel_mag(void)
{
    return accel_mag_g;
}

void robot_control_get_accel(float *x, float *y, float *z)
{
    if (x) *x = accel_x_last;
    if (y) *y = accel_y_last;
    if (z) *z = accel_z_last;
}

void robot_control_manual_drive(float target, int ms)
{
    if (target < -12.0f) target = -12.0f;
    if (target > 12.0f) target = 12.0f;
    if (ms < 0) ms = 0;
    if (ms > 3000) ms = 3000;
    manual_target = target;
    manual_ticks = ms;   /* control loop runs at ~1 kHz, so 1 ms ~ 1 tick */
}

bool robot_control_manual_active(void)
{
    return manual_ticks > 0;
}

int robot_control_manual_ticks(void)
{
    return manual_ticks;
}

void robot_control_set_getup(int on)
{
    if (on) {
        getup_request = 1;
    } else {
        getup_request = 0;
        getup_state = 0;
        getup_state_pub = 0;
    }
}

void robot_control_set_getup_params(float torque, float release_deg, int sign)
{
    if (torque >= 0.0f && torque <= 30.0f) {
        getup_torque = torque;
    }
    if (release_deg >= 2.0f && release_deg <= 45.0f) {
        getup_release_deg = release_deg;
    }
    getup_sign = (sign < 0) ? -1 : 1;
}

float robot_control_getup_torque(void)
{
    return getup_torque;
}

float robot_control_getup_release(void)
{
    return getup_release_deg;
}

int robot_control_getup_sign(void)
{
    return getup_sign;
}

int robot_control_getup_state(void)
{
    return getup_state_pub;
}

int robot_control_airborne(void)
{
    return airborne;
}

/* NVS persistence so a level calibration survives reboots. */
#define NVS_NS "wlbot"
#define NVS_KEY_ROLL_BIAS "roll_bias"

static void roll_bias_save(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NS, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_blob(handle, NVS_KEY_ROLL_BIAS, &roll_bias, sizeof(roll_bias));
        nvs_commit(handle);
        nvs_close(handle);
    }
}

static void roll_bias_load(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NS, NVS_READONLY, &handle) != ESP_OK) {
        return;
    }
    float value = roll_bias;
    size_t length = sizeof(value);
    if (nvs_get_blob(handle, NVS_KEY_ROLL_BIAS, &value, &length) == ESP_OK &&
        length == sizeof(value) && value > -10.0f && value < 10.0f) {
        roll_bias = value;
    }
    nvs_close(handle);
}

float robot_control_calibrate_level(void)
{
    const int previous_mode = roll_mode;
    roll_mode = 0;   /* legs to their symmetric nominal pose */

    vTaskDelay(pdMS_TO_TICKS(900));
    float sum = 0.0f;
    int samples = 0;
    for (int i = 0; i < 100; ++i) {
        sum += last_roll_angle;
        samples++;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (samples > 0) {
        float bias = -sum / (float)samples;
        if (bias > -10.0f && bias < 10.0f) {
            roll_bias = bias;
        } else {
            ESP_LOGW("robot_control", "level calibration rejected (rb=%.1f)", bias);
        }
    }
    roll_bias_save();

    roll_mode = (previous_mode == 0) ? 1 : previous_mode;
    ESP_LOGI("robot_control", "level calibrated: rb=%.2f", roll_bias);
    return roll_bias;
}

void robot_control_set_jump_profile(int height, int land_height, int speed,
                                    int acc, int land_ticks)
{
    if (height >= LEG_HEIGHT_MIN && height <= LEG_HEIGHT_MAX) {
        jump_height = height;
    }
    if (land_height >= LEG_HEIGHT_MIN && land_height <= LEG_HEIGHT_MAX) {
        jump_land_height = land_height;
    }
    if (speed >= 0 && speed <= 2000) {
        jump_speed = speed;
    }
    if (acc >= 0 && acc <= 100) {
        jump_acc = acc;
    }
    if (land_ticks > 5 && land_ticks < 200) {
        jump_land_ticks = land_ticks;
    }
}

void robot_control_get_jump_profile(int *height, int *land_height, int *speed,
                                    int *acc, int *land_ticks)
{
    if (height) *height = jump_height;
    if (land_height) *land_height = jump_land_height;
    if (speed) *speed = jump_speed;
    if (acc) *acc = jump_acc;
    if (land_ticks) *land_ticks = jump_land_ticks;
}

void robot_control_set_jump_crouch(int height, int ticks)
{
    if (height >= LEG_HEIGHT_MIN && height <= LEG_HEIGHT_MAX) {
        jump_crouch = height;
    }
    if (ticks > 10 && ticks < 400) {
        jump_crouch_ticks = ticks;
    }
}

void robot_control_get_jump_crouch(int *height, int *ticks)
{
    if (height) *height = jump_crouch;
    if (ticks) *ticks = jump_crouch_ticks;
}

/* Last leg positions commanded to the servos and the jump state machine flag,
 * for diagnosing whether a jump command reaches the servos. */
void robot_control_get_leg_diag(int16_t *target1, int16_t *target2, int *jump_state)
{
    if (target1) *target1 = last_leg_target1;
    if (target2) *target2 = last_leg_target2;
    if (jump_state) *jump_state = jump_flag;
}

/* PIDController::clear_error() only resets the error history; the integral
 * accumulator (integral_prev) needs reset() as well, otherwise pid_lqr_u can
 * stay wound up after a fault/fall. */
static void reset_pids(void)
{
    pid_angle.reset();
    pid_gyro.reset();
    pid_distance.reset();
    pid_speed.reset();
    pid_yaw_angle.reset();
    pid_yaw_gyro.reset();
    pid_lqr_u.reset();
    pid_zeropoint.reset();
    pid_roll_angle.reset();
}

/* ------------------------------------------------------------------------- */
/* Leg output                                                                */
/* ------------------------------------------------------------------------- */

static void leg_output(int16_t position1, int16_t position2, uint16_t speed, uint8_t acceleration)
{
    if (leg_queue == NULL) {
        return;
    }
    last_leg_target1 = position1;
    last_leg_target2 = position2;
    const leg_pose_t pose = {
        .position1 = position1,
        .position2 = position2,
        .speed = speed,
        .acceleration = acceleration,
    };
    xQueueOverwrite(leg_queue, &pose);
}

static void leg_task(void *arg)
{
    (void)arg;
    leg_pose_t pose = {};
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        if (xQueueReceive(leg_queue, &pose, portMAX_DELAY) == pdTRUE) {
            const uint8_t ids[] = {1, 2};
            const int16_t positions[] = {pose.position1, pose.position2};
            servo_sts_sync_write_position(ids, positions, 2, pose.speed, pose.acceleration);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LEG_OUTPUT_PERIOD_MS));
    }
}

/* ------------------------------------------------------------------------- */
/* Control loops                                                             */
/* ------------------------------------------------------------------------- */

static void yaw_angle_addup(const mpu6050_sample_t *imu)
{
    YAW_angle = imu->angle_z;
    YAW_gyro = imu->gyro_z_dps;

    if (YAW_angle_zero_point == -10.0f) {
        YAW_angle_zero_point = YAW_angle;
    }

    float yaw_angle_1;
    float yaw_angle_2;
    float yaw_addup_angle;
    /* angle_z is accumulated in degrees, so the wrap-around correction is a
     * full 360 degrees (the reference firmware used 2*PI here by mistake). */
    if (YAW_angle > YAW_angle_last) {
        yaw_angle_1 = YAW_angle - YAW_angle_last;
        yaw_angle_2 = YAW_angle - YAW_angle_last - 360.0f;
    } else {
        yaw_angle_1 = YAW_angle - YAW_angle_last;
        yaw_angle_2 = YAW_angle - YAW_angle_last + 360.0f;
    }

    if (fabsf(yaw_angle_1) > fabsf(yaw_angle_2)) {
        yaw_addup_angle = yaw_angle_2;
    } else {
        yaw_addup_angle = yaw_angle_1;
    }

    YAW_angle_total = YAW_angle_total + yaw_addup_angle;
    YAW_angle_last = YAW_angle;
}

static void yaw_loop(const mpu6050_sample_t *imu, const robot_command_t *cmd)
{
    yaw_angle_addup(imu);

    YAW_angle_total += (float)cmd->joy_x * 0.002f;
    float yaw_angle_control = pid_yaw_angle(YAW_angle_total);
    float yaw_gyro_control = pid_yaw_gyro(YAW_gyro);
    YAW_output = yaw_angle_control + yaw_gyro_control;
}

static void lqr_balance_loop(const motor_feedback_t *left, const motor_feedback_t *right,
                             const mpu6050_sample_t *imu, const robot_command_t *cmd)
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
        arm_request = false;
    }

    /* Balance zero compensates for the height-dependent CoM position. */
    float zero = angle_zeropoint -
                 LEG_BALANCE_ZERO_SLOPE * (leg_height_cmd - (float)LEG_HEIGHT_DEFAULT);
    last_balance_zero = zero;
    angle_control = pid_angle(LQR_angle - zero);
    gyro_control = pid_gyro(LQR_gyro);

    if (cmd->joy_y != 0) {
        distance_zeropoint = LQR_distance;
        pid_lqr_u.clear_error();
    }

    if ((prev_joy_x != 0 && cmd->joy_x == 0) || (prev_joy_y != 0 && cmd->joy_y == 0)) {
        wrobot_move_stop_flag = 1;
    }
    if (wrobot_move_stop_flag == 1 && fabsf(LQR_speed) < 0.5f) {
        distance_zeropoint = LQR_distance;
        wrobot_move_stop_flag = 0;
    }

    if (fabsf(LQR_speed) > 15.0f) {
        distance_zeropoint = LQR_distance;
    }

    distance_control = pid_distance(LQR_distance - distance_zeropoint);
    speed_control = pid_speed(LQR_speed - 0.1f * lpf_joy_y((float)cmd->joy_y));

    robot_speed_last = robot_speed;
    robot_speed = LQR_speed;
    if (fabsf(robot_speed - robot_speed_last) > 10.0f || fabsf(robot_speed) > 50.0f ||
        jump_flag != 0) {
        distance_zeropoint = LQR_distance;
        LQR_u = angle_control + gyro_control;
        pid_lqr_u.clear_error();
    } else {
        LQR_u = angle_control + gyro_control + distance_control + speed_control;
    }

    if (fabsf(LQR_u) < 5.0f && cmd->joy_y == 0 && fabsf(distance_control) < 4.0f &&
        jump_flag == 0) {
        LQR_u = pid_lqr_u(LQR_u);
        /* Slow adaptation to the stationary lean angle, hard-bounded around the
         * configured base so a push/held robot can never wind the zero away. */
        angle_zeropoint -= pid_zeropoint(lpf_zeropoint(distance_control));
        if (angle_zeropoint < angle_zeropoint_base - LEG_BALANCE_ZERO_ADAPT) {
            angle_zeropoint = angle_zeropoint_base - LEG_BALANCE_ZERO_ADAPT;
        } else if (angle_zeropoint > angle_zeropoint_base + LEG_BALANCE_ZERO_ADAPT) {
            angle_zeropoint = angle_zeropoint_base + LEG_BALANCE_ZERO_ADAPT;
        }
    } else {
        pid_lqr_u.clear_error();
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
    if (accel_mag_g < air_thresh_g) {
        if (air_count < air_hold_ticks) {
            air_count++;
        }
    } else if (air_count > 0) {
        air_count--;
    }
    airborne = (air_count > 0) ? 1 : 0;
    if (airborne) {
        LQR_u *= air_scale;
        pid_lqr_u.clear_error();
    }
}

static int16_t clamp_position(float value, int16_t low, int16_t high)
{
    if (value < (float)low) {
        return low;
    }
    if (value > (float)high) {
        return high;
    }
    return (int16_t)value;
}

static void jump_legs_to(int height, int speed, int acc)
{
    int16_t p1 = (int16_t)(LEG_POSITION_CENTER + LEG_MOUNT_OFFSET +
                           LEG_HEIGHT_STEP * (height - LEG_HEIGHT_MIN));
    int16_t p2 = (int16_t)(LEG_POSITION_CENTER - LEG_MOUNT_OFFSET -
                           LEG_HEIGHT_STEP * (height - LEG_HEIGHT_MIN));
    leg_output(p1, p2, speed, acc);
}

/* Three-phase jump gait, one command per control tick (~500 Hz):
 *   1. crouch to jump_crouch at max speed, hold jump_crouch_ticks to settle,
 *   2. slam up to jump_height (launch),
 *   3. after jump_land_ticks retract to jump_land_height to catch the landing. */
static void jump_loop(const robot_command_t *cmd)
{
    if (prev_dir == ROBOT_JUMP && cmd->dir == ROBOT_STOP && jump_flag == 0) {
        jump_flag = 1;
    }
    if (jump_flag == 0) {
        return;
    }
    jump_flag++;

    const int slam_tick = jump_crouch_ticks + 2;
    const int land_tick = slam_tick + jump_land_ticks;
    if (jump_flag == 2) {
        jump_legs_to(jump_crouch, 0, 0);
    } else if (jump_flag == slam_tick) {
        jump_legs_to(jump_height, jump_speed, jump_acc);
    } else if (jump_flag == land_tick) {
        jump_legs_to(jump_land_height, jump_speed, jump_acc);
    }
    if (jump_flag > land_tick + 160) {
        jump_flag = 0;
    }
}

static void leg_loop(const mpu6050_sample_t *imu, const robot_command_t *cmd)
{
    jump_loop(cmd);
    if (jump_flag != 0) {
        return;
    }

    float roll_angle = imu->angle_x + roll_bias - (float)cmd->roll;
    last_roll_angle = imu->angle_x;
    if (fault_reason != FAULT_NONE) {
        /* While faulted the robot is usually on its side and the roll error is
         * huge; chasing it would wind the integral up and slam the legs to the
         * travel clamps. Hold the legs level and keep the loop unwound so the
         * recovery starts from a neutral pose. */
        pid_roll_angle.reset();
        leg_position_add = 0.0f;
    } else {
        float roll_out = pid_roll_angle(lpf_roll(roll_angle));
        leg_position_add = (roll_mode == 0) ? 0.0f : (float)roll_mode * roll_out;
    }

    /* Slew the commanded height so a slider jump does not kick the chassis. */
    float target_height = (float)cmd->height;
    if (target_height > leg_height_cmd) {
        leg_height_cmd += fminf(LEG_HEIGHT_SLEW, target_height - leg_height_cmd);
    } else {
        leg_height_cmd -= fminf(LEG_HEIGHT_SLEW, leg_height_cmd - target_height);
    }

    float height_offset = LEG_HEIGHT_STEP * (leg_height_cmd - LEG_HEIGHT_MIN);
    float position1 = LEG_POSITION_CENTER + LEG_MOUNT_OFFSET + height_offset - leg_position_add;
    float position2 = LEG_POSITION_CENTER - LEG_MOUNT_OFFSET - height_offset - leg_position_add;

    int16_t p1 = clamp_position(position1, LEG_POS1_MIN, LEG_POS1_MAX);
    int16_t p2 = clamp_position(position2, LEG_POS2_MIN, LEG_POS2_MAX);
    leg_output(p1, p2, LEG_MOVE_SPEED, LEG_MOVE_ACC);
}

static void apply_motor_targets(const robot_command_t *cmd)
{
    if (cmd->go == 0) {
        motor_foc_set_target(MOTOR_LEFT, 0.0f);
        motor_foc_set_target(MOTOR_RIGHT, 0.0f);
        leg_position_add = 0.0f;
        return;
    }
    float yaw = (yaw_mode == 0) ? 0.0f : ((yaw_mode > 0) ? YAW_output : -YAW_output);
    motor_foc_set_target(MOTOR_LEFT, (-0.5f) * (LQR_u + yaw));
    motor_foc_set_target(MOTOR_RIGHT, (-0.5f) * (LQR_u - yaw));
}

/* ------------------------------------------------------------------------- */
/* Fault handling                                                            */
/* ------------------------------------------------------------------------- */

#define ATTITUDE_RECOVER_DEG  10.0f
#define RECOVER_HOLD_TICKS    200
#define BATTERY_CHECK_PERIOD  50
static float battery_voltage_throttled(void)
{
    static int ticks;
    static float cached = -1.0f;
    if (cached < 0.0f || ++ticks >= BATTERY_CHECK_PERIOD) {
        ticks = 0;
        cached = board_battery_voltage();
    }
    return cached;
}

/* The battery sense input is noisy on some builds, so only cut the motors after
 * the reading stays low for a while, and ignore implausibly low values (a
 * powered 2S pack cannot read below ~4 V; that means a bad connection). */
#define BATTERY_LOW_DEBOUNCE  40      /* BATTERY_CHECK_PERIOD ticks -> ~2 s */
#define BATTERY_MIN_PLAUSIBLE 4.0f

static bool battery_is_low(void)
{
    static int low_count;
    float voltage = battery_voltage_throttled();

    if (voltage < BATTERY_MIN_PLAUSIBLE) {
        low_count = 0;
        return false;
    }
    if (voltage < BOARD_BATTERY_LOW_VOLTAGE) {
        if (low_count < BATTERY_LOW_DEBOUNCE) {
            low_count++;
        }
        return low_count >= BATTERY_LOW_DEBOUNCE;
    }
    low_count = 0;
    return false;
}

static void enter_fault(fault_reason_t reason, bool operator_go)
{
    if (fault_reason != FAULT_NONE) {
        return;
    }
    fault_reason = reason;
    recover_ticks = 0;
    motor_foc_stop();
    reset_pids();
    robot_control_set_go(false);
    /* Auto-recovery only for attitude faults while the operator wanted to run;
     * battery faults always need a manual restart. */
    go_auto_latch = (reason == FAULT_ATTITUDE) && operator_go;
    robot_state_set(ROBOT_STATE_FAULT);
    ESP_LOGE("robot_control", "fault (%s), motors disabled%s",
             reason == FAULT_BATTERY ? "battery low" : "attitude",
             go_auto_latch ? "; will auto-recover when upright" : "; set go=1 when resolved");
}

static void fault_recover(const robot_command_t *cmd, const mpu6050_sample_t *imu)
{
    LQR_angle = imu->angle_y;

    bool condition_ok = (fault_reason == FAULT_BATTERY)
        ? (board_battery_voltage() > BOARD_BATTERY_RECOVER_VOLTAGE)
        : (fabsf(LQR_angle) < ATTITUDE_RECOVER_DEG);

    if (condition_ok && (cmd->go || go_auto_latch)) {
        if (++recover_ticks >= RECOVER_HOLD_TICKS) {
            recover_ticks = 0;
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
        recover_ticks = 0;
    }
}

/* ------------------------------------------------------------------------- */
/* Control task                                                              */
/* ------------------------------------------------------------------------- */

static void control_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    motor_feedback_t left = {};
    motor_feedback_t right = {};
    mpu6050_sample_t imu = {};

    while (true) {
        robot_command_t cmd = robot_control_get_command();
        __atomic_fetch_add(&control_loop_count, 1, __ATOMIC_RELAXED);

        /* Latched direction buttons drive like a held joystick (the reference
         * firmware left ROBOT_FORWARD/BACK/LEFT/RIGHT unwired; only the jump
         * edge used cmd.dir). Magnitudes match the tested joystick range. */
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

            /* Research mode: direct wheel torque, bypasses balance + fault. */
            if (manual_ticks > 0) {
                if (!manual_armed) {
                    motor_foc_enable_torque();
                    manual_armed = true;
                }
                motor_foc_set_target(MOTOR_LEFT, manual_target);
                motor_foc_set_target(MOTOR_RIGHT, manual_target);
                leg_loop(&imu, &cmd);
                int remaining = manual_ticks - 1;
                manual_ticks = remaining;
                if (remaining == 0) {
                    motor_foc_stop();
                    manual_armed = false;
                }
                prev_dir = cmd.dir;
                prev_joy_x = cmd.joy_x;
                prev_joy_y = cmd.joy_y;
                motor_foc_step();
                vTaskDelayUntil(&last_wake, 1);
                continue;
            }

            if (fault_reason != FAULT_NONE) {
                LQR_angle = imu.angle_y;
                if (getup_request && fault_reason == FAULT_ATTITUDE) {
                    getup_request = 0;
                    getup_state = 1;
                    getup_ticks = 0;
                    motor_foc_enable_torque();
                }
                if (getup_state == 1) {
                    /* Drive the wheels toward the side that lifts the chassis
                     * back over the axle; release near upright. */
                    const float dir = (LQR_angle > 0.0f) ? 1.0f : -1.0f;
                    const float t = (float)getup_sign * dir * getup_torque;
                    motor_foc_set_target(MOTOR_LEFT, t);
                    motor_foc_set_target(MOTOR_RIGHT, t);
                    leg_loop(&imu, &cmd);
                    int ticks = getup_ticks + 1;
                    getup_ticks = ticks;
                    if (fabsf(LQR_angle) < getup_release_deg || ticks > 3000) {
                        getup_state = 0;
                        getup_ticks = 0;
                        reset_pids();
                        arm_request = true;
                        go_auto_latch = false;
                        fault_reason = FAULT_NONE;
                        robot_control_set_go(true);
                        robot_state_set(ROBOT_STATE_RUNNING);
                        ESP_LOGI("robot_control", "get-up released at %.1f deg",
                                 LQR_angle);
                    }
                } else {
                    fault_recover(&cmd, &imu);
                    leg_loop(&imu, &cmd);
                }
                getup_state_pub = getup_state;
            } else {
                motor_foc_get_feedback(MOTOR_LEFT, &left);
                motor_foc_get_feedback(MOTOR_RIGHT, &right);

                lqr_balance_loop(&left, &right, &imu, &cmd);
                yaw_loop(&imu, &cmd);
                leg_loop(&imu, &cmd);

                if (fabsf(LQR_angle) > attitude_fault_deg) {
                    enter_fault(FAULT_ATTITUDE, cmd.go);
                } else if (cmd.go && battery_is_low()) {
                    enter_fault(FAULT_BATTERY, cmd.go);
                } else {
                    apply_motor_targets(&cmd);
                    robot_state_set(cmd.go ? ROBOT_STATE_RUNNING : ROBOT_STATE_READY);
                }
            }

            prev_dir = cmd.dir;
            prev_joy_x = cmd.joy_x;
            prev_joy_y = cmd.joy_y;
        }

        {
            static float a_min = 1e9f;
            static float a_max = -1e9f;
            static int pp_count;
            if (LQR_angle < a_min) a_min = LQR_angle;
            if (LQR_angle > a_max) a_max = LQR_angle;
            if (++pp_count >= 500) {
                angle_pp = a_max - a_min;
                a_min = 1e9f;
                a_max = -1e9f;
                pp_count = 0;
            }
        }

        motor_foc_step();
        vTaskDelayUntil(&last_wake, 1);
    }
}

esp_err_t robot_control_start(void)
{
    if (control_started) {
        return ESP_OK;
    }

    leg_queue = xQueueCreate(1, sizeof(leg_pose_t));
    if (leg_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* Control starts before wifi_net (which also inits NVS), so make sure the
     * key-value store exists before loading the saved level calibration. */
    nvs_flash_init();
    roll_bias_load();
    if (xTaskCreatePinnedToCore(control_task, "control_task", 6144, NULL, CONTROL_TASK_PRIO,
                                NULL, ROBOT_TASK_CORE) != pdPASS) {
        vQueueDelete(leg_queue);
        leg_queue = NULL;
        return ESP_FAIL;
    }
    if (xTaskCreatePinnedToCore(leg_task, "leg_task", 4096, NULL, LEG_TASK_PRIO,
                                NULL, ROBOT_TASK_CORE) != pdPASS) {
        return ESP_FAIL;
    }

    control_started = true;
    robot_state_set(ROBOT_STATE_READY);
    ESP_LOGI("robot_control", "control + leg tasks started (LQR + yaw + leg)");
    return ESP_OK;
}

float robot_control_lqr_angle(void)
{
    return LQR_angle;
}

float robot_control_lqr_u(void)
{
    return LQR_u;
}

float robot_control_yaw_output(void)
{
    return YAW_output;
}

float robot_control_yaw_total(void)
{
    return YAW_angle_total;
}

float robot_control_left_velocity(void)
{
    return last_left_velocity;
}

float robot_control_right_velocity(void)
{
    return last_right_velocity;
}

float robot_control_gyro_z(void)
{
    return last_gyro_z;
}

void robot_control_get_terms(float *angle, float *gyro, float *distance, float *speed)
{
    if (angle) *angle = angle_control;
    if (gyro) *gyro = gyro_control;
    if (distance) *distance = distance_control;
    if (speed) *speed = speed_control;
}

float robot_control_leg_add(void)
{
    return leg_position_add;
}

float robot_control_roll_angle(void)
{
    return last_roll_angle;
}

bool robot_control_faulted(void)
{
    return fault_reason != FAULT_NONE;
}

uint32_t robot_control_loop_count(void)
{
    return __atomic_load_n(&control_loop_count, __ATOMIC_RELAXED);
}
