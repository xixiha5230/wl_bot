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

static StabPID pid_angle(1.0f, 0.0f, 0.0f, 100000.0f, 8.0f);
static StabPID pid_gyro(0.06f, 0.0f, 0.0f, 100000.0f, 8.0f);
static StabPID pid_distance(0.5f, 0.0f, 0.0f, 100000.0f, 8.0f);
static StabPID pid_speed(0.7f, 0.0f, 0.0f, 100000.0f, 8.0f);
static StabPID pid_yaw_angle(1.0f, 0.0f, 0.0f, 100000.0f, 8.0f);
static StabPID pid_yaw_gyro(0.04f, 0.0f, 0.0f, 100000.0f, 8.0f);
static StabPID pid_lqr_u(1.0f, 15.0f, 0.0f, 100000.0f, 8.0f);
static StabPID pid_zeropoint(0.001f, 0.0f, 0.0f, 100000.0f, 4.0f);
static StabPID pid_roll_angle(4.0f, 0.0f, 0.0f, 100000.0f, 450.0f);

static LowPassFilter lpf_joy_y(0.2f);
static LowPassFilter lpf_zeropoint(0.1f);
static LowPassFilter lpf_roll(0.6f);

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
static float angle_control;
static float gyro_control;
static float speed_control;
static float distance_control;
/* Balance zero point: base value adjusted by the height-dependent offset. The
 * reference -2.25 assumes a different sensor orientation; this is the value
 * measured on this build (see robot_config.h). 'zero' changes the base. */
static float angle_zeropoint = LEG_BALANCE_ZERO_DEFAULT;
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
static int yaw_mode = 1;   /* 1 = normal, -1 = inverted, 0 = disabled */
static bool arm_request;   /* reset distance zero/PIDs when go turns on */

/* Attitude/battery fault latch: motors are disabled and stay off until the
 * operator commands go=1 again once the fault condition has cleared. */
typedef enum {
    FAULT_NONE = 0,
    FAULT_ATTITUDE,
    FAULT_BATTERY,
} fault_reason_t;

static volatile fault_reason_t fault_reason = FAULT_NONE;
static int recover_ticks;

static bool control_started;
static QueueHandle_t leg_queue;
static uint32_t control_loop_count;

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
}

float robot_control_get_angle_zeropoint(void)
{
    return angle_zeropoint;
}

float robot_control_get_balance_zero(void)
{
    return last_balance_zero;
}

void robot_control_set_go(bool go)
{
    portENTER_CRITICAL(&cmd_mux);
    command.go = go;
    portEXIT_CRITICAL(&cmd_mux);
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

static void reset_pids(void)
{
    pid_angle.clear_error();
    pid_gyro.clear_error();
    pid_distance.clear_error();
    pid_speed.clear_error();
    pid_yaw_angle.clear_error();
    pid_yaw_gyro.clear_error();
    pid_lqr_u.clear_error();
    pid_zeropoint.clear_error();
    pid_roll_angle.clear_error();
}

/* ------------------------------------------------------------------------- */
/* Leg output                                                                */
/* ------------------------------------------------------------------------- */

static void leg_output(int16_t position1, int16_t position2, uint16_t speed, uint8_t acceleration)
{
    if (leg_queue == NULL) {
        return;
    }
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
        angle_zeropoint -= pid_zeropoint(lpf_zeropoint(distance_control));
    } else {
        pid_lqr_u.clear_error();
    }

    if (cmd->height < 50) {
        pid_speed.P = 0.7f;
    } else if (cmd->height < 64) {
        pid_speed.P = 0.6f;
    } else {
        pid_speed.P = 0.5f;
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

static void jump_loop(const robot_command_t *cmd)
{
    if (prev_dir == ROBOT_JUMP && cmd->dir == ROBOT_STOP && jump_flag == 0) {
        int16_t p1 = (int16_t)(LEG_POSITION_CENTER + LEG_MOUNT_OFFSET +
                               LEG_HEIGHT_STEP * (LEG_JUMP_HEIGHT - LEG_HEIGHT_MIN));
        int16_t p2 = (int16_t)(LEG_POSITION_CENTER - LEG_MOUNT_OFFSET -
                               LEG_HEIGHT_STEP * (LEG_JUMP_HEIGHT - LEG_HEIGHT_MIN));
        leg_output(p1, p2, LEG_JUMP_SPEED, LEG_JUMP_ACC);
        jump_flag = 1;
    }

    if (jump_flag > 0) {
        jump_flag++;
        if (jump_flag > 30 && jump_flag < 35) {
            int16_t p1 = (int16_t)(LEG_POSITION_CENTER + LEG_MOUNT_OFFSET +
                                   LEG_HEIGHT_STEP * (LEG_JUMP_LAND_HEIGHT - LEG_HEIGHT_MIN));
            int16_t p2 = (int16_t)(LEG_POSITION_CENTER - LEG_MOUNT_OFFSET -
                                   LEG_HEIGHT_STEP * (LEG_JUMP_LAND_HEIGHT - LEG_HEIGHT_MIN));
            leg_output(p1, p2, LEG_JUMP_SPEED, LEG_JUMP_ACC);
            jump_flag = 40;
        }
        if (jump_flag > 200) {
            jump_flag = 0;
        }
    }
}

static void leg_loop(const mpu6050_sample_t *imu, const robot_command_t *cmd)
{
    jump_loop(cmd);
    if (jump_flag != 0) {
        return;
    }

    float roll_angle = imu->angle_x + 2.0f;
    last_roll_angle = imu->angle_x;
    leg_position_add = pid_roll_angle(lpf_roll(roll_angle));

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

#define ATTITUDE_FAULT_DEG    25.0f
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

static void enter_fault(fault_reason_t reason)
{
    if (fault_reason != FAULT_NONE) {
        return;
    }
    fault_reason = reason;
    recover_ticks = 0;
    motor_foc_stop();
    reset_pids();
    robot_control_set_go(false);
    robot_state_set(ROBOT_STATE_FAULT);
    ESP_LOGE("robot_control", "fault (%s), motors disabled; set go=1 when resolved",
             reason == FAULT_BATTERY ? "battery low" : "attitude");
}

static void fault_recover(const robot_command_t *cmd, const mpu6050_sample_t *imu)
{
    LQR_angle = imu->angle_y;

    bool condition_ok = (fault_reason == FAULT_BATTERY)
        ? (board_battery_voltage() > BOARD_BATTERY_RECOVER_VOLTAGE)
        : (fabsf(LQR_angle) < ATTITUDE_RECOVER_DEG);

    if (condition_ok && cmd->go) {
        if (++recover_ticks >= RECOVER_HOLD_TICKS) {
            recover_ticks = 0;
            reset_pids();
            if (motor_foc_enable_torque() == ESP_OK) {
                fault_reason = FAULT_NONE;
                robot_state_set(ROBOT_STATE_READY);
                ESP_LOGI("robot_control", "recovered from fault");
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

        if (sensors_read_imu(&imu) == ESP_OK) {
            if (cmd.go && !prev_go) {
                arm_request = true;
            }
            prev_go = cmd.go;

            if (fault_reason != FAULT_NONE) {
                fault_recover(&cmd, &imu);
                leg_loop(&imu, &cmd);
            } else {
                motor_foc_get_feedback(MOTOR_LEFT, &left);
                motor_foc_get_feedback(MOTOR_RIGHT, &right);

                lqr_balance_loop(&left, &right, &imu, &cmd);
                yaw_loop(&imu, &cmd);
                leg_loop(&imu, &cmd);

                if (fabsf(LQR_angle) > ATTITUDE_FAULT_DEG) {
                    enter_fault(FAULT_ATTITUDE);
                } else if (cmd.go && battery_is_low()) {
                    enter_fault(FAULT_BATTERY);
                } else {
                    apply_motor_targets(&cmd);
                    robot_state_set(cmd.go ? ROBOT_STATE_RUNNING : ROBOT_STATE_READY);
                }
            }

            prev_dir = cmd.dir;
            prev_joy_x = cmd.joy_x;
            prev_joy_y = cmd.joy_y;
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
