#pragma once

/*
 * Private interface between the robot_control translation units.
 *
 * `robot_control.cpp` owns the real-time control task and defines all of the
 * state below; `robot_control_api.cpp` is the public setter/getter surface that
 * the console, HTTP and WebSocket tasks call from other cores. Sharing the
 * snapshot structures and their locks here keeps that boundary explicit instead
 * of exposing it through the public header.
 *
 * Not for use outside the robot_control module.
 */

#include "robot_control.h"
#include "robot_config.h"
#include "robot_logic.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define REQUEST_QUEUE_LEN    8
#define WHEEL_SEQ_MAX        8

/* All app tasks share core 1, leaving core 0 to the Wi-Fi/system stack, like the
 * single-core Arduino reference where the control loop never competes with Wi-Fi. */
#define ROBOT_TASK_CORE      1
#define CONTROL_TASK_PRIO    7
#define LEG_TASK_PRIO        4

typedef struct {
    float p, i, d, limit;
} pid_gains_t;

typedef struct {
    /* Remote-control command: written by the console/HTTP/WebSocket tasks,
     * snapshotted by the control task every loop. */
    robot_command_t command;

    /* Tunables (never touched by the control task directly). */
    pid_gains_t pid[ROBOT_PID_COUNT];
    float lpf_tf[ROBOT_LPF_COUNT];
    int leg1_min, leg1_max, leg2_min, leg2_max;
    int jump_height, jump_land_height, jump_speed, jump_acc, jump_land_ticks;
    int jump_crouch, jump_crouch_ticks;
    int bump_amp, bump_ticks, bump_speed, bump_acc;
    float getup_torque, getup_release_deg;
    int getup_sign, getup_final_height;
    int yaw_mode, roll_mode;
    float fault_deg;
    float air_thresh_g, air_scale;
    float zero_trim;
    float yaw_wheel_scale, yaw_wheel_corr;
    float roll_bias;
    float angle_zeropoint;        /* base balance zero (the 'zero=' setting) */

    /* Manual leg hold. `height`/`bump` clear it so the operator can always get
     * back to normal height/roll control over Wi-Fi. */
    bool manual_leg_enable;
    int manual_leg1, manual_leg2;

    /* Set by the arm of a wheel sequence; consumed and cleared by the control
     * task when it hands over to the balance loop. */
    bool wheel_seq_arm;
} robot_shared_t;

/* robot_telemetry_t is public (robot_telemetry.h) because /api/status reads a
 * whole frame at once. */

/* Discrete one-shot actions. Kept out of the shared snapshot so they are never
 * lost or applied twice. */
typedef enum {
    REQ_ATTITUDE_RESET,
    REQ_ZERO_AUTO_RESET,
    REQ_GO_OFF,          /* operator stop: cancel a pending auto-recovery */
    REQ_BUMP,            /* u.i[0] = 1 left, 2 right, 0 cancel */
    REQ_GETUP,           /* u.i[0] = 0/1 */
    REQ_MANUAL_DRIVE,    /* u.f[0] = target, u.i[0] = ms */
    REQ_WHEEL_SEQ,       /* u.seq */
} robot_request_kind_t;

typedef struct {
    robot_request_kind_t kind;
    union {
        int i[4];
        float f[2];
        struct {
            float targets[WHEEL_SEQ_MAX];
            int durations_ms[WHEEL_SEQ_MAX];
            int count;
        } seq;
    } u;
} robot_request_t;

/* Defined in robot_control.cpp. */
extern robot_shared_t shared;
extern robot_telemetry_t telemetry;
extern SemaphoreHandle_t shared_mutex;
extern SemaphoreHandle_t telemetry_mutex;
extern QueueHandle_t request_queue;

/* Defined in robot_control.cpp: lock helpers and the ordered-request mailbox. */
bool shared_ready(void);
void shared_lock(void);
void shared_unlock(void);
void telemetry_lock(void);
void telemetry_unlock(void);
void post_request(const robot_request_t *request);

/* Defined in robot_control_persist.cpp. roll_bias_load is called by the
 * control task at start-up; roll_bias_save follows every calibration change. */
void roll_bias_save(void);
void roll_bias_load(void);

/* Defined in robot_control_legs.cpp: the low-rate servo output path. The
 * control task publishes the newest pose; the leg task writes it to the bus. */
esp_err_t robot_control_leg_io_start(void);
void robot_control_leg_output(int16_t position1, int16_t position2,
                              uint16_t speed, uint8_t acceleration);
void robot_control_leg_targets(int16_t *target1, int16_t *target2);

#ifdef __cplusplus
}
#endif
