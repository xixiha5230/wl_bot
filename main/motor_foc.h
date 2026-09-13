#pragma once

#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1,
} motor_id_t;

typedef enum {
    MOTOR_MODE_DISABLED = 0,
    MOTOR_MODE_TORQUE,
    MOTOR_MODE_VELOCITY,
    MOTOR_MODE_ANGLE,
    MOTOR_MODE_VELOCITY_OPENLOOP,
    MOTOR_MODE_ANGLE_OPENLOOP,
} motor_mode_t;

typedef struct {
    float target;
    float angle;
    float velocity;
    float sensor_angle;
    float voltage_q;
} motor_feedback_t;

/* Create the LEDC 3PWM drivers and link them to the AS5600 sensors. Does not
 * move the motors; the L6234 outputs stay at zero volts. */
esp_err_t motor_foc_init(void);

/* Run the SimpleFOC sensor/motor alignment. This energizes the motors and
 * moves each wheel slightly, so the robot must be secured. On failure the
 * motors are left disabled. */
esp_err_t motor_foc_align(void);

/* Align a single motor (id 0 = left, 1 = right). Disables it on failure. */
esp_err_t motor_foc_align_motor(motor_id_t motor);

/* Override the pole pair count and invalidate the previous alignment. */
void motor_foc_set_pole_pairs(motor_id_t motor, int pole_pairs);

/* Override the alignment voltage used by the next align of this motor. */
void motor_foc_set_align_voltage(motor_id_t motor, float volts);

void motor_foc_set_mode(motor_mode_t mode);
motor_mode_t motor_foc_get_mode(void);
bool motor_foc_is_aligned(void);
void motor_foc_set_target(motor_id_t motor, float target);

/* Re-enable both drivers in torque mode after a stop/fault (requires alignment). */
esp_err_t motor_foc_enable_torque(void);

/* Disable both drivers and zero the targets. */
void motor_foc_stop(void);

void motor_foc_get_feedback(motor_id_t motor, motor_feedback_t *feedback);

/* Run one FOC iteration (sensor update + torque + shaft feedback). The control
 * task calls this once per loop, mirroring the reference single-loop design. */
void motor_foc_step(void);

/* Number of FOC-loop iterations since boot (for rate diagnostics). */
uint32_t motor_foc_loop_count(void);

#ifdef __cplusplus
}
#endif
