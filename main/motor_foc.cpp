#include "motor_foc.h"

#include "board.h"
#include "esp_log.h"
#include "esp_simplefoc.h"
#include "sensors.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <math.h>

static const char *TAG = "motor_foc";

/* Forwards SimpleFOC debug output (MOT: ...) to ESP-IDF logging. */
class LogPrint : public Print {
public:
    size_t write(uint8_t c) override
    {
        if (c == '\n' || len_ >= sizeof(buf_) - 1) {
            flush();
            return 1;
        }
        if (c != '\r') {
            buf_[len_++] = (char)c;
        }
        return 1;
    }

    size_t write(const uint8_t *buffer, size_t size) override
    {
        for (size_t i = 0; i < size; ++i) {
            write(buffer[i]);
        }
        return size;
    }

private:
    void flush(void)
    {
        if (len_ > 0) {
            buf_[len_] = '\0';
            ESP_LOGI("sfoc", "%s", buf_);
            len_ = 0;
        }
    }

    char buf_[160] = {};
    size_t len_ = 0;
};

static LogPrint log_print;

#define AS5600_COUNTS_PER_REV   4096.0f
#define MOTOR_POLE_PAIRS        7
#define MOTOR_SUPPLY_VOLTAGE    8.0f
#define MOTOR_ALIGN_VOLTAGE     6.0f

/* SimpleFOC sensor fed by the existing AS5600 driver, so the managed AS5600
 * (legacy i2c_bus) does not fight with our i2c_master buses. */
class BoardEncoder : public Sensor {
public:
    explicit BoardEncoder(uint8_t index) : index_(index) {}

    void begin(void)
    {
        Sensor::init();
    }

protected:
    float getSensorAngle(void) override
    {
        uint16_t raw = 0;
        if (sensors_read_encoder(index_, &raw) != ESP_OK) {
            return -1.0f; /* Sensor::update() treats negative values as errors. */
        }
        return (float)raw * (_2PI / AS5600_COUNTS_PER_REV);
    }

private:
    uint8_t index_;
};

static BLDCMotor motor_left(MOTOR_POLE_PAIRS);
static BLDCMotor motor_right(MOTOR_POLE_PAIRS);
static BLDCDriver3PWM driver_left(BOARD_MOTOR_LEFT_U, BOARD_MOTOR_LEFT_V,
                                  BOARD_MOTOR_LEFT_W, BOARD_MOTOR_LEFT_ENABLE);
static BLDCDriver3PWM driver_right(BOARD_MOTOR_RIGHT_U, BOARD_MOTOR_RIGHT_V,
                                   BOARD_MOTOR_RIGHT_W, BOARD_MOTOR_RIGHT_ENABLE);
static BoardEncoder encoder_left(0);
static BoardEncoder encoder_right(1);

/* Shared state between the control task and the console/alignment callers. */
static portMUX_TYPE foc_mux = portMUX_INITIALIZER_UNLOCKED;
static motor_mode_t current_mode = MOTOR_MODE_DISABLED;
static float target_left = 0.0f;
static float target_right = 0.0f;
static bool foc_ready = false;
static bool foc_aligned = false;
static bool foc_suspended = false;

/* Serializes motor stepping (control task) against sensor alignment. */
static SemaphoreHandle_t foc_mutex;
static uint32_t foc_loop_count;

static MotionControlType component_mode(motor_mode_t mode)
{
    switch (mode) {
    case MOTOR_MODE_VELOCITY:
        return MotionControlType::velocity;
    case MOTOR_MODE_ANGLE:
        return MotionControlType::angle;
    case MOTOR_MODE_VELOCITY_OPENLOOP:
        return MotionControlType::velocity_openloop;
    case MOTOR_MODE_ANGLE_OPENLOOP:
        return MotionControlType::angle_openloop;
    case MOTOR_MODE_TORQUE:
    default:
        return MotionControlType::torque;
    }
}

static bool mode_needs_alignment(motor_mode_t mode)
{
    return mode == MOTOR_MODE_TORQUE || mode == MOTOR_MODE_VELOCITY || mode == MOTOR_MODE_ANGLE;
}

static void configure_motor(BLDCMotor &motor, BLDCDriver3PWM &driver, BoardEncoder &encoder)
{
    driver.voltage_power_supply = MOTOR_SUPPLY_VOLTAGE;
    driver.voltage_limit = MOTOR_SUPPLY_VOLTAGE;

    motor.linkDriver(&driver);
    motor.linkSensor(&encoder);
    motor.voltage_limit = MOTOR_SUPPLY_VOLTAGE;
    motor.voltage_sensor_align = MOTOR_ALIGN_VOLTAGE;
    motor.torque_controller = TorqueControlType::voltage;
    motor.controller = MotionControlType::torque;

    motor.PID_velocity.P = 0.05f;
    motor.PID_velocity.I = 1.0f;
    motor.PID_velocity.D = 0.0f;
    motor.LPF_velocity.Tf = 0.005f;
}

esp_err_t motor_foc_init(void)
{
    portENTER_CRITICAL(&foc_mux);
    bool already_ready = foc_ready;
    portEXIT_CRITICAL(&foc_mux);
    if (already_ready) {
        return ESP_OK;
    }

    foc_mutex = xSemaphoreCreateMutex();
    if (foc_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    encoder_left.begin();
    encoder_right.begin();

    SimpleFOCDebug::enable(&log_print);

    configure_motor(motor_left, driver_left, encoder_left);
    configure_motor(motor_right, driver_right, encoder_right);

    if (!driver_left.init({0, 1, 2})) {
        ESP_LOGE(TAG, "left driver init failed");
        return ESP_FAIL;
    }
    if (!driver_right.init({3, 4, 5})) {
        ESP_LOGE(TAG, "right driver init failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "LEDC 3PWM drivers ready (ch 0-2, 3-5), motors not aligned");

    motor_left.init();
    motor_right.init();

    portENTER_CRITICAL(&foc_mux);
    foc_ready = true;
    portEXIT_CRITICAL(&foc_mux);
    return ESP_OK;
}

/* Run one FOC iteration: update sensors, apply torque, update shaft feedback.
 * Called from the control task at ~1 kHz, matching the reference single loop
 * (mpu update -> control -> loopFOC -> move). */
void motor_foc_step(void)
{
    portENTER_CRITICAL(&foc_mux);
    bool ready = foc_ready;
    bool suspended = foc_suspended;
    portEXIT_CRITICAL(&foc_mux);
    if (!ready || suspended) {
        return;
    }
    if (xSemaphoreTake(foc_mutex, 0) != pdTRUE) {
        return;
    }

    __atomic_fetch_add(&foc_loop_count, 1, __ATOMIC_RELAXED);

    motor_mode_t mode;
    float left_target;
    float right_target;
    portENTER_CRITICAL(&foc_mux);
    mode = current_mode;
    left_target = target_left;
    right_target = target_right;
    portEXIT_CRITICAL(&foc_mux);

    /* Set the target first, then update the sensor and apply the voltage in the
     * same iteration. (loopFOC applies current_sp, so doing move() afterwards
     * would delay the new torque by one control cycle.) */
    motor_left.move(mode == MOTOR_MODE_DISABLED ? 0.0f : left_target);
    motor_right.move(mode == MOTOR_MODE_DISABLED ? 0.0f : right_target);
    motor_left.loopFOC();
    motor_right.loopFOC();

    xSemaphoreGive(foc_mutex);
}

/* Take the motor lock and stop the control task from stepping while aligning. */
static void begin_align(void)
{
    portENTER_CRITICAL(&foc_mux);
    foc_suspended = true;
    portEXIT_CRITICAL(&foc_mux);
    xSemaphoreTake(foc_mutex, portMAX_DELAY);
}

static void end_align(void)
{
    xSemaphoreGive(foc_mutex);
    portENTER_CRITICAL(&foc_mux);
    foc_suspended = false;
    portEXIT_CRITICAL(&foc_mux);
}

esp_err_t motor_foc_align(void)
{
    portENTER_CRITICAL(&foc_mux);
    bool ready = foc_ready;
    portEXIT_CRITICAL(&foc_mux);
    if (!ready) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG, "aligning motors: wheels will move");
    begin_align();

    motor_left.enable();
    motor_right.enable();
    int left = motor_left.initFOC();
    int right = motor_right.initFOC();

    end_align();

    if (!left || !right) {
        motor_left.disable();
        motor_right.disable();
        portENTER_CRITICAL(&foc_mux);
        foc_aligned = false;
        current_mode = MOTOR_MODE_DISABLED;
        portEXIT_CRITICAL(&foc_mux);
        ESP_LOGE(TAG, "alignment failed (left=%d right=%d), motors disabled", left, right);
        return ESP_FAIL;
    }

    portENTER_CRITICAL(&foc_mux);
    foc_aligned = true;
    current_mode = MOTOR_MODE_TORQUE;
    target_left = 0.0f;
    target_right = 0.0f;
    portEXIT_CRITICAL(&foc_mux);
    ESP_LOGI(TAG, "aligned: zero_elec L=%.4f R=%.4f dir L=%d R=%d",
             motor_left.zero_electric_angle, motor_right.zero_electric_angle,
             (int)motor_left.sensor_direction, (int)motor_right.sensor_direction);
    return ESP_OK;
}

esp_err_t motor_foc_align_motor(motor_id_t motor)
{
    portENTER_CRITICAL(&foc_mux);
    bool ready = foc_ready;
    portEXIT_CRITICAL(&foc_mux);
    if (!ready) {
        return ESP_ERR_INVALID_STATE;
    }

    BLDCMotor &m = (motor == MOTOR_LEFT) ? motor_left : motor_right;
    motor_foc_set_target(motor, 0.0f);
    begin_align();

    m.enable();
    int ok = m.initFOC();

    end_align();

    if (!ok) {
        m.disable();
        ESP_LOGE(TAG, "align motor %d failed, disabled", (int)motor);
        return ESP_FAIL;
    }

    portENTER_CRITICAL(&foc_mux);
    foc_aligned = true;
    current_mode = MOTOR_MODE_TORQUE;
    portEXIT_CRITICAL(&foc_mux);
    ESP_LOGI(TAG, "align motor %d -> %d (zero=%.4f dir=%d)",
             (int)motor, ok, m.zero_electric_angle, (int)m.sensor_direction);
    return ESP_OK;
}

void motor_foc_set_pole_pairs(motor_id_t motor, int pole_pairs)
{
    if (pole_pairs < 1) {
        return;
    }
    BLDCMotor &m = (motor == MOTOR_LEFT) ? motor_left : motor_right;
    m.pole_pairs = pole_pairs;
    m.zero_electric_angle = NOT_SET;
    m.sensor_direction = Direction::UNKNOWN;
    portENTER_CRITICAL(&foc_mux);
    foc_aligned = false;
    portEXIT_CRITICAL(&foc_mux);
}

void motor_foc_set_align_voltage(motor_id_t motor, float volts)
{
    BLDCMotor &m = (motor == MOTOR_LEFT) ? motor_left : motor_right;
    m.voltage_sensor_align = _constrain(volts, 0.0f, m.voltage_limit);
}

void motor_foc_set_mode(motor_mode_t mode)
{
    portENTER_CRITICAL(&foc_mux);
    bool ready = foc_ready;
    bool aligned = foc_aligned;
    portEXIT_CRITICAL(&foc_mux);
    if (!ready) {
        return;
    }
    if (mode_needs_alignment(mode) && !aligned) {
        ESP_LOGW(TAG, "not aligned yet, run 'm_align' first");
        return;
    }
    motor_left.controller = component_mode(mode);
    motor_right.controller = component_mode(mode);
    portENTER_CRITICAL(&foc_mux);
    current_mode = mode;
    portEXIT_CRITICAL(&foc_mux);
}

motor_mode_t motor_foc_get_mode(void)
{
    portENTER_CRITICAL(&foc_mux);
    motor_mode_t mode = current_mode;
    portEXIT_CRITICAL(&foc_mux);
    return mode;
}

bool motor_foc_is_aligned(void)
{
    portENTER_CRITICAL(&foc_mux);
    bool aligned = foc_aligned;
    portEXIT_CRITICAL(&foc_mux);
    return aligned;
}

void motor_foc_set_target(motor_id_t motor, float target)
{
    portENTER_CRITICAL(&foc_mux);
    if (motor == MOTOR_LEFT) {
        target_left = target;
    } else {
        target_right = target;
    }
    portEXIT_CRITICAL(&foc_mux);
}

esp_err_t motor_foc_enable_torque(void)
{
    portENTER_CRITICAL(&foc_mux);
    bool ready = foc_ready;
    bool aligned = foc_aligned;
    portEXIT_CRITICAL(&foc_mux);
    if (!ready || !aligned) {
        return ESP_ERR_INVALID_STATE;
    }
    motor_left.controller = MotionControlType::torque;
    motor_right.controller = MotionControlType::torque;
    motor_left.enable();
    motor_right.enable();
    portENTER_CRITICAL(&foc_mux);
    target_left = 0.0f;
    target_right = 0.0f;
    current_mode = MOTOR_MODE_TORQUE;
    portEXIT_CRITICAL(&foc_mux);
    return ESP_OK;
}

void motor_foc_stop(void)
{
    portENTER_CRITICAL(&foc_mux);
    target_left = 0.0f;
    target_right = 0.0f;
    current_mode = MOTOR_MODE_DISABLED;
    portEXIT_CRITICAL(&foc_mux);
    if (foc_ready) {
        motor_left.disable();
        motor_right.disable();
    }
}

void motor_foc_get_feedback(motor_id_t motor, motor_feedback_t *feedback)
{
    if (feedback == NULL) {
        return;
    }
    BLDCMotor &m = (motor == MOTOR_LEFT) ? motor_left : motor_right;
    portENTER_CRITICAL(&foc_mux);
    float left_target = target_left;
    float right_target = target_right;
    portEXIT_CRITICAL(&foc_mux);
    feedback->target = (motor == MOTOR_LEFT) ? left_target : right_target;
    feedback->angle = m.shaft_angle;
    feedback->velocity = m.shaft_velocity;
    feedback->sensor_angle = m.sensor ? m.sensor->getAngle() : 0.0f;
    feedback->voltage_q = m.voltage.q;
}

uint32_t motor_foc_loop_count(void)
{
    return __atomic_load_n(&foc_loop_count, __ATOMIC_RELAXED);
}
