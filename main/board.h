#pragma once

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "hal/adc_types.h"

#include <stdbool.h>

/* Pin map copied from the original ESP32 controller firmware. */
#define BOARD_I2C_ENCODER_PORT       I2C_NUM_0
#define BOARD_I2C_ENCODER_SDA        GPIO_NUM_19
#define BOARD_I2C_ENCODER_SCL        GPIO_NUM_18
#define BOARD_I2C_IMU_PORT           I2C_NUM_1
#define BOARD_I2C_IMU_SDA            GPIO_NUM_23
#define BOARD_I2C_IMU_SCL            GPIO_NUM_5

#define BOARD_MOTOR_LEFT_U           GPIO_NUM_32
#define BOARD_MOTOR_LEFT_V           GPIO_NUM_33
#define BOARD_MOTOR_LEFT_W           GPIO_NUM_25
#define BOARD_MOTOR_LEFT_ENABLE      GPIO_NUM_22
#define BOARD_MOTOR_RIGHT_U          GPIO_NUM_26
#define BOARD_MOTOR_RIGHT_V          GPIO_NUM_27
#define BOARD_MOTOR_RIGHT_W          GPIO_NUM_14
#define BOARD_MOTOR_RIGHT_ENABLE     GPIO_NUM_12

/* Battery divider on GPIO35, which is ADC1 channel 7 on the classic ESP32. */
#define BOARD_BATTERY_ADC_UNIT       ADC_UNIT_1
#define BOARD_BATTERY_ADC_CHANNEL    ADC_CHANNEL_7
#define BOARD_BATTERY_LED_GPIO       GPIO_NUM_13
#define BOARD_BATTERY_LED_THRESHOLD  7.8f
#define BOARD_BATTERY_LED_HYSTERESIS 0.1f
/* Below this the robot refuses to run and a running robot is stopped. */
#define BOARD_BATTERY_LOW_VOLTAGE    6.8f
#define BOARD_BATTERY_RECOVER_VOLTAGE (BOARD_BATTERY_LOW_VOLTAGE + 0.2f)
#define BOARD_SERVO_UART_NUM         UART_NUM_2
#define BOARD_SERVO_UART_TX          GPIO_NUM_17
#define BOARD_SERVO_UART_RX          GPIO_NUM_16

extern i2c_master_bus_handle_t board_encoder_i2c_bus;
extern i2c_master_bus_handle_t board_imu_i2c_bus;

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t board_init(void);

/* Battery voltage in volts (ADC pin voltage scaled by the divider ratio). */
float board_battery_voltage(void);

/* Raw ADC sample, for diagnostics. Returns -1 if the ADC is unavailable. */
int board_battery_raw(void);

/* Battery indicator LED. */
void board_battery_led_set(bool on);

#ifdef __cplusplus
}
#endif
