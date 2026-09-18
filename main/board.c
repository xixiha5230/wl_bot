#include "board.h"
#include "board_internal.h"

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"

/*
 * Board bring-up: the two I2C buses (encoder + IMU) and the servo UART. The
 * battery ADC and indicator LED live in battery.c.
 */

static const char *TAG = "board";
i2c_master_bus_handle_t board_encoder_i2c_bus;
i2c_master_bus_handle_t board_imu_i2c_bus;

static esp_err_t init_i2c(i2c_port_num_t port, gpio_num_t sda, gpio_num_t scl,
                          i2c_master_bus_handle_t *bus_handle)
{
    const i2c_master_bus_config_t config = {
        .i2c_port = port,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&config, bus_handle);
}

esp_err_t board_init(void)
{
    ESP_RETURN_ON_ERROR(init_i2c(BOARD_I2C_ENCODER_PORT, BOARD_I2C_ENCODER_SDA,
                                 BOARD_I2C_ENCODER_SCL, &board_encoder_i2c_bus),
                        TAG, "encoder I2C init failed");
    ESP_RETURN_ON_ERROR(init_i2c(BOARD_I2C_IMU_PORT, BOARD_I2C_IMU_SDA,
                                 BOARD_I2C_IMU_SCL, &board_imu_i2c_bus),
                        TAG, "IMU I2C init failed");

    ESP_RETURN_ON_ERROR(battery_init(), TAG, "battery init failed");

    const uart_config_t servo_uart = {
        .baud_rate = 1000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(BOARD_SERVO_UART_NUM, 512, 512, 0, NULL, 0),
                        TAG, "servo UART install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(BOARD_SERVO_UART_NUM, &servo_uart),
                        TAG, "servo UART config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(BOARD_SERVO_UART_NUM, BOARD_SERVO_UART_TX,
                                    BOARD_SERVO_UART_RX, UART_PIN_NO_CHANGE,
                                    UART_PIN_NO_CHANGE), TAG, "servo UART pins failed");

    ESP_LOGI(TAG, "board peripherals initialized; motor outputs remain disabled");
    return ESP_OK;
}
