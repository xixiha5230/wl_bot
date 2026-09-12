#include "board.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "board";
i2c_master_bus_handle_t board_encoder_i2c_bus;
i2c_master_bus_handle_t board_imu_i2c_bus;

static adc_oneshot_unit_handle_t s_battery_adc;
static adc_cali_handle_t s_battery_cali;
static bool s_battery_cali_enabled;
static float s_battery_filtered;
static bool s_battery_filtered_valid;

/* Original firmware scales the ADC pin voltage with a 3.97 divider ratio. */
#define BOARD_BATTERY_DIVIDER_RATIO 3.97f

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

    gpio_config_t led = {
        .pin_bit_mask = 1ULL << BOARD_BATTERY_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&led), TAG, "LED GPIO init failed");
    gpio_set_level(BOARD_BATTERY_LED_GPIO, 0);

    adc_oneshot_unit_handle_t adc_unit;
    const adc_oneshot_unit_init_cfg_t adc_init = {
        .unit_id = BOARD_BATTERY_ADC_UNIT,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    const adc_oneshot_chan_cfg_t adc_channel = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&adc_init, &adc_unit), TAG, "ADC init failed");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc_unit, BOARD_BATTERY_ADC_CHANNEL,
                                                   &adc_channel),
                        TAG, "ADC channel failed");
    s_battery_adc = adc_unit;

    const adc_cali_line_fitting_config_t cali_config = {
        .unit_id = BOARD_BATTERY_ADC_UNIT,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    s_battery_cali_enabled =
        (adc_cali_create_scheme_line_fitting(&cali_config, &s_battery_cali) == ESP_OK);
    ESP_LOGI(TAG, "battery ADC ready (calibration %s)",
             s_battery_cali_enabled ? "on" : "off");

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

int board_battery_raw(void)
{
    if (s_battery_adc == NULL) {
        return -1;
    }
    int raw = 0;
    if (adc_oneshot_read(s_battery_adc, BOARD_BATTERY_ADC_CHANNEL, &raw) != ESP_OK) {
        return -1;
    }
    return raw;
}

float board_battery_voltage(void)
{
    if (s_battery_adc == NULL) {
        return 0.0f;
    }
    int raw = 0;
    if (adc_oneshot_read(s_battery_adc, BOARD_BATTERY_ADC_CHANNEL, &raw) != ESP_OK) {
        return s_battery_filtered_valid ? s_battery_filtered : 0.0f;
    }
    int millivolts = 0;
    if (s_battery_cali_enabled) {
        if (adc_cali_raw_to_voltage(s_battery_cali, raw, &millivolts) != ESP_OK) {
            return s_battery_filtered_valid ? s_battery_filtered : 0.0f;
        }
    } else {
        millivolts = raw * 3100 / 4095;
    }
    float voltage = (float)millivolts * BOARD_BATTERY_DIVIDER_RATIO / 1000.0f;

    /* Light exponential smoothing so the LED and low-voltage checks do not
     * react to single noisy samples. */
    if (!s_battery_filtered_valid) {
        s_battery_filtered = voltage;
        s_battery_filtered_valid = true;
    } else {
        s_battery_filtered += 0.2f * (voltage - s_battery_filtered);
    }
    return s_battery_filtered;
}

void board_battery_led_set(bool on)
{
    gpio_set_level(BOARD_BATTERY_LED_GPIO, on ? 1 : 0);
}
