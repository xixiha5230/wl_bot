#include "board.h"
#include "board_internal.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"

/*
 * Battery sense: a divider on the ADC pin, a lightly smoothed voltage for the
 * LED and low-voltage checks, and the indicator LED itself.
 */

static const char *TAG = "battery";
static adc_oneshot_unit_handle_t s_battery_adc;
static adc_cali_handle_t s_battery_cali;
static bool s_battery_cali_enabled;
static float s_battery_filtered;
static bool s_battery_filtered_valid;

/* Original firmware scales the ADC pin voltage with a 3.97 divider ratio. */
#define BOARD_BATTERY_DIVIDER_RATIO 3.97f

esp_err_t battery_init(void)
{
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
