#include "console.h"
#include "console_internal.h"

#include "esp_console.h"
#include "esp_check.h"
#include "esp_log.h"
#include "ota.h"

#include <stdio.h>

static const char *TAG = "console";

/* Firmware over Wi-Fi from the serial console: `ota <url>`. */
static int cmd_ota(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: ota <http://host:port/image.bin>\n");
        return 1;
    }
    esp_err_t status = ota_start_from_url(argv[1]);
    if (status != ESP_OK) {
        printf("ota start failed: %s\n", esp_err_to_name(status));
        return 1;
    }
    printf("ota started; the robot will reboot when the transfer completes\n");
    return 0;
}

static esp_err_t register_commands(void)
{
    ESP_RETURN_ON_ERROR(console_servo_register(), TAG, "servo commands failed");
    ESP_RETURN_ON_ERROR(console_motor_register(), TAG, "motor commands failed");
    ESP_RETURN_ON_ERROR(console_control_register(), TAG, "control commands failed");
    ESP_RETURN_ON_ERROR(console_diag_register(), TAG, "diag commands failed");

    const esp_console_cmd_t commands[] = {
        {.command = "ota", .help = "ota <url>: flash firmware over Wi-Fi", .func = cmd_ota},
    };
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
        ESP_RETURN_ON_ERROR(esp_console_cmd_register(&commands[i]), TAG,
                            "register %s failed", commands[i].command);
    }
    ESP_RETURN_ON_ERROR(esp_console_register_help_command(), TAG, "help register failed");
    return ESP_OK;
}

esp_err_t console_start(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "wlrobot>";
    repl_config.max_cmdline_length = 128;
    repl_config.task_core_id = 0;   /* keep the control loop core free */
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();

    ESP_RETURN_ON_ERROR(esp_console_new_repl_uart(&uart_config, &repl_config, &repl),
                        TAG, "console repl init failed");
    ESP_RETURN_ON_ERROR(register_commands(), TAG, "console commands failed");
    ESP_RETURN_ON_ERROR(esp_console_start_repl(repl), TAG, "console repl start failed");
    return ESP_OK;
}
