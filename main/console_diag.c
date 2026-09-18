#include "console_internal.h"

#include "esp_check.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "board.h"
#include "motor_foc.h"
#include "robot_control.h"
#include "sensors.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "console";

/* Sensor and rate diagnostics: encoders, IMU, gyro re-zero, jitter, battery. */

static int cmd_enc(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    as5600_sample_t left = {0};
    as5600_sample_t right = {0};
    esp_err_t status = sensors_read_encoders(&left, &right);
    if (status != ESP_OK) {
        printf("encoder read -> %s\n", esp_err_to_name(status));
        return 1;
    }
    printf("L raw=%u angle=%.1f cont=%.1f vel=%.1f\n",
           left.raw_angle, left.angle_degrees,
           left.continuous_angle_degrees, left.velocity_degrees_per_second);
    printf("R raw=%u angle=%.1f cont=%.1f vel=%.1f\n",
           right.raw_angle, right.angle_degrees,
           right.continuous_angle_degrees, right.velocity_degrees_per_second);
    return 0;
}

static int cmd_imu(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    mpu6050_sample_t imu = {0};
    esp_err_t status = sensors_read_imu(&imu);
    if (status != ESP_OK) {
        printf("imu read -> %s\n", esp_err_to_name(status));
        return 1;
    }
    printf("acc  x=%+.3fg y=%+.3fg z=%+.3fg\n", imu.accel_x_g, imu.accel_y_g, imu.accel_z_g);
    printf("gyro x=%+.1f y=%+.1f z=%+.1f dps\n", imu.gyro_x_dps, imu.gyro_y_dps, imu.gyro_z_dps);
    printf("angle x=%+.2f y=%+.2f deg\n", imu.angle_x, imu.angle_y);
    return 0;
}

static int cmd_gcal(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("recalibrating gyro - keep the robot perfectly still...\n");
    esp_err_t status = sensors_calibrate_gyro();
    printf("gyro recalibration %s\n", status == ESP_OK ? "ok" : esp_err_to_name(status));
    return status == ESP_OK ? 0 : 1;
}

static int cmd_rate(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    uint32_t control0 = robot_control_loop_count();
    uint32_t foc0 = motor_foc_loop_count();
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("control=%lu Hz  foc=%lu Hz\n",
           (unsigned long)(robot_control_loop_count() - control0),
           (unsigned long)(motor_foc_loop_count() - foc0));
    return 0;
}

static int cmd_jit(int argc, char **argv)
{
    int seconds = argc >= 2 ? atoi(argv[1]) : 3;
    if (seconds < 1) {
        seconds = 1;
    }
    float a_min = 1e9f, a_max = -1e9f;
    float r_min = 1e9f, r_max = -1e9f;
    float l_min = 1e9f, l_max = -1e9f;
    int64_t end = esp_timer_get_time() + (int64_t)seconds * 1000000;
    while (esp_timer_get_time() < end) {
        float a = robot_control_lqr_angle();
        float r = robot_control_roll_angle();
        float l = robot_control_leg_add();
        if (a < a_min) a_min = a;
        if (a > a_max) a_max = a;
        if (r < r_min) r_min = r;
        if (r > r_max) r_max = r;
        if (l < l_min) l_min = l;
        if (l > l_max) l_max = l;
    }
    printf("jit %ds lqr_angle %.2f..%.2f pp=%.2f | roll %.2f..%.2f pp=%.2f | leg_add %.1f..%.1f pp=%.1f\n",
           seconds, a_min, a_max, a_max - a_min,
           r_min, r_max, r_max - r_min,
           l_min, l_max, l_max - l_min);
    return 0;
}

static int cmd_bat(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("battery=%.2f V raw=%d (threshold %.1f)\n",
           board_battery_voltage(), board_battery_raw(),
           (double)BOARD_BATTERY_LED_THRESHOLD);
    return 0;
}


esp_err_t console_diag_register(void)
{
    const esp_console_cmd_t commands[] = {
        {.command = "enc", .help = "read both AS5600 encoders", .func = cmd_enc},
        {.command = "imu", .help = "read accel/gyro/angle", .func = cmd_imu},
        {.command = "gcal", .help = "re-zero the gyro (keep robot still)", .func = cmd_gcal},
        {.command = "rate", .help = "measure control/FOC loop rates", .func = cmd_rate},
        {.command = "jit", .help = "jit [sec]: measure high-rate jitter", .func = cmd_jit},
        {.command = "bat", .help = "read battery voltage", .func = cmd_bat},
    };
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
        ESP_RETURN_ON_ERROR(esp_console_cmd_register(&commands[i]), TAG,
                            "register %s failed", commands[i].command);
    }
    return ESP_OK;
}
