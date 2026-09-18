#include "console_internal.h"

#include "esp_check.h"
#include "esp_console.h"
#include "esp_log.h"
#include "motor_foc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "console";

/* FOC motor commands: init/align, mode, target, feedback and pole pairs. */

static const char *mode_name(motor_mode_t mode)
{
    switch (mode) {
    case MOTOR_MODE_TORQUE:           return "torque";
    case MOTOR_MODE_VELOCITY:         return "velocity";
    case MOTOR_MODE_ANGLE:            return "angle";
    case MOTOR_MODE_VELOCITY_OPENLOOP:return "velocity_openloop";
    case MOTOR_MODE_ANGLE_OPENLOOP:   return "angle_openloop";
    default:                          return "disabled";
    }
}

static int cmd_m_init(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    esp_err_t status = motor_foc_init();
    printf("motor init -> %s\n", esp_err_to_name(status));
    return status == ESP_OK ? 0 : 1;
}

static int cmd_m_align(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    esp_err_t status = motor_foc_align();
    printf("motor align -> %s\n", esp_err_to_name(status));
    return status == ESP_OK ? 0 : 1;
}

static int cmd_m_mode(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: m_mode <disabled|torque|velocity|angle|vopen|aopen>\n");
        return 1;
    }
    motor_mode_t mode = MOTOR_MODE_DISABLED;
    if (strcmp(argv[1], "torque") == 0) {
        mode = MOTOR_MODE_TORQUE;
    } else if (strcmp(argv[1], "velocity") == 0) {
        mode = MOTOR_MODE_VELOCITY;
    } else if (strcmp(argv[1], "angle") == 0) {
        mode = MOTOR_MODE_ANGLE;
    } else if (strcmp(argv[1], "vopen") == 0) {
        mode = MOTOR_MODE_VELOCITY_OPENLOOP;
    } else if (strcmp(argv[1], "aopen") == 0) {
        mode = MOTOR_MODE_ANGLE_OPENLOOP;
    }
    motor_foc_set_mode(mode);
    printf("motor mode -> %s\n", mode_name(motor_foc_get_mode()));
    return 0;
}

static int cmd_m_target(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: m_target <left> <right>\n");
        return 1;
    }
    motor_foc_set_target(MOTOR_LEFT, strtof(argv[1], NULL));
    motor_foc_set_target(MOTOR_RIGHT, strtof(argv[2], NULL));
    printf("motor target L=%s R=%s\n", argv[1], argv[2]);
    return 0;
}

static int cmd_m_state(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    const motor_id_t ids[] = {MOTOR_LEFT, MOTOR_RIGHT};
    const char *names[] = {"left", "right"};
    for (int i = 0; i < 2; ++i) {
        motor_feedback_t fb = {0};
        motor_foc_get_feedback(ids[i], &fb);
        printf("%-5s target=%7.3f angle=%8.3f rad vel=%8.3f rad/s sensor=%8.3f rad Uq=%6.3f V\n",
               names[i], fb.target, fb.angle, fb.velocity, fb.sensor_angle, fb.voltage_q);
    }
    printf("mode=%s\n", mode_name(motor_foc_get_mode()));
    return 0;
}

static int cmd_m_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    motor_foc_stop();
    printf("motor stopped (outputs disabled)\n");
    return 0;
}

static int cmd_m_pp(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: m_pp <1|2> <pole_pairs>\n");
        return 1;
    }
    motor_id_t id = (atoi(argv[1]) == 2) ? MOTOR_RIGHT : MOTOR_LEFT;
    int pp = atoi(argv[2]);
    motor_foc_set_pole_pairs(id, pp);
    printf("motor %d pole_pairs -> %d (realign required)\n", (int)id, pp);
    return 0;
}

static int cmd_m_align1(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: m_align1 <1|2> [align_volts]\n");
        return 1;
    }
    motor_id_t id = (atoi(argv[1]) == 2) ? MOTOR_RIGHT : MOTOR_LEFT;
    if (argc >= 3) {
        motor_foc_set_align_voltage(id, strtof(argv[2], NULL));
    }
    esp_err_t status = motor_foc_align_motor(id);
    printf("align motor %d -> %s\n", (int)id, esp_err_to_name(status));
    return status == ESP_OK ? 0 : 1;
}

esp_err_t console_motor_register(void)
{
    const esp_console_cmd_t commands[] = {
        {.command = "m_init", .help = "initialize FOC motor drivers", .func = cmd_m_init},
        {.command = "m_align", .help = "align FOC sensors/motors (wheels move)", .func = cmd_m_align},
        {.command = "m_mode", .help = "m_mode <disabled|torque|velocity|angle|vopen|aopen>", .func = cmd_m_mode},
        {.command = "m_target", .help = "m_target <left> <right>", .func = cmd_m_target},
        {.command = "m_state", .help = "print FOC motor feedback", .func = cmd_m_state},
        {.command = "m_stop", .help = "disable motor outputs", .func = cmd_m_stop},
        {.command = "m_pp", .help = "m_pp <1|2> <pole_pairs>", .func = cmd_m_pp},
        {.command = "m_align1", .help = "m_align1 <1|2> [align_volts]: align one motor", .func = cmd_m_align1},
    };
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
        ESP_RETURN_ON_ERROR(esp_console_cmd_register(&commands[i]), TAG,
                            "register %s failed", commands[i].command);
    }
    return ESP_OK;
}
