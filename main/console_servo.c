#include "console_internal.h"

#include "esp_check.h"
#include "esp_console.h"
#include "esp_log.h"
#include "robot_config.h"
#include "robot_control.h"
#include "servo_sts.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "console";

/* Servo-bus commands: raw STS reads/writes, travel calibration and torque. */

static void print_feedback(uint8_t id, const servo_sts_feedback_t *fb)
{
    printf("id=%u position=%d speed=%d load=%d current=%d voltage=%u temp=%u status=0x%02x\n",
           id, fb->position, fb->speed, fb->load, fb->current,
           fb->voltage, fb->temperature, fb->status);
}

static int cmd_read(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    for (uint8_t id = 1; id <= 2; ++id) {
        servo_sts_feedback_t fb = {0};
        esp_err_t status = servo_sts_read_feedback(id, &fb);
        if (status == ESP_OK) {
            print_feedback(id, &fb);
        } else {
            printf("id=%u read failed: %s\n", id, esp_err_to_name(status));
        }
    }
    return 0;
}

static int cmd_move(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: move <id> <position> [speed]\n");
        return 1;
    }
    uint8_t id = (uint8_t)atoi(argv[1]);
    int16_t position = (int16_t)atoi(argv[2]);
    uint16_t speed = argc >= 4 ? (uint16_t)atoi(argv[3]) : 600;
    if (id < 1 || id > 2) {
        printf("invalid id\n");
        return 1;
    }
    esp_err_t status = servo_sts_sync_write_position(&id, &position, 1, speed, 10);
    printf("move id=%u position=%d speed=%u -> %s\n", id, position, speed,
           esp_err_to_name(status));
    return status == ESP_OK ? 0 : 1;
}

static int cmd_sync(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: sync <pos1> <pos2> [speed]\n");
        return 1;
    }
    const uint8_t ids[] = {1, 2};
    const int16_t positions[] = {(int16_t)atoi(argv[1]), (int16_t)atoi(argv[2])};
    uint16_t speed = argc >= 4 ? (uint16_t)atoi(argv[3]) : 600;
    esp_err_t status = servo_sts_sync_write_position(ids, positions, 2, speed, 10);
    printf("sync id1=%d id2=%d speed=%u -> %s\n",
           positions[0], positions[1], speed, esp_err_to_name(status));
    return status == ESP_OK ? 0 : 1;
}

static int cmd_torque(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: torque <id|all> <0|1>\n");
        return 1;
    }
    bool enable = atoi(argv[2]) != 0;
    if (strcmp(argv[1], "all") == 0) {
        for (uint8_t id = 1; id <= 2; ++id) {
            servo_sts_set_torque(id, enable);
        }
    } else {
        servo_sts_set_torque((uint8_t)atoi(argv[1]), enable);
    }
    printf("torque %s\n", enable ? "enabled" : "disabled");
    return 0;
}

static int cmd_cal(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: cal <id|all>\n");
        return 1;
    }
    if (robot_control_get_command().go) {
        printf("refuse: disable balance output with 'go 0' before calibration\n");
        return 1;
    }

    if (strcmp(argv[1], "all") == 0) {
        for (uint8_t id = 1; id <= 2; ++id) {
            servo_sts_travel_t travel = {0};
            if (servo_sts_calibrate_travel(id, &travel) == ESP_OK) {
                printf("cal id=%u low=%d high=%d span=%d\n",
                       id, travel.minimum, travel.maximum,
                       travel.maximum - travel.minimum);
            } else {
                printf("cal id=%u failed\n", id);
            }
        }
    } else {
        uint8_t id = (uint8_t)atoi(argv[1]);
        if (id < 1 || id > 2) {
            printf("invalid id\n");
            return 1;
        }
        servo_sts_travel_t travel = {0};
        if (servo_sts_calibrate_travel(id, &travel) != ESP_OK) {
            printf("cal id=%u failed\n", id);
            return 1;
        }
        printf("cal id=%u low=%d high=%d span=%d\n",
               id, travel.minimum, travel.maximum,
               travel.maximum - travel.minimum);
    }
    return 0;
}

static int cmd_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    for (uint8_t id = 1; id <= 2; ++id) {
        servo_sts_set_torque(id, false);
    }
    printf("all servo torque disabled\n");
    return 0;
}

static int cmd_limits(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("mechanical: id1 %d..%d  id2 %d..%d\n",
           SERVO_ID1_MECH_MIN, SERVO_ID1_MECH_MAX,
           SERVO_ID2_MECH_MIN, SERVO_ID2_MECH_MAX);
    printf("safe(+/-%d): id1 %d..%d  id2 %d..%d\n",
           SERVO_TRAVEL_MARGIN,
           SERVO_ID1_SAFE_MIN, SERVO_ID1_SAFE_MAX,
           SERVO_ID2_SAFE_MIN, SERVO_ID2_SAFE_MAX);
    printf("mirror axis %d  neutral id1=%d id2=%d\n",
           SERVO_MIRROR_AXIS, SERVO_NEUTRAL_ID1, SERVO_NEUTRAL_ID2);
    return 0;
}

esp_err_t console_servo_register(void)
{
    const esp_console_cmd_t commands[] = {
        {.command = "read", .help = "read both servo feedback", .func = cmd_read},
        {.command = "move", .help = "move <id> <position> [speed]", .func = cmd_move},
        {.command = "sync", .help = "sync <pos1> <pos2> [speed]", .func = cmd_sync},
        {.command = "torque", .help = "torque <id|all> <0|1>", .func = cmd_torque},
        {.command = "cal", .help = "cal <id|all>: measure mechanical travel", .func = cmd_cal},
        {.command = "limits", .help = "print calibrated servo travel limits", .func = cmd_limits},
        {.command = "stop", .help = "disable all servo torque", .func = cmd_stop},
    };
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
        ESP_RETURN_ON_ERROR(esp_console_cmd_register(&commands[i]), TAG,
                            "register %s failed", commands[i].command);
    }
    return ESP_OK;
}
