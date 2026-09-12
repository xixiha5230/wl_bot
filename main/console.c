#include "console.h"

#include "esp_console.h"
#include "esp_check.h"
#include "esp_log.h"
#include "board.h"
#include "motor_foc.h"
#include "robot_config.h"
#include "robot_control.h"
#include "robot_state.h"
#include "sensors.h"
#include "servo_sts.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "console";

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

static int cmd_go(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: go <0|1>\n");
        return 1;
    }
    bool go = atoi(argv[1]) != 0;
    robot_control_set_go(go);
    printf("go=%d\n", go ? 1 : 0);
    return 0;
}

static int cmd_height(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: height <32..80>\n");
        return 1;
    }
    robot_control_set_height(atoi(argv[1]));
    printf("height=%d\n", atoi(argv[1]));
    return 0;
}

static int cmd_joy(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: joy <x> <y>\n");
        return 1;
    }
    robot_control_set_joy(atoi(argv[1]), atoi(argv[2]));
    printf("joyx=%d joyy=%d\n", atoi(argv[1]), atoi(argv[2]));
    return 0;
}

static int cmd_dir(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: dir <0..5> (fwd back right left stop jump)\n");
        return 1;
    }
    robot_control_set_dir(atoi(argv[1]));
    printf("dir=%d\n", atoi(argv[1]));
    return 0;
}

static int cmd_rc(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    robot_command_t cmd = robot_control_get_command();
    printf("go=%d height=%d joyx=%d joyy=%d dir=%d\n",
           cmd.go ? 1 : 0, cmd.height, cmd.joy_x, cmd.joy_y, cmd.dir);
    printf("lqr_angle=%.2f lqr_u=%.3f fault=%d state=%s\n",
           robot_control_lqr_angle(), robot_control_lqr_u(),
           robot_control_faulted() ? 1 : 0,
           robot_state_name(robot_state_get()));
    return 0;
}

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

static int cmd_bat(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("battery=%.2f V raw=%d (threshold %.1f)\n",
           board_battery_voltage(), board_battery_raw(),
           (double)BOARD_BATTERY_LED_THRESHOLD);
    return 0;
}

static esp_err_t register_commands(void)
{
    const esp_console_cmd_t commands[] = {
        {.command = "read", .help = "read both servo feedback", .func = cmd_read},
        {.command = "move", .help = "move <id> <position> [speed]", .func = cmd_move},
        {.command = "sync", .help = "sync <pos1> <pos2> [speed]", .func = cmd_sync},
        {.command = "torque", .help = "torque <id|all> <0|1>", .func = cmd_torque},
        {.command = "cal", .help = "cal <id|all>: measure mechanical travel", .func = cmd_cal},
        {.command = "limits", .help = "print calibrated servo travel limits", .func = cmd_limits},
        {.command = "m_init", .help = "initialize FOC motor drivers", .func = cmd_m_init},
        {.command = "m_align", .help = "align FOC sensors/motors (wheels move)", .func = cmd_m_align},
        {.command = "m_mode", .help = "m_mode <disabled|torque|velocity|angle|vopen|aopen>", .func = cmd_m_mode},
        {.command = "m_target", .help = "m_target <left> <right>", .func = cmd_m_target},
        {.command = "m_state", .help = "print FOC motor feedback", .func = cmd_m_state},
        {.command = "m_stop", .help = "disable motor outputs", .func = cmd_m_stop},
        {.command = "m_pp", .help = "m_pp <1|2> <pole_pairs>", .func = cmd_m_pp},
        {.command = "m_align1", .help = "m_align1 <1|2> [align_volts]: align one motor", .func = cmd_m_align1},
        {.command = "go", .help = "go <0|1>: enable balance output", .func = cmd_go},
        {.command = "height", .help = "height <32..80>: leg height", .func = cmd_height},
        {.command = "joy", .help = "joy <x> <y>: virtual joystick", .func = cmd_joy},
        {.command = "dir", .help = "dir <0..5>: motion direction (incl. jump)", .func = cmd_dir},
        {.command = "rc", .help = "print control state", .func = cmd_rc},
        {.command = "enc", .help = "read both AS5600 encoders", .func = cmd_enc},
        {.command = "bat", .help = "read battery voltage", .func = cmd_bat},
        {.command = "stop", .help = "disable all servo torque", .func = cmd_stop},
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
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();

    ESP_RETURN_ON_ERROR(esp_console_new_repl_uart(&uart_config, &repl_config, &repl),
                        TAG, "console repl init failed");
    ESP_RETURN_ON_ERROR(register_commands(), TAG, "console commands failed");
    ESP_RETURN_ON_ERROR(esp_console_start_repl(repl), TAG, "console repl start failed");
    return ESP_OK;
}
