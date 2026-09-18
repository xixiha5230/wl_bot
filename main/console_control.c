#include "console_internal.h"

#include "esp_check.h"
#include "esp_console.h"
#include "esp_log.h"
#include "robot_config.h"
#include "robot_control.h"
#include "robot_state.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "console";

/* Motion and tuning commands: balance, height, joystick, gait and PID/LPF. */

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
        printf("usage: height <32..72>\n");
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

static int cmd_yaw(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: yaw <1|-1|0> (normal / inverted / off)\n");
        return 1;
    }
    int mode = atoi(argv[1]);
    robot_control_set_yaw_mode(mode);
    printf("yaw mode %d\n", mode > 0 ? 1 : (mode < 0 ? -1 : 0));
    return 0;
}

static int cmd_jump(int argc, char **argv)
{
    int h, l, s, a, t;
    robot_control_get_jump_profile(&h, &l, &s, &a, &t);
    if (argc >= 2 && argc < 6) {
        printf("usage: jump [h land speed acc ticks] (no args = show)\n");
        return 1;
    }
    if (argc >= 6) {
        robot_control_set_jump_profile(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]),
                                       atoi(argv[4]), atoi(argv[5]));
        robot_control_get_jump_profile(&h, &l, &s, &a, &t);
    }
    printf("jump h=%d land=%d speed=%d acc=%d ticks=%d\n", h, l, s, a, t);
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
    printf("yaw_out=%.2f yaw_total=%.2f yaw_mode=%d\n",
           robot_control_yaw_output(), robot_control_yaw_total(),
           robot_control_get_yaw_mode());
    float angle_c, gyro_c, dist_c, speed_c;
    robot_control_get_terms(&angle_c, &gyro_c, &dist_c, &speed_c);
    printf("terms angle=%.2f gyro=%.2f dist=%.2f speed=%.2f zero=%.2f\n",
           angle_c, gyro_c, dist_c, speed_c, robot_control_get_balance_zero());
    printf("leg_add=%.1f roll=%.2f\n",
           robot_control_leg_add(), robot_control_roll_angle());
    return 0;
}

static int cmd_pid(int argc, char **argv)
{
    if (argc == 1) {
        for (int idx = 0; idx < robot_control_pid_count(); ++idx) {
            float p, i, d, limit;
            robot_control_get_pid(idx, &p, &i, &d, &limit);
            printf("%-11s P=%.5g I=%.5g D=%.5g limit=%.5g\n",
                   robot_control_pid_name(idx), p, i, d, limit);
        }
        return 0;
    }
    if (argc < 4) {
        printf("usage: pid <name> <P> <I> [D] [limit]\n");
        return 1;
    }
    int which = -1;
    for (int idx = 0; idx < robot_control_pid_count(); ++idx) {
        if (strcmp(argv[1], robot_control_pid_name(idx)) == 0) {
            which = idx;
            break;
        }
    }
    if (which < 0) {
        printf("unknown pid '%s'\n", argv[1]);
        return 1;
    }
    float p = strtof(argv[2], NULL);
    float ki = strtof(argv[3], NULL);
    float kd = argc >= 5 ? strtof(argv[4], NULL) : -1.0f;
    float limit = argc >= 6 ? strtof(argv[5], NULL) : -1.0f;
    robot_control_set_pid(which, p, ki, kd, limit);
    printf("pid %s <- P=%g I=%g\n", argv[1], p, ki);
    return 0;
}

static int cmd_lpf(int argc, char **argv)
{
    if (argc < 3) {
        for (int idx = 0; idx < robot_control_lpf_count(); ++idx) {
            float tf;
            robot_control_get_lpf(idx, &tf);
            printf("%-11s Tf=%.5g\n", robot_control_lpf_name(idx), tf);
        }
        return 0;
    }
    int which = -1;
    for (int idx = 0; idx < robot_control_lpf_count(); ++idx) {
        if (strcmp(argv[1], robot_control_lpf_name(idx)) == 0) {
            which = idx;
            break;
        }
    }
    if (which < 0) {
        printf("unknown lpf '%s'\n", argv[1]);
        return 1;
    }
    float tf = strtof(argv[2], NULL);
    robot_control_set_lpf(which, tf);
    printf("lpf %s <- Tf=%g\n", argv[1], tf);
    return 0;
}

static int cmd_zero(int argc, char **argv)
{
    if (argc < 2) {
        printf("angle_zeropoint=%.2f\n", robot_control_get_angle_zeropoint());
        return 0;
    }
    robot_control_set_angle_zeropoint(strtof(argv[1], NULL));
    printf("angle_zeropoint=%.2f\n", robot_control_get_angle_zeropoint());
    return 0;
}


esp_err_t console_control_register(void)
{
    const esp_console_cmd_t commands[] = {
        {.command = "go", .help = "go <0|1>: enable balance output", .func = cmd_go},
        {.command = "yaw", .help = "yaw <0|1>: enable yaw correction", .func = cmd_yaw},
        {.command = "height", .help = "height <32..72>: leg height", .func = cmd_height},
        {.command = "joy", .help = "joy <x> <y>: virtual joystick", .func = cmd_joy},
        {.command = "dir", .help = "dir <0..5>: motion direction (incl. jump)", .func = cmd_dir},
        {.command = "jump", .help = "jump [h land speed acc ticks]: show/set jump profile", .func = cmd_jump},
        {.command = "rc", .help = "print control state", .func = cmd_rc},
        {.command = "pid", .help = "pid [name P I [D] [limit]]: show/set gains", .func = cmd_pid},
        {.command = "lpf", .help = "lpf [name Tf]: show/set low-pass filters", .func = cmd_lpf},
        {.command = "zero", .help = "zero [deg]: show/set balance zero point", .func = cmd_zero},
    };
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
        ESP_RETURN_ON_ERROR(esp_console_cmd_register(&commands[i]), TAG,
                            "register %s failed", commands[i].command);
    }
    return ESP_OK;
}
