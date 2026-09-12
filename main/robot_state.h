#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ROBOT_STATE_SAFE = 0,
    ROBOT_STATE_CALIBRATING,
    ROBOT_STATE_READY,
    ROBOT_STATE_RUNNING,
    ROBOT_STATE_FAULT,
} robot_state_t;

void robot_state_init(void);

/* Thread-safe: may be called from any task; the HTTP status handler reads it. */
void robot_state_set(robot_state_t state);
robot_state_t robot_state_get(void);
const char *robot_state_name(robot_state_t state);

#ifdef __cplusplus
}
#endif
