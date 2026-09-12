#include "robot_state.h"

#include "freertos/FreeRTOS.h"

static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
static robot_state_t state = ROBOT_STATE_SAFE;

void robot_state_init(void)
{
    robot_state_set(ROBOT_STATE_SAFE);
}

void robot_state_set(robot_state_t value)
{
    portENTER_CRITICAL(&state_mux);
    state = value;
    portEXIT_CRITICAL(&state_mux);
}

robot_state_t robot_state_get(void)
{
    portENTER_CRITICAL(&state_mux);
    robot_state_t value = state;
    portEXIT_CRITICAL(&state_mux);
    return value;
}

const char *robot_state_name(robot_state_t value)
{
    switch (value) {
    case ROBOT_STATE_SAFE: return "safe";
    case ROBOT_STATE_CALIBRATING: return "calibrating";
    case ROBOT_STATE_READY: return "ready";
    case ROBOT_STATE_RUNNING: return "running";
    case ROBOT_STATE_FAULT: return "fault";
    default: return "unknown";
    }
}
