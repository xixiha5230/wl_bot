#include "robot_control_internal.h"

#include "esp_err.h"
#include "servo_sts.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/*
 * Leg-servo output path.
 *
 * The 1 kHz control loop only publishes the newest pose into an overwrite
 * mailbox; this low-rate task writes it to the STS bus. A busy or locked UART
 * therefore delays the legs but can never stall balancing.
 */

#define LEG_OUTPUT_PERIOD_MS 10

static QueueHandle_t leg_queue;
static int16_t last_leg_target1;
static int16_t last_leg_target2;

typedef struct {
    int16_t position1;
    int16_t position2;
    uint16_t speed;
    uint8_t acceleration;
} leg_pose_t;

void robot_control_leg_output(int16_t position1, int16_t position2,
                              uint16_t speed, uint8_t acceleration)
{
    if (leg_queue == NULL) {
        return;
    }
    last_leg_target1 = position1;
    last_leg_target2 = position2;
    const leg_pose_t pose = {
        .position1 = position1,
        .position2 = position2,
        .speed = speed,
        .acceleration = acceleration,
    };
    xQueueOverwrite(leg_queue, &pose);
}

void robot_control_leg_targets(int16_t *target1, int16_t *target2)
{
    if (target1) *target1 = last_leg_target1;
    if (target2) *target2 = last_leg_target2;
}

static void leg_task(void *arg)
{
    (void)arg;
    leg_pose_t pose = {};
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        if (xQueueReceive(leg_queue, &pose, portMAX_DELAY) == pdTRUE) {
            const uint8_t ids[] = {1, 2};
            const int16_t positions[] = {pose.position1, pose.position2};
            servo_sts_sync_write_position(ids, positions, 2, pose.speed, pose.acceleration);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LEG_OUTPUT_PERIOD_MS));
    }
}

esp_err_t robot_control_leg_io_start(void)
{
    leg_queue = xQueueCreate(1, sizeof(leg_pose_t));
    if (leg_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreatePinnedToCore(leg_task, "leg_task", 4096, NULL, LEG_TASK_PRIO,
                                NULL, ROBOT_TASK_CORE) != pdPASS) {
        vQueueDelete(leg_queue);
        leg_queue = NULL;
        return ESP_FAIL;
    }
    return ESP_OK;
}
