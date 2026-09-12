#include "servo_sts.h"

#include "board.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <stdlib.h>
#include <string.h>

#define STS_INST_READ                 0x02
#define STS_INST_WRITE                0x03
#define STS_INST_SYNC_WRITE           0x83
#define STS_TORQUE_ENABLE             40
#define STS_ACC                       41
#define STS_PRESENT_POSITION_L        56
#define STS_PRESENT_SPEED_L           58
#define STS_PRESENT_TEMPERATURE       63
#define STS_PRESENT_CURRENT_L         69
#define STS_FEEDBACK_LENGTH           19
#define STS_READ_TIMEOUT_MS           20

#define STS_CAL_STEP                  20
#define STS_CAL_DWELL_MS              400
#define STS_CAL_BACKOFF               60
#define STS_CAL_STALL_MARGIN          30
#define STS_CAL_STALL_MOVED           6
#define STS_CAL_STALL_COUNT           2
#define STS_CAL_MAX_TRAVEL            660
#define STS_CAL_SPEED                 600
#define STS_CAL_ACC                   10
#define STS_CAL_TEMP_LIMIT            65

static const char *TAG = "servo";
static SemaphoreHandle_t servo_mutex;

static esp_err_t lock_bus(void)
{
    if (servo_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return xSemaphoreTake(servo_mutex, portMAX_DELAY) == pdTRUE ? ESP_OK : ESP_FAIL;
}

static void unlock_bus(void)
{
    xSemaphoreGive(servo_mutex);
}

static uint8_t checksum(const uint8_t *data, size_t length)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < length; ++i) {
        sum += data[i];
    }
    return (uint8_t)~sum;
}

static int16_t signed_word(uint8_t low, uint8_t high)
{
    uint16_t value = (uint16_t)low | ((uint16_t)high << 8);
    if (value & 0x8000) {
        return -(int16_t)(value & 0x7FFF);
    }
    return (int16_t)value;
}

static esp_err_t read_exact(uint8_t *buffer, size_t length)
{
    size_t received = 0;
    while (received < length) {
        int count = uart_read_bytes(BOARD_SERVO_UART_NUM, buffer + received,
                                    length - received,
                                    pdMS_TO_TICKS(STS_READ_TIMEOUT_MS));
        if (count <= 0) {
            return ESP_ERR_TIMEOUT;
        }
        received += (size_t)count;
    }
    return ESP_OK;
}

static esp_err_t read_header(void)
{
    uint8_t byte;
    uint8_t previous = 0;
    for (int i = 0; i < 32; ++i) {
        ESP_RETURN_ON_ERROR(read_exact(&byte, 1), TAG, "servo response timeout");
        if (previous == 0xFF && byte == 0xFF) {
            return ESP_OK;
        }
        previous = byte;
    }
    return ESP_ERR_INVALID_RESPONSE;
}

esp_err_t servo_sts_init(void)
{
    servo_mutex = xSemaphoreCreateMutex();
    if (servo_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "STS UART ready at 1 Mbps; read-only diagnostics enabled");
    return ESP_OK;
}

static esp_err_t read_feedback_locked(uint8_t id, servo_sts_feedback_t *feedback)
{
    if (feedback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t request[] = {
        0xFF, 0xFF, id, 0x04, STS_INST_READ,
        STS_PRESENT_POSITION_L, 15,
        0,
    };
    request[7] = checksum(&request[2], 5);
    uart_flush_input(BOARD_SERVO_UART_NUM);
    int written = uart_write_bytes(BOARD_SERVO_UART_NUM, request, sizeof(request));
    if (written != (int)sizeof(request)) {
        ESP_LOGE(TAG, "servo request wrote %d/%u bytes", written, (unsigned)sizeof(request));
        return ESP_FAIL;
    }
    ESP_RETURN_ON_ERROR(uart_wait_tx_done(BOARD_SERVO_UART_NUM, pdMS_TO_TICKS(20)),
                        TAG, "servo transmit timeout");

    ESP_RETURN_ON_ERROR(read_header(), TAG, "servo response header timeout");
    uint8_t response[STS_FEEDBACK_LENGTH];
    ESP_RETURN_ON_ERROR(read_exact(response, sizeof(response)), TAG, "servo response timeout");

    if (response[0] != id || response[1] != 17) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    uint8_t expected = checksum(response, 18);
    if (expected != response[18]) {
        return ESP_ERR_INVALID_CRC;
    }
    if (response[2] != 0) {
        feedback->status = response[2];
        ESP_LOGW(TAG, "servo id=%u returned status 0x%02x", id, response[2]);
        return ESP_ERR_INVALID_STATE;
    }

    feedback->status = 0;
    feedback->position = signed_word(response[3], response[4]);
    feedback->speed = signed_word(response[5], response[6]);
    feedback->load = signed_word(response[7], response[8]);
    feedback->voltage = response[9];
    feedback->temperature = response[10];
    feedback->current = signed_word(response[16], response[17]);
    return ESP_OK;
}

esp_err_t servo_sts_read_feedback(uint8_t id, servo_sts_feedback_t *feedback)
{
    if (feedback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(lock_bus(), TAG, "servo bus lock failed");
    esp_err_t err = read_feedback_locked(id, feedback);
    unlock_bus();
    return err;
}

static esp_err_t sync_write_position_locked(const uint8_t *ids, const int16_t *positions,
                                            size_t count, uint16_t speed, uint8_t acceleration)
{
    if (ids == NULL || positions == NULL || count == 0 || count > 2) {
        return ESP_ERR_INVALID_ARG;
    }

    // STS SyncWritePosEx: ACC, position, time(0), speed for every servo.
    uint8_t packet[7 + 8 * 2 + 1];
    size_t length = 7 + count * 8 + 1;
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFE;
    packet[3] = (uint8_t)(count * 8 + 4);
    packet[4] = STS_INST_SYNC_WRITE;
    packet[5] = 41;
    packet[6] = 7;

    size_t offset = 7;
    for (size_t i = 0; i < count; ++i) {
        uint16_t position = positions[i] < 0
            ? (uint16_t)(-positions[i]) | 0x8000
            : (uint16_t)positions[i];
        packet[offset++] = ids[i];
        packet[offset++] = acceleration;
        packet[offset++] = position & 0xFF;
        packet[offset++] = position >> 8;
        packet[offset++] = 0;
        packet[offset++] = 0;
        packet[offset++] = speed & 0xFF;
        packet[offset++] = speed >> 8;
    }
    packet[offset] = checksum(&packet[2], offset - 2);

    int written = uart_write_bytes(BOARD_SERVO_UART_NUM, packet, length);
    if (written != (int)length) {
        ESP_LOGE(TAG, "servo sync write wrote %d/%u bytes", written, (unsigned)length);
        return ESP_FAIL;
    }
    ESP_RETURN_ON_ERROR(uart_wait_tx_done(BOARD_SERVO_UART_NUM, pdMS_TO_TICKS(20)),
                        TAG, "servo sync write timeout");
    return ESP_OK;
}

esp_err_t servo_sts_sync_write_position(const uint8_t *ids, const int16_t *positions,
                                        size_t count, uint16_t speed, uint8_t acceleration)
{
    if (ids == NULL || positions == NULL || count == 0 || count > 2) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(lock_bus(), TAG, "servo bus lock failed");
    esp_err_t err = sync_write_position_locked(ids, positions, count, speed, acceleration);
    unlock_bus();
    return err;
}

static esp_err_t set_torque_locked(uint8_t id, bool enable)
{
    uint8_t packet[] = {
        0xFF, 0xFF, id, 0x04, STS_INST_WRITE,
        STS_TORQUE_ENABLE, enable ? 1 : 0,
        0,
    };
    packet[7] = checksum(&packet[2], 5);
    int written = uart_write_bytes(BOARD_SERVO_UART_NUM, packet, sizeof(packet));
    if (written != (int)sizeof(packet)) {
        return ESP_FAIL;
    }
    return uart_wait_tx_done(BOARD_SERVO_UART_NUM, pdMS_TO_TICKS(20));
}

esp_err_t servo_sts_set_torque(uint8_t id, bool enable)
{
    ESP_RETURN_ON_ERROR(lock_bus(), TAG, "servo bus lock failed");
    esp_err_t err = set_torque_locked(id, enable);
    unlock_bus();
    return err;
}

esp_err_t servo_sts_calibrate_travel(uint8_t id, servo_sts_travel_t *travel)
{
    if (travel == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    servo_sts_feedback_t feedback = {0};
    ESP_RETURN_ON_ERROR(servo_sts_read_feedback(id, &feedback), TAG, "cal start read failed");
    const int16_t start = feedback.position;
    ESP_LOGI(TAG, "cal id=%u start=%d temp=%u", id, start, feedback.temperature);

    int16_t reached[2] = {start, start};
    const int step_count = STS_CAL_MAX_TRAVEL / STS_CAL_STEP;

    for (int direction = 0; direction < 2; ++direction) {
        const int sign = direction == 0 ? -1 : 1;
        int16_t command = start;
        int16_t last = start;
        int stall = 0;
        int16_t limit = start;
        const uint8_t single_id[] = {id};

        for (int step = 0; step < step_count; ++step) {
            command = (int16_t)(command + sign * STS_CAL_STEP);
            int16_t target = command;
            if (servo_sts_sync_write_position(single_id, &target, 1,
                                              STS_CAL_SPEED, STS_CAL_ACC) != ESP_OK) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(STS_CAL_DWELL_MS));
            if (servo_sts_read_feedback(id, &feedback) != ESP_OK) {
                break;
            }

            limit = feedback.position;
            const int moved = abs((int)feedback.position - (int)last);
            const int error = abs((int)feedback.position - (int)command);
            if (moved <= STS_CAL_STALL_MOVED && error >= STS_CAL_STALL_MARGIN) {
                stall++;
            } else {
                stall = 0;
            }
            last = feedback.position;

            ESP_LOGI(TAG, "cal id=%u dir=%+d cmd=%d pos=%d moved=%d err=%d load=%d temp=%u",
                     id, sign, command, feedback.position, moved, error,
                     feedback.load, feedback.temperature);

            if (feedback.temperature > STS_CAL_TEMP_LIMIT) {
                ESP_LOGW(TAG, "cal id=%u aborted: temperature %u", id, feedback.temperature);
                break;
            }
            if (stall >= STS_CAL_STALL_COUNT) {
                ESP_LOGW(TAG, "cal id=%u limit reached near %d", id, feedback.position);
                break;
            }
        }

        reached[direction] = limit;

        int16_t backoff = (int16_t)(limit - sign * STS_CAL_BACKOFF);
        servo_sts_sync_write_position(single_id, &backoff, 1, STS_CAL_SPEED, STS_CAL_ACC);
        vTaskDelay(pdMS_TO_TICKS(STS_CAL_DWELL_MS));
    }

    travel->minimum = reached[0];
    travel->maximum = reached[1];

    const uint8_t single_id[] = {id};
    int16_t restore = start;
    servo_sts_sync_write_position(single_id, &restore, 1, STS_CAL_SPEED, STS_CAL_ACC);
    vTaskDelay(pdMS_TO_TICKS(STS_CAL_DWELL_MS));

    ESP_LOGI(TAG, "cal id=%u RESULT low=%d high=%d span=%d",
             id, travel->minimum, travel->maximum,
             travel->maximum - travel->minimum);
    return ESP_OK;
}
