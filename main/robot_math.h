#pragma once

/*
 * Pure, dependency-free geometry/codec helpers shared by the firmware and the
 * host test suite (`test/host/`). Nothing here touches ESP-IDF, FreeRTOS or
 * hardware, so it can be compiled and exercised with a plain C compiler.
 *
 * Keep behaviour identical to what the control loops need; the point is that
 * the firmware and the tests run the *same* code, not a copy.
 */

#include "robot_config.h"

#include <stddef.h>
#include <stdint.h>

/* Clamp a float servo command into [low, high]. */
static inline int16_t robot_clamp_servo(float value, int16_t low, int16_t high)
{
    if (value < (float)low) {
        return low;
    }
    if (value > (float)high) {
        return high;
    }
    return (int16_t)value;
}

/*
 * Leg servo commands for a height command plus a symmetric roll correction
 * (`roll_add` is subtracted from both legs). height is in the
 * [LEG_HEIGHT_MIN, LEG_HEIGHT_MAX] domain; the result is clamped to the
 * caller's travel limits. The mirror pair keeps p1 + p2 ~ 2 * center.
 */
static inline void robot_leg_positions(float height, float roll_add,
                                       int16_t leg1_min, int16_t leg1_max,
                                       int16_t leg2_min, int16_t leg2_max,
                                       int16_t *position1, int16_t *position2)
{
    const float offset = LEG_HEIGHT_STEP * (height - (float)LEG_HEIGHT_MIN);
    *position1 = robot_clamp_servo(
        LEG_POSITION_CENTER + LEG_MOUNT_OFFSET + offset - roll_add, leg1_min, leg1_max);
    *position2 = robot_clamp_servo(
        LEG_POSITION_CENTER - LEG_MOUNT_OFFSET - offset - roll_add, leg2_min, leg2_max);
}

/* Balance zero (deg) for a leg height: the modelled height slope plus the
 * self-calibrated offset. */
static inline float robot_balance_zero(float base, float height, float zero_auto)
{
    return base - LEG_BALANCE_ZERO_SLOPE * (height - (float)LEG_HEIGHT_DEFAULT) + zero_auto;
}

/* Shortest signed delta between two 12-bit AS5600 reads. */
static inline int robot_encoder_delta(uint16_t previous, uint16_t current)
{
    int delta = (int)current - (int)previous;
    if (delta > 2048) {
        delta -= 4096;
    }
    if (delta < -2048) {
        delta += 4096;
    }
    return delta;
}

/*
 * Feetech STS present-position/speed/current encoding is sign-magnitude, not
 * two's complement: bit 15 is the sign and the low 15 bits the magnitude. So
 * 0xFFFF decodes to -32767 (not -1) and encoding -1 gives 0x8001.
 */
static inline int16_t robot_sts_decode_signed(uint8_t low, uint8_t high)
{
    uint16_t value = (uint16_t)low | ((uint16_t)high << 8);
    if (value & 0x8000U) {
        return -(int16_t)(value & 0x7FFFU);
    }
    return (int16_t)value;
}

static inline uint16_t robot_sts_encode_signed(int16_t value)
{
    if (value < 0) {
        return (uint16_t)(-(int32_t)value) | 0x8000U;
    }
    return (uint16_t)value;
}

/* STS checksum: the complement of the sum of the packet bytes (id..params). */
static inline uint8_t robot_sts_checksum(const uint8_t *data, size_t length)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < length; ++i) {
        sum += data[i];
    }
    return (uint8_t)~sum;
}