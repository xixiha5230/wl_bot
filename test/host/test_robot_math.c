/*
 * Host test suite for the pure firmware helpers in main/robot_math.h.
 *
 * These are the parts that regressed on real hardware before (leg travel
 * clamping at the top of the height range, sign-magnitude servo decoding),
 * so they are worth pinning down without a robot or an ESP-IDF toolchain.
 *
 * Run with:  test/host/run.sh
 */

#include "robot_math.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

static int failures;

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
            failures++;                                                     \
        }                                                                   \
    } while (0)

static void test_clamp_servo(void)
{
    CHECK(robot_clamp_servo(0.0f, 100, 200) == 100);
    CHECK(robot_clamp_servo(150.0f, 100, 200) == 150);
    CHECK(robot_clamp_servo(250.0f, 100, 200) == 200);
    CHECK(robot_clamp_servo(-5.0f, -10, 200) == -5);
}

static void test_leg_positions_mirror(void)
{
    int16_t p1, p2;
    for (int height = LEG_HEIGHT_MIN; height <= LEG_HEIGHT_MAX; ++height) {
        robot_leg_positions((float)height, 0.0f,
                            LEG_POS1_MIN, LEG_POS1_MAX, LEG_POS2_MIN, LEG_POS2_MAX,
                            &p1, &p2);
        CHECK(p1 >= LEG_POS1_MIN && p1 <= LEG_POS1_MAX);
        CHECK(p2 >= LEG_POS2_MIN && p2 <= LEG_POS2_MAX);
        /* The legs are a mirror pair around LEG_POSITION_CENTER, but only while
         * neither is clamped: the travel limits are slightly asymmetric, so the
         * very bottom of the range pins leg2. */
        const bool clamped = p1 == LEG_POS1_MIN || p1 == LEG_POS1_MAX ||
                             p2 == LEG_POS2_MIN || p2 == LEG_POS2_MAX;
        if (!clamped) {
            const int sum = (int)p1 + (int)p2;
            CHECK(abs(sum - (int)(2 * LEG_POSITION_CENTER)) <= 2);
        }
    }
}

static void test_height_has_roll_headroom(void)
{
    /* Regression: at the maximum height leg1 used to pin on LEG_POS1_MAX,
     * leaving the roll levelling no travel and winding its integrator up
     * until the chassis shook into a fault. The cap must leave headroom. */
    int16_t p1, p2;
    robot_leg_positions((float)LEG_HEIGHT_MAX, 0.0f,
                        LEG_POS1_MIN, LEG_POS1_MAX, LEG_POS2_MIN, LEG_POS2_MAX,
                        &p1, &p2);
    CHECK(p1 < LEG_POS1_MAX);
    CHECK(p2 > LEG_POS2_MIN);
    printf("  height %d: leg1=%d (headroom %d), leg2=%d (headroom %d)\n",
           LEG_HEIGHT_MAX, p1, LEG_POS1_MAX - p1, p2, p2 - LEG_POS2_MIN);
}

static void test_out_of_range_height_is_clamped(void)
{
    /* A requested height above the cap (e.g. the old jump default of 80) must
     * not push the servo past its travel limit. */
    int16_t p1, p2;
    robot_leg_positions(80.0f, 0.0f,
                        LEG_POS1_MIN, LEG_POS1_MAX, LEG_POS2_MIN, LEG_POS2_MAX,
                        &p1, &p2);
    CHECK(p1 <= LEG_POS1_MAX);
    CHECK(p2 >= LEG_POS2_MIN);

    robot_leg_positions(-100.0f, 0.0f,
                        LEG_POS1_MIN, LEG_POS1_MAX, LEG_POS2_MIN, LEG_POS2_MAX,
                        &p1, &p2);
    CHECK(p1 >= LEG_POS1_MIN);
    CHECK(p2 <= LEG_POS2_MAX);
}

static void test_roll_correction_clamps(void)
{
    int16_t p1, p2;
    /* A roll correction far beyond the travel must saturate, not overflow. */
    robot_leg_positions((float)LEG_HEIGHT_DEFAULT, 100000.0f,
                        LEG_POS1_MIN, LEG_POS1_MAX, LEG_POS2_MIN, LEG_POS2_MAX,
                        &p1, &p2);
    CHECK(p1 <= LEG_POS1_MAX && p1 >= LEG_POS1_MIN);
    CHECK(p2 <= LEG_POS2_MAX && p2 >= LEG_POS2_MIN);
}

static void test_balance_zero(void)
{
    const float base = 4.40f;
    /* At the default height the modelled slope contributes nothing. */
    CHECK(robot_balance_zero(base, (float)LEG_HEIGHT_DEFAULT, 0.0f) == base);
    CHECK(robot_balance_zero(base, (float)LEG_HEIGHT_DEFAULT, 1.5f) == base + 1.5f);
    /* Higher legs need a smaller zero on this build. */
    CHECK(robot_balance_zero(base, (float)LEG_HEIGHT_MAX, 0.0f) < base);
    CHECK(robot_balance_zero(base, (float)LEG_HEIGHT_MIN, 0.0f) > base);
}

static void test_encoder_delta(void)
{
    CHECK(robot_encoder_delta(100, 140) == 40);
    CHECK(robot_encoder_delta(140, 100) == -40);
    /* Wrap up (through zero) and down (through 4095). */
    CHECK(robot_encoder_delta(4000, 100) == 196);
    CHECK(robot_encoder_delta(100, 4000) == -196);
    CHECK(robot_encoder_delta(0, 2047) == 2047);
    CHECK(robot_encoder_delta(0, 2049) == -2047);
}

static void test_sts_signed_decode(void)
{
    /* Sign-magnitude, not two's complement. */
    CHECK(robot_sts_decode_signed(0x01, 0x00) == 1);
    CHECK(robot_sts_decode_signed(0xFF, 0x7F) == 32767);
    CHECK(robot_sts_decode_signed(0x01, 0x80) == -1);
    CHECK(robot_sts_decode_signed(0xFF, 0xFF) == -32767);
    /* Round-trip through the encoder. */
    const int16_t values[] = {0, 1, -1, 2048, -2048, 32767, -32767};
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
        uint16_t encoded = robot_sts_encode_signed(values[i]);
        int16_t decoded = robot_sts_decode_signed(
            (uint8_t)(encoded & 0xFF), (uint8_t)(encoded >> 8));
        CHECK(decoded == values[i]);
        if (decoded != values[i]) {
            printf("  round-trip failed for %d (encoded 0x%04X, decoded %d)\n",
                   values[i], encoded, decoded);
        }
    }
}

static void test_sts_checksum(void)
{
    const uint8_t packet[] = {0x01, 0x04, 0x03, 0x28, 0x01};
    CHECK(robot_sts_checksum(packet, sizeof(packet)) == 0xCE);
}

int main(void)
{
    test_clamp_servo();
    test_leg_positions_mirror();
    test_height_has_roll_headroom();
    test_out_of_range_height_is_clamped();
    test_roll_correction_clamps();
    test_balance_zero();
    test_encoder_delta();
    test_sts_signed_decode();
    test_sts_checksum();

    if (failures == 0) {
        printf("robot_math: all checks passed\n");
        return 0;
    }
    printf("robot_math: %d check(s) failed\n", failures);
    return 1;
}