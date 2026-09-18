/*
 * Host test suite for the pure control logic in main/robot_logic.h.
 *
 * These are the state machines and predicates the control task uses (yaw
 * fusion/give-up, airborne detection, the jump and bump gait timing, fault
 * recovery and the balance-zero trim), so they can be exercised without a
 * robot or an ESP-IDF toolchain.
 *
 * Run with:  test/host/run.sh
 */

#include "robot_logic.h"

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

static void test_yaw_fusion(void)
{
    robot_yaw_t y = {0};
    /* Pure gyro integration: 90 dps for 10 ms steps. */
    for (int i = 0; i < 10; ++i) {
        robot_yaw_step(&y, 90.0f, 0.0f, 0.0f, 0, 0.001f);
    }
    CHECK(y.fused > 0.89f && y.fused < 0.91f);
    CHECK(y.setpoint > 0.89f && y.setpoint < 0.91f);
    CHECK(!y.gave_up);

    /* Wheel fusion pulls the gyro heading toward the wheel heading. */
    robot_yaw_t f = {0};
    robot_yaw_step(&f, 0.0f, 10.0f, 1.0f, 0, 0.1f);
    CHECK(f.wheel > 0.99f && f.wheel < 1.01f);
    CHECK(f.fused > 0.09f && f.fused < 0.11f);
    CHECK(f.wheel_rate == 10.0f);

    /* The yaw stick ramps the setpoint at YAW_STICK_RATE_DPS per unit. */
    robot_yaw_t s = {0};
    robot_yaw_step(&s, 0.0f, 0.0f, 0.0f, 10, 0.1f);
    CHECK(s.setpoint > 1.99f && s.setpoint < 2.01f);
}

static void test_yaw_give_up(void)
{
    /* A hard uncommanded rotation re-references immediately. */
    robot_yaw_t y = {0};
    robot_yaw_step(&y, 100.0f, 0.0f, 0.0f, 0, 0.001f);
    CHECK(y.gave_up);
    CHECK(y.setpoint == 0.0f);

    /* A large accumulated setpoint also re-references once the stick is
     * released. While the stick is held there is no give-up. */
    robot_yaw_t z = {0};
    for (int i = 0; i < 100; ++i) {
        robot_yaw_step(&z, 0.0f, 0.0f, 0.0f, 50, 0.01f);
    }
    CHECK(z.setpoint > YAW_GIVEUP_DEG);
    CHECK(!z.gave_up);
    robot_yaw_step(&z, 0.0f, 0.0f, 0.0f, 0, 0.001f);
    CHECK(z.gave_up);
    CHECK(z.setpoint == 0.0f);
}

static void test_yaw_hold(void)
{
    robot_yaw_t y = {0};
    robot_yaw_step(&y, 10.0f, 0.0f, 0.0f, 5, 0.1f);
    CHECK(y.setpoint != 0.0f);
    robot_yaw_hold(&y);
    CHECK(y.setpoint == 0.0f);
    CHECK(y.fused_last == y.fused);
}

static void test_yaw_bias_residual(void)
{
    /* Within the trusted window the gyro/wheel difference is the residual. */
    CHECK(robot_yaw_bias_residual(2.0f, 1.0f, 25.0f) == 1.0f);
    CHECK(robot_yaw_bias_residual(0.0f, -1.0f, 25.0f) == 1.0f);
    /* Outside it the wheel reference is not trusted (slip / real rotation). */
    CHECK(robot_yaw_bias_residual(30.0f, 1.0f, 25.0f) == 0.0f);
    CHECK(robot_yaw_bias_residual(1.0f, -30.0f, 25.0f) == 0.0f);
}

static void test_airborne(void)
{
    robot_airborne_t a = {0};
    /* Below the threshold the flag asserts and the hold timer is capped. */
    for (int i = 0; i < 5; ++i) {
        robot_airborne_step(&a, 0.5f, 0.6f, 0.01f);
    }
    CHECK(a.airborne == 1);
    CHECK(a.hold_s <= AIR_HOLD_S);

    /* It stays asserted for AIR_HOLD_S after the magnitude recovers. */
    robot_airborne_step(&a, 1.0f, 0.6f, 0.01f);
    CHECK(a.airborne == 1);
    robot_airborne_step(&a, 1.0f, 0.6f, AIR_HOLD_S);
    CHECK(a.airborne == 0);
}

static void test_slew(void)
{
    CHECK(robot_slew(0.0f, 10.0f, 2.0f) == 2.0f);
    CHECK(robot_slew(10.0f, 0.0f, 2.0f) == 8.0f);
    CHECK(robot_slew(9.0f, 10.0f, 2.0f) == 10.0f);
    CHECK(robot_slew(1.0f, 0.0f, 2.0f) == 0.0f);
    CHECK(robot_slew(5.0f, 5.0f, 2.0f) == 5.0f);
}

static void test_jump_phase(void)
{
    /* crouch 100 ms (+2) then land after 32 ms, then settle. */
    CHECK(robot_jump_phase(1, 0.10f, 100, 32) == 1);
    CHECK(robot_jump_phase(1, 0.11f, 100, 32) == 2);
    CHECK(robot_jump_phase(2, 0.14f, 100, 32) == 3);
    CHECK(robot_jump_phase(3, 0.30f, 100, 32) == 0);
    CHECK(robot_jump_phase(0, 1.0f, 100, 32) == 0);
}

static void test_bump_phase(void)
{
    /* 110 ms extend, then 110 ms retract. */
    CHECK(robot_bump_phase(0.10f, 110) == 1);
    CHECK(robot_bump_phase(0.12f, 110) == 2);
    CHECK(robot_bump_phase(0.21f, 110) == 2);
    CHECK(robot_bump_phase(0.23f, 110) == 0);
}

static void test_fault(void)
{
    CHECK(!robot_attitude_faulted(34.9f, 35.0f));
    CHECK(robot_attitude_faulted(35.1f, 35.0f));
    CHECK(robot_attitude_faulted(-40.0f, 35.0f));

    CHECK(robot_fault_recover_condition(FAULT_ATTITUDE, 5.0f, 0.0f, 7.0f));
    CHECK(!robot_fault_recover_condition(FAULT_ATTITUDE, 15.0f, 0.0f, 7.0f));
    CHECK(robot_fault_recover_condition(FAULT_BATTERY, 0.0f, 7.5f, 7.0f));
    CHECK(!robot_fault_recover_condition(FAULT_BATTERY, 0.0f, 6.5f, 7.0f));
}

static void test_hold_elapsed(void)
{
    float hold = 0.0f;
    CHECK(!robot_hold_elapsed(&hold, true, 0.1f, 0.2f));
    CHECK(robot_hold_elapsed(&hold, true, 0.1f, 0.2f));
    CHECK(hold == 0.0f);
    CHECK(!robot_hold_elapsed(&hold, false, 0.1f, 0.2f));
    CHECK(hold == 0.0f);
}

static void test_zero_trim(void)
{
    CHECK(robot_zero_trim(0.0f, 4.0f, 1.0f, 1.0f) == 4.0f);
    /* A big error is a push, not a bias: no adaptation. */
    CHECK(robot_zero_trim(0.0f, 20.0f, 1.0f, 1.0f) == 0.0f);
    CHECK(robot_zero_trim(11.0f, 4.0f, 1.0f, 1.0f) == LEG_BALANCE_ZERO_TRIM_MAX);
    CHECK(robot_zero_trim(-11.0f, -4.0f, 1.0f, 1.0f) == -LEG_BALANCE_ZERO_TRIM_MAX);
}

int main(void)
{
    test_yaw_fusion();
    test_yaw_give_up();
    test_yaw_hold();
    test_yaw_bias_residual();
    test_airborne();
    test_slew();
    test_jump_phase();
    test_bump_phase();
    test_fault();
    test_hold_elapsed();
    test_zero_trim();

    if (failures == 0) {
        printf("robot_logic: all checks passed\n");
        return 0;
    }
    printf("robot_logic: %d check(s) failed\n", failures);
    return 1;
}
