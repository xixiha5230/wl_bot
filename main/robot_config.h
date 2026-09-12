#pragma once

/*
 * Calibrated travel of the two leg servos (STS3032), measured on the physical
 * robot through the 'cal' console command.
 *
 * ID1 mechanical range: 2030 .. 2575
 * ID2 mechanical range: 1512 .. 2077
 *
 * The two servos are mounted as a mirror pair, so id1 + id2 is about 4096.
 */

#define SERVO_MIRROR_AXIS        4096

#define SERVO_ID1_MECH_MIN       2030
#define SERVO_ID1_MECH_MAX       2575
#define SERVO_ID2_MECH_MIN       1512
#define SERVO_ID2_MECH_MAX       2077

/* Safety margin kept away from the mechanical limits. */
#define SERVO_TRAVEL_MARGIN      40

#define SERVO_ID1_SAFE_MIN       (SERVO_ID1_MECH_MIN + SERVO_TRAVEL_MARGIN)
#define SERVO_ID1_SAFE_MAX       (SERVO_ID1_MECH_MAX - SERVO_TRAVEL_MARGIN)
#define SERVO_ID2_SAFE_MIN       (SERVO_ID2_MECH_MIN + SERVO_TRAVEL_MARGIN)
#define SERVO_ID2_SAFE_MAX       (SERVO_ID2_MECH_MAX - SERVO_TRAVEL_MARGIN)

/* Aligned neutral pose, taken at the center of the safe range. */
#define SERVO_NEUTRAL_ID1        2302
#define SERVO_NEUTRAL_ID2        (SERVO_MIRROR_AXIS - SERVO_NEUTRAL_ID1)

/*
 * Leg kinematics, from the original firmware. The two legs are a mirror pair
 * mounted +/- LEG_MOUNT_OFFSET around LEG_POSITION_CENTER; height changes by
 * LEG_HEIGHT_STEP counts per height unit.
 */
#define LEG_POSITION_CENTER      2048.0f
#define LEG_MOUNT_OFFSET         12.0f
#define LEG_HEIGHT_STEP          8.4f
#define LEG_HEIGHT_MIN           32
#define LEG_HEIGHT_MAX           80
#define LEG_HEIGHT_DEFAULT       38

/* Software travel clamp, copied from the reference firmware. Kept inside the
 * calibrated safe range above. */
#define LEG_POS1_MIN             2110
#define LEG_POS1_MAX             2510
#define LEG_POS2_MIN             1586
#define LEG_POS2_MAX             1986

/* Leg actuator speed/acceleration for normal height tracking and jumps. */
#define LEG_MOVE_SPEED           200
#define LEG_MOVE_ACC             8
#define LEG_JUMP_SPEED           0
#define LEG_JUMP_ACC             0
#define LEG_JUMP_HEIGHT          80
#define LEG_JUMP_LAND_HEIGHT     40
