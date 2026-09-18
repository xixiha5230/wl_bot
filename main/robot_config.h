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
/* Capped at 72, not the mechanical 80: near the top of the leg travel leg1
 * pins on LEG_POS1_MAX, the roll levelling loses all authority, its integrator
 * winds up and the chassis shakes itself into a fault. 72 leaves ~57 counts of
 * roll headroom while keeping most of the working range. */
#define LEG_HEIGHT_MAX           72
#define LEG_HEIGHT_DEFAULT       38

/* Global leg travel limits (servo command counts). Bench readback at the
 * height extremes (the boundaries beyond which the tyres start to rub):
 *   highest from ground: leg1 = 2438, leg2 = 1635
 *   lowest  from ground: leg1 = 2062, leg2 = 2023
 * The readback is offset from the command by ~15 counts (mirrored:
 * leg1 readback = command - 15, leg2 readback = command + 16), so these
 * command-space limits keep the physical leg inside that readback range.
 * Enforced on every leg command path (height, roll correction and jumps). */
#define LEG_POS1_MIN             2077
#define LEG_POS1_MAX             2453
#define LEG_POS2_MIN             1619
#define LEG_POS2_MAX             2007

/* Leg actuator speed/acceleration for normal height tracking and jumps. */
#define LEG_MOVE_SPEED           200
#define LEG_MOVE_ACC             8
#define LEG_JUMP_SPEED           0
#define LEG_JUMP_ACC             0
/* Jump profile defaults, in the same [LEG_HEIGHT_MIN, LEG_HEIGHT_MAX] domain as
 * every other height so the launch never pins the leg on its travel stop. */
#define LEG_JUMP_HEIGHT          LEG_HEIGHT_MAX
#define LEG_JUMP_LAND_HEIGHT     40

/* Max leg-height change per second. Softens a slider jump so the leg motion
 * does not kick the chassis hard enough to lose balance. Scaled by the measured
 * control-loop period, so it is independent of the actual loop rate. */
#define LEG_HEIGHT_SLEW_RATE     50.0f

/* One-shot leg bump ("iron mountain lean" / 抖肩): quickly extend one leg then
 * hand back to the normal height loop. amp = extension in servo counts,
 * ticks = extend (and retract) time in ms, speed/acc = STS goal speed and
 * acceleration (0 = max for both). Runtime-tunable as
 * bumpamp / bumpms / bumpspeed / bumpacc. Because it is a timed two-phase pulse
 * it can never latch a hold that would freeze height/roll control. */
#define LEG_BUMP_AMP             120
#define LEG_BUMP_TICKS           110
#define LEG_BUMP_SPEED           0
#define LEG_BUMP_ACC             0

/*
 * Balance zero point vs leg height. Changing the leg extension moves the CoM,
 * so the chassis tilt at which the robot balances changes too. Measured on this
 * build: ~4.0 deg at height 32, ~3.2 at 52 and ~0.4 at 80 => about -0.075 deg
 * per height unit. The value is the balance angle at LEG_HEIGHT_DEFAULT.
 */
#define LEG_BALANCE_ZERO_DEFAULT 4.40f
#define LEG_BALANCE_ZERO_SLOPE   0.075f

/*
 * Self-calibrating balance zero. The hard-coded height slope above is only a
 * rough model (measured ~0.155 deg/unit on this build vs 0.075 configured), so
 * while the robot is balancing straight and slow the effective zero walks
 * toward the pitch it actually rests at. That makes it stand with minimal
 * effort at any height without re-measuring the model. rate is in deg/s
 * (0 = off); max bounds the learned offset from the modelled zero.
 */
#define LEG_BALANCE_ZERO_TRIM_RATE 0.3f
#define LEG_BALANCE_ZERO_TRIM_MAX  12.0f
/* Only self-calibrate while the pitch error is small (a bigger error is a
 * push/fall, not a bias) and the robot is nearly stationary. */
#define LEG_BALANCE_ZERO_TRIM_BAND  8.0f
#define LEG_BALANCE_ZERO_TRIM_SPEED 3.0f

/*
 * Yaw heading. The gyro is fast and, once calibrated, drift-free enough; the
 * wheel odometry is only trustworthy while the wheels roll without slipping
 * (a forced/fast rotation makes them slip, and then correcting toward their
 * heading drags the fused heading so the loop chases it into a slow spin).
 * YAW_WHEEL_SCALE converts the wheel velocity difference (rad/s, shaft) into a
 * body yaw rate (deg/s), measured on this build (turn in place, compare the
 * gyro and wheel differential integrals). YAW_WHEEL_CORR is the complementary
 * correction rate (1/s); it defaults to 0 - the wheel fusion is a diagnostic
 * aid, not the primary heading reference.
 */
#define YAW_WHEEL_SCALE    (-10.4f)
#define YAW_WHEEL_CORR     0.0f
/* Give up (re-reference) the accumulated heading instead of fighting a rotation
 * the operator did not ask for: beyond this error with no yaw command, and when
 * the body is being rotated faster than this (picked up / shoved). */
#define YAW_GIVEUP_DEG     35.0f
#define YAW_GIVEUP_RATE    90.0f
/* Heading setpoint change per joystick unit per second (deg/s per unit). */
#define YAW_STICK_RATE_DPS 2.0f

/*
 * Control-loop timing. The nominal period is only used until the first
 * measurement; the control task then times every iteration and every rate /
 * integrator below is scaled by that measured dt.
 */
#define CONTROL_DT_DEFAULT       0.001f
#define LQR_ANGLE_PP_WINDOW_S    0.5f   /* jitter metric window (s) */

/*
 * Wheel-odometry trust thresholds (wheel-shaft deg/s). The distance loop
 * integrates the wheel angle, so it is only a valid position reference while
 * the robot rolls on the ground without slipping.
 */
#define LQR_ODOMETRY_STOP_SPEED  0.5f
#define LQR_ODOMETRY_FAST_SPEED  15.0f
#define LQR_ODOMETRY_SLIP_SPEED  50.0f
#define LQR_ODOMETRY_SLIP_STEP   10.0f
/* Joystick -> wheel speed setpoint gain (deg/s per unit). */
#define LQR_SPEED_JOY_GAIN       0.1f
/* Only trim the LQR_u bias while the output and the distance term are small
 * and the operator is not driving. */
#define LQR_U_TRIM_BAND          5.0f
#define LQR_U_DISTANCE_BAND      4.0f
/* Initial distance reference; any real wheel position replaces it on arm. */
#define LQR_DISTANCE_SENTINEL    (-256.0f)

/*
 * Airborne / drop detection. While the wheels are off the ground the balance
 * loop's drive just spins them up, and the leftover wheel speed makes the
 * robot lunge forward on landing. Below AIR_THRESH_G of specific force it is
 * treated as airborne and the output is scaled by AIR_SCALE.
 */
#define AIR_THRESH_G       0.60f
#define AIR_SCALE          0.25f
#define AIR_HOLD_S         0.02f   /* debounce before trusting the flag (s) */

/*
 * Gyro Z auto-trim while disarmed and at rest. The window must exceed a
 * plausible bad boot offset (MPU6050 ZRO is +/-20 dps) or the trim can never
 * reach it; real rotations are far larger and excluded.
 */
#define GYRO_TRIM_WINDOW_DPS 60.0f
#define GYRO_TRIM_REST_S     0.5f   /* at rest before trusting the reading */
#define GYRO_TRIM_WINDOW_S   3.0f   /* max time the rest window keeps growing */
#define GYRO_TRIM_TAU_S      1.0f   /* offset correction time constant */

/* Fault handling. */
#define ATTITUDE_FAULT_DEG   35.0f  /* pitch that latches an attitude fault */
#define ATTITUDE_RECOVER_DEG 10.0f  /* pitch required to auto-recover */
#define RECOVER_HOLD_S       0.2f   /* condition must hold this long (s) */
#define BATTERY_SAMPLE_S     0.05f  /* ADC sample period (s) */
#define BATTERY_LOW_HOLD_S   2.0f   /* reading must stay low this long (s) */
#define BATTERY_MIN_PLAUSIBLE 4.0f  /* below this the sense wiring is broken */

/* Self-right (get-up). */
#define GETUP_LOW_HEIGHT     35     /* legs lowered before the rock */
#define GETUP_LOWER_S        0.7f   /* time allowed for the legs to get there */
#define GETUP_ROCK_BACK_MS   120    /* rock phase durations (ms) */
#define GETUP_ROCK_FWD_MS    200
#define GETUP_TORQUE         12.0f  /* rock wheel magnitude */
#define GETUP_RELEASE_DEG    20.0f  /* hand over to balance below this pitch */

/* Time after the jump landing command before the gait hands back (s). */
#define JUMP_SETTLE_S        0.16f
