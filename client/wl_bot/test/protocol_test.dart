import 'package:flutter_test/flutter_test.dart';
import 'package:wl_bot/models/drive_command.dart';
import 'package:wl_bot/models/robot_status.dart';

void main() {
  test('DriveCommand JSON matches the firmware WS protocol', () {
    final json = DriveCommand(
      dir: 'stop',
      height: 42,
      roll: -3,
      stable: true,
      joyX: -55,
      joyY: 60,
    ).toJson();
    expect(json, {
      'mode': 'basic',
      'dir': 'stop',
      'height': 42,
      'roll': -3,
      'linear': 0,
      'angular': 0,
      'stable': 1,
      'joy_x': -55,
      'joy_y': 60,
    });
  });

  test('DriveCommand.adopt keeps go on for a balancing robot', () {
    final cmd = DriveCommand();
    expect(cmd.stable, isFalse);
    cmd.adopt(RobotStatus.fromJson(const {'go': 1, 'height': 52}));
    expect(cmd.stable, isTrue);
    expect(cmd.height, 52);
    expect(cmd.toJson()['stable'], 1);
  });

  test('RobotStatus parses the firmware status payload', () {    final payload = <String, dynamic>{
      'state': 'running',
      'battery': 8.13,
      'go': 1,
      'height': 38,
      'lqr_angle': 4.42,
      'lqr_u': -0.312,
      'fault': 0,
      'joy_x': 0,
      'joy_y': 0,
      'dir': 4,
      'balance_zero': 4.4,
      'angle_pp': 0.42,
      'yaw_total': -0.1,
      'roll_angle': 0.52,
      'velocity_left': -0.64,
      'velocity_right': -0.48,
      'uptime': 615,
      'jump_height': 80,
      'jump_land_height': 40,
      'jump_speed': 0,
      'jump_acc': 0,
      'jump_land_ms': 32,
      'jump_crouch': 34,
      'jump_crouch_ms': 150,
    };
    final status = RobotStatus.fromJson(payload, latencyMs: 7);
    expect(status.state, 'running');
    expect(status.go, isTrue);
    expect(status.fault, isFalse);
    expect(status.battery, 8.13);
    expect(status.lqrAngle, 4.42);
    expect(status.height, 38);
    expect(status.jumpCrouch, 34);
    expect(status.jumpLandTicks, 32);
    expect(status.latencyMs, 7);
  });

  test('RobotStatus tolerates missing fields', () {
    final status = RobotStatus.fromJson(const {});
    expect(status.state, '');
    expect(status.go, isFalse);
    expect(status.height, 38);
  });

  test('RobotStatus parses the extended telemetry fields', () {
    final payload = <String, dynamic>{
      'roll_mode': 0,
      'roll_bias': 0.52,
      'fault_deg': 30.0,
      'accel_mag': 0.36,
      'air': 1,
      'air_thresh': 0.6,
      'air_scale': 0.25,
      'motor_aligned': 1,
      'getup_state': 1,
      'fault_reason': 1,
      'p1min': 2077,
      'p1max': 2453,
      'p2min': 1619,
      'p2max': 2007,
      'leg1_target': 2110,
      'leg2_target': 1985,
      'jump_state': 0,
      'manual_legs': 1,
      'bump_leg': 2,
      'bump_amp': 60,
      'bump_ms': 140,
      'bump_speed': 0,
    };
    final s = RobotStatus.fromJson(payload);
    expect(s.rollMode, 0);
    expect(s.rollLevelOn, isFalse);
    expect(s.air, isTrue);
    expect(s.amag, 0.36);
    expect(s.faultReasonName, 'attitude');
    expect(s.selfRighting, isTrue);
    expect(s.motorAligned, isTrue);
    expect(s.p1min, 2077);
    expect(s.p2max, 2007);
    expect(s.legTarget1, 2110);
    expect(s.manualLegs, isTrue);
    expect(s.bumpLeg, 2);
    expect(s.bumping, isTrue);
    expect(s.bumpAmp, 60);
    expect(s.bumpMs, 140);
  });

  test('RobotStatus defaults the bump fields to idle', () {
    final s = RobotStatus.fromJson(const {});
    expect(s.manualLegs, isFalse);
    expect(s.bumpLeg, 0);
    expect(s.bumping, isFalse);
    expect(s.bumpAmp, 120);
    expect(s.bumpMs, 110);
    expect(s.bumpSpeed, 0);
    expect(s.bumpAcc, 0);
  });
}
