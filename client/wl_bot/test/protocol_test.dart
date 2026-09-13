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

  test('RobotStatus parses the firmware status payload', () {
    final payload = <String, dynamic>{
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
      'zero': 4.4,
      'angle_pp': 0.42,
      'yaw': -0.1,
      'roll': 0.52,
      'vl': -0.64,
      'vr': -0.48,
      'uptime': 615,
      'jh': 80,
      'jl': 40,
      'js': 0,
      'jacc': 0,
      'jlt': 32,
      'jc': 34,
      'jct': 150,
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
}
