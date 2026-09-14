import 'robot_status.dart';

/// Desired robot command, mirrored 1:1 from the firmware's WS protocol
/// (ws_server.c handle_basic_json). The connection layer serializes this
/// to JSON at a fixed rate.
class DriveCommand {
  DriveCommand({
    this.dir = 'stop',
    this.height = 38,
    this.roll = 0,
    this.stable = false,
    this.joyX = 0,
    this.joyY = 0,
  });

  /// One of stop / forward / back / left / right / jump.
  String dir;
  int height;
  int roll;
  bool stable;
  int joyX;
  int joyY;

  /// Adopt the robot's live state so a fresh connection does not fight what it
  /// is already doing. `stable` mirrors the firmware `go` flag; leaving it at
  /// the default OFF would disengage the balance loop and drop a standing
  /// robot. `roll` is deliberately not adopted: the status field is the
  /// measured angle, not the slider target.
  void adopt(RobotStatus status) {
    stable = status.go;
    height = status.height;
  }

  bool equalsTo(DriveCommand o) =>
      dir == o.dir &&
      height == o.height &&
      roll == o.roll &&
      stable == o.stable &&
      joyX == o.joyX &&
      joyY == o.joyY;

  DriveCommand copy() => DriveCommand(
        dir: dir,
        height: height,
        roll: roll,
        stable: stable,
        joyX: joyX,
        joyY: joyY,
      );

  Map<String, dynamic> toJson() => {
        'mode': 'basic',
        'dir': dir,
        'height': height,
        'roll': roll,
        'linear': 0,
        'angular': 0,
        'stable': stable ? 1 : 0,
        'joy_x': joyX,
        'joy_y': joyY,
      };
}
