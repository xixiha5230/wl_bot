/// Parsed snapshot of the robot's GET /api/status JSON.
///
/// Field names mirror the firmware's http_server.c status_handler exactly.
class RobotStatus {
  const RobotStatus({
    this.state = '',
    this.battery = 0,
    this.go = false,
    this.height = 38,
    this.lqrAngle = 0,
    this.lqrU = 0,
    this.fault = false,
    this.joyX = 0,
    this.joyY = 0,
    this.dir = 0,
    this.zero = 0,
    this.anglePp = 0,
    this.yaw = 0,
    this.roll = 0,
    this.vl = 0,
    this.vr = 0,
    this.uptime = 0,
    this.jumpHeight = 80,
    this.jumpLand = 40,
    this.jumpSpeed = 0,
    this.jumpAcc = 0,
    this.jumpLandTicks = 32,
    this.jumpCrouch = 34,
    this.jumpCrouchTicks = 150,
    this.latencyMs = 0,
  });

  final String state;
  final double battery;
  final bool go;
  final int height;
  final double lqrAngle;
  final double lqrU;
  final bool fault;
  final int joyX;
  final int joyY;
  final int dir;
  final double zero;
  final double anglePp;
  final double yaw;
  final double roll;
  final double vl;
  final double vr;
  final int uptime;
  final int jumpHeight;
  final int jumpLand;
  final int jumpSpeed;
  final int jumpAcc;
  final int jumpLandTicks;
  final int jumpCrouch;
  final int jumpCrouchTicks;
  final int latencyMs;

  static double _d(Map<String, dynamic> j, String k, double def) =>
      (j[k] as num?)?.toDouble() ?? def;

  static int _i(Map<String, dynamic> j, String k, int def) =>
      (j[k] as num?)?.toInt() ?? def;

  factory RobotStatus.fromJson(Map<String, dynamic> json, {int latencyMs = 0}) {
    return RobotStatus(
      state: (json['state'] as String?) ?? '',
      battery: _d(json, 'battery', 0),
      go: ((json['go'] as num?)?.toInt() ?? 0) != 0,
      height: _i(json, 'height', 38),
      lqrAngle: _d(json, 'lqr_angle', 0),
      lqrU: _d(json, 'lqr_u', 0),
      fault: ((json['fault'] as num?)?.toInt() ?? 0) != 0,
      joyX: _i(json, 'joy_x', 0),
      joyY: _i(json, 'joy_y', 0),
      dir: _i(json, 'dir', 0),
      zero: _d(json, 'zero', 0),
      anglePp: _d(json, 'angle_pp', 0),
      yaw: _d(json, 'yaw', 0),
      roll: _d(json, 'roll', 0),
      vl: _d(json, 'vl', 0),
      vr: _d(json, 'vr', 0),
      uptime: _i(json, 'uptime', 0),
      jumpHeight: _i(json, 'jh', 80),
      jumpLand: _i(json, 'jl', 40),
      jumpSpeed: _i(json, 'js', 0),
      jumpAcc: _i(json, 'jacc', 0),
      jumpLandTicks: _i(json, 'jlt', 32),
      jumpCrouch: _i(json, 'jc', 34),
      jumpCrouchTicks: _i(json, 'jct', 150),
      latencyMs: latencyMs,
    );
  }
}
