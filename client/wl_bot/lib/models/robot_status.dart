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
    this.yawMode = 1,
    this.rollMode = 1,
    this.rollBias = 0.52,
    this.faultDeg = 35,
    this.amag = 1,
    this.air = false,
    this.airThresh = 0.60,
    this.airScale = 0.25,
    this.ax = 0,
    this.ay = 0,
    this.az = 0,
    this.manualTicks = 0,
    this.motorMode = 0,
    this.motorAligned = false,
    this.getupState = 0,
    this.faultReason = 0,
    this.p1min = 2077,
    this.p1max = 2453,
    this.p2min = 1619,
    this.p2max = 2007,
    this.anglePp = 0,
    this.ta = 0,
    this.tg = 0,
    this.td = 0,
    this.ts = 0,
    this.legAdd = 0,
    this.yaw = 0,
    this.yawOut = 0,
    this.roll = 0,
    this.vl = 0,
    this.vr = 0,
    this.gz = 0,
    this.uptime = 0,
    this.jumpHeight = 80,
    this.jumpLand = 40,
    this.jumpSpeed = 0,
    this.jumpAcc = 0,
    this.jumpLandTicks = 32,
    this.jumpCrouch = 34,
    this.jumpCrouchTicks = 150,
    this.legTarget1 = 0,
    this.legTarget2 = 0,
    this.jumpState = 0,
    this.manualLegs = false,
    this.bumpLeg = 0,
    this.bumpAmp = 120,
    this.bumpMs = 110,
    this.bumpSpeed = 0,
    this.bumpAcc = 0,
    this.yawFused = 0,
    this.yawWheelRate = 0,
    this.yawWheelScale = -10.4,
    this.yawWheelCorr = 0.5,
    this.zeroAuto = 0,
    this.zeroTrimRate = 0.3,
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
  final int yawMode;
  final int rollMode;
  final double rollBias;
  final double faultDeg;
  final double amag;
  final bool air;
  final double airThresh;
  final double airScale;
  final double ax;
  final double ay;
  final double az;
  final int manualTicks;
  final int motorMode;
  final bool motorAligned;
  final int getupState;
  final int faultReason;
  final int p1min;
  final int p1max;
  final int p2min;
  final int p2max;
  final double anglePp;
  final double ta;
  final double tg;
  final double td;
  final double ts;
  final double legAdd;
  final double yaw;
  final double yawOut;
  final double roll;
  final double vl;
  final double vr;
  final double gz;
  final int uptime;
  final int jumpHeight;
  final int jumpLand;
  final int jumpSpeed;
  final int jumpAcc;
  final int jumpLandTicks;
  final int jumpCrouch;
  final int jumpCrouchTicks;
  final int legTarget1;
  final int legTarget2;
  final int jumpState;

  /// Research manual-leg hold latched in the firmware. Non-zero means height
  /// and roll correction are bypassed, so the leg controls look dead.
  final bool manualLegs;

  /// One-shot leg bump currently running: 0 idle, 1 left, 2 right.
  final int bumpLeg;

  /// Leg bump tuning (extension counts, phase time ms, STS speed 0 = max,
  /// STS acceleration 0 = max).
  final int bumpAmp;
  final int bumpMs;
  final int bumpSpeed;
  final int bumpAcc;

  /// Fused yaw heading (gyro + wheel odometry) and the wheel-derived yaw rate.
  final double yawFused;
  final double yawWheelRate;
  final double yawWheelScale;
  final double yawWheelCorr;

  /// Self-calibrating balance zero: the learned offset (deg) and its rate.
  final double zeroAuto;
  final double zeroTrimRate;

  final int latencyMs;

  bool get rollLevelOn => rollMode != 0;

  bool get selfRighting => getupState != 0;

  /// Jump gait state exposed by the firmware (0 = idle).
  bool get jumping => jumpState != 0;

  /// Leg bump currently running (LB/RB "iron mountain lean").
  bool get bumping => bumpLeg != 0;

  /// 0 none, 1 attitude, 2 battery.
  String get faultReasonName => switch (faultReason) {
        1 => 'attitude',
        2 => 'battery',
        _ => '',
      };

  static double _d(Map<String, dynamic> j, String k, double def) =>
      (j[k] as num?)?.toDouble() ?? def;

  static int _i(Map<String, dynamic> j, String k, int def) =>
      (j[k] as num?)?.toInt() ?? def;

  static bool _b(Map<String, dynamic> j, String k) =>
      ((j[k] as num?)?.toInt() ?? 0) != 0;

  factory RobotStatus.fromJson(Map<String, dynamic> json, {int latencyMs = 0}) {
    return RobotStatus(
      state: (json['state'] as String?) ?? '',
      battery: _d(json, 'battery', 0),
      go: _b(json, 'go'),
      height: _i(json, 'height', 38),
      lqrAngle: _d(json, 'lqr_angle', 0),
      lqrU: _d(json, 'lqr_u', 0),
      fault: _b(json, 'fault'),
      joyX: _i(json, 'joy_x', 0),
      joyY: _i(json, 'joy_y', 0),
      dir: _i(json, 'dir', 0),
      zero: _d(json, 'balance_zero', 0),
      yawMode: _i(json, 'yaw_mode', 1),
      rollMode: _i(json, 'roll_mode', 1),
      rollBias: _d(json, 'roll_bias', 0),
      faultDeg: _d(json, 'fault_deg', 35),
      amag: _d(json, 'accel_mag', 1),
      air: _b(json, 'air'),
      airThresh: _d(json, 'air_thresh', 0.60),
      airScale: _d(json, 'air_scale', 0.25),
      ax: _d(json, 'ax', 0),
      ay: _d(json, 'ay', 0),
      az: _d(json, 'az', 0),
      manualTicks: _i(json, 'manual_ms', 0),
      motorMode: _i(json, 'motor_mode', 0),
      motorAligned: _b(json, 'motor_aligned'),
      getupState: _i(json, 'getup_state', 0),
      faultReason: _i(json, 'fault_reason', 0),
      p1min: _i(json, 'p1min', 2077),
      p1max: _i(json, 'p1max', 2453),
      p2min: _i(json, 'p2min', 1619),
      p2max: _i(json, 'p2max', 2007),
      anglePp: _d(json, 'angle_pp', 0),
      ta: _d(json, 'term_angle', 0),
      tg: _d(json, 'term_gyro', 0),
      td: _d(json, 'term_distance', 0),
      ts: _d(json, 'term_speed', 0),
      legAdd: _d(json, 'leg_add', 0),
      yaw: _d(json, 'yaw_total', 0),
      yawOut: _d(json, 'yaw_out', 0),
      roll: _d(json, 'roll_angle', 0),
      vl: _d(json, 'velocity_left', 0),
      vr: _d(json, 'velocity_right', 0),
      gz: _d(json, 'gyro_z', 0),
      uptime: _i(json, 'uptime', 0),
      jumpHeight: _i(json, 'jump_height', 80),
      jumpLand: _i(json, 'jump_land_height', 40),
      jumpSpeed: _i(json, 'jump_speed', 0),
      jumpAcc: _i(json, 'jump_acc', 0),
      jumpLandTicks: _i(json, 'jump_land_ms', 32),
      jumpCrouch: _i(json, 'jump_crouch', 34),
      jumpCrouchTicks: _i(json, 'jump_crouch_ms', 150),
      legTarget1: _i(json, 'leg1_target', 0),
      legTarget2: _i(json, 'leg2_target', 0),
      jumpState: _i(json, 'jump_state', 0),
      manualLegs: _b(json, 'manual_legs'),
      bumpLeg: _i(json, 'bump_leg', 0),
      bumpAmp: _i(json, 'bump_amp', 120),
      bumpMs: _i(json, 'bump_ms', 110),
      bumpSpeed: _i(json, 'bump_speed', 0),
      bumpAcc: _i(json, 'bump_acc', 0),
      yawFused: _d(json, 'yaw_fused', 0),
      yawWheelRate: _d(json, 'yaw_wheel_rate', 0),
      yawWheelScale: _d(json, 'yaw_wheel_scale', -10.4),
      yawWheelCorr: _d(json, 'yaw_wheel_corr', 0.5),
      zeroAuto: _d(json, 'zauto', 0),
      zeroTrimRate: _d(json, 'zero_trim_rate', 0.3),
      latencyMs: latencyMs,
    );
  }
}
