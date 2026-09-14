import 'dart:async';
import 'dart:math' as math;

import 'package:gamepads/gamepads.dart';

/// Normalized stick vector, -1..1 per axis.
typedef JoyVector = ({double x, double y});

/// Maps a standard gamepad (Xbox/PS layout) onto the full robot command
/// surface:
///
///   left stick    -> drive joystick (dead zone + rescale, as the touch pad)
///   A / Cross     -> jump
///   B / Circle    -> reset to default state (stop + height 100 + legs centered)
///   X / Square    -> toggle roll auto-level
///   Y / Triangle  -> self-right (get up)
///   Start / Menu  -> toggle GO
///   Back / Share  -> emergency stop (stop + GO off)
///   D-pad         -> momentary direction commands
///   RT / LT       -> height up / down while held
///   RB            -> right leg "iron mountain lean" bump
///   LB            -> left leg "iron mountain lean" bump
///   right stick   -> height (Y) and roll (X) trim
///
/// Silently no-ops on platforms without gamepad support.
class GamepadService {
  GamepadService({
    required this.onJoy,
    required this.onJump,
    required this.onStopCmd,
    required this.onEmergencyStop,
    required this.onGoToggle,
    required this.onSelfRight,
    required this.onRollLevelToggle,
    required this.onDirPress,
    required this.onDirRelease,
    required this.onHeightDelta,
    required this.onRollDelta,
    required this.onLegBump,
    required this.onResetToDefault,
  });

  final void Function(JoyVector joy) onJoy;
  final void Function() onJump;
  final void Function() onStopCmd;
  final void Function() onEmergencyStop;
  final void Function() onGoToggle;
  final void Function() onSelfRight;
  final void Function() onRollLevelToggle;
  final void Function(String dir) onDirPress;
  final void Function() onDirRelease;
  final void Function(int delta) onHeightDelta;
  final void Function(int delta) onRollDelta;
  final void Function(int leg) onLegBump;
  final void Function() onResetToDefault;

  static const double _deadzone = 0.15;
  static const double _trimThreshold = 0.4;
  static const double _triggerThreshold = 0.3;

  StreamSubscription<NormalizedGamepadEvent>? _sub;
  Timer? _trimTimer;
  String? _heldDir;

  double _stickX = 0;
  double _stickY = 0;
  double _rightX = 0;
  double _rightY = 0;
  double _rt = 0;
  double _lt = 0;

  bool _listening = false;
  bool get listening => _listening;

  void start() {
    if (_listening) {
      return;
    }
    try {
      _sub = Gamepads.normalizedEvents.listen(_handle, onError: (_) {});
      _listening = true;
    } catch (_) {
      _listening = false;
    }
  }

  Future<void> stop() async {
    await _sub?.cancel();
    _sub = null;
    _trimTimer?.cancel();
    _trimTimer = null;
    _listening = false;
  }

  void _handle(NormalizedGamepadEvent e) {
    final axis = e.axis;
    if (axis != null) {
      switch (axis) {
        case GamepadAxis.leftStickX:
          _stickX = e.value;
          onJoy(_applyDeadzone(_stickX, _stickY));
        case GamepadAxis.leftStickY:
          _stickY = e.value;
          onJoy(_applyDeadzone(_stickX, _stickY));
        case GamepadAxis.rightStickX:
          _rightX = e.value;
          _ensureTrimTimer();
        case GamepadAxis.rightStickY:
          _rightY = e.value;
          _ensureTrimTimer();
        case GamepadAxis.rightTrigger:
          _rt = e.value;
          _ensureTrimTimer();
        case GamepadAxis.leftTrigger:
          _lt = e.value;
          _ensureTrimTimer();
      }
      return;
    }

    final button = e.button;
    if (button == null) {
      return;
    }
    final pressed = e.value > 0.5;
    switch (button) {
      case GamepadButton.a:
        if (pressed) onJump();
      case GamepadButton.b:
        if (pressed) onResetToDefault();
      case GamepadButton.x:
        if (pressed) onRollLevelToggle();
      case GamepadButton.y:
        if (pressed) onSelfRight();
      case GamepadButton.start:
        if (pressed) onGoToggle();
      case GamepadButton.back:
        if (pressed) onEmergencyStop();
      case GamepadButton.leftBumper:
        if (pressed) onLegBump(1);
      case GamepadButton.rightBumper:
        if (pressed) onLegBump(2);
      case GamepadButton.dpadUp:
        _dpad('forward', pressed);
      case GamepadButton.dpadDown:
        _dpad('back', pressed);
      case GamepadButton.dpadLeft:
        _dpad('left', pressed);
      case GamepadButton.dpadRight:
        _dpad('right', pressed);
      default:
        break;
    }
  }

  void _dpad(String dir, bool pressed) {
    if (pressed) {
      _heldDir = dir;
      onDirPress(dir);
    } else if (_heldDir == dir) {
      _heldDir = null;
      onDirRelease();
    }
  }

  bool get _trimActive =>
      _rt > _triggerThreshold ||
      _lt > _triggerThreshold ||
      _rightX.abs() > _trimThreshold ||
      _rightY.abs() > _trimThreshold;

  /// Steps height/roll while a trim input is held, so triggers and the
  /// right stick behave like the on-screen sliders.
  void _ensureTrimTimer() {
    if (_trimTimer != null) {
      return;
    }
    _trimTimer = Timer.periodic(
      const Duration(milliseconds: 90),
      (t) => _applyTrim(t),
    );
  }

  void _applyTrim(Timer t) {
    var height = 0;
    var roll = 0;

    if (_rt > _triggerThreshold) height += 1;
    if (_lt > _triggerThreshold) height -= 1;
    if (_rightY > _trimThreshold) height += 1;
    if (_rightY < -_trimThreshold) height -= 1;

    if (_rightX > _trimThreshold) roll += 1;
    if (_rightX < -_trimThreshold) roll -= 1;

    if (height != 0) onHeightDelta(height);
    if (roll != 0) onRollDelta(roll);

    if (!_trimActive) {
      t.cancel();
      _trimTimer = null;
    }
  }

  JoyVector _applyDeadzone(double x, double y) {
    final m = math.sqrt(x * x + y * y);
    if (m < _deadzone) {
      return (x: 0.0, y: 0.0);
    }
    final s = (m - _deadzone) / (1 - _deadzone) / m;
    return (x: x * s, y: y * s);
  }
}
