import 'dart:async';
import 'dart:math' as math;

import 'package:gamepads/gamepads.dart';

/// Normalized stick vector, -1..1 per axis.
typedef JoyVector = ({double x, double y});

/// Maps a standard gamepad (Xbox/PS layout) onto the robot command surface:
///   left stick   -> joystick (dead zone + rescale, same as the touch pad)
///   A / Cross    -> jump (momentary; the drive screen reverts to stop)
///   B / Circle   -> stop (zero joystick + dir stop)
///   Start/Menu   -> toggle GO
///   D-pad        -> momentary direction commands
///   RT / LT      -> height up / down while held
///
/// Silently no-ops on platforms without gamepad support.
class GamepadService {
  GamepadService({
    required this.onJoy,
    required this.onJump,
    required this.onStopCmd,
    required this.onGoToggle,
    required this.onDirPress,
    required this.onDirRelease,
    required this.onHeightDelta,
  });

  final void Function(JoyVector joy) onJoy;
  final void Function() onJump;
  final void Function() onStopCmd;
  final void Function() onGoToggle;
  final void Function(String dir) onDirPress;
  final void Function() onDirRelease;
  final void Function(int delta) onHeightDelta;

  StreamSubscription<NormalizedGamepadEvent>? _sub;
  String? _heldDir;
  double _rt = 0;
  double _lt = 0;
  Timer? _triggerTimer;

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
    _triggerTimer?.cancel();
    _triggerTimer = null;
    _listening = false;
  }

  void _handle(NormalizedGamepadEvent e) {
    final axis = e.axis;
    if (axis != null) {
      switch (axis) {
        case GamepadAxis.leftStickX:
        case GamepadAxis.leftStickY:
          final x = axis == GamepadAxis.leftStickX ? e.value : _lastStickX;
          final y = axis == GamepadAxis.leftStickY ? e.value : _lastStickY;
          if (axis == GamepadAxis.leftStickX) {
            _lastStickX = e.value;
          } else {
            _lastStickY = e.value;
          }
          onJoy(_applyDeadzone(x, y));
        case GamepadAxis.rightTrigger:
          _rt = e.value;
          _ensureTriggerTimer();
        case GamepadAxis.leftTrigger:
          _lt = e.value;
          _ensureTriggerTimer();
        default:
          break;
      }
      return;
    }

    final button = e.button;
    if (button == null) {
      return;
    }
    final pressed = e.value > 0.5;
    switch (button) {
      case GamepadButton.a when pressed:
        onJump();
      case GamepadButton.b when pressed:
        onStopCmd();
      case GamepadButton.start when pressed:
        onGoToggle();
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

  double _lastStickX = 0;
  double _lastStickY = 0;

  void _dpad(String dir, bool pressed) {
    if (pressed) {
      _heldDir = dir;
      onDirPress(dir);
    } else if (_heldDir == dir) {
      _heldDir = null;
      onDirRelease();
    }
  }

  JoyVector _applyDeadzone(double x, double y) {
    const dead = 0.15;
    final m = math.sqrt(x * x + y * y);
    if (m < dead) {
      return (x: 0.0, y: 0.0);
    }
    final s = (m - dead) / (1 - dead) / m;
    return (x: x * s, y: y * s);
  }

  void _ensureTriggerTimer() {
    if (_triggerTimer != null) {
      return;
    }
    _triggerTimer = Timer.periodic(const Duration(milliseconds: 160), (t) {
      if (_rt < 0.3 && _lt < 0.3) {
        t.cancel();
        _triggerTimer = null;
        return;
      }
      if (_rt >= 0.3) {
        onHeightDelta(1);
      } else if (_lt >= 0.3) {
        onHeightDelta(-1);
      }
    });
  }
}
