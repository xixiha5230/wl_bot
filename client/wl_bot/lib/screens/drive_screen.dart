import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import '../main.dart' show wlBotGreen;
import '../models/robot_status.dart';
import '../services/gamepad_service.dart';
import '../services/robot_connection.dart';
import '../services/settings_store.dart';
import '../widgets/battery_gauge.dart';
import '../widgets/discover_sheet.dart';
import '../widgets/joystick.dart';
import 'settings_screen.dart';

class DriveScreen extends StatefulWidget {
  const DriveScreen({super.key, required this.settings});

  final SettingsStore settings;

  @override
  State<DriveScreen> createState() => _DriveScreenState();
}

class _DriveScreenState extends State<DriveScreen> {
  final RobotConnection _conn = RobotConnection();
  final TextEditingController _hostCtrl = TextEditingController();
  late final GamepadService _gamepad;
  Timer? _jumpTimer;
  String _activeDir = 'stop';

  @override
  void initState() {
    super.initState();
    _hostCtrl.text = widget.settings.lastHost ?? '192.168.1.195';
    _conn.addListener(_onLinkChanged);
    _gamepad = GamepadService(
      onJoy: _onJoyNorm,
      onJump: () => _setDir('jump'),
      onStopCmd: () {
        _setDir('stop');
        _onJoyNorm(const (x: 0.0, y: 0.0));
      },
      onGoToggle: () => _setGo(!_conn.desired.stable),
      onDirPress: _setDir,
      onDirRelease: () => _setDir('stop'),
      onHeightDelta: (delta) => setState(() {
        _conn.desired.height =
            (_conn.desired.height + delta).clamp(32, 80);
      }),
    );
    _gamepad.start();
  }

  @override
  void dispose() {
    _jumpTimer?.cancel();
    _gamepad.stop();
    _conn.removeListener(_onLinkChanged);
    _conn.dispose();
    _hostCtrl.dispose();
    super.dispose();
  }

  void _onLinkChanged() {
    if (mounted) {
      setState(() {});
    }
  }

  Future<void> _connect() async {
    final host = _hostCtrl.text.trim();
    if (host.isEmpty) {
      return;
    }
    await widget.settings.rememberHost(host);
    setState(() {});
    await _conn.connect(host);
  }

  Future<void> _openDiscover() async {
    if (!mounted) {
      return;
    }
    await showModalBottomSheet<void>(
      context: context,
      isScrollControlled: true,
      builder: (context) => DraggableScrollableSheet(
        expand: false,
        initialChildSize: 0.7,
        maxChildSize: 0.92,
        builder: (context, controller) => DiscoverSheet(
          settings: widget.settings,
          onPick: (host) {
            _hostCtrl.text = host;
            _connect();
          },
        ),
      ),
    );
    if (mounted) {
      setState(() {});
    }
  }

  void _setDir(String dir) {
    setState(() => _activeDir = dir);
    _conn.desired.dir = dir;
    if (dir == 'jump') {
      // The firmware fires the jump on the jump -> stop edge.
      _jumpTimer?.cancel();
      _jumpTimer = Timer(const Duration(milliseconds: 400), () {
        if (_activeDir == 'jump') {
          _setDir('stop');
        }
      });
    }
  }

  Future<void> _setGo(bool on) async {
    _conn.desired.stable = on;
    try {
      await _conn.apiSet({'go': on ? '1' : '0'});
    } catch (_) {
      // The WS stable field still carries the intent.
    }
    setState(() {});
  }

  @override
  Widget build(BuildContext context) {
    final status = _conn.status;
    return Scaffold(
      body: SafeArea(
        child: Focus(
          autofocus: true,
          onKeyEvent: _onKey,
          child: Column(
            children: [
              _topBar(status),
              const Divider(height: 1, color: Color(0xFF2B313A)),
              Expanded(
                child: LayoutBuilder(
                  builder: (context, box) {
                    final wide = box.maxWidth > 820;
                    if (wide) {
                      return Row(
                        crossAxisAlignment: CrossAxisAlignment.center,
                        children: [
                          Expanded(
                            child: Center(
                              child: Joystick(
                                size: 300,
                                onChanged: _onJoy,
                              ),
                            ),
                          ),
                          SizedBox(
                            width: 360,
                            child: Column(
                              mainAxisAlignment: MainAxisAlignment.center,
                              children: [
                                _statusPanel(status),
                                _sliders(),
                                _dirPad(),
                                _goStop(),
                              ],
                            ),
                          ),
                          const SizedBox(width: 16),
                        ],
                      );
                    }
                    return SingleChildScrollView(
                      padding: const EdgeInsets.all(12),
                      child: Column(
                        children: [
                          _statusPanel(status),
                          const SizedBox(height: 12),
                          Joystick(size: 240, onChanged: _onJoy),
                          const SizedBox(height: 12),
                          _sliders(),
                          _dirPad(),
                          _goStop(),
                        ],
                      ),
                    );
                  },
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  KeyEventResult _onKey(FocusNode node, KeyEvent event) {
    if (event is! KeyDownEvent && event is! KeyRepeatEvent) {
      return KeyEventResult.ignored;
    }
    const speed = 60;
    if (event.logicalKey == LogicalKeyboardKey.space) {
      _setDir('jump');
      return KeyEventResult.handled;
    }
    if (event.logicalKey == LogicalKeyboardKey.keyW ||
        event.logicalKey == LogicalKeyboardKey.arrowUp) {
      _conn.desired.joyY = speed;
      setState(() {});
      return KeyEventResult.handled;
    }
    if (event.logicalKey == LogicalKeyboardKey.keyS ||
        event.logicalKey == LogicalKeyboardKey.arrowDown) {
      _conn.desired.joyY = -speed;
      setState(() {});
      return KeyEventResult.handled;
    }
    if (event.logicalKey == LogicalKeyboardKey.keyA ||
        event.logicalKey == LogicalKeyboardKey.arrowLeft) {
      _conn.desired.joyX = -speed;
      setState(() {});
      return KeyEventResult.handled;
    }
    if (event.logicalKey == LogicalKeyboardKey.keyD ||
        event.logicalKey == LogicalKeyboardKey.arrowRight) {
      _conn.desired.joyX = speed;
      setState(() {});
      return KeyEventResult.handled;
    }
    if (event.logicalKey == LogicalKeyboardKey.escape ||
        event.logicalKey == LogicalKeyboardKey.keyX) {
      _setDir('stop');
      _conn.desired.joyX = 0;
      _conn.desired.joyY = 0;
      setState(() {});
      return KeyEventResult.handled;
    }
    return KeyEventResult.ignored;
  }

  void _onJoy(Offset v) {
    _conn.desired.joyX = (v.dx * 100).round();
    _conn.desired.joyY = (v.dy * 100).round();
  }

  /// Gamepad stick input already carries the dead zone; values are -1..1.
  void _onJoyNorm(({double x, double y}) v) {
    _conn.desired.joyX = (v.x * 100).round();
    _conn.desired.joyY = (v.y * 100).round();
    setState(() {});
  }

  Widget _topBar(RobotStatus? status) {
    final dotColor = switch (_conn.state) {
      LinkState.connected => wlBotGreen,
      LinkState.connecting => const Color(0xFFD8A12E),
      LinkState.error => const Color(0xFFB3382F),
      LinkState.disconnected => const Color(0xFFB3382F),
    };
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
      child: Row(
        children: [
          Icon(Icons.circle, size: 12, color: dotColor),
          const SizedBox(width: 8),
          Expanded(
            child: TextField(
              controller: _hostCtrl,
              enabled: !_conn.isConnected,
              decoration: const InputDecoration(
                hintText: 'robot ip, e.g. 192.168.1.195',
                isDense: true,
              ),
              onSubmitted: (_) => _connect(),
            ),
          ),
          IconButton(
            tooltip: 'Find robots on LAN',
            icon: const Icon(Icons.radar),
            onPressed: _conn.isConnected ? null : _openDiscover,
          ),
          const SizedBox(width: 8),
          _conn.isConnected
              ? OutlinedButton(
                  onPressed: _conn.disconnect,
                  child: const Text('Disconnect'),
                )
              : FilledButton(
                  onPressed: _conn.state == LinkState.connecting
                      ? null
                      : _connect,
                  child: const Text('Connect'),
                ),
          if (_conn.isConnected) ...[
            const SizedBox(width: 8),
            Chip(
              label: Text(
                '${status?.latencyMs ?? 0} ms',
                style: const TextStyle(fontSize: 12),
              ),
              visualDensity: VisualDensity.compact,
            ),
            IconButton(
              tooltip: 'Settings',
              icon: const Icon(Icons.tune),
              onPressed: () => _openSettings(context),
            ),
          ],
        ],
      ),
    );
  }

  Widget _statusPanel(RobotStatus? status) {
    final s = status;
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: s == null
            ? const Text('not connected',
                style: TextStyle(color: Color(0xFF6F7B8A)))
            : Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Row(
                    children: [
                      _pill(s.state.toUpperCase(),
                          s.state == 'running' ? wlBotGreen : null),
                      if (s.fault) ...[
                        const SizedBox(width: 6),
                        const _Pill('FAULT', Color(0xFFB3382F)),
                      ],
                      const Spacer(),
                      BatteryGauge(voltage: s.battery),
                    ],
                  ),
                  const SizedBox(height: 8),
                  _monoRow('angle', '${s.lqrAngle.toStringAsFixed(2)} deg'),
                  _monoRow('pp', s.anglePp.toStringAsFixed(2)),
                  _monoRow('lqr_u', s.lqrU.toStringAsFixed(3)),
                  _monoRow('height', '${s.height}   zero ${s.zero.toStringAsFixed(2)}'),
                ],
              ),
      ),
    );
  }

  Widget _pill(String text, [Color? color]) =>
      _Pill(text, color ?? const Color(0xFF26303A));

  Widget _monoRow(String k, String v) => Padding(
        padding: const EdgeInsets.symmetric(vertical: 1),
        child: Row(
          children: [
            SizedBox(
              width: 64,
              child: Text(k,
                  style: const TextStyle(
                      color: Color(0xFF6F7B8A), fontSize: 12)),
            ),
            Text(v, style: const TextStyle(fontSize: 13)),
          ],
        ),
      );

  Widget _sliders() {
    return Column(
      children: [
        _slider(
          label: 'Height',
          value: _conn.desired.height.toDouble(),
          min: 32,
          max: 80,
          display: '${_conn.desired.height}',
          onChanged: (v) => setState(() => _conn.desired.height = v.round()),
        ),
        _slider(
          label: 'Roll',
          value: _conn.desired.roll.toDouble(),
          min: -30,
          max: 30,
          display: '${_conn.desired.roll}',
          onChanged: (v) => setState(() => _conn.desired.roll = v.round()),
        ),
      ],
    );
  }

  Widget _slider({
    required String label,
    required double value,
    required double min,
    required double max,
    required String display,
    required ValueChanged<double> onChanged,
  }) {
    return Row(
      children: [
        SizedBox(width: 56, child: Text(label, style: const TextStyle(fontSize: 12))),
        Expanded(
          child: Slider(
            value: value.clamp(min, max),
            min: min,
            max: max,
            divisions: (max - min).round(),
            onChanged: _conn.isConnected ? onChanged : null,
          ),
        ),
        SizedBox(width: 34, child: Text(display, style: const TextStyle(fontSize: 12))),
      ],
    );
  }

  Widget _dirPad() {
    Widget btn(String dir, [IconData? icon, String? label]) {
      final active = _activeDir == dir && dir != 'stop';
      final isJump = dir == 'jump';
      return OutlinedButton(
        style: OutlinedButton.styleFrom(
          backgroundColor: active
              ? (isJump ? const Color(0xFFD8A12E) : const Color(0xFF2F6FED))
              : const Color(0xFF26303A),
          foregroundColor: Colors.white,
          side: BorderSide(
            color: active
                ? (isJump ? const Color(0xFFD8A12E) : const Color(0xFF2F6FED))
                : const Color(0xFF333B45),
          ),
          padding: const EdgeInsets.symmetric(vertical: 14),
        ),
        onPressed:
            _conn.isConnected ? () => _setDir(isJump ? 'jump' : dir) : null,
        child: icon != null
            ? Icon(icon, size: 22)
            : Text(label ?? dir, style: const TextStyle(fontSize: 12)),
      );
    }

    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 8),
      child: GridView.count(
        crossAxisCount: 3,
        shrinkWrap: true,
        physics: const NeverScrollableScrollPhysics(),
        mainAxisSpacing: 6,
        crossAxisSpacing: 6,
        childAspectRatio: 2.6,
        children: [
          btn('forward', Icons.arrow_upward),
          btn('jump', Icons.rocket_launch),
          btn('left', Icons.arrow_back),
          btn('stop', Icons.stop),
          btn('right', Icons.arrow_forward),
          btn('back', Icons.arrow_downward),
        ],
      ),
    );
  }

  Widget _goStop() {
    final go = _conn.desired.stable;
    return Row(
      children: [
        Expanded(
          child: FilledButton.icon(
            style: FilledButton.styleFrom(
              backgroundColor: go ? wlBotGreen : const Color(0xFF26303A),
              foregroundColor: Colors.white,
              padding: const EdgeInsets.symmetric(vertical: 14),
            ),
            onPressed:
                _conn.isConnected ? () => _setGo(!go) : null,
            icon: Icon(go ? Icons.pause : Icons.play_arrow),
            label: Text(go ? 'GOING' : 'GO'),
          ),
        ),
        const SizedBox(width: 8),
        Expanded(
          child: FilledButton.icon(
            style: FilledButton.styleFrom(
              backgroundColor: const Color(0xFFB3382F),
              foregroundColor: Colors.white,
              padding: const EdgeInsets.symmetric(vertical: 14),
            ),
            onPressed: _conn.isConnected
                ? () {
                    _setDir('stop');
                    _conn.desired.joyX = 0;
                    _conn.desired.joyY = 0;
                    _setGo(false);
                  }
                : null,
            icon: const Icon(Icons.emergency),
            label: const Text('STOP'),
          ),
        ),
      ],
    );
  }

  void _openSettings(BuildContext context) {
    Navigator.of(context).push(
      MaterialPageRoute<void>(
        builder: (_) => SettingsScreen(connection: _conn),
      ),
    );
  }
}

class _Pill extends StatelessWidget {
  const _Pill(this.text, this.color);

  final String text;
  final Color? color;

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 2),
      decoration: BoxDecoration(
        color: (color ?? const Color(0xFF26303A)).withValues(alpha: 0.9),
        borderRadius: BorderRadius.circular(10),
      ),
      child: Text(
        text,
        style: TextStyle(
          fontSize: 11,
          color: color != null ? Colors.white : Colors.white70,
        ),
      ),
    );
  }
}
