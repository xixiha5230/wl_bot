import 'package:flutter/material.dart';

import '../services/robot_connection.dart';

class SettingsScreen extends StatefulWidget {
  const SettingsScreen({super.key, required this.connection});

  final RobotConnection connection;

  @override
  State<SettingsScreen> createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen> {
  final _pidName = TextEditingController(text: 'angle');
  final _pidP = TextEditingController();
  final _pidI = TextEditingController();
  final _pidD = TextEditingController();
  final _pidLimit = TextEditingController();

  final _lpfName = TextEditingController(text: 'joyy');
  final _lpfTf = TextEditingController();

  final _zero = TextEditingController();

  final _jh = TextEditingController();
  final _jl = TextEditingController();
  final _js = TextEditingController();
  final _jacc = TextEditingController();
  final _jlt = TextEditingController();
  final _jc = TextEditingController();
  final _jct = TextEditingController();

  final _otaUrl = TextEditingController();

  bool _otaWaiting = false;
  int? _uptimeBefore;

  static const _pidNames = [
    'angle', 'gyro', 'distance', 'speed', 'yaw_angle', 'yaw_gyro',
    'lqr_u', 'zeropoint', 'roll_angle',
  ];
  static const _lpfNames = ['joyy', 'zeropoint', 'roll'];

  @override
  void dispose() {
    for (final c in [
      _pidName, _pidP, _pidI, _pidD, _pidLimit, _lpfName, _lpfTf, _zero,
      _jh, _jl, _js, _jacc, _jlt, _jc, _jct, _otaUrl,
    ]) {
      c.dispose();
    }
    super.dispose();
  }

  Future<void> _apply(Map<String, String> params, String okMessage) async {
    try {
      final reply = await widget.connection.apiSet(params);
      if (mounted) {
        ScaffoldMessenger.of(context)
            .showSnackBar(SnackBar(content: Text(reply.trim())));
      }
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(context)
            .showSnackBar(SnackBar(content: Text('failed: $e')));
      }
    }
  }

  Future<void> _flash() async {
    final url = _otaUrl.text.trim();
    if (url.isEmpty) {
      return;
    }
    final ok = await showDialog<bool>(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('Flash firmware?'),
        content: Text('The robot will download $url and reboot.'),
        actions: [
          TextButton(
              onPressed: () => Navigator.pop(context, false),
              child: const Text('Cancel')),
          FilledButton(
              onPressed: () => Navigator.pop(context, true),
              child: const Text('Flash')),
        ],
      ),
    );
    if (ok != true) {
      return;
    }
    try {
      await widget.connection.sendOta(url);
      _uptimeBefore = widget.connection.status?.uptime;
      setState(() => _otaWaiting = true);
      widget.connection.addListener(_otaWatch);
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(context)
            .showSnackBar(SnackBar(content: Text('ota failed: $e')));
      }
    }
  }

  void _otaWatch() {
    if (!_otaWaiting) {
      return;
    }
    final s = widget.connection.status;
    if (s == null || _uptimeBefore == null) {
      return;
    }
    // The robot reports a fresh uptime after flashing; the connection layer
    // reconnects on its own while the robot reboots.
    if (s.uptime + 10 < _uptimeBefore!) {
      widget.connection.removeListener(_otaWatch);
      if (mounted) {
        setState(() => _otaWaiting = false);
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('rebooted into new firmware (uptime ${s.uptime}s)')),
        );
      }
    }
  }
  @override
  Widget build(BuildContext context) {
    final status = widget.connection.status;
    return Scaffold(
      appBar: AppBar(title: const Text('Settings')),
      body: Center(
        child: ConstrainedBox(
          constraints: const BoxConstraints(maxWidth: 640),
          child: ListView(
            padding: const EdgeInsets.all(12),
            children: [
              _card(
                title: 'PID tuning',
                children: [
                  DropdownButtonFormField<String>(
                    initialValue: _pidName.text,
                    items: [
                      for (final n in _pidNames)
                        DropdownMenuItem(value: n, child: Text(n))
                    ],
                    onChanged: (v) => setState(() => _pidName.text = v ?? 'angle'),
                    decoration: const InputDecoration(labelText: 'PID'),
                  ),
                  const SizedBox(height: 8),
                  _row([
                    _field(_pidP, 'P'),
                    _field(_pidI, 'I (opt)'),
                    _field(_pidD, 'D (opt)'),
                    _field(_pidLimit, 'limit (opt)'),
                  ]),
                  const SizedBox(height: 8),
                  FilledButton(
                    onPressed: () => _apply({
                      'pid': _pidName.text,
                      'p': _pidP.text.isEmpty ? '0' : _pidP.text,
                      if (_pidI.text.isNotEmpty) 'i': _pidI.text,
                      if (_pidD.text.isNotEmpty) 'd': _pidD.text,
                      if (_pidLimit.text.isNotEmpty) 'limit': _pidLimit.text,
                    }, 'pid applied'),
                    child: const Text('Apply PID'),
                  ),
                ],
              ),
              _card(
                title: 'Low-pass filter',
                children: [
                  _row([
                    DropdownButtonFormField<String>(
                      initialValue: _lpfName.text,
                      items: [
                        for (final n in _lpfNames)
                          DropdownMenuItem(value: n, child: Text(n))
                      ],
                      onChanged: (v) => setState(() => _lpfName.text = v ?? 'joyy'),
                      decoration: const InputDecoration(labelText: 'filter'),
                    ),
                    _field(_lpfTf, 'Tf'),
                  ]),
                  const SizedBox(height: 8),
                  FilledButton(
                    onPressed: () => _apply({
                      'lpf': _lpfName.text,
                      'tf': _lpfTf.text.isEmpty ? '0' : _lpfTf.text,
                    }, 'lpf applied'),
                    child: const Text('Apply LPF'),
                  ),
                ],
              ),
              _card(
                title: 'Balance & yaw',
                children: [
                  _row([
                    _field(_zero, 'balance zero (deg)'),
                    OutlinedButton(
                      onPressed: () =>
                          _apply({'zero': _zero.text}, 'zero applied'),
                      child: const Text('Set'),
                    ),
                  ]),
                  const SizedBox(height: 8),
                  _row([
                    OutlinedButton(
                      onPressed: () => _apply({'yaw': '1'}, 'yaw on'),
                      child: const Text('Yaw on'),
                    ),
                    OutlinedButton(
                      onPressed: () => _apply({'yaw': '0'}, 'yaw off'),
                      child: const Text('Yaw off'),
                    ),
                    OutlinedButton(
                      onPressed: () async {
                        final ok = await showDialog<bool>(
                          context: context,
                          builder: (context) => AlertDialog(
                            title: const Text('Recalibrate gyro?'),
                            content: const Text(
                                'Keep the robot perfectly still during calibration.'),
                            actions: [
                              TextButton(
                                  onPressed: () =>
                                      Navigator.pop(context, false),
                                  child: const Text('Cancel')),
                              FilledButton(
                                  onPressed: () =>
                                      Navigator.pop(context, true),
                                  child: const Text('Calibrate')),
                            ],
                          ),
                        );
                        if (ok == true) {
                          await _apply({'gcal': '1'}, 'gyro calibrated');
                        }
                      },
                      child: const Text('Gyro cal'),
                    ),
                  ]),
                  const SizedBox(height: 8),
                  OutlinedButton.icon(
                    onPressed: () async {
                      final ok = await showDialog<bool>(
                        context: context,
                        builder: (context) => AlertDialog(
                          title: const Text('Calibrate level?'),
                          content: const Text(
                              'Stand the robot on flat ground. Roll correction is '
                              'briefly disabled so both legs return to their '
                              'symmetric pose, then the IMU reading is stored.'),
                          actions: [
                            TextButton(
                                onPressed: () => Navigator.pop(context, false),
                                child: const Text('Cancel')),
                            FilledButton(
                                onPressed: () => Navigator.pop(context, true),
                                child: const Text('Calibrate')),
                          ],
                        ),
                      );
                      if (ok == true) {
                        await _apply(
                            {'rblevel': '1'}, 'level calibrated');
                      }
                    },
                    icon: const Icon(Icons.straighten),
                    label: const Text('Calibrate level (auto)'),
                  ),
                ],
              ),
              _card(
                title: 'Jump profile',
                subtitle: status == null
                    ? null
                    : 'current  h=${status.jumpHeight} land=${status.jumpLand} '
                        's=${status.jumpSpeed} a=${status.jumpAcc} '
                        't=${status.jumpLandTicks} c=${status.jumpCrouch} '
                        'ct=${status.jumpCrouchTicks}',
                children: [
                  _row([
                    _field(_jh, 'height'),
                    _field(_jl, 'land'),
                    _field(_js, 'speed 0=max'),
                    _field(_jacc, 'acc'),
                  ]),
                  const SizedBox(height: 8),
                  _row([
                    _field(_jlt, 'land ticks'),
                    _field(_jc, 'crouch h'),
                    _field(_jct, 'crouch ticks'),
                  ]),
                  const SizedBox(height: 8),
                  FilledButton(
                    onPressed: () => _apply({
                      if (_jh.text.isNotEmpty) 'jh': _jh.text,
                      if (_jl.text.isNotEmpty) 'jl': _jl.text,
                      if (_js.text.isNotEmpty) 'js': _js.text,
                      if (_jacc.text.isNotEmpty) 'jacc': _jacc.text,
                      if (_jlt.text.isNotEmpty) 'jlt': _jlt.text,
                      if (_jc.text.isNotEmpty) 'jc': _jc.text,
                      if (_jct.text.isNotEmpty) 'jct': _jct.text,
                    }, 'jump profile applied'),
                    child: const Text('Apply jump profile'),
                  ),
                ],
              ),
              _card(
                title: 'Firmware OTA',
                subtitle: status == null
                    ? 'uptime unknown'
                    : 'uptime ${status.uptime}s  ',
                children: [
                  _field(_otaUrl,
                      'firmware url, e.g. http://192.168.1.193:8000/micro_wheeled_leg_bot.bin'),
                  const SizedBox(height: 8),
                  FilledButton.icon(
                    style: FilledButton.styleFrom(
                      backgroundColor: const Color(0xFFB3382F),
                    ),
                    onPressed: _otaWaiting ? null : _flash,
                    icon: const Icon(Icons.flash_on),
                    label: Text(_otaWaiting
                        ? 'waiting for reboot...'
                        : 'Flash over Wi-Fi'),
                  ),
                ],
              ),
              const SizedBox(height: 24),
            ],
          ),
        ),
      ),
    );
  }

  Widget _card({
    required String title,
    String? subtitle,
    required List<Widget> children,
  }) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(title,
                style: const TextStyle(
                    fontSize: 13,
                    letterSpacing: 0.6,
                    color: Color(0xFF8B97A6))),
            if (subtitle != null)
              Padding(
                padding: const EdgeInsets.only(top: 2),
                child: Text(subtitle,
                    style: const TextStyle(
                        fontSize: 12, color: Color(0xFF6F7B8A))),
              ),
            const SizedBox(height: 10),
            ...children,
          ],
        ),
      ),
    );
  }

  Widget _row(List<Widget> children) => Row(
        children: [
          for (final c in children) ...[
            Expanded(child: c),
            const SizedBox(width: 8),
          ],
        ],
      );

  Widget _field(TextEditingController ctrl, String hint) => TextField(
        controller: ctrl,
        decoration: InputDecoration(hintText: hint, isDense: true),
        keyboardType: TextInputType.text,
      );
}
