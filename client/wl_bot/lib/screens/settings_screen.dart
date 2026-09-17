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

  final _rb = TextEditingController();
  final _faultDeg = TextEditingController();
  final _zrate = TextEditingController();
  final _airThresh = TextEditingController();
  final _airScale = TextEditingController();

  final _p1min = TextEditingController();
  final _p1max = TextEditingController();
  final _p2min = TextEditingController();
  final _p2max = TextEditingController();
  final _lp1 = TextEditingController();
  final _lp2 = TextEditingController();
  final _height = TextEditingController();

  final _bumpAmp = TextEditingController();
  final _bumpMs = TextEditingController();
  final _bumpSpeed = TextEditingController();
  final _bumpAcc = TextEditingController();

  final _gTorque = TextEditingController();
  final _gRelease = TextEditingController();
  final _gSign = TextEditingController();

  final _wdrive = TextEditingController();
  final _wms = TextEditingController();
  final _seq = TextEditingController();
  bool _seqArm = false;

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
      _rb, _faultDeg, _zrate, _airThresh, _airScale,
      _p1min, _p1max, _p2min, _p2max, _lp1, _lp2, _height,
      _bumpAmp, _bumpMs, _bumpSpeed, _bumpAcc,
      _gTorque, _gRelease, _gSign,
      _wdrive, _wms, _seq,
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
                title: 'Roll auto-level & attitude',
                subtitle: status == null
                    ? null
                    : 'roll ${status.roll.toStringAsFixed(2)} deg  '
                        'mode ${status.rollMode}  '
                        'rb ${status.rollBias.toStringAsFixed(2)}  '
                        'fault ${status.faultDeg.toStringAsFixed(0)} deg  '
                        'air ${status.airThresh.toStringAsFixed(2)}'
                        'x${status.airScale.toStringAsFixed(2)}  '
                        'amag ${status.amag.toStringAsFixed(2)} g  '
                        'zeroAuto ${status.zeroAuto.toStringAsFixed(2)} '
                        '@${status.zeroTrimRate.toStringAsFixed(2)}/s',
                children: [
                  _row([
                    OutlinedButton(
                      onPressed: () =>
                          _apply({'rollmode': '1'}, 'roll level on'),
                      child: const Text('Level on'),
                    ),
                    OutlinedButton(
                      onPressed: () =>
                          _apply({'rollmode': '0'}, 'roll level off'),
                      child: const Text('Level off'),
                    ),
                    OutlinedButton(
                      onPressed: () =>
                          _apply({'rollmode': '-1'}, 'roll inverted'),
                      child: const Text('Invert'),
                    ),
                  ]),
                  const SizedBox(height: 8),
                  _row([
                    _field(_zrate, 'zero self-cal deg/s (${status?.zeroTrimRate ?? 0.3})'),
                    OutlinedButton(
                      onPressed: () => _apply(
                          {'zadapt': _zrate.text.isEmpty ? '0' : _zrate.text},
                          'zero trim set'),
                      child: const Text('Set'),
                    ),
                    OutlinedButton(
                      onPressed: () =>
                          _apply({'zauto': '0'}, 'zero offset reset'),
                      child: const Text('Reset'),
                    ),
                  ]),
                  const SizedBox(height: 8),
                  _row([
                    _field(_rb, 'roll bias deg (rb)'),
                    OutlinedButton(
                      onPressed: () => _apply({'rb': _rb.text}, 'rb applied'),
                      child: const Text('Set rb'),
                    ),
                  ]),
                  const SizedBox(height: 8),
                  _row([
                    _field(_faultDeg, 'fault deg (35)'),
                    OutlinedButton(
                      onPressed: () => _apply(
                          {'faultdeg': _faultDeg.text}, 'faultdeg applied'),
                      child: const Text('Set'),
                    ),
                  ]),
                  const SizedBox(height: 8),
                  _row([
                    _field(_airThresh, 'airborne thresh g (0.6)'),
                    _field(_airScale, 'air scale (0.25)'),
                  ]),
                  const SizedBox(height: 8),
                  FilledButton(
                    onPressed: () => _apply({
                      if (_airThresh.text.isNotEmpty) 'airth': _airThresh.text,
                      if (_airScale.text.isNotEmpty) 'airscale': _airScale.text,
                    }, 'airborne applied'),
                    child: const Text('Apply airborne'),
                  ),
                ],
              ),
              _card(
                title: 'Leg travel limits',
                subtitle: status == null
                    ? null
                    : 'p1[${status.p1min}..${status.p1max}] '
                        'p2[${status.p2min}..${status.p2max}]  '
                        'lt1=${status.legTarget1} lt2=${status.legTarget2}',
                children: [
                  _row([
                    _field(_p1min, 'p1min'),
                    _field(_p1max, 'p1max'),
                    _field(_p2min, 'p2min'),
                    _field(_p2max, 'p2max'),
                  ]),
                  const SizedBox(height: 8),
                  FilledButton(
                    onPressed: () => _apply({
                      if (_p1min.text.isNotEmpty) 'p1min': _p1min.text,
                      if (_p1max.text.isNotEmpty) 'p1max': _p1max.text,
                      if (_p2min.text.isNotEmpty) 'p2min': _p2min.text,
                      if (_p2max.text.isNotEmpty) 'p2max': _p2max.text,
                    }, 'limits applied'),
                    child: const Text('Apply limits'),
                  ),
                  const SizedBox(height: 8),
                  _row([
                    _field(_height, 'height 32..72 (h)'),
                    OutlinedButton(
                      onPressed: () =>
                          _apply({'h': _height.text}, 'height applied'),
                      child: const Text('Set h'),
                    ),
                  ]),
                  const Divider(height: 24, color: Color(0xFF2B313A)),
                  const Text('Manual leg hold (research)',
                      style: TextStyle(fontSize: 12, color: Color(0xFF6F7B8A))),
                  const SizedBox(height: 8),
                  _row([
                    _field(_lp1, 'lp1'),
                    _field(_lp2, 'lp2'),
                  ]),
                  const SizedBox(height: 8),
                  _row([
                    OutlinedButton(
                      onPressed: () => _apply({
                        if (_lp1.text.isNotEmpty) 'lp1': _lp1.text,
                        if (_lp2.text.isNotEmpty) 'lp2': _lp2.text,
                      }, 'legs held'),
                      child: const Text('Hold legs'),
                    ),
                    OutlinedButton(
                      onPressed: () => _apply({'lp': '0'}, 'legs released'),
                      child: const Text('Release'),
                    ),
                  ]),
                ],
              ),
              _card(
                title: 'Leg bump (铁山靠)',
                subtitle: status == null
                    ? null
                    : 'running=${status.bumpLeg}  '
                        'amp=${status.bumpAmp} ms=${status.bumpMs} '
                        'speed=${status.bumpSpeed} acc=${status.bumpAcc}  '
                        '${status.manualLegs ? 'LEGS MANUAL' : 'control live'}',
                children: [
                  const Text(
                    'Extends one leg for a short pulse, then snaps back at the '
                    'same speed. speed/acc 0 = max.',
                    style: TextStyle(fontSize: 12, color: Color(0xFF6F7B8A)),
                  ),
                  const SizedBox(height: 8),
                  _row([
                    _field(_bumpAmp, 'amp (${status?.bumpAmp ?? 120})'),
                    _field(_bumpMs, 'ms (${status?.bumpMs ?? 110})'),
                  ]),
                  const SizedBox(height: 8),
                  _row([
                    _field(_bumpSpeed, 'speed (${status?.bumpSpeed ?? 0})'),
                    _field(_bumpAcc, 'acc (${status?.bumpAcc ?? 0})'),
                  ]),
                  const SizedBox(height: 8),
                  FilledButton(
                    onPressed: () => _apply({
                      if (_bumpAmp.text.isNotEmpty) 'bumpamp': _bumpAmp.text,
                      if (_bumpMs.text.isNotEmpty) 'bumpms': _bumpMs.text,
                      if (_bumpSpeed.text.isNotEmpty) 'bumpspeed': _bumpSpeed.text,
                      if (_bumpAcc.text.isNotEmpty) 'bumpacc': _bumpAcc.text,
                    }, 'bump params applied'),
                    child: const Text('Apply bump params'),
                  ),
                  const SizedBox(height: 8),
                  _row([
                    OutlinedButton(
                      onPressed: () => _apply({'bump': '1'}, 'left leg bump'),
                      child: const Text('Left bump'),
                    ),
                    OutlinedButton(
                      onPressed: () => _apply({'bump': '2'}, 'right leg bump'),
                      child: const Text('Right bump'),
                    ),
                    OutlinedButton(
                      onPressed: () => _apply({'reset': '1'}, 'reset to default'),
                      child: const Text('Reset'),
                    ),
                  ]),
                ],
              ),
              _card(
                title: 'Self-right',
                subtitle: status == null
                    ? null
                    : 'state ${status.getupState} '
                        '${status.selfRighting ? '(getting up)' : '(idle)'}',
                children: [
                  _row([
                    FilledButton.icon(
                      onPressed: () =>
                          _apply({'getup': '1'}, 'self-right started'),
                      icon: const Icon(Icons.accessibility_new),
                      label: const Text('Get up'),
                    ),
                    OutlinedButton(
                      onPressed: () =>
                          _apply({'getup': '0'}, 'self-right cancelled'),
                      child: const Text('Cancel'),
                    ),
                  ]),
                  const SizedBox(height: 8),
                  _row([
                    _field(_gTorque, 'torque'),
                    _field(_gRelease, 'release deg'),
                    _field(_gSign, 'sign +/-1'),
                  ]),
                  const SizedBox(height: 8),
                  FilledButton(
                    onPressed: () => _apply({
                      if (_gTorque.text.isNotEmpty) 'gtorque': _gTorque.text,
                      if (_gRelease.text.isNotEmpty) 'grelease': _gRelease.text,
                      if (_gSign.text.isNotEmpty) 'gsign': _gSign.text,
                    }, 'self-right params applied'),
                    child: const Text('Apply params'),
                  ),
                ],
              ),
              _card(
                title: 'Manual wheel drive (research)',
                subtitle: status == null
                    ? null
                    : 'manual ticks ${status.manualTicks}  '
                        'motor mode ${status.motorMode}',
                children: [
                  _row([
                    _field(_wdrive, 'torque -30..30'),
                    _field(_wms, 'ms (300)'),
                    OutlinedButton(
                      onPressed: () => _apply({
                        'wdrive': _wdrive.text.isEmpty ? '0' : _wdrive.text,
                        if (_wms.text.isNotEmpty) 'wms': _wms.text,
                      }, 'wheel drive sent'),
                      child: const Text('Drive'),
                    ),
                  ]),
                  const SizedBox(height: 8),
                  _field(_seq, 'sequence  v:ms, v:ms, ...'),
                  Row(
                    children: [
                      Checkbox(
                        value: _seqArm,
                        onChanged: (v) => setState(() => _seqArm = v ?? false),
                      ),
                      const Expanded(
                          child: Text('arm (hand over on release angle)')),
                    ],
                  ),
                  const SizedBox(height: 8),
                  FilledButton(
                    onPressed: () => _apply({
                      if (_seq.text.isNotEmpty) 'seq': _seq.text,
                      if (_seqArm) 'arm': '1',
                    }, 'sequence sent'),
                    child: const Text('Run sequence'),
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
