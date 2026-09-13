import 'package:flutter/material.dart';

import '../services/discovery.dart';
import '../services/settings_store.dart';

/// Bottom sheet for finding robots: LAN scan results, tap to connect, and
/// the remembered host history below.
class DiscoverSheet extends StatefulWidget {
  const DiscoverSheet({
    super.key,
    required this.settings,
    required this.onPick,
  });

  final SettingsStore settings;
  final ValueChanged<String> onPick;

  @override
  State<DiscoverSheet> createState() => _DiscoverSheetState();
}

class _DiscoverSheetState extends State<DiscoverSheet> {
  bool _scanning = false;
  int _scanned = 0;
  int _total = 0;
  List<FoundRobot> _found = [];

  @override
  void initState() {
    super.initState();
    _scan();
  }

  Future<void> _scan() async {
    setState(() {
      _scanning = true;
      _scanned = 0;
      _total = 0;
      _found = [];
    });
    final found = await DiscoveryService().scanSubnet(
      onProgress: (scanned, total) {
        if (mounted) {
          setState(() {
            _scanned = scanned;
            _total = total;
          });
        }
      },
    );
    if (mounted) {
      setState(() {
        _scanning = false;
        _found = found;
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return SafeArea(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            Row(
              children: [
                const Text('Robots on LAN',
                    style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold)),
                const Spacer(),
                if (_scanning)
                  Text('$_scanned/$_total',
                      style: const TextStyle(
                          fontSize: 12, color: Color(0xFF6F7B8A)))
                else
                  TextButton.icon(
                    onPressed: _scan,
                    icon: const Icon(Icons.refresh, size: 16),
                    label: const Text('Rescan'),
                  ),
                IconButton(
                  icon: const Icon(Icons.close),
                  onPressed: () => Navigator.pop(context),
                ),
              ],
            ),
            if (_scanning) const LinearProgressIndicator(),
            const SizedBox(height: 8),
            Flexible(
              child: ListView(
                shrinkWrap: true,
                children: [
                  if (!_scanning && _found.isEmpty)
                    const Padding(
                      padding: EdgeInsets.symmetric(vertical: 16),
                      child: Center(
                        child: Text('No robots found. Check Wi-Fi.',
                            style: TextStyle(color: Color(0xFF6F7B8A))),
                      ),
                    ),
                  for (final robot in _found) _robotTile(robot),
                  if (widget.settings.hosts.isNotEmpty) ...[
                    const Padding(
                      padding: EdgeInsets.only(top: 12, bottom: 4),
                      child: Text('History',
                          style: TextStyle(
                              fontSize: 12, color: Color(0xFF6F7B8A))),
                    ),
                    for (final host in widget.settings.hosts)
                      ListTile(
                        dense: true,
                        leading: const Icon(Icons.history, size: 18),
                        title: Text(host),
                        onTap: () {
                          Navigator.pop(context);
                          widget.onPick(host);
                        },
                      ),
                  ],
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _robotTile(FoundRobot robot) {
    final s = robot.status;
    final running = s.state == 'running';
    return ListTile(
      leading: Icon(
        Icons.smart_toy,
        color: running ? const Color(0xFF1F8A4C) : const Color(0xFF6F7B8A),
      ),
      title: Text(robot.host,
          style: const TextStyle(fontFamily: 'monospace')),
      subtitle: Text(
        '${s.state}  ${s.battery.toStringAsFixed(2)}V  '
        'angle ${s.lqrAngle.toStringAsFixed(1)}°',
        style: const TextStyle(fontSize: 12),
      ),
      trailing: const Icon(Icons.arrow_forward_ios, size: 16),
      onTap: () {
        Navigator.pop(context);
        widget.onPick(robot.host);
      },
    );
  }
}
