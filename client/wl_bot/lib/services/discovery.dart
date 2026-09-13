import 'dart:async';

import '../models/robot_status.dart';
import 'local_subnet.dart';
import 'robot_connection.dart';

/// A robot found by subnet scan or remembered in history.
class FoundRobot {
  const FoundRobot({required this.host, required this.status});

  final String host;
  final RobotStatus status;
}

/// LAN discovery for WLROBOTs: probes every address of the local /24 for an
/// /api/status endpoint. No mDNS required, so it works with the current
/// firmware on all platforms.
class DiscoveryService {
  static const _chunk = 48;

  /// Scans all local /24 subnets; reports progress as scanned/total.
  /// Returns after all probes settle (dead hosts just time out).
  Future<List<FoundRobot>> scanSubnet({
    void Function(int scanned, int total)? onProgress,
    Duration timeout = const Duration(milliseconds: 300),
  }) async {
    final prefixes = await localSubnetPrefixes();
    final targets = <String>[
      for (final prefix in prefixes)
        for (var i = 1; i < 255; i++) '$prefix.$i',
    ];
    final found = <FoundRobot>[];
    var scanned = 0;
    for (var start = 0; start < targets.length; start += _chunk) {
      final end = (start + _chunk).clamp(0, targets.length);
      final results = await Future.wait(
        [for (var i = start; i < end; i++) _probe(targets[i], timeout)],
      );
      for (final robot in results) {
        if (robot != null) {
          found.add(robot);
        }
      }
      scanned = end;
      onProgress?.call(scanned, targets.length);
    }
    return found;
  }

  Future<FoundRobot?> _probe(String host, Duration timeout) async {
    final status = await RobotConnection.probe(host, timeout: timeout);
    if (status == null) {
      return null;
    }
    return FoundRobot(host: host, status: status);
  }
}
