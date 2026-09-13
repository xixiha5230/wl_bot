import 'package:flutter_test/flutter_test.dart';
import 'package:wl_bot/services/discovery.dart';
import 'package:wl_bot/services/local_subnet.dart';

/// Live LAN-scan test against a real robot. Opt-in via ROBOT_HOST so CI
/// stays green without hardware:
///   ROBOT_HOST=192.168.1.195 flutter test test/live_scan_test.dart
void main() {
  const host = String.fromEnvironment('ROBOT_HOST');

  test('subnet scan finds the robot', () async {
    if (host.isEmpty) {
      // No hardware available (e.g. CI): pass vacuously.
      return;
    }
    final prefixes = await localSubnetPrefixes();
    expect(prefixes, isNotEmpty, reason: 'no local /24 found');

    final found = await DiscoveryService().scanSubnet();
    final hosts = found.map((r) => r.host).toList();
    // ignore: avoid_print
    print('prefixes=$prefixes found=$hosts');
    expect(hosts, contains(host));
    final robot = found.firstWhere((r) => r.host == host);
    expect(robot.status.battery, greaterThan(0));
  }, timeout: const Timeout(Duration(seconds: 60)));
}
