import 'package:flutter_test/flutter_test.dart';
import 'package:wl_bot/services/robot_connection.dart';

/// Live smoke test against a real robot on the LAN. Opt-in via ROBOT_HOST so
/// CI stays green without hardware:
///   ROBOT_HOST=192.168.1.195 flutter test test/live_test.dart
void main() {
  const host = String.fromEnvironment('ROBOT_HOST');

  test('connects, streams status and drives the command pump', () async {
    // ignore: avoid_print
    print('ROBOT_HOST=$host');
    if (host.isEmpty) {
      // No hardware available (e.g. CI): pass vacuously.
      return;
    }
    final conn = RobotConnection(
      pollInterval: const Duration(milliseconds: 100),
      sendInterval: const Duration(milliseconds: 40),
    );
    await conn.connect(host);
    await Future<void>.delayed(const Duration(seconds: 2));
    expect(conn.state, LinkState.connected,
        reason: 'connection error: ${conn.error}');

    final status = conn.status;
    expect(status, isNotNull);
    // ignore: avoid_print
    print('state=${status!.state} battery=${status.battery} '
        'angle=${status.lqrAngle} latency=${status.latencyMs}ms');

    conn.desired.stable = false;
    conn.desired.height = 38;
    await Future<void>.delayed(const Duration(milliseconds: 150));
    await conn.disconnect();

    expect(conn.state, LinkState.disconnected);
  }, timeout: const Timeout(Duration(seconds: 20)));
}
