import 'dart:async';
import 'dart:convert';

import 'package:flutter/foundation.dart';
import 'package:http/http.dart' as http;
import 'package:web_socket_channel/web_socket_channel.dart';

import '../models/drive_command.dart';
import '../models/robot_status.dart';

enum LinkState { disconnected, connecting, connected, error }

/// Owns the WS control link (/ws) and the /api/status poll, both on port 80.
///
/// - Outgoing DriveCommands are serialized at [sendInterval] and only when
///   they change (plus an idle heartbeat so the robot never keeps a stale
///   joystick value).
/// - Status is polled at [pollInterval]; round-trip time is surfaced as
///   RobotStatus.latencyMs.
/// - While [shouldStayConnected] the link reconnects with exponential
///   backoff (1 s .. 10 s).
class RobotConnection extends ChangeNotifier {
  RobotConnection({
    this.sendInterval = const Duration(milliseconds: 40),
    this.heartbeatInterval = const Duration(milliseconds: 250),
    this.pollInterval = const Duration(milliseconds: 250),
  });

  final Duration sendInterval;
  final Duration heartbeatInterval;
  final Duration pollInterval;

  LinkState _state = LinkState.disconnected;
  String _host = '';
  String _error = '';
  RobotStatus? _status;

  WebSocketChannel? _ws;
  StreamSubscription<dynamic>? _wsSub;
  Timer? _sendTimer;
  Timer? _pollTimer;
  Timer? _reconnectTimer;
  http.Client? _http;

  DriveCommand _desired = DriveCommand();
  DriveCommand? _lastSent;
  DateTime _lastSentAt = DateTime.fromMillisecondsSinceEpoch(0);
  Duration _backoff = const Duration(seconds: 1);

  bool shouldStayConnected = false;

  LinkState get state => _state;
  String get host => _host;
  String get error => _error;
  RobotStatus? get status => _status;

  /// The mutable command the UI writes into; sent at the throttle rate.
  DriveCommand get desired => _desired;

  bool get isConnected => _state == LinkState.connected;

  Future<void> connect(String host) async {
    await disconnect(keepDesired: true);
    _host = _normalizeHost(host);
    shouldStayConnected = true;
    _backoff = const Duration(seconds: 1);
    _setState(LinkState.connecting);
    await _openLink();
  }

  Future<void> disconnect({bool keepDesired = false}) async {
    shouldStayConnected = false;
    _reconnectTimer?.cancel();
    _reconnectTimer = null;
    _sendTimer?.cancel();
    _sendTimer = null;
    _pollTimer?.cancel();
    _pollTimer = null;
    await _wsSub?.cancel();
    _wsSub = null;
    _ws?.sink.close();
    _ws = null;
    _http?.close();
    _http = null;
    _lastSent = null;
    if (!keepDesired) {
      _desired = DriveCommand();
    }
    _status = null;
    _setState(LinkState.disconnected);
  }

  Future<void> _openLink() async {
    _setState(LinkState.connecting);
    final wsUri = Uri.parse('ws://$_host/ws');
    final client = _http ??= http.Client();
    try {
      final probe = await client
          .get(Uri.parse('http://$_host/api/status'))
          .timeout(const Duration(seconds: 3));
      if (probe.statusCode != 200) {
        throw Exception('status ${probe.statusCode}');
      }
      _status = RobotStatus.fromJson(
        jsonDecode(probe.body) as Map<String, dynamic>,
      );
    } catch (e) {
      _error = 'robot unreachable: $e';
      _setState(LinkState.error);
      _scheduleReconnect();
      return;
    }

    try {
      final channel = WebSocketChannel.connect(wsUri);
      await channel.ready.timeout(const Duration(seconds: 3));
      _ws = channel;
      _wsSub = channel.stream.listen(
        (_) {},
        onError: (Object e) => _onLinkLost('ws error: $e'),
        onDone: () => _onLinkLost('ws closed'),
      );
    } catch (e) {
      _error = 'websocket failed: $e';
      _setState(LinkState.error);
      _scheduleReconnect();
      return;
    }

    _error = '';
    _setState(LinkState.connected);
    _adoptFromStatus();
    unawaited(_clearStaleManualLegs());
    _lastSent = null;
    _sendTimer?.cancel();
    _sendTimer = Timer.periodic(sendInterval, (_) => _pump());
    _pollTimer?.cancel();
    _pollTimer = Timer.periodic(pollInterval, (_) => _pollStatus());
  }

  void _onLinkLost(String reason) {
    if (!shouldStayConnected) {
      _setState(LinkState.disconnected);
      return;
    }
    _error = reason;
    _setState(LinkState.connecting);
    _wsSub?.cancel();
    _wsSub = null;
    _ws = null;
    _lastSent = null;
    _scheduleReconnect();
  }

  void _scheduleReconnect() {
    if (!shouldStayConnected || _reconnectTimer != null) {
      return;
    }
    _reconnectTimer = Timer(_backoff, () async {
      _reconnectTimer = null;
      if (shouldStayConnected) {
        _backoff = (_backoff * 2) > const Duration(seconds: 10)
            ? const Duration(seconds: 10)
            : _backoff * 2;
        await _openLink();
      }
    });
  }

  void _pump() {
    final sink = _ws?.sink;
    if (sink == null || _state != LinkState.connected) {
      return;
    }
    final now = DateTime.now();
    final changed = _lastSent == null || !_desired.equalsTo(_lastSent!);
    final heartbeat = now.difference(_lastSentAt) > heartbeatInterval;
    if (!changed && !heartbeat) {
      return;
    }
    sink.add(jsonEncode(_desired.toJson()));
    _lastSent = _desired.copy();
    _lastSentAt = now;
  }

  Future<void> _pollStatus() async {
    if (_state != LinkState.connected) {
      return;
    }
    final client = _http;
    if (client == null) {
      return;
    }
    final watch = Stopwatch()..start();
    try {
      final response = await client
          .get(Uri.parse('http://$_host/api/status'))
          .timeout(pollInterval * 4);
      watch.stop();
      if (response.statusCode == 200) {
        _status = RobotStatus.fromJson(
          jsonDecode(response.body) as Map<String, dynamic>,
          latencyMs: watch.elapsedMilliseconds,
        );
        notifyListeners();
      }
    } catch (e) {
      // A transient status-poll failure must not tear down the control link:
      // the WS has its own onDone/onError and keeps commands flowing. Surface
      // the error for the UI and let the next poll retry.
      _error = 'status poll failed: $e';
      notifyListeners();
    }
  }

  void _setState(LinkState s) {
    if (_state == s) {
      return;
    }
    _state = s;
    notifyListeners();
  }

  /// Seed the outgoing command from the robot's live state so connecting does
  /// not fight what it is already doing. `stable` maps to the firmware's `go`
  /// flag, and the default (false) would drop a robot that was balancing.
  /// `roll` is deliberately not adopted: the status field is the measured
  /// angle (last_roll_angle), not the slider target.
  void _adoptFromStatus() {
    final s = _status;
    if (s == null) {
      return;
    }
    _desired.adopt(s);
    notifyListeners();
  }

  /// The research manual-leg hold latches in the firmware until `lp=0`. If a
  /// previous session left it on, height and roll control look dead, so clear
  /// it on every fresh link to guarantee normal control comes back.
  Future<void> _clearStaleManualLegs() async {
    try {
      await apiSet({'lp': '0'});
    } catch (_) {
      // Best effort: never block the link on this.
    }
  }

  static String _normalizeHost(String raw) {
    var host = raw.trim();
    host = host.replaceFirst(RegExp(r'^https?://'), '');
    host = host.replaceFirst(RegExp(r'^ws://'), '');
    host = host.split('/').first;
    host = host.split(':').first;
    return host;
  }

  // -- REST helpers (one-shot calls, independent of the poll loop) ----------

  Future<String> apiSet(Map<String, String> params) async {
    final query = params.entries
        .map((e) =>
            '${Uri.encodeQueryComponent(e.key)}='
            '${Uri.encodeQueryComponent(e.value)}')
        .join('&');
    final response = await http
        .get(Uri.parse('http://$_host/api/set?$query'))
        .timeout(const Duration(seconds: 4));
    if (response.statusCode != 200) {
      throw Exception('set failed: ${response.statusCode} ${response.body}');
    }
    return response.body;
  }

  Future<void> sendOta(String firmwareUrl) async {
    final response = await http
        .post(Uri.parse('http://$_host/api/ota'), body: firmwareUrl)
        .timeout(const Duration(seconds: 10));
    if (response.statusCode != 200) {
      throw Exception('ota rejected: ${response.statusCode}');
    }
  }

  static Future<RobotStatus?> probe(String host,
      {Duration timeout = const Duration(seconds: 2)}) async {
    try {
      final normalized = _normalizeHost(host);
      final response = await http
          .get(Uri.parse('http://$normalized/api/status'))
          .timeout(timeout);
      if (response.statusCode != 200) {
        return null;
      }
      return RobotStatus.fromJson(jsonDecode(response.body) as Map<String, dynamic>);
    } catch (_) {
      return null;
    }
  }
}
