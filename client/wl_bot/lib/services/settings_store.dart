import 'dart:convert';

import 'package:shared_preferences/shared_preferences.dart';

/// Persisted app settings: recent robot hosts, last used host, and the
/// joystick side preference.
class SettingsStore {
  static const _keyHosts = 'hosts';
  static const _keyLastHost = 'last_host';
  static const _keyJoystickRight = 'joystick_right';
  static const _maxHosts = 8;

  List<String> _hosts = [];
  bool _joystickRight = false;

  List<String> get hosts => List.unmodifiable(_hosts);
  bool get joystickRight => _joystickRight;

  Future<void> load() async {
    final prefs = await SharedPreferences.getInstance();
    _hosts = prefs.getStringList(_keyHosts) ?? [];
    _joystickRight = prefs.getBool(_keyJoystickRight) ?? false;
  }

  String? get lastHost => _hosts.isNotEmpty ? _hosts.first : null;

  Future<void> rememberHost(String host) async {
    _hosts
      ..removeWhere((h) => h == host)
      ..insert(0, host);
    if (_hosts.length > _maxHosts) {
      _hosts = _hosts.sublist(0, _maxHosts);
    }
    final prefs = await SharedPreferences.getInstance();
    await prefs.setStringList(_keyHosts, _hosts);
    await prefs.setString(_keyLastHost, host);
  }

  Future<void> setJoystickRight(bool right) async {
    _joystickRight = right;
    final prefs = await SharedPreferences.getInstance();
    await prefs.setBool(_keyJoystickRight, right);
  }

  static List<String> decodeHosts(String? raw) =>
      raw == null ? [] : (jsonDecode(raw) as List).cast<String>();
}
