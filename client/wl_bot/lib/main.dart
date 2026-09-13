import 'package:flutter/material.dart';

import 'screens/drive_screen.dart';
import 'services/settings_store.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  final settings = SettingsStore();
  await settings.load();
  runApp(WlBotApp(settings: settings));
}

const wlBotGreen = Color(0xFF1F8A4C);
const wlBotBlue = Color(0xFF2F6FED);

class WlBotApp extends StatelessWidget {
  const WlBotApp({super.key, required this.settings});

  final SettingsStore settings;

  @override
  Widget build(BuildContext context) {
    const seed = Color(0xFF2F6FED);
    return MaterialApp(
      title: 'WLROBOT',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        useMaterial3: true,
        brightness: Brightness.dark,
        colorScheme: ColorScheme.fromSeed(
          seedColor: seed,
          brightness: Brightness.dark,
        ),
        scaffoldBackgroundColor: const Color(0xFF14171C),
        cardTheme: CardThemeData(
          color: const Color(0xFF1D2128),
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(12),
            side: const BorderSide(color: Color(0xFF2B313A)),
          ),
        ),
        inputDecorationTheme: InputDecorationTheme(
          filled: true,
          fillColor: const Color(0xFF12151A),
          border: OutlineInputBorder(
            borderRadius: BorderRadius.circular(8),
            borderSide: const BorderSide(color: Color(0xFF333B45)),
          ),
        ),
      ),
      home: DriveScreen(settings: settings),
    );
  }
}
