import 'package:flutter/material.dart';

/// Battery indicator matching the web UI gauge: voltage + estimated percent
/// for a 2S pack (full 8.4 V, cut-off 6.8 V), color-coded health.
class BatteryGauge extends StatelessWidget {
  const BatteryGauge({super.key, required this.voltage});

  final double voltage;

  static const _full = 8.4;
  static const _low = 6.8;
  static const _warn = 7.6;
  static const _critical = 7.2;

  @override
  Widget build(BuildContext context) {
    final pct = ((voltage - _low) / (_full - _low) * 100)
        .clamp(0.0, 100.0)
        .toDouble();
    final color = voltage <= _critical
        ? const Color(0xFFB3382F)
        : voltage <= _warn
            ? const Color(0xFFD8A12E)
            : const Color(0xFF1F8A4C);
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Container(
          width: 44,
          height: 20,
          padding: const EdgeInsets.all(2),
          decoration: BoxDecoration(
            border: Border.all(color: color, width: 2),
            borderRadius: BorderRadius.circular(5),
          ),
          child: Align(
            alignment: Alignment.centerLeft,
            child: FractionallySizedBox(
              widthFactor: (pct / 100).clamp(0.04, 1.0),
              child: Container(
                decoration: BoxDecoration(
                  color: color,
                  borderRadius: BorderRadius.circular(2),
                ),
              ),
            ),
          ),
        ),
        const SizedBox(width: 4),
        Container(
          width: 3,
          height: 8,
          decoration: BoxDecoration(
            color: color,
            borderRadius: const BorderRadius.horizontal(right: Radius.circular(2)),
          ),
        ),
        const SizedBox(width: 8),
        Text(
          '${voltage.toStringAsFixed(2)} V  ${pct.toStringAsFixed(0)}%',
          style: TextStyle(
            fontSize: 13,
            color: voltage <= _critical ? const Color(0xFFE05A4F) : null,
          ),
        ),
      ],
    );
  }
}
