import 'package:flutter/material.dart';

/// Virtual joystick matching the web UI's behaviour: 0.15 dead zone,
/// radial re-scaling. The knob position is driven by the parent via [value]
/// so it always reflects the latest input source (touch, gamepad or keyboard).
class Joystick extends StatelessWidget {
  const Joystick({
    super.key,
    required this.value,
    required this.onChanged,
    this.size = 240,
  });

  /// Current position, normalised -1..1 (y positive = up). The parent owns
  /// this state; the widget never mutates it.
  final Offset value;

  /// Called on every drag update with the dead-zoned, rescaled value.
  final ValueChanged<Offset> onChanged;

  final double size;

  @override
  Widget build(BuildContext context) {
    final active = value != Offset.zero;
    return GestureDetector(
      behavior: HitTestBehavior.opaque,
      onPanDown: (d) => _handleTouch(d.localPosition),
      onPanUpdate: (d) => _handleTouch(d.localPosition),
      onPanEnd: (_) => onChanged(Offset.zero),
      onPanCancel: () => onChanged(Offset.zero),
      child: Container(
        width: size,
        height: size,
        decoration: BoxDecoration(
          color: const Color(0xFF12151A),
          borderRadius: BorderRadius.circular(20),
          border: Border.all(
            color: active ? const Color(0xFF2F6FED) : const Color(0xFF333B45),
            width: 2,
          ),
        ),
        child: Stack(
          children: [
            Center(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Icon(Icons.keyboard_arrow_up,
                      color: Colors.white.withValues(alpha: 0.15)),
                  Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(Icons.keyboard_arrow_left,
                          color: Colors.white.withValues(alpha: 0.15)),
                      const SizedBox(width: 48),
                      Icon(Icons.keyboard_arrow_right,
                          color: Colors.white.withValues(alpha: 0.15)),
                    ],
                  ),
                  Icon(Icons.keyboard_arrow_down,
                      color: Colors.white.withValues(alpha: 0.15)),
                ],
              ),
            ),
            Center(
              child: Transform.translate(
                offset: Offset(value.dx, -value.dy) * (size * 0.30),
                child: Container(
                  width: size * 0.22,
                  height: size * 0.22,
                  decoration: BoxDecoration(
                    color: active
                        ? const Color(0xFF2F6FED)
                        : const Color(0xFF26303A),
                    shape: BoxShape.circle,
                    boxShadow: [
                      BoxShadow(
                        color: Colors.black.withValues(alpha: 0.4),
                        blurRadius: 8,
                      ),
                    ],
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  void _handleTouch(Offset local) {
    final center = Offset(size / 2, size / 2);
    final radius = size / 2;
    var v = (local - center) / radius;
    v = Offset(
      (v.dx * 2).clamp(-1.0, 1.0),
      (-v.dy * 2).clamp(-1.0, 1.0),
    );
    const dead = 0.15;
    final m = v.distance;
    if (m < dead) {
      onChanged(Offset.zero);
      return;
    }
    final scaled = (m - dead) / (1 - dead) / m;
    onChanged(Offset(v.dx * scaled, v.dy * scaled));
  }
}
