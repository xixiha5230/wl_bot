import 'package:flutter/material.dart';

/// Virtual joystick matching the web UI's behaviour: 0.15 dead zone,
/// radial re-scaling, spring back to center with zeroed output on release.
class Joystick extends StatefulWidget {
  const Joystick({
    super.key,
    required this.onChanged,
    this.size = 240,
  });

  final ValueChanged<Offset> onChanged;
  final double size;

  @override
  State<Joystick> createState() => _JoystickState();
}

class _JoystickState extends State<Joystick> {
  Offset _knob = Offset.zero; // normalized -1..1, y up

  void _update(Offset normalized) {
    setState(() => _knob = normalized);
    widget.onChanged(normalized);
  }

  void _fromLocalPosition(Offset local) {
    final center = Offset(widget.size / 2, widget.size / 2);
    final radius = widget.size / 2;
    var v = (local - center) / radius;
    v = Offset(
      (v.dx * 2).clamp(-1.0, 1.0),
      (-v.dy * 2).clamp(-1.0, 1.0),
    );
    const dead = 0.15;
    final m = v.distance;
    if (m < dead) {
      _update(Offset.zero);
      return;
    }
    final scaled = (m - dead) / (1 - dead) / m;
    _update(Offset(v.dx * scaled, v.dy * scaled));
  }

  @override
  Widget build(BuildContext context) {
    final active = _knob != Offset.zero;
    return GestureDetector(
      behavior: HitTestBehavior.opaque,
      onPanDown: (DragDownDetails d) => _fromLocalPosition(d.localPosition),
      onPanUpdate: (DragUpdateDetails d) => _fromLocalPosition(d.localPosition),
      onPanEnd: (DragEndDetails d) => _update(Offset.zero),
      onPanCancel: () => _update(Offset.zero),
      child: Container(
        width: widget.size,
        height: widget.size,
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
                offset: Offset(_knob.dx, -_knob.dy) * (widget.size * 0.30),
                child: Container(
                  width: widget.size * 0.22,
                  height: widget.size * 0.22,
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
}
