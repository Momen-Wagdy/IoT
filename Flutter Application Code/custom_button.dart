import 'package:flutter/material.dart';

enum UGVControlDirection { up, down, left, right }

class UGVControlButton extends StatelessWidget {
  final UGVControlDirection direction;
  final Color color;
  final VoidCallback onPressed;
  final VoidCallback onLongPress;
  final GestureLongPressEndCallback onLongPressEnd;
  final double size;
  final IconData icon;

  const UGVControlButton({
    super.key,
    required this.direction,
    required this.color,
    required this.onPressed,
    required this.onLongPress,
    required this.onLongPressEnd,
    required this.size,
    required this.icon,
  });

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: size,
      height: size,
      child: GestureDetector(
        onTap: onPressed,
        onLongPress: onLongPress,
        onLongPressEnd: onLongPressEnd,
        child: Stack(
          alignment: Alignment.center,
          children: [
            CustomPaint(
              size: Size(size, size),
              painter: UGVControlArrowPainter(direction: direction, color: color),
            ),
            Icon(
              icon,
              color: Colors.white,
              size: size * 0.5,
            ),
          ],
        ),
      ),
    );
  }
}

class UGVControlArrowPainter extends CustomPainter {
  final UGVControlDirection direction;
  final Color color;

  UGVControlArrowPainter({
    required this.direction,
    required this.color,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final Paint paint = Paint()
      ..color = color
      ..style = PaintingStyle.fill;

    final Path path = Path();

    switch (direction) {
      case UGVControlDirection.up:
        path.moveTo(size.width / 2, 0);
        path.lineTo(size.width, size.height);
        path.lineTo(0, size.height);
        path.close();
        break;
      case UGVControlDirection.down:
        path.moveTo(size.width / 2, size.height);
        path.lineTo(size.width, 0);
        path.lineTo(0, 0);
        path.close();
        break;
      case UGVControlDirection.left:
        path.moveTo(0, size.height / 2);
        path.lineTo(size.width, 0);
        path.lineTo(size.width, size.height);
        path.close();
        break;
      case UGVControlDirection.right:
        path.moveTo(size.width, size.height / 2);
        path.lineTo(0, 0);
        path.lineTo(0, size.height);
        path.close();
        break;
    }

    canvas.drawPath(path, paint);
  }

  @override
  bool shouldRepaint(CustomPainter oldDelegate) {
    return oldDelegate is UGVControlArrowPainter &&
        (oldDelegate.direction != direction || oldDelegate.color != color);
  }
}
