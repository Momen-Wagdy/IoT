import 'package:flutter/material.dart';
import 'dart:async';

class CameraView extends StatefulWidget {
  final String imageUrl;

  const CameraView({Key? key, required this.imageUrl}) : super(key: key);

  @override
  _CameraViewState createState() => _CameraViewState();
}

class _CameraViewState extends State<CameraView> {
  late String _currentImageUrl;
  late Timer _timer;

  @override
  void initState() {
    super.initState();
    _currentImageUrl = widget.imageUrl;
    _startImageUpdate();
  }

  void _startImageUpdate() {
    _timer = Timer.periodic(Duration(milliseconds: 300), (Timer t) {
      setState(() {
        _currentImageUrl = widget.imageUrl + '${DateTime.now().millisecondsSinceEpoch}';
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Camera View'),
      ),
      body: Center(
        child: Image.network(
          _currentImageUrl,
          key: ValueKey(_currentImageUrl), // Ensure the image updates
          loadingBuilder: (BuildContext context, Widget child, ImageChunkEvent? loadingProgress) {
            if (loadingProgress == null) {
              return child;
            } else {
              return Center(
                child: CircularProgressIndicator(
                  value: loadingProgress.expectedTotalBytes != null
                      ? loadingProgress.cumulativeBytesLoaded / (loadingProgress.expectedTotalBytes ?? 1)
                      : null,
                ),
              );
            }
          },
          errorBuilder: (BuildContext context, Object error, StackTrace? stackTrace) {
            return Icon(Icons.error);
          },
        ),
      ),
    );
  }

  @override
  void dispose() {
    _timer.cancel();
    super.dispose();
  }
}
