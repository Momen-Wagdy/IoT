import 'package:flutter/material.dart';
import 'package:mqtt_client/mqtt_server_client.dart';
import 'package:ugv_control_app/bluetooth_control.dart';
import 'package:ugv_control_app/camera_view.dart';
import 'package:ugv_control_app/mqtt_data_files.dart';

class UGVHomePage extends StatefulWidget {
  final MqttServerClient client;

  UGVHomePage({required this.client});

  @override
  _UGVHomePageState createState() => _UGVHomePageState();
}

class _UGVHomePageState extends State<UGVHomePage> {
  int _currentIndex = 0;

  late List<Widget> _children;

  @override
  void initState() {
    super.initState();
    _children = [
      MqttDataPage(client: widget.client),
      const UGVBluetoothController(imageUrl: "http://192.168.76.208:19999"),
      const CameraView(imageUrl: "http://192.168.76.208:19999/image/")
    ];
  }

  void onTabTapped(int index) {
    setState(() {
      _currentIndex = index;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: _children[_currentIndex],
      bottomNavigationBar: BottomNavigationBar(
        currentIndex: _currentIndex,
        onTap: onTabTapped,
        items: const [
          BottomNavigationBarItem(
            icon: Icon(Icons.data_usage),
            label: 'MQTT Data',
          ),
          BottomNavigationBarItem(
            icon: Icon(Icons.bluetooth),
            label: 'Bluetooth Control',
          ),
          BottomNavigationBarItem(
            icon: Icon(Icons.camera),
            label: 'Camera Feed',
          ),
        ],
      ),
    );
  }
}
