import 'package:flutter/material.dart';
import 'package:flutter_blue/flutter_blue.dart';
import 'dart:async';
import 'dart:io';

import 'package:ugv_control_app/custom_button.dart';

class UGVBluetoothController extends StatefulWidget {
  const UGVBluetoothController({super.key});

  @override
  State<UGVBluetoothController> createState() => _UGVBluetoothControllerState();
}

class _UGVBluetoothControllerState extends State<UGVBluetoothController> {
  Timer? timer;
  bool pressed = false;
  BluetoothCharacteristic? movementCharacteristic;
  BluetoothDevice? connectedDevice;

  @override
  void initState() {
    super.initState();
    scanDevices();
  }

  void scanDevices() async {
    FlutterBlue flutterBlue = FlutterBlue.instance;
    flutterBlue.startScan();
    flutterBlue.scanResults.listen((results) {
      for (ScanResult r in results) {
        if (r.device.name == 'esp32') {
          connectDevice(r.device);
          flutterBlue.stopScan();
          break;
        }
      }
    });
  }

  void connectDevice(BluetoothDevice device) async {
    await device.connect();
    setState(() {
      connectedDevice = device;
    });
  }

  void sendInstruction(String command) async {
    if (movementCharacteristic != null) {
      await movementCharacteristic!.write(command.codeUnits);
    }
  }

  void _onLongPressEnd(LongPressEndDetails details) {
    pressed = false;
    timer?.cancel();
    sendInstruction('S');
  }

  void startStream(String command) {
    timer?.cancel();
    timer = Timer.periodic(const Duration(milliseconds: 50), (timer) {
      if (pressed) {
        sendInstruction(command);
      } else {
        timer.cancel();
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    return Scaffold(
      appBar: AppBar(
        title: const Text('UGV Bluetooth Controller'),
        centerTitle: true,
        backgroundColor: Colors.cyan,
      ),
      backgroundColor: theme.colorScheme.inversePrimary,
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            // Forward Button (Up)
            Padding(
              padding: const EdgeInsets.all(8.0),
              child: UGVControlButton(
                direction: UGVControlDirection.up,
                color: Colors.blueAccent,
                onPressed: () {
                  sendInstruction('F');
                  sleep(const Duration(microseconds: 20));
                  sendInstruction('S');
                },
                onLongPress: () {
                  pressed = true;
                  startStream('F');
                },
                onLongPressEnd: _onLongPressEnd,
                size: 100,
                icon: Icons.arrow_upward,
              ),
            ),
            // Middle Row with Left and Right Buttons
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                // Left Button
                Padding(
                  padding: const EdgeInsets.all(8.0),
                  child: UGVControlButton(
                    direction: UGVControlDirection.left,
                    color: Colors.blueAccent,
                    onPressed: () {
                      sendInstruction('L');
                      sleep(const Duration(microseconds: 20));
                      sendInstruction('S');
                    },
                    onLongPress: () {
                      pressed = true;
                      startStream('L');
                    },
                    onLongPressEnd: _onLongPressEnd,
                    size: 100,
                    icon: Icons.arrow_left,
                  ),
                ),
                const SizedBox(width: 130),
                // Right Button
                Padding(
                  padding: const EdgeInsets.all(8.0),
                  child: UGVControlButton(
                    direction: UGVControlDirection.right,
                    color: Colors.blueAccent,
                    onPressed: () {
                      sendInstruction('R');
                      sleep(const Duration(microseconds: 20));
                      sendInstruction('S');
                    },
                    onLongPress: () {
                      pressed = true;
                      startStream('R');
                    },
                    onLongPressEnd: _onLongPressEnd,
                    size: 100,
                    icon: Icons.arrow_right,
                  ),
                ),
              ],
            ),
            // Backward Button (Down)
            Padding(
              padding: const EdgeInsets.all(8.0),
              child: UGVControlButton(
                direction: UGVControlDirection.down,
                color: Colors.blueAccent,
                onPressed: () {
                  sendInstruction('B');
                  sleep(const Duration(microseconds: 20));
                  sendInstruction('S');
                },
                onLongPress: () {
                  pressed = true;
                  startStream('B');
                },
                onLongPressEnd: _onLongPressEnd,
                size: 100,
                icon: Icons.arrow_downward,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
