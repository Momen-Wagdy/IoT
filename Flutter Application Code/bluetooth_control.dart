import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';
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
  BluetoothConnection? connection;
  BluetoothDevice? connectedDevice;

  @override
  void initState() {
    super.initState();
    scanDevices();
  }

  void scanDevices() async {
    // Request paired devices, then select one to connect
    List<BluetoothDevice> devices = await FlutterBluetoothSerial.instance.getBondedDevices();

    for (BluetoothDevice device in devices) {
      print("${device.address} ${device.name}");
      if (device.address == 'EC:64:C9:87:36:86') { // Replace with your ESP32 MAC address
        print("Connecting..");
        connectDevice(device);
        break;
      }
    }
  }

  void connectDevice(BluetoothDevice device) async {
    try {
      // Establish Bluetooth connection
      await BluetoothConnection.toAddress(device.address).then((_connection) {
        print('Connected to the device');
        setState(() {
          connectedDevice = device;
          connection = _connection;
        });

        connection!.input!.listen(_onDataReceived).onDone(() {
          print('Disconnected by remote request');
        });
      });
    } catch (e) {
      print('Could not connect to the device: $e');
    }
  }

  // Send data to the connected Bluetooth device
  void sendInstruction(String command) async {
    if (connection != null && connection!.isConnected) {
      connection!.output.add(Uint8List.fromList(command.codeUnits));
      await connection!.output.allSent;
    }
  }

  // Handle data received from the Bluetooth device
  void _onDataReceived(Uint8List data) {
    print('Data received: $data');
  }

  void _onLongPressEnd(LongPressEndDetails details) {
    pressed = false;
    timer?.cancel();
    sendInstruction('S');
  }

  void startStream(String command) {
    timer?.cancel();
    timer = Timer.periodic(const Duration(milliseconds: 1), (timer) {
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
                  sleep(const Duration(microseconds: 10));
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
                const SizedBox(width: 120),
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

  @override
  void dispose() {
    connection?.dispose();
    connection = null;
    super.dispose();
  }
}
