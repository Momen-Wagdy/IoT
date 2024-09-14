import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';
import 'dart:async';
import 'dart:io';
import 'package:ugv_control_app/custom_button.dart';
import 'camera_view.dart';
import 'package:http/http.dart' as http;

class UGVBluetoothController extends StatefulWidget {
  const UGVBluetoothController({super.key, required this.imageUrl});
  final String imageUrl;
  @override
  State<UGVBluetoothController> createState() => _UGVBluetoothControllerState(this.imageUrl);
}

class _UGVBluetoothControllerState extends State<UGVBluetoothController> {
  Timer? timer;
  late Timer _timer;
  double speed = 150;
  bool pressed = false;
  bool led = false;
  BluetoothConnection? connection;
  BluetoothDevice? connectedDevice;
  String imageUrl = '';
  _UGVBluetoothControllerState(this.imageUrl);

 @override
  void initState() {
    super.initState();
    scanDevices();
    _startImageUpdate();
  }
void newMessage(String suggestion) {
    if (suggestion.isNotEmpty) {
      final parsed = parser(suggestion);
      if (parsed.isNotEmpty){
      setState(() {
        _appendMessage(parsed);
      });}
    }
  }

  void _startImageUpdate() {
    _timer = Timer.periodic(const Duration(milliseconds: 300), (Timer t) async {
      
        var response = await http.get(Uri.parse("$imageUrl/suggest"));
        print(response.body);
        if (response.statusCode == 200) {
          newMessage(response.body);
        }
      
    });
  }
  String parser(String suggestion) {
    final parts = suggestion.split('|');
    if (parts.length == 4) {
      final ls = parts[0];
      final rs = parts[1];
      final led = parts[2];
      final buzzer = parts[3];
      return "Left Motor Speed: $ls, Right Motor Speed: $rs, LED: $led, Buzzer: $buzzer";
    }
    return "";
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
      String finalCommand = '$command';
      print(finalCommand);
      connection!.output.add(Uint8List.fromList(finalCommand.codeUnits));
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
  List<String> messages = [];

  TextEditingController _controller = TextEditingController();

  void _appendMessage(String message) {
    setState(() {
      messages.add(message); // Add new message to the list
    });
    _controller.clear(); // Clear the input field after sending
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
          mainAxisAlignment: MainAxisAlignment.start,
          children: <Widget>[
          
            // Forward Button (Up)
              Padding(
                  padding: const EdgeInsets.all(8.0),
                  child: UGVControlButton(
                    direction: UGVControlDirection.up,
                    color: Colors.blueAccent,
                    onPressed: () {
                      startStream('L');
                      sleep(const Duration(microseconds: 20));
                      sendInstruction('S');
                    },
                    onLongPress: () {
                      pressed = true;
                      startStream('L');
                    },
                    onLongPressEnd: _onLongPressEnd,
                    size: 60,
                    icon: Icons.arrow_upward,
                  ),
                ),
                const SizedBox(height: 15,),
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
                          sendInstruction('B');
                          sleep(const Duration(microseconds: 20));
                          sendInstruction('S');
                        },
                        onLongPress: () {
                          pressed = true;
                          startStream('B');
                        },
                        onLongPressEnd: _onLongPressEnd,
                        size: 60,
                        icon: Icons.arrow_left,
                      ),
                    ),
                    SizedBox(width: 15,),
                    SizedBox(
                      width: 70,
                      height: 70,
                      child: ElevatedButton(
                        onPressed: () {
                          setState(() {
                            if (!led) {
                              sendInstruction('A');
                              led = true;
                            } else {
                              sendInstruction('T');
                              led = false;
                            }
                          });
                        },
                        style: ElevatedButton.styleFrom(
                          backgroundColor: led ? Colors.green : Colors.red,
                          foregroundColor: Colors.blue,
                        ),
                        child: const Text(
                          "LED",
                          style: TextStyle(fontSize: 12),
                        ),
                      ),
                    ),
                    const SizedBox(width: 15),
                    // Right Button
                    Padding(
                      padding: const EdgeInsets.all(8.0),
                      child: UGVControlButton(
                        direction: UGVControlDirection.right,
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
                        size: 60,
                        icon: Icons.arrow_right,
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 15,),
                // Backward Button (Down)
                Padding(
                  padding: const EdgeInsets.all(8.0),
                  child: UGVControlButton(
                    direction: UGVControlDirection.down,
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
                    size: 60,
                    icon: Icons.arrow_downward,
                  ),
                ),
          SizedBox(height: 30),
          Row(children: [
           Transform.rotate(
            angle: 90 * 3.1415926535897932 / 180, // Convert 90 degrees to radians
            child: Container(
              width: 200,
              height: 200,
              color: Color.fromARGB(19, 0, 0, 0),
              child:Column(
        children: [
          Expanded(
            child: ListView.builder(
              itemCount: messages.length,
              itemBuilder: (context, index) {
                return Padding(
                  padding: const EdgeInsets.all(1.0),
                  child: Text(messages[index]),
                );
              },
            ),
          ),
                  ],
      ),
            ),
          ),
           Transform.rotate(
            angle: 90 * 3.1415926535897932 / 180, // Convert 90 degrees to radians
            child: Container(
              width: 150,
              height: 150,
              color: const Color.fromARGB(0, 33, 149, 243),
              child: CameraView(imageUrl: "$imageUrl/image/")
            ),
          ),
          ],)
          
          
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
