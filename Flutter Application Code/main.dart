import 'package:flutter/material.dart';
import 'package:mqtt_client/mqtt_client.dart';
import 'package:mqtt_client/mqtt_server_client.dart';
import 'package:firebase_core/firebase_core.dart';

import 'dart:io';
import 'package:flutter/services.dart';
import 'package:ugv_control_app/login.dart';


void main() async {
  WidgetsFlutterBinding.ensureInitialized(); // Ensure Flutter bindings are initialized
  await Firebase.initializeApp();
  
  MqttServerClient client = MqttServerClient('bae6e1004af84917878e457c24d59cce.s1.eu.hivemq.cloud', 'id122');
  runApp(MyApp(client: client,));

  await connectToMqtt(client);
}


Future<void> connectToMqtt(MqttServerClient client) async {
  // Set port for TLS/SSL
  client.port = 8883;

  // Set logging for debugging
  client.logging(on: true);

  // Keep-alive period
  client.keepAlivePeriod = 60;

  // Set the security context with SSL/TLS
  client.secure = true;
  client.securityContext = SecurityContext.defaultContext;

  // Load the CA certificate if needed (Optional, depending on broker requirements)
  // This is necessary if the broker uses a self-signed certificate.
  try {
    final byteData = await rootBundle.load('assets/ca.crt');
    client.securityContext.setTrustedCertificatesBytes(byteData.buffer.asUint8List());
  } catch (e) {
    print('Failed to load CA certificate: $e');
  }

  // Set connection message with credentials
  final connMessage = MqttConnectMessage()
      .withClientIdentifier('dart_client')
      .authenticateAs("esp32", 'esppass')
      .withWillTopic('willtopic')
      .withWillMessage('My Will message')
      .startClean()
      .withWillQos(MqttQos.atLeastOnce);

  client.connectionMessage = connMessage;
  client.onDisconnected = onDisconnected;
  client.onConnected = onConnected;

  // Attempt to connect
  try {
    await client.connect();
  } catch (e) {
    print('Exception: $e');
    client.disconnect();
  }

  // Check connection status
  if (client.connectionStatus!.state == MqttConnectionState.connected) {
    print('MQTT client connected');
  } else {
    print('ERROR: MQTT client connection failed - ${client.connectionStatus}');
    client.disconnect();
  }
}
void onConnected() {
  print('Connected');
}

void onDisconnected() {
  print('Disconnected');
}
class MyApp extends StatelessWidget {
  MqttServerClient? client;
  MyApp({super.key, this.client});

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Demo',
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.deepPurple),
        useMaterial3: true,
      ),
      home:LoginPage(client: client,),
    );
  }
}

