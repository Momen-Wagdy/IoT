import 'package:flutter/material.dart';
import 'package:mqtt_client/mqtt_client.dart';
import 'package:mqtt_client/mqtt_server_client.dart';

class MqttDataPage extends StatefulWidget {
  final MqttServerClient client;

  const MqttDataPage({Key? key, required this.client}) : super(key: key);

  @override
  _MqttDataPageState createState() => _MqttDataPageState();
}

class _MqttDataPageState extends State<MqttDataPage> {
  String rightFuzzyData = "Waiting for data...";
  String leftFuzzyData = "Waiting for data...";
  String distanceData = "Waiting for data...";
  String orientationData = "Waiting for data...";

  @override
  void initState() {
    super.initState();
    _subscribeToTopics();
  }

  void _subscribeToTopics() {
    const String rightFuzzyTopic = "UGV/RF";
    const String leftFuzzyTopic = "UGV/LF";
    const String distanceTopic = "UGV/Distance";
    const String orientationTopic = "UGV/Orient";

    widget.client.subscribe(rightFuzzyTopic, MqttQos.atMostOnce);
    widget.client.subscribe(leftFuzzyTopic, MqttQos.atMostOnce);
    widget.client.subscribe(distanceTopic, MqttQos.atMostOnce);
    widget.client.subscribe(orientationTopic, MqttQos.atMostOnce);

    widget.client.updates?.listen((List<MqttReceivedMessage<MqttMessage?>>? c) {
      final MqttPublishMessage recMess = c![0].payload as MqttPublishMessage;
      final String pt =
          MqttPublishPayload.bytesToStringAsString(recMess.payload.message);

      setState(() {
        switch (c[0].topic) {
          case rightFuzzyTopic:
            rightFuzzyData = pt;
            break;
          case leftFuzzyTopic:
            leftFuzzyData = pt;
            break;
          case distanceTopic:
            distanceData = pt;
            break;
          case orientationTopic:
            orientationData = pt;
            break;
        }
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('MQTT Data'),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text('Right Fuzzy: $rightFuzzyData'),
            const SizedBox(height: 8),
            Text('Left Fuzzy: $leftFuzzyData'),
            const SizedBox(height: 8),
            Text('Distance: $distanceData'),
            const SizedBox(height: 8),
            Text('Orientation: $orientationData'),
          ],
        ),
      ),
    );
  }

  @override
  void dispose() {
    const String rightFuzzyTopic = "UGV/RF";
    const String leftFuzzyTopic = "UGV/LF";
    const String distanceTopic = "UGV/Distance";
    const String orientationTopic = "UGV/Orient";
    widget.client.unsubscribe(rightFuzzyTopic);
    widget.client.unsubscribe(leftFuzzyTopic);
    widget.client.unsubscribe(distanceTopic);
    widget.client.unsubscribe(orientationTopic);
    super.dispose();
  }
}
