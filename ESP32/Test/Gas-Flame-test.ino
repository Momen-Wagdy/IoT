// Define the pins for the sensors
const int flameSensorPin = 26;
const int mq2SensorPin = 27;
const int mq9SensorPin = 25;

// Variables to store sensor values
int flameValue = 0;
int mq2Value = 0;
int mq9Value = 0;

void setup() {
  // Initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  
  // Configure the pins as input
  pinMode(flameSensorPin, INPUT);
  pinMode(mq2SensorPin, INPUT);
  pinMode(mq9SensorPin, INPUT);
}

void loop() {
  // Read the analog value from the flame sensor
  flameValue = analogRead(flameSensorPin);
  
  // Read the analog value from the MQ2 sensor
  mq2Value = analogRead(mq2SensorPin);
  
  // Read the analog value from the MQ9 sensor
  mq9Value = analogRead(mq9SensorPin);

  // Print out the sensor values
  Serial.print("Flame Sensor Value: ");
  Serial.println(flameValue);
  
  Serial.print("MQ2 Sensor Value: ");
  Serial.println(mq2Value);
  
  Serial.print("MQ9 Sensor Value: ");
  Serial.println(mq9Value);
  
  // Wait for 1 second before reading again
  delay(1000);
}
