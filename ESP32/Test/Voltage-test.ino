int current_sensor_pin = 0;
float sensitivity = 66;
int VCC = 5;

void setup() {
  Serial.begin(115200);
  pinMode(current_sensor_pin, INPUT);
}

void loop() {
  int reading = analogRead(current_sensor_pin);

  float voltage = reading * (VCC / 4096.0);
  voltage = reading*(30000+7500) / 7500;
  float current = (voltage - 25) / sensitivity;
  Serial.printf("Voltage: %0.2fV\n",voltage);
  Serial.printf("Current: %0.2fA\n", current);
  delay(1000);
}
