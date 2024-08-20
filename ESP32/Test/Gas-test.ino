
// Define the analog pin connected to the MQ-9 sensor
const int mq9_pin = 26;  // Analog pin A0 on the Arduino

void setup() {
  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);

  // Setup the analog pin (no need for width/attenuation on Arduino)
  pinMode(mq9_pin, INPUT);
}

int read_gas() {
  int gas_value = analogRead(mq9_pin);  // Read the analog value from the MQ-9 sensor
  return gas_value;
}

void loop() {
  int gas_level = read_gas();
  Serial.print("Gas Level: ");
  Serial.println(gas_level);
  
  // Trigger actions based on the gas level
  if (gas_level > 2000) {  // Example threshold, adjust based on your needs
    Serial.println("Warning: High gas concentration detected!");
  }
  
  delay(1000);  // Delay for 1 second before the next reading
}
