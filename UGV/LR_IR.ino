// Define the pin for the IR sensor for each wheel
const int leftIRSensorPin = 2; // IR sensor pin for the left wheel
const int rightIRSensorPin = 3; // IR sensor pin for the right wheel

// Define constants for the wheel and tick counting
const float wheelRadius = 0.1;  // Radius of the wheel in meters (e.g., 10 cm)
const int ticksPerRevolution = 8; // Number of ticks per revolution (depends on the number of markers)

// Variables to keep track of ticks for each wheel
volatile long leftWheelTicks = 0;
volatile long rightWheelTicks = 0;

// Interrupt Service Routine (ISR) for left wheel IR sensor
void leftWheelTick() {
  leftWheelTicks++;
}

// Interrupt Service Routine (ISR) for right wheel IR sensor
void rightWheelTick() {
  rightWheelTicks++;
}

// Function to calculate distance traveled
float getDistance(long ticks, float wheelRadius, int ticksPerRevolution) {
  // Calculate the circumference of the wheel
  float wheelCircumference = 2 * PI * wheelRadius;

  // Calculate distance traveled
  float distance = (float(ticks) / ticksPerRevolution) * wheelCircumference;

  return distance; // Return distance in meters
}

void setup() {
  Serial.begin(9600);  // Start serial communication

  // Set IR sensor pins as input
  pinMode(leftIRSensorPin, INPUT);
  pinMode(rightIRSensorPin, INPUT);

  // Attach interrupts to the IR sensor pins
  attachInterrupt(digitalPinToInterrupt(leftIRSensorPin), leftWheelTick, RISING);  // Detect rising edge for the left wheel
  attachInterrupt(digitalPinToInterrupt(rightIRSensorPin), rightWheelTick, RISING);  // Detect rising edge for the right wheel
}

void loop() {
  // Calculate the distances traveled by each wheel
  float leftWheelDistance = getDistance(leftWheelTicks, wheelRadius, ticksPerRevolution);
  float rightWheelDistance = getDistance(rightWheelTicks, wheelRadius, ticksPerRevolution);
  
  // Print the distances to the Serial Monitor
  Serial.print("Left Wheel Distance: ");
  Serial.print(leftWheelDistance);
  Serial.println(" meters");

  Serial.print("Right Wheel Distance: ");
  Serial.print(rightWheelDistance);
  Serial.println(" meters");

  // Optionally reset ticks if needed to calculate the next interval's distance
  leftWheelTicks = 0;
  rightWheelTicks = 0;

  delay(1000);  // Delay for readability
}
