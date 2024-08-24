#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

// Motor A
int in1_pin = 2;  // Update these pins to valid GPIOs
int in2_pin = 0;
int en1_pin = 15;
// Motor B
int in3_pin = 4;
int in4_pin = 16;
int en2_pin = 17;

// IR front and back word
int IR_F = 5;  // ADD BINS
int IR_B = 18;

// Flag to interrupt motor control
volatile bool interruptFlag = false;

// initial values for output speed for each motor and direction
float speed_left = 0;
float speed_right = 0;
float direction = 0;

// Define the pin for the IR sensor for each wheel
const int leftIRSensorPin = 2;   // IR sensor pin for the left wheel
const int rightIRSensorPin = 3;  // IR sensor pin for the right wheel

// Define constants for the wheel and tick counting
const float wheelRadius = 6.5;     // Radius of the wheel in meters (e.g., 10 cm)
const int ticksPerRevolution = 8;  // Number of ticks per revolution (depends on the number of markers)

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

  return distance;  // Return distance in meters
}

// fuzzy interference system
void FIS(float orientation, float distance, float &speed_left, float &speed_right) {
  // fuzzification of orientation and distance
  float right = triangular_MF(orientation, -320 - 1e-10, -320, -50);
  float center = triangular_MF(orientation, -75, 75);
  float left = triangular_MF(orientation, 50, 320, 320 + 1e-10);
  float close = triangular_MF(distance, -1e-10, 0, 25);
  float medium = triangular_MF(distance, 20, 50);
  float far = triangular_MF(distance, 50, 90, 90 + 1e-10);

  // initializing of output possibilities
  float speed_left_very_low = 0;
  float speed_left_low = 0;
  float speed_left_medium = 0;
  float speed_left_high = 0;
  float speed_right_very_low = 0;
  float speed_right_low = 0;
  float speed_right_medium = 0;
  float speed_right_high = 0;
  float direction_left = 0;
  float direction_center = 0;
  float direction_right = 0;

  // applying the fuzzy rules
  float rule1 = min(right, far);
  speed_left_medium = max(speed_left_medium, rule1);
  speed_right_high = max(speed_right_high, rule1);
  direction_right = max(direction_right, rule1);

  float rule2 = min(left, far);
  speed_left_high = max(speed_left_high, rule2);
  speed_right_medium = max(speed_right_medium, rule2);
  direction_left = max(direction_left, rule2);

  float rule3 = min(right, medium);
  speed_left_low = max(speed_left_low, rule3);
  speed_right_medium = max(speed_right_medium, rule3);
  direction_right = max(direction_right, rule3);

  float rule4 = min(left, medium);
  speed_left_medium = max(speed_left_medium, rule4);
  speed_right_low = max(speed_right_low, rule4);
  direction_left = max(direction_left, rule4);

  float rule5 = min(right, close);
  speed_left_very_low = max(speed_left_very_low, rule5);
  speed_right_low = max(speed_right_low, rule5);
  direction_right = max(direction_right, rule5);

  float rule6 = min(left, close);
  speed_left_low = max(speed_left_low, rule6);
  speed_right_very_low = max(speed_right_very_low, rule6);
  direction_left = max(direction_left, rule6);

  float rule7 = min(center, far);
  speed_left_high = max(speed_left_high, rule7);
  speed_right_high = max(speed_right_high, rule7);
  direction_center = max(direction_center, rule7);

  float rule8 = min(center, medium);
  speed_left_medium = max(speed_left_medium, rule8);
  speed_right_medium = max(speed_right_medium, rule8);
  direction_center = max(direction_center, rule8);

  float rule9 = min(center, close);
  speed_left_low = max(speed_left_low, rule9);
  speed_right_low = max(speed_right_low, rule9);
  direction_center = max(direction_center, rule9);

  // using defuzzification to get the crisp output for motors' speed and direction
  speed_left = defuzzification(speed_left_very_low, speed_left_low, speed_left_medium, speed_left_high);
  speed_right = defuzzification(speed_right_very_low, speed_right_low, speed_right_medium, speed_right_high);
  direction = defuzzification(direction_left, direction_center, direction_right);
}

// triangular membership function
float triangular_MF(float x, float a, float c) {
  float b = (a + c) / 2;
  float first_term = (x-a) / (b-a);
  float second_term = (c-x) / (c-b);
  float inner_term = first_term > 1? 1: first_term > second_term? second_term:first_term;
  float final_membership = inner_term > 0? inner_term:0;
  return final_membership;
}
// triangular membership function
float triangular_MF(float x, float a, float b, float c) {  
  float first_term = (x-a) / (b-a);
  float second_term = (c-x) / (c-b);
  float inner_term = first_term > 1? 1: first_term > second_term? second_term:first_term;
  float final_membership = inner_term > 0? inner_term:0;
  return final_membership;
}

// defuzzification using centroid method
float defuzzification(float low, float medium, float high) {
  float low_a = 111-1e-10;
  float low_b = 111;
  float low_c = 155;
  float med_a = 140;
  float med_b = 167.5;
  float med_c = 195;
  float high_a = 180;
  float high_b = 255;
  float high_c = 255+1+e-10;

  float low1 = low*(low_b - low_a) + low_a;
  float low2 = low_c - low*(low_c - low_b);

  float med1 = med*(med_b - med_a) + med_a;
  float med2 = med_c - med*(med_c - med_b);

  float high1 = high*(high_b - high_a) + high_a;
  float high2 = high_c - high*(high_c - high_b);

  return (low1*low + low2*low + med1*med + med2*med + high1*high + high2*high) / (2*(low+med+high));
}

void moveForward(int intensity_A, int intensity_B) {
  // Motor A forward
  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, HIGH);
  analogWrite(en1_pin, intensity_A);  // Full speed

  // Motor B forward
  digitalWrite(in3_pin, LOW);
  digitalWrite(in4_pin, HIGH);
  analogWrite(en2_pin, intensity_B);  // Full speed
}

void moveBackward(int intensity_A, int intensity_B) {
  // Motor A backward
  digitalWrite(in1_pin, HIGH);
  digitalWrite(in2_pin, LOW);
  analogWrite(en1_pin, intensity_A);  // Full speed

  // Motor B backward
  digitalWrite(in3_pin, HIGH);
  digitalWrite(in4_pin, LOW);
  analogWrite(en2_pin, intensity_B);  // Full speed
}

void turnLeft(int intensity_A, int intensity_B) {
  // Motor A stop
  analogWrite(en1_pin, intensity_A);
  digitalWrite(in1_pin, HIGH);
  digitalWrite(in2_pin, LOW);

  // Motor B forward
  digitalWrite(in3_pin, LOW);
  digitalWrite(in4_pin, HIGH);
  analogWrite(en2_pin, intensity_B);  // Full speed
}

void turnRight(int intensity_A, int intensity_B) {
  // Motor B stop
  analogWrite(en2_pin, intensity_B);
  digitalWrite(in3_pin, HIGH);
  digitalWrite(in4_pin, LOW);

  // Motor A forward
  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, HIGH);
  analogWrite(en1_pin, intensity_A);  // Full speed
}

void stopMotors() {
  // Motor A stop
  analogWrite(en1_pin, 0);

  // Motor B stop
  analogWrite(en2_pin, 0);
}

// Function that runs automatically when detecting any IR sensor going low
void onSensorChange() {
  interruptFlag = true;
  if (digitalRead(IR_F) == LOW && digitalRead(IR_B) == HIGH) {
    moveBackward(111, 111);
  } else if (digitalRead(IR_F) == HIGH && digitalRead(IR_B) == LOW) {
    moveForward(111, 111);
  }
}

void setup() {
  Serial.begin(115200);

  if (!SerialBT.begin("esp32")) {  // Check if Bluetooth starts properly
    Serial.println("An error occurred initializing Bluetooth");
  } else {
    Serial.println("Bluetooth initialized. Device is ready to pair.");
  }

  // Set IR sensor pins as input
  pinMode(leftIRSensorPin, INPUT);
  pinMode(rightIRSensorPin, INPUT);

  // Attach interrupts to the IR sensor pins
  attachInterrupt(digitalPinToInterrupt(leftIRSensorPin), leftWheelTick, RISING);    // Detect rising edge for the left wheel
  attachInterrupt(digitalPinToInterrupt(rightIRSensorPin), rightWheelTick, RISING);  // Detect rising edge for the right wheel

  // Initialize pins for motors
  pinMode(in1_pin, OUTPUT);
  pinMode(in2_pin, OUTPUT);
  pinMode(en1_pin, OUTPUT);

  pinMode(in3_pin, OUTPUT);
  pinMode(in4_pin, OUTPUT);
  pinMode(en2_pin, OUTPUT);

  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, LOW);
  digitalWrite(en1_pin, LOW);
  digitalWrite(in3_pin, LOW);
  digitalWrite(in4_pin, LOW);
  digitalWrite(en2_pin, LOW);

  Serial.println("Setup complete.");

  // Initialize IR sensor pins
  pinMode(IR_F, INPUT);
  pinMode(IR_B, INPUT);

  // Attach interrupts to IR sensors
  attachInterrupt(digitalPinToInterrupt(IR_F), onSensorChange, FALLING);
  attachInterrupt(digitalPinToInterrupt(IR_B), onSensorChange, FALLING);
}

void loop() {
  if (SerialBT.hasClient()) {
    Serial.println("a device is paired");

    // if there is a paired device, move using manual instructions

    if (Serial.available()) {
      SerialBT.write(Serial.read());
    }

    if (SerialBT.available()) {
      Serial.write(SerialBT.read());
      delay(5);
      char command = (char)SerialBT.read();
      Serial.println(command);

      if (interruptFlag) {
        stopMotors();           // Stop motors if interrupt flag is set
        interruptFlag = false;  // Clear the flag
      }

      switch (command) {
        case 'F':  // Move forward
          moveForward(111, 111);
          break;
        case 'B':  // Move backward
          moveBackward(111, 111);
          break;
        case 'L':  // Turn left
          turnLeft(111, 111);
          break;
        case 'R':  // Turn right
          turnRight(111, 111);
          break;
        case 'S':  // Stop
          stopMotors();
          break;
        default:
          stopMotors();
          break;
      }
    }
  } else {
    Serial.println("No device is connected.");

    if (interruptFlag) {
      stopMotors();           // Stop motors if interrupt flag is set
      interruptFlag = false;  // Clear the flag
    }
    // get orientation and distance values from cam
    float orientation = 0;
    float distance = 0;

    // if there is no paired device, work using fuzzy inference system
    FIS(orientation, distance, speed_left, speed_right);

    if (distance < 245) {
      turnLeft(speed_left, speed_right);
    } else if (distance > 395) {
      turnRight(speed_left, speed_right);
    } else {
      moveForward(speed_left, speed_right);
    }
  }

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
}
