#include <BluetoothSerial.h>  // Add the missing #

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
int IR_F = 5;  // Replace with actual GPIO pin numbers for the front IR sensor
int IR_B = 18; // Replace with actual GPIO pin numbers for the back IR sensor

// Flag to interrupt motor control
volatile bool interruptFlag = false;

void setup() {
  Serial.begin(115200);

  if (!SerialBT.begin("esp32")) {  // Check if Bluetooth starts properly
    Serial.println("An error occurred initializing Bluetooth");
  } else {
    Serial.println("Bluetooth initialized. Device is ready to pair.");
  }

  // Initialize pins for motors
  pinMode(in1_pin, OUTPUT);
  pinMode(in2_pin, OUTPUT);
  pinMode(en1_pin, OUTPUT);

  pinMode(in3_pin, OUTPUT);
  pinMode(in4_pin, OUTPUT);
  pinMode(en2_pin, OUTPUT);

  digitalWrite(en1_pin, LOW);
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
  if (SerialBT.available()) {
    char command = SerialBT.read();
    Serial.println(command);

    if (interruptFlag) {
      stopMotors(); // Stop motors if interrupt flag is set
      interruptFlag = false; // Clear the flag
    }

    switch (command) {
      case 'F': // Move forward
        moveForward(255);
        break;
      case 'B': // Move backward
        moveBackward(255);
        break;
      case 'L': // Turn left
        turnLeft(255);
        break;
      case 'R': // Turn right
        turnRight(255);
        break;
      case 'S': // Stop
        stopMotors();
        break;
      default:
        stopMotors();
        break;
    }
  }
}

void moveForward(int intensity) {
  // Motor A forward
  digitalWrite(in1_pin, HIGH);
  digitalWrite(in2_pin, LOW);
  analogWrite(en1_pin, intensity); // Full speed

  // Motor B forward
  digitalWrite(in3_pin, HIGH);
  digitalWrite(in4_pin, LOW);
  analogWrite(en2_pin, intensity); // Full speed
}

void moveBackward(int intensity) {
  // Motor A backward
  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, HIGH);
  analogWrite(en1_pin, intensity); // Full speed

  // Motor B backward
  digitalWrite(in3_pin, LOW);
  digitalWrite(in4_pin, HIGH);
  analogWrite(en2_pin, intensity); // Full speed
}

void turnLeft(int intensity) {
  // Motor A stop
  analogWrite(en1_pin, intensity);
  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, HIGH);

  // Motor B forward
  digitalWrite(in3_pin, HIGH);
  digitalWrite(in4_pin, LOW);
  analogWrite(en2_pin, intensity); // Full speed
}

void turnRight(int intensity) {
  // Motor B stop
  analogWrite(en2_pin, intensity);
  digitalWrite(in3_pin, LOW);
  digitalWrite(in4_pin, HIGH);

  // Motor A forward
  digitalWrite(in1_pin, HIGH);
  digitalWrite(in2_pin, LOW);
  analogWrite(en1_pin, intensity); // Full speed
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
    moveBackward(111);
  } else if (digitalRead(IR_F) == HIGH && digitalRead(IR_B) == LOW) {
    moveForward(111);
  }
}
