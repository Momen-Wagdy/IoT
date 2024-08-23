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
int IR_F = 0 ; // ADD BINS
int IR_B = 0 ;

// Flag to interrupt motor control
volatile bool interruptFlag = false;

float speed_left = 0;
float speed_right = 0;

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

// Function the run automatic when detecting any ir sensor going low 
void onSensorChange() {
 if (Front_sensor == LOW && Back_sensor == High) {
   moveBackward(111);
  } else if (Front_sensor == HIGH && Back_sensor == LOW) {
   moveForword(111);
  }
  }

void FIS(float orientation, float distance, float &speed_left, float &speed_right) {
  float right = triangular_MF(orientation, -320, -50);
  float center = triangular_MF(orientation, -75, 75);
  float left = triangular_MF(orientation, 50, 320);
  float close = triangular_MF(distance, 0, 25);
  float medium = triangular_MF(distance, 20, 50);
  float far = triangular_MF(distance, 50, 90);

  float speed_left_low = 0;
  float speed_left_medium = 0;
  float speed_left_high = 0;
  float speed_right_low = 0;
  float speed_right_medium = 0;
  float speed_right_high = 0;

  float rule1 = min(right, far);
  speed_left_medium = max(speed_left_medium, rule1);
  speed_right_high = max(speed_right_high, rule1);

  float rule2 = min(left, far);
  speed_left_high = max(speed_left_high, rule2);
  speed_right_medium = max(speed_right_medium, rule2);

  // rule 3
  speed_left_high = max(speed_left_high, far);
  speed_right_high = max(speed_right_high, far);

  // rule 4
  speed_left_medium = max(speed_left_medium, medium);
  speed_right_medium = max(speed_right_medium, medium);
  
  // rule 5
  speed_left_low = max(speed_left_low, close);
  speed_right_low = max(speed_right_low, close);

  speed_left = defuzzification(speed_left_low, speed_left_medium, speed_left_high);
  speed_right = defuzzification(speed_right_low, speed_right_medium, speed_right_high);
}

float triangular_MF(float x, float a, float c) {
  float b = (a + c)/2;
  if (x <= a) {
    return 0;
  } else if (a <= x && x <= b) {
    return (x - a)/(b - a);
  } else if (b <= x && x <= c) {
    return (c - x)/(c - b);
  } else {
    return 0;
  }
}

float defuzzification(float low, float medium, float high) {
  if (low + medium + high == 0) {
    return 0;
  } else {
    return ((low * 45) + (medium * 122.5) + (high * 202.5))/(low + medium + high);
  }
}