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

void FIS(float orientation, float distance) {
    // Fuzzification of orientation and distance
    float right = trapizoidal_MF(orientation, 0, 1, 120, 240);
    float center = triangular_MF(orientation, 230, 410);
    float left = trapizoidal_MF(orientation, 390,480, 640, 641);
    float close = trapizoidal_MF(distance, 0., 0.1, 15,25);
    float medium = triangular_MF(distance, 20, 55);
    float far = trapizoidal_MF(distance, 50,70, 90, 120);

    // Initialize output possibilities
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

    // Apply fuzzy rules
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

    speed_left = defuzzification(speed_left_low, speed_left_medium, speed_left_high);
    speed_right = defuzzification(speed_right_low, speed_right_medium, speed_right_high);
    direction = defuzzification(direction_left, direction_center, direction_right);

}

// triangular membership function
float triangular_MF(float x, float a, float c) {
    float b = (a + c) / 2;
    if (x <= a || x >= c) return 0;
    if (x == b) return 1;
    return (x < b) ? (x - a) / (b - a) : (c - x) / (c - b);
}

// triangular membership function
float triangular_MF(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0;
    if (x == b) return 1;
    return (x < b) ? (x - a) / (b - a) : (c - x) / (c - b);
}

// Triangular membership function
float trapizoidal_MF(float x, float a, float b, float c, float d) {
    if (x <= a || x >= d) {
        return 0.0;  // Outside the trapezoid
    } else if (x >= b && x <= c) {
        return 1.0;  // Top flat part of the trapezoid
    } else if (x > a && x < b) {
        return (x - a) / (b - a);  // Rising edge
    } else if (x > c && x < d) {
        return (d - x) / (d - c);  // Falling edge
    }
    return 0.0;  // Should never reach here, but just in case
}
// defuzzification using centroid method
float defuzzification(float low, float med, float high) {
  float low_a = 111-1e-10;
  float low_b = 111;
  float low_c = 155;
  float med_a = 140;
  float med_b = 167.5;
  float med_c = 195;
  float high_a = 180;
  float high_b = 255;
  float high_c = 255+1e-10;

  float low1 = low*(low_b - low_a) + low_a;
  float low2 = low_c - low*(low_c - low_b);

  float med1 = med*(med_b - med_a) + med_a;
  float med2 = med_c - med*(med_c - med_b);

  float high1 = high*(high_b - high_a) + high_a;
  float high2 = high_c - high*(high_c - high_b);

  if (low+med+high == 0) return 0;

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
    moveBackward(111, 111);
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1,14,12);

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
  } else{
    if (Serial1.available() > 0) {
        String distanceData = "";
        String orientationData = "";
        bool readD = false;
         // A string to hold incoming data
        while (Serial1.available() > 0) {
            char incomingByte = Serial1.read();
            if  (incomingByte == '|') readD = true;
            else if (readD){
              distanceData += incomingByte;
            }
            else{
            orientationData += incomingByte;       // Append the byte to the string
            }
            
        }
        Serial.println(orientationData);
        Serial.println(distanceData);
        // Example: If you expect orientation and distance values
        float orientation = orientationData.toFloat();
        float distance = distanceData.toFloat();

        // If an interrupt flag is set, stop motors
        if (interruptFlag) {
            stopMotors();
            interruptFlag = false;
        }

        // If no paired device, work using the fuzzy inference system
        FIS(orientation, distance);

        // Control the robot based on orientation and distance
        if (orientation < 230) {
            turnLeft(speed_left, speed_right);
            Serial.print("L");
        } else if (distance > 410) {
            turnRight(speed_left, speed_right);
            Serial.print("R");
        } else {
            moveForward(speed_left, speed_right);
            Serial.print("F");
        }

        // Calculate and print the distances traveled by each wheel
        float leftWheelDistance = getDistance(leftWheelTicks, wheelRadius, ticksPerRevolution);
        float rightWheelDistance = getDistance(rightWheelTicks, wheelRadius, ticksPerRevolution);

        Serial.print("Left Intensity: ");
        Serial.print(speed_left);
        Serial.println(" fuzzs");

        Serial.print("Right Intesity: ");
        Serial.print(speed_right);
        Serial.println(" fuzzs");
    } else{
      
    stopMotors();  
      
    }
  }
}
