#include <BluetoothSerial.h>   // Library for Bluetooth communication
#include <WiFiClientSecure.h>  // Library for secure Wi-Fi connection
#include <esp_crt_bundle.h>    // ESP32 certificate bundle
#include <WiFi.h>              // Library for Wi-Fi functions
#include <PubSubClient.h>
#include <math.h>

BluetoothSerial SerialBT;


// MQTT Data
const char* ssid = "m";
const char* password = "11111111";
const char* mqtt_broker = "bae6e1004af84917878e457c24d59cce.s1.eu.hivemq.cloud";
const char* mqtt_username = "esp32";
const char* mqtt_password = "esppass";
const int mqttPort = 8883;
const char* right_Fuzzy_topic = "UGV/RF";
const char* left_Fuzzy_topic = "UGV/LF";
const char* distance_topic = "UGV/Distance";
const char* orientation_topic = "UGV/Orient";
// WiFi Data
WiFiClientSecure espClient;
PubSubClient client(espClient);

// Motor A
int ena_pin = 15;
int in1_pin = 2;
int in2_pin = 0;
// Motor B
int in3_pin = 4;
int in4_pin = 16;
int enb_pin = 17;

// IR front and back word
int IR_F = 36;
int IR_B = 39;

// Flag to interrupt motor control
volatile bool interruptFlag = false;

// initial values for output speed for each motor and direction
float speed_left = 0;
float speed_right = 0;
float direction = 0;

// Define the pin for the IR sensor for each wheel
const int FrontleftIRSensorPin = 35;
const int FrontRightIRSensorPin = 34;
const int BackleftIRSensorPin = 32;
const int BackRightIRSensorPin = 33;


// Define constants for the wheel and tick counting
const float wheelRadius = 6.5;     // Radius of the wheel in meters (e.g., 10 cm)
const int ticksPerRevolution = 8;  // Number of ticks per revolution (depends on the number of markers)
const float wheelBase = 12;

// Variables for precise location control
float x = 0;
float y = 0;
float theta = 0;
bool preciseControl = false;


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

// Fuzzy inference system
void FIS(float orientation, float distance) {
  // Fuzzification of orientation and distance
  float right = trapizoidal_MF(orientation, 0, 1, 120, 240);
  float center = triangular_MF(orientation, 230, 410);
  float left = trapizoidal_MF(orientation, 390, 480, 640, 641);
  float close = trapizoidal_MF(distance, 0., 0.1, 15, 25);
  float medium = triangular_MF(distance, 20, 55);
  float far = trapizoidal_MF(distance, 50, 70, 90, 120);

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
  // If orientation is right and distance is far then direction is right, left motor's rotation speed is medium and right motor's rotation speed is high
  float rule1 = min(right, far);
  speed_left_medium = max(speed_left_medium, rule1);
  speed_right_high = max(speed_right_high, rule1);
  direction_right = max(direction_right, rule1);

  // If orientation is left and distance is far then direction is left, left motor's rotation speed is high and right motor's rotation speed is medium
  float rule2 = min(left, far);
  speed_left_high = max(speed_left_high, rule2);
  speed_right_medium = max(speed_right_medium, rule2);
  direction_left = max(direction_left, rule2);

  // If orientation is right and distance is medium then direction is right, left motor's rotation speed is low and right motor's rotation speed is medium
  float rule3 = min(right, medium);
  speed_left_low = max(speed_left_low, rule3);
  speed_right_medium = max(speed_right_medium, rule3);
  direction_right = max(direction_right, rule3);

  // If orientation is left and distance is medium then direction is left, left motor's rotation speed is medium and right motor's rotation speed is low
  float rule4 = min(left, medium);
  speed_left_medium = max(speed_left_medium, rule4);
  speed_right_low = max(speed_right_low, rule4);
  direction_left = max(direction_left, rule4);

  // If orientation is right and distance is close then direction is right, left motor's rotation speed is very low and right motor's rotation speed is low
  float rule5 = min(right, close);
  speed_left_very_low = max(speed_left_very_low, rule5);
  speed_right_low = max(speed_right_low, rule5);
  direction_right = max(direction_right, rule5);

  // If orientation is left and distance is close then direction is left, left motor's rotation speed is low and right motor's rotation speed is very low
  float rule6 = min(left, close);
  speed_left_low = max(speed_left_low, rule6);
  speed_right_very_low = max(speed_right_very_low, rule6);
  direction_left = max(direction_left, rule6);

  // If orientation is center and distance is far then direction is center, left motor's rotation speed is high and right motor's rotation speed is high
  float rule7 = min(center, far);
  speed_left_high = max(speed_left_high, rule7);
  speed_right_high = max(speed_right_high, rule7);
  direction_center = max(direction_center, rule7);

  // If orientation is center and distance is medium then direction is center, left motor's rotation speed is medium and right motor's rotation speed is medium
  float rule8 = min(center, medium);
  speed_left_medium = max(speed_left_medium, rule8);
  speed_right_medium = max(speed_right_medium, rule8);
  direction_center = max(direction_center, rule8);

  // If orientation is center and distance is close then direction is center, left motor's rotation speed is low and right motor's rotation speed is low
  float rule9 = min(center, close);
  speed_left_low = max(speed_left_low, rule9);
  speed_right_low = max(speed_right_low, rule9);
  direction_center = max(direction_center, rule9);

  // Defuzzification to get the final crisp output values using center of gravity method
  speed_left = defuzzification(speed_left_low, speed_left_medium, speed_left_high);
  speed_right = defuzzification(speed_right_low, speed_right_medium, speed_right_high);
  direction = defuzzification(direction_left, direction_center, direction_right);
}

// Symmetric triangular membership function
float triangular_MF(float x, float a, float c) {
  float b = (a + c) / 2;
  if (x <= a || x >= c) return 0;
  if (x == b) return 1;
  return (x < b) ? (x - a) / (b - a) : (c - x) / (c - b);
}

// Asymmetric triangular membership function
float triangular_MF(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0;
  if (x == b) return 1;
  return (x < b) ? (x - a) / (b - a) : (c - x) / (c - b);
}

// Trapizoidal membership function
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
  // Fuzzy ranges for motor speeds
  // The speeds use a triangular membership function
  float low_a = 111 - 1e-10;
  float low_b = 111;
  float low_c = 155;
  float med_a = 140;
  float med_b = 167.5;
  float med_c = 195;
  float high_a = 180;
  float high_b = 255;
  float high_c = 255 + 1e-10;

  // Solve the inverse for membership functions

  float low1 = low * (low_b - low_a) + low_a;
  float low2 = low_c - low * (low_c - low_b);

  float med1 = med * (med_b - med_a) + med_a;
  float med2 = med_c - med * (med_c - med_b);

  float high1 = high * (high_b - high_a) + high_a;
  float high2 = high_c - high * (high_c - high_b);

  // Returns 0 if values are out of bound
  if (low + med + high == 0) return 0;

  // Uses center of gravity weighted mean to calculate wheels speed
  return (low1 * low + low2 * low + med1 * med + med2 * med + high1 * high + high2 * high) / (2 * (low + med + high));
}

void moveForward(int intensity_A, int intensity_B) {
  // Motor A forward
  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, HIGH);
  analogWrite(ena_pin, intensity_A);

  // Motor B forward
  digitalWrite(in3_pin, LOW);
  digitalWrite(in4_pin, HIGH);
  analogWrite(enb_pin, intensity_B);
}

void moveBackward(int intensity_A, int intensity_B) {
  // Motor A backward
  digitalWrite(in1_pin, HIGH);
  digitalWrite(in2_pin, LOW);
  analogWrite(ena_pin, intensity_A);

  // Motor B backward
  digitalWrite(in3_pin, HIGH);
  digitalWrite(in4_pin, LOW);
  analogWrite(enb_pin, intensity_B);
}

void turnLeft(int intensity_A, int intensity_B) {
  // Motor A stop
  analogWrite(ena_pin, intensity_A);
  digitalWrite(in1_pin, HIGH);
  digitalWrite(in2_pin, LOW);

  // Motor B forward
  digitalWrite(in3_pin, LOW);
  digitalWrite(in4_pin, HIGH);
  analogWrite(enb_pin, intensity_B);
}

void turnRight(int intensity_A, int intensity_B) {
  // Motor B stop
  analogWrite(enb_pin, intensity_B);
  digitalWrite(in3_pin, HIGH);
  digitalWrite(in4_pin, LOW);

  // Motor A forward
  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, HIGH);
  analogWrite(ena_pin, intensity_A);
}

void stopMotors() {
  // Motor A stop
  analogWrite(ena_pin, 0);

  // Motor B stop
  analogWrite(enb_pin, 0);
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

void goToXY(float xn, float yn) {
  // The distance between the designated point and the current point
  float eucledianDistance = sqrt(pow(xn - x, 2) + pow(yn - y, 2));

  // Angle between designated coordinates vector and x-axis
  float newTheta = arctan2(yn, xn);
  // Required change from robots angle to designated angle
  float changeInTheta = newTheta - theta;

  // The length of the circular arc drawn by the right wheels
  float totalRightMovement = (2 * eucledianDistance - changeInTheta * wheelBase) / 2;
  // The length of the circular arc drawn by the left wheels
  float totalLeftMovement = (2 * eucledianDistance - totalRightMovement);

  // If the designated area is approximately reached, hand back control to fuzzy control
  if (abs(totalRightMovement) < 1 && abs(totalLeftMovement) < 1) preciseControl = false;


  // If both arcs are approximately the same, move the robot forward
  if (abs(totalLeftMovement - totalRightMovement) < 10) {

    moveForward(150,150);

  } else {

    // If the right arc is significant
    if (abs(totalRightMovement) > 1) {
      // If it is a positive arc
      if (totalRightMovement > 0) {
        // Turn the right wheels forward
        for (int i = 0; i < 100000; i++) {
          turnRight(150, 0);
        }
        else {
          // Turn the right wheels backward
          for (int i = 0; i < 100000; i++) {
            turnLeft(150, 0);
          }
        }
      }
    }

    // If the left arc is significant
    if (abs(totalLeftMovement) > 1) {
      // If the arc is positive
      if (totalLeftMovement > 0) {
        // Turn the left wheels forward
        for (int i = 0; i < 100000; i++) {
          turnLeft(0, 150);
        }
        else {
          // Turn the left wheel backward
          for (int i = 0; i < 100000; i++) {
            turnRight(0, 150);
          }
        }
      }
    }
  }
}

void setup() {

  delay(10);

  // Begin WiFi connection
  WiFi.begin(ssid, password);
  // Tries to connect to WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Sets ESP communication as insecure, MQTT does not work otherwise
  espClient.setInsecure();
  Serial.println("Connected.");
  // Set data for MQTT client
  client.setServer(mqtt_broker, mqttPort);

  // Connects to MQTT client
  while (!client.connected()) {
    Serial.print("Connecting to MQTT Broker...");
    // ID to connect to client with
    String client_id = "esp32-client-" + String(WiFi.macAddress());
    // Tries to connect using given username and password
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT Broker");
    } else {
      // If connection fails, log the error code
      Serial.print("Failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }

  // Begins serial printing
  Serial.begin(115200);
  // Start serial communication with ESPCam over RX2 and TX2
  Serial1.begin(115200, SERIAL_8N1, 14, 12);

  // Starts bluetooth serial
  if (!SerialBT.begin("esp32")) {  // Check if Bluetooth starts properly
    Serial.println("An error occurred initializing Bluetooth");
  } else {
    Serial.println("Bluetooth initialized. Device is ready to pair.");
  }

  // Sets econder IR sensor pins as input
  pinMode(FrontleftIRSensorPin, INPUT);
  pinMode(FrontRightIRSensorPin, INPUT);
  pinMode(BackleftIRSensorPin, INPUT);
  pinMode(BackRightIRSensorPin, INPUT);

  // Attach interrupts to the IR sensor pins
  attachInterrupt(digitalPinToInterrupt(FrontleftIRSensorPin), leftWheelTick, RISING);    // Detect rising edge for the left wheel
  attachInterrupt(digitalPinToInterrupt(FrontRightIRSensorPin), rightWheelTick, RISING);  // Detect rising edge for the right wheel
  attachInterrupt(digitalPinToInterrupt(BackleftIRSensorPin), leftWheelTick, RISING);     // Detect rising edge for the left wheel
  attachInterrupt(digitalPinToInterrupt(BackRightIRSensorPin), rightWheelTick, RISING);   // Detect rising edge for the right wheel

  // Initialize pins for motors
  pinMode(in1_pin, OUTPUT);
  pinMode(in2_pin, OUTPUT);
  pinMode(ena_pin, OUTPUT);

  pinMode(in3_pin, OUTPUT);
  pinMode(in4_pin, OUTPUT);
  pinMode(enb_pin, OUTPUT);

  // Set all motor pins as low
  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, LOW);
  digitalWrite(ena_pin, LOW);
  digitalWrite(in3_pin, LOW);
  digitalWrite(in4_pin, LOW);
  digitalWrite(enb_pin, LOW);

  Serial.println("Setup complete.");

  // Initialize IR sensor pins
  pinMode(IR_F, INPUT);
  pinMode(IR_B, INPUT);

  // Attach interrupts to front and back IR sensors
  attachInterrupt(digitalPinToInterrupt(IR_F), onSensorChange, FALLING);
  attachInterrupt(digitalPinToInterrupt(IR_B), onSensorChange, FALLING);
}

void loop() {
  // If there is a paired device, move using manual instructions
  if (SerialBT.hasClient()) {

    // Send data between serial and bluetooth to keep the conncetion alive
    if (Serial.available()) {
      SerialBT.write(Serial.read());
    }
    if (SerialBT.available()) {
      Serial.write(SerialBT.read());
      delay(5);

      // Reads the sent command
      char command = (char)SerialBT.read();

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
  } else if (preciseControl) {
    // Coordinates of the bin on the map
    goToXY(30, 45);
  } else {
    // Reads data from ESPCam attached over serial on RX2 and TX2
    if (Serial1.available() > 0) {
      // Strings to read serial data
      String distanceData = "";
      String orientationData = "";
      // Flag to determine which variable is being read
      bool readD = false;
      // Loops while the serial communication has data
      while (Serial1.available() > 0) {
        // Reads the serial character
        char incomingByte = Serial1.read();
        if (incomingByte == '|') readD = true;
        else if (readD) {
          distanceData += incomingByte;  // Append the byte to the string
        } else {
          orientationData += incomingByte;  // Append the byte to the string
        }
      }

      // Convert read data to floats
      float orientation = orientationData.toFloat();
      float distance = distanceData.toFloat();


      // If an interrupt flag is set, stop motors
      if (interruptFlag) {
        stopMotors();
        interruptFlag = false;
      }

      // If the object is in the robots pickers range
      if (distance < 10) {
        // Use precise control to go to bin
        preciseControl = true;
      } else {
        // Calculate the speeds using the inference system
        FIS(orientation, distance);

        // Convert defuzzified valeus to integer
        int speedR = (int)speed_right;
        int speedL = (int)speed_left;

        // Control the robot based on orientation and fuzzy values
        // The center of the image is at 320,320
        if (orientation < 230) {
          for (int i = 0; i < 150000; i++) {
            turnLeft(speedL, speedR);
          }
        } else if (distance > 410) {
          for (int i = 0; i < 150000; i++) {
            turnRight(speedL, speedR);
          }
        } else {
          for (int i = 0; i < 150000; i++) {
            moveForward(speedL, speedR);
          }
        }
        // Publish fuzzy data to MQTT
        client.publish(right_Fuzzy_topic, String(speedR).c_str());
        client.publish(left_Fuzzy_topic, String(speedL).c_str());
        client.publish(distance_topic, String(distance).c_str());
        client.publish(orientation_topic, String(orientation).c_str());

        Serial.print("Left Intensity: ");
        Serial.print(speedL);
        Serial.println(" fuzzs");

        Serial.print("Right Intesity: ");
        Serial.print(speedR);
        Serial.println(" fuzzs");
      }
    } else {
      // If no communication occurs, stop the motors
      stopMotors();
    }
  }

  // Calculate the distances traveled by each wheel pair
  float leftWheelDistance = getDistance(leftWheelTicks / 2, wheelRadius, ticksPerRevolution);
  float rightWheelDistance = getDistance(rightWheelTicks / 2, wheelRadius, ticksPerRevolution);

  // Total distance covered by both wheels
  float linearDistance = (leftWheelDistance + rightWheelDistance) / 2;
  // Angle between 2 circular arcs formed by both wheels
  float angularDistance = (leftWheelDistance - rightWheelDistance) / wheelBase;

  // Updating the x value
  x += linearDistance * cos(angularDistance);
  // Updating the y value
  y += linearDistance * sin(angularDistance);
  // Updating the theta value
  theta -= angularDistance;

  // Resetting the ticks for another calculation
  leftWheelTicks = 0;
  rightWheelTicks = 0;
}
