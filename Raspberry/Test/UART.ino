#include <HardwareSerial.h>
#include <Keypad.h>

// Define GPIO pins
const int ROWS = 4;
const int COLS = 4;
const int buzzer_pin = 14;
const int speaker_first_pin = 4;
const int speaker_second_pin = 2;

// Keypad configuration
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {34, 35, 32, 33};  // Connect to the row pins of the keypad
byte colPins[COLS] = {25, 26, 27, 14};  // Connect to the column pins of the keypad

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

HardwareSerial mySerial(1); // Use UART1 for communication

void setup() {
  mySerial.begin(115200, SERIAL_8N1, 16, 17); // RX, TX pins
  Serial.begin(115200); // For debugging

  pinMode(buzzer_pin, OUTPUT);
  pinMode(speaker_first_pin, OUTPUT);
  pinMode(speaker_second_pin, OUTPUT);
  
  digitalWrite(buzzer_pin, LOW);
  digitalWrite(speaker_first_pin, LOW);
  digitalWrite(speaker_second_pin, LOW);
}

void loop() {
  char key = keypad.getKey();
  if (key) {
    mySerial.print("KEY:");
    mySerial.println(key);
  }

  // Check if data is available on UART
  if (mySerial.available()) {
    char command = (char)mySerial.read(); // Read the command
    if (command == 'B') {
      digitalWrite(buzzer_pin, HIGH); // Set GPIO pin state
    } else if (command == 'b') {
      digitalWrite(buzzer_pin, LOW);
    } else if (command == 'S') {
      String message = "";
      while (mySerial.available()) {
        char incomingChar = (char)mySerial.read();
        if (incomingChar == '\n') { // End of the message
          break;
        }
        message += incomingChar; // Append character to message
      }
      // speaker_handler(message); // Process the message for the speaker
    }
  }
}

// void speaker_handler()
