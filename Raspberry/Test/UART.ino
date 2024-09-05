#include <HardwareSerial.h>
#include <Keypad.h>

const int ROWS = 4;
const int COLS = 4;
const int buzzer_pin = 14;
const int speaker_first_pin = 4;
const int speaker_second_pin = 2;

char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {34, 35, 32, 33};
byte colPins[COLS] = {25, 26, 27, 14};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

HardwareSerial mySerial(1);

void setup() {
  mySerial.begin(115200, SERIAL_8N1, 16, 17); // RX, TX pins
  Serial.begin(115200);

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

  if (mySerial.available()) {
    char command = (char)mySerial.read();
    if (command == 'B') {
      long_beep()
    } else if (command == 'b') {
      short_beep()
    } else if (command == 'S') {
      String message = "";
      while (mySerial.available()) {
        char incomingChar = (char)mySerial.read();
        if (incomingChar == '\n') {
          break;
        }
        message += incomingChar;
      }
      // speaker_handler(message); // Process the message for the speaker
    }
  }
}

void short_beep() {
  digitalWrite(buzzer_pin, HIGH);
  delay(250);
  digitalWrite(buzzer_pin, LOW);
  delay(200);
}

void long_beep() {
  digitalWrite(buzzer_pin, HIGH);
  delay(500);
  digitalWrite(buzzer_pin, LOW);
  delay(200);
}

// void speaker_handler()
