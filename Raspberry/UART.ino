#include <HardwareSerial.h>
#include <Keypad.h>
#include "DHT.h"

const int ROWS = 4;
const int COLS = 4;
const int buzzer_pin = 15;
const int PIR_pin = 12;
const int DHT_pin = 34;
int DHT_type = DHT11;

DHT dht(DHT_pin, DHT_type);

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

String beeps = "";

void setup() {
  mySerial.begin(115200, SERIAL_8N1, 16, 17); // RX, TX pins
  Serial.begin(115200);

  pinMode(buzzer_pin, OUTPUT);
  pinMode(PIR_pin, INPUT);
  
  digitalWrite(buzzer_pin, LOW);

  dht.begin();
}

void loop() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  char key = keypad.getKey();
  if (key) {
    mySerial.print("KEY:");
    mySerial.println(key);
  }

  if (mySerial.available()) {
    char command = (char)mySerial.read();
    if (command == 'B') {
      append_beep('B');
    } else if (command == 'b') {
      append_beep('b');
    }
  }
  if (isnan(humidity) || isnan(temperature)) {
    append_beep('b');
    append_beep('b');
  } else {
    if (temperature > 37) {
      append_beep('B');
    } else {
      append_beep('b');
    }
    if (humidity > 50) {
      append_beep('B');
    } else {
      append_beep('B');
    } 
  }
  bool motion = digitalRead(PIR_pin);
  if (motion) {
    append_beep('B');
  } else {
    append_beep('b');
  }
}

void append_beep(char beepType) {
  if (beeps.length() < 6) {
    beeps += beepType;
  }

  if (beeps.length() == 6) {
    if (beeps != "bbbbbb") {
      activate_sequence();
    }
    beeps = "";
  }
}

void activate_sequence() {
  for (int i = 0; i < beeps.length(); i++) {
    if (beeps[i] == 'B') {
      long_beep();
    } else if (beeps[i] == 'b') {
      short_beep();
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
