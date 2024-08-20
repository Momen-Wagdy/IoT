#define START_PIN 14
#define ALE_PIN 14
#define EOC_PIN 4
#define OE_PIN 5
#define ADD_A_PIN 25
#define ADD_B_PIN 26
#define ADD_C_PIN 27
#define D0_PIN 32
#define D1_PIN 23
#define D2_PIN 22
#define D3_PIN 21
#define D4_PIN 19
#define D5_PIN 18
#define D6_PIN 17
#define D7_PIN 16

void setup() {
  pinMode(START_PIN, OUTPUT);
  pinMode(ALE_PIN, OUTPUT);
  pinMode(EOC_PIN, INPUT);
  pinMode(OE_PIN, OUTPUT);
  pinMode(ADD_A_PIN, OUTPUT);
  pinMode(ADD_B_PIN, OUTPUT);
  pinMode(ADD_C_PIN, OUTPUT);
  pinMode(D0_PIN, INPUT);
  pinMode(D1_PIN, INPUT);
  pinMode(D2_PIN, INPUT);
  pinMode(D3_PIN, INPUT);
  pinMode(D4_PIN, INPUT);
  pinMode(D5_PIN, INPUT);
  pinMode(D6_PIN, INPUT);
  pinMode(D7_PIN, INPUT);

  Serial.begin(115200);
}

void loop() {
  // Select the analog input channel (example: IN0)
  digitalWrite(ADD_A_PIN, LOW);
  digitalWrite(ADD_B_PIN, LOW);
  digitalWrite(ADD_C_PIN, LOW);

  // Start conversion
  digitalWrite(START_PIN, HIGH);
  digitalWrite(ALE_PIN, HIGH);
  delayMicroseconds(50); // Wait for the address to latch
  digitalWrite(START_PIN, LOW);
  digitalWrite(ALE_PIN, LOW);

  // Wait for the end of conversion
  while (digitalRead(EOC_PIN) == LOW);

  // Read the converted data
  digitalWrite(OE_PIN, HIGH);
  int adcValue = (digitalRead(D7_PIN) << 7) |
                 (digitalRead(D6_PIN) << 6) |
                 (digitalRead(D5_PIN) << 5) |
                 (digitalRead(D4_PIN) << 4) |
                 (digitalRead(D3_PIN) << 3) |
                 (digitalRead(D2_PIN) << 2) |
                 (digitalRead(D1_PIN) << 1) |
                 digitalRead(D0_PIN);
  digitalWrite(OE_PIN, LOW);

  Serial.println(adcValue);

  delay(1000); // Wait for a while before the next read
}
