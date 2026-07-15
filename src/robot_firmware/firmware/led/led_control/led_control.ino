void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == '1') {
      digitalWrite(13, HIGH);
      Serial.println("LED ON");
    }
    else if (cmd == '0') {
      digitalWrite(13, LOW);
      Serial.println("LED OFF");
    }
  }
}