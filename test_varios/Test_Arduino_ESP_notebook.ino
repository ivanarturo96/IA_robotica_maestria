#include <SoftwareSerial.h>

// En mi conexión RX=10, TX=11 vistos desde el Arduino
SoftwareSerial esp(10, 11);

void setup() {
  Serial.begin(9600);
  esp.begin(9600);

  Serial.println("Puente UNO <-> ESP-01 listo");
}

void loop() {
  // ESP -> PC
  while (esp.available()) {
    Serial.write(esp.read());
  }

  // PC -> ESP
  while (Serial.available()) {
    esp.write(Serial.read());
  }
}