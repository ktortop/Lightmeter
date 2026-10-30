// Hardware diagnostic for the Lightmeter. Upload, then read Serial at 115200.
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
Sd2Card card;
void lvl(const char* tag) {
  Serial.print(tag); Serial.print(" SDA="); Serial.print(digitalRead(20));
  Serial.print(" SCL="); Serial.print(digitalRead(21));
  Serial.print(" RX1(19,GPS TX)="); Serial.print(digitalRead(19));
  Serial.print(" RX2(17,IMU TX)="); Serial.println(digitalRead(17));
}
void imuTry(long baud) {
  Serial2.begin(baud); delay(200); while (Serial2.available()) Serial2.read();
  Serial2.write(0xAA); Serial2.write(0x01); Serial2.write((uint8_t)0); Serial2.write((uint8_t)1); delay(150);
  Serial.print("IMU @"); Serial.print(baud); Serial.print(" reply:");
  int n = 0; while (Serial2.available() && n < 16) { Serial.print(" 0x"); Serial.print(Serial2.read(), HEX); n++; }
  Serial.println(n ? "" : " (none)");
  Serial2.end();
}
void setup() {
  Serial.begin(115200); delay(500);
  Serial.println("=== HWDIAG ===");
  pinMode(20, INPUT); pinMode(21, INPUT); pinMode(19, INPUT); pinMode(17, INPUT); delay(10);
  lvl("no-pullups ");
  Wire.begin(); Wire.setWireTimeout(25000, true);
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a); uint8_t e = Wire.endTransmission();
    if (Wire.getWireTimeoutFlag()) { Serial.println("I2C TIMEOUT (bus stuck)"); Wire.clearWireTimeoutFlag(); break; }
    if (e == 0) { Serial.print("I2C device 0x"); Serial.println(a, HEX); found++; }
  }
  Serial.print("I2C found="); Serial.println(found);
  imuTry(115200);
  for (int i = 0; i < 3; i++) imuTry(115200);
  long bauds[] = {9600, 38400};
  for (int b = 0; b < 2; b++) {
    Serial1.begin(bauds[b]); delay(50); while (Serial1.available()) Serial1.read();
    unsigned long t = millis(); int n = 0; char s[81]; int si = 0;
    while (millis() - t < 2000) if (Serial1.available()) { char c = Serial1.read(); n++; if (si < 80 && c >= 32) s[si++] = c; }
    s[si] = 0;
    Serial.print("GPS @"); Serial.print(bauds[b]); Serial.print(" bytes="); Serial.print(n); Serial.print(" sample="); Serial.println(s);
    Serial1.end();
  }
  pinMode(53, OUTPUT); digitalWrite(53, HIGH);
  bool ok = card.init(SPI_QUARTER_SPEED, 53);
  Serial.print("SD init "); Serial.print(ok ? "OK" : "FAIL"); Serial.print(" err=0x"); Serial.println(card.errorCode(), HEX);
  Serial.println("DIAG DONE");
}
void loop() {}
