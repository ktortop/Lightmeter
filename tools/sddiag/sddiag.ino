// SD card wiring probe: bit-bangs the SD reset (CMD0) with normal and swapped
// data pins, and reports what the card answers. Expected healthy reply: 0x01.
#include <Arduino.h>
const uint8_t P50 = 50, P51 = 51, SCK_P = 52, CS_P = 53;
uint8_t xfer(uint8_t mosi, uint8_t miso, uint8_t out) {
  uint8_t in = 0;
  for (int b = 7; b >= 0; b--) {
    digitalWrite(mosi, (out >> b) & 1); delayMicroseconds(20);
    digitalWrite(SCK_P, HIGH); delayMicroseconds(20);
    in = (in << 1) | digitalRead(miso);
    digitalWrite(SCK_P, LOW); delayMicroseconds(20);
  }
  return in;
}
void tryCmd0(const char* tag, uint8_t mosi, uint8_t miso) {
  pinMode(mosi, OUTPUT); pinMode(miso, INPUT_PULLUP); pinMode(SCK_P, OUTPUT); pinMode(CS_P, OUTPUT);
  digitalWrite(CS_P, HIGH); digitalWrite(SCK_P, LOW);
  for (int i = 0; i < 12; i++) xfer(mosi, miso, 0xFF);      // >74 clocks with CS high
  digitalWrite(CS_P, LOW);
  xfer(mosi, miso, 0xFF);
  uint8_t cmd[6] = {0x40, 0, 0, 0, 0, 0x95};
  for (int i = 0; i < 6; i++) xfer(mosi, miso, cmd[i]);
  Serial.print(tag); Serial.print(" CMD0 replies:");
  bool got = false;
  for (int i = 0; i < 10; i++) { uint8_t r = xfer(mosi, miso, 0xFF); Serial.print(" "); Serial.print(r, HEX); if (r == 0x01) got = true; }
  digitalWrite(CS_P, HIGH); xfer(mosi, miso, 0xFF);
  Serial.println(got ? "  <-- CARD ANSWERED" : "");
}
void setup() {
  Serial.begin(115200); delay(300);
  Serial.println("=== SDDIAG ===");
  pinMode(CS_P, OUTPUT); digitalWrite(CS_P, HIGH);
  pinMode(P50, INPUT); delay(5); int a = digitalRead(P50);
  pinMode(P50, INPUT_PULLUP); delay(5); int b = digitalRead(P50);
  digitalWrite(CS_P, LOW); pinMode(P50, INPUT); delay(5); int c = digitalRead(P50);
  pinMode(P50, INPUT_PULLUP); delay(5); int d = digitalRead(P50); digitalWrite(CS_P, HIGH);
  Serial.print("pin50 (DO) CS high: float="); Serial.print(a); Serial.print(" pullup="); Serial.print(b);
  Serial.print(" | CS low: float="); Serial.print(c); Serial.print(" pullup="); Serial.println(d);
  tryCmd0("normal  (51 out, 50 in)", P51, P50);
  tryCmd0("swapped (50 out, 51 in)", P50, P51);
  Serial.println("DIAG DONE");
}
void loop() {}
