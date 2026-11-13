// Streams datalog.csv from the SD card over USB between BEGIN/END markers.
#include <SPI.h>
#include <SD.h>
void setup() {
  Serial.begin(115200); delay(1500);
  if (!SD.begin(53)) { Serial.println("SDDUMP ERROR: SD.begin failed"); return; }
  File d = SD.open("datalog.csv");
  if (!d) { Serial.println("SDDUMP ERROR: datalog.csv not found"); return; }
  Serial.print("SDDUMP BEGIN size="); Serial.println(d.size());
  uint8_t buf[64];
  while (d.available()) { int n = d.read(buf, sizeof(buf)); Serial.write(buf, n); }
  d.close();
  Serial.println(); Serial.println("SDDUMP END");
}
void loop() {}
