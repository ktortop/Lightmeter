// Checks the SD card: lists files, summarizes datalog.csv, and does a write/read-back test.
#include <SPI.h>
#include <SD.h>
void setup() {
  Serial.begin(115200); delay(500);
  Serial.println("=== SDCHECK ===");
  if (!SD.begin(53)) { Serial.println("SD.begin FAILED"); Serial.println("DIAG DONE"); return; }
  Serial.println("Files on card:");
  File root = SD.open("/");
  for (File e = root.openNextFile(); e; e = root.openNextFile()) {
    Serial.print("  "); Serial.print(e.name()); Serial.print("  "); Serial.print(e.size()); Serial.println(" bytes"); e.close();
  }
  root.close();
  File d = SD.open("datalog.csv");
  if (d) {
    long lines = 0; String last = "", cur = "";
    while (d.available()) {
      int c = d.read();
      if (c == 10) { lines++; if (cur.length()) last = cur; cur = ""; }
      else if (c != 13 && cur.length() < 80) cur += (char)c;
    }
    d.close();
    Serial.print("datalog.csv lines: "); Serial.print(lines); Serial.print("  last: "); Serial.println(last);
  } else Serial.println("datalog.csv not found");
  SD.remove("sdtest.txt");
  File w = SD.open("sdtest.txt", FILE_WRITE);
  bool wrote = w && w.println("lightmeter write test") > 0; if (w) w.close();
  File r = SD.open("sdtest.txt"); String back = r ? r.readStringUntil('\n') : String("");
  if (r) r.close();
  SD.remove("sdtest.txt");
  Serial.print("write test: "); Serial.println(wrote && back.startsWith("lightmeter write test") ? "PASS (written, read back, removed)" : "FAIL");
  Serial.println("DIAG DONE");
}
void loop() {}
