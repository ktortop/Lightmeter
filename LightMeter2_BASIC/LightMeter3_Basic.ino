#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ── Constants ─────────────────────────────────────────────────────────────────
const int   chipSelect       = 53;
const float fc_conversion    = 10.76391f;
const float batteryMax       = 4.35f;
const float batteryMin       = 3.30f;
const float spacingThreshold = 1.524f;  // 5 feet in metres
const float levelTolerance   = 5.0f;   // degrees of tilt allowed before skipping reading

// ── LED Pins ──────────────────────────────────────────────────────────────────
const int gps_red   = 2;
const int gps_green = 3;
const int lux_red   = 5;
const int lux_green = 6;
const int imu_red   = 7;
const int imu_green = 8;

// ── GPS Averaging / Outlier Filtering ────────────────────────────────────────
// Collects GPS_ARRAY_SIZE fixes, removes outliers beyond 2 standard deviations,
// then averages the remainder into a stable anchor (absLat / absLon).
// This prevents single bad GPS jumps from corrupting the position estimate.
const int GPS_ARRAY_SIZE = 30;
double latArray[GPS_ARRAY_SIZE];
double lonArray[GPS_ARRAY_SIZE];
int    gpsCount = 0;

// ── Position State ────────────────────────────────────────────────────────────
double absLat        = 0.0;  // averaged GPS anchor (degrees)
double absLon        = 0.0;
double currentLat    = 0.0;  // current estimated position
double currentLon    = 0.0;
double lastLoggedLat = 0.0;  // position of last CSV write
double lastLoggedLon = 0.0;
bool   hasLoggedFirstPoint = false;

// ── IMU Dead-Reckoning ────────────────────────────────────────────────────────
float northOffset = 0.0f;  // metres north of anchor
float eastOffset  = 0.0f;  // metres east of anchor
float velocity    = 0.0f;  // forward velocity (m/s)

// ── Flags & Timers ────────────────────────────────────────────────────────────
bool          hasAnchor   = false;
unsigned long lastIMUTime = 0;

// ── Light Reading ─────────────────────────────────────────────────────────────
float lastLux = 0.0f;  // most recent lux reading

// ── Hardware Objects ──────────────────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_TSL2591  tsl = Adafruit_TSL2591(2591);
Adafruit_BNO055   bno = Adafruit_BNO055(55, 0x28);
SoftwareSerial    mySerial(11, 10);  // RX=11, TX=10
Adafruit_GPS      GPS(&mySerial);

#define GPSECHO false  // set true only for raw NMEA debug output

// ─────────────────────────────────────────────────────────────────────────────
// configureSensor
// ─────────────────────────────────────────────────────────────────────────────
void configureSensor()
{
  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);

  Serial.println(F("------------------------------------"));
  Serial.print(F("Gain: "));
  tsl2591Gain_t gain = tsl.getGain();
  switch (gain)
  {
    case TSL2591_GAIN_LOW:  Serial.println(F("1x (Low)"));     break;
    case TSL2591_GAIN_MED:  Serial.println(F("25x (Medium)")); break;
    case TSL2591_GAIN_HIGH: Serial.println(F("428x (High)"));  break;
    case TSL2591_GAIN_MAX:  Serial.println(F("9876x (Max)"));  break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// setLED  –  color: "RED" | "GREEN" | "YELLOW" | anything else = OFF
// ─────────────────────────────────────────────────────────────────────────────
void setLED(int rPin, int gPin, const char* color)
{
  if      (strcmp(color, "RED")    == 0) { digitalWrite(rPin, LOW);  digitalWrite(gPin, HIGH); }
  else if (strcmp(color, "GREEN")  == 0) { digitalWrite(rPin, HIGH); digitalWrite(gPin, LOW);  }
  else if (strcmp(color, "YELLOW") == 0) { digitalWrite(rPin, LOW);  digitalWrite(gPin, LOW);  }
  else                                   { digitalWrite(rPin, HIGH); digitalWrite(gPin, HIGH); }
}

// ─────────────────────────────────────────────────────────────────────────────
// getDisplacement  –  Haversine distance between two lat/lon pairs (metres)
//   Uses double throughout to preserve sub-metre precision.
// ─────────────────────────────────────────────────────────────────────────────
double getDisplacement(double lat1, double lon1, double lat2, double lon2)
{
  const double R = 6371000.0;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);

  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  return R * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

// ─────────────────────────────────────────────────────────────────────────────
// isMoving  –  returns false once linear accel AND velocity have been
//              near-zero for >0.5 s, which zeroes velocity to prevent drift.
// ─────────────────────────────────────────────────────────────────────────────
bool isMoving(float accForward, float vel, float dt)
{
  static float stationaryTime = 0.0f;

  if (fabsf(accForward) < 0.05f && fabsf(vel) < 0.05f)
    stationaryTime += dt;
  else
    stationaryTime = 0.0f;

  return (stationaryTime <= 0.5f);
}

// ─────────────────────────────────────────────────────────────────────────────
// processGPSData
//   Called once latArray[] / lonArray[] are full (GPS_ARRAY_SIZE fixes).
//   1. Computes mean lat/lon across all fixes.
//   2. Filters fixes that are >2 standard deviations from the mean.
//   3. Averages the survivors into absLat / absLon (the IMU anchor).
//   4. Resets IMU offsets so dead-reckoning restarts from the new anchor.
//
//   If ALL fixes are rejected as outliers the old anchor is preserved
//   and offsets are left unchanged.
// ─────────────────────────────────────────────────────────────────────────────
void processGPSData()
{
  // 1. Mean
  double sumLat = 0.0, sumLon = 0.0;
  for (int i = 0; i < GPS_ARRAY_SIZE; i++) {
    sumLat += latArray[i];
    sumLon += lonArray[i];
  }
  double meanLat = sumLat / GPS_ARRAY_SIZE;
  double meanLon = sumLon / GPS_ARRAY_SIZE;

  // 2. Standard deviation
  double varLat = 0.0, varLon = 0.0;
  for (int i = 0; i < GPS_ARRAY_SIZE; i++) {
    varLat += pow(latArray[i] - meanLat, 2);
    varLon += pow(lonArray[i] - meanLon, 2);
  }
  double stdLat = sqrt(varLat / GPS_ARRAY_SIZE);
  double stdLon = sqrt(varLon / GPS_ARRAY_SIZE);

  // 3. Filter outliers, average survivors
  double finalSumLat = 0.0, finalSumLon = 0.0;
  int validCount = 0;
  for (int i = 0; i < GPS_ARRAY_SIZE; i++) {
    if (fabs(latArray[i] - meanLat) <= 2.0 * stdLat &&
        fabs(lonArray[i] - meanLon) <= 2.0 * stdLon)
    {
      finalSumLat += latArray[i];
      finalSumLon += lonArray[i];
      validCount++;
    }
  }

  if (validCount > 0) {
    absLat = finalSumLat / validCount;
    absLon = finalSumLon / validCount;

    // Reset dead-reckoning to new anchor
    northOffset = 0.0f;
    eastOffset  = 0.0f;

    if (!hasAnchor) {
      hasAnchor   = true;
      lastIMUTime = millis();
      setLED(gps_red, gps_green, "GREEN");
      Serial.println(F("GPS anchor acquired"));
    } else {
      Serial.println(F("GPS anchor updated"));
    }
  } else {
    // All fixes were outliers — keep old anchor, don't reset offsets
    Serial.println(F("WARNING: all GPS fixes rejected, anchor unchanged"));
  }

  gpsCount = 0;  // reset buffer for next batch
}

// ─────────────────────────────────────────────────────────────────────────────
// setup
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  delay(500);

  // LED pins — must be initialised before any setLED() call
  pinMode(gps_red,   OUTPUT);
  pinMode(gps_green, OUTPUT);
  pinMode(lux_red,   OUTPUT);
  pinMode(lux_green, OUTPUT);
  setLED(gps_red, gps_green, "RED");   // red until anchor acquired
  setLED(lux_red, lux_green, "RED");   // red until light sensor confirmed
  setLED(imu_red, imu_green, "RED");   // red until IMU reading is obtained

  // LCD
  Serial.println(F("LCD STARTUP"));
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("LIGHT METER STARTING");

  // GPS
  Serial.println(F("GPS CHECK"));
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  Serial.println(F("GPS.PASS"));

  // IMU
  Serial.println(F("IMU CHECK"));
  if (!bno.begin()) {
    Serial.println(F("Failed to initialize IMU - halting"));
    while (1);
  }
  Serial.println(F("IMU found"));
  setLED(imu_red, imu_green, "GREEN");

  // Light sensor
  Serial.println(F("LIGHT SENSOR CHECK"));
  if (!tsl.begin()) {
    Serial.println(F("No TSL2591 found - check wiring - halting"));
    while (1);
  }
  Serial.println(F("TSL2591 found"));
  setLED(lux_red, lux_green, "GREEN");

  // SD card — fatal if missing, no point running without logging
  if (!SD.begin(chipSelect)) {
    Serial.println(F("SD FAIL - halting"));
    lcd.setCursor(0, 1);
    lcd.print("SD CARD FAIL - HALT");
    while (1);
  }
  Serial.println(F("SD OK"));

  // Light sensor config
  Serial.println(F("SENSOR CONFIG"));
  configureSensor();
  Serial.println(F("SENSOR CONFIG PASS"));

  Serial.println(F("STARTING LOOP..."));
}

// ─────────────────────────────────────────────────────────────────────────────
// loop
// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
  // ── 1. Feed GPS parser ──────────────────────────────────────────────────────
  char c = GPS.read();
  if (GPSECHO && c) Serial.write(c);

  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())) {
      if (GPS.fix) {
        latArray[gpsCount] = GPS.latitudeDegrees;  // double — no precision loss
        lonArray[gpsCount] = GPS.longitudeDegrees;
        gpsCount++;

        if (gpsCount >= GPS_ARRAY_SIZE) {
          // Batch full — compute stable averaged anchor
          processGPSData();
        }
      }
    }
  }

  // ── 2. Wait until a valid anchor exists ─────────────────────────────────────
  if (!hasAnchor) {
    return;
  }

  // ── 3. IMU dead-reckoning ───────────────────────────────────────────────────
  unsigned long now = millis();
  float dt = (now - lastIMUTime) / 1000.0f;
  lastIMUTime = now;  // always advance so next dt is correct

  if (dt > 0.0f && dt <= 0.5f) {
    imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    float accForward = linAccel.x();  // ensure x-axis points forward on cart

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float headingRad = euler.x() * PI / 180.0f;

    velocity += accForward * dt;
    if (!isMoving(accForward, velocity, dt)) {
      velocity = 0.0f;
    }

    float d   = velocity * dt;
    northOffset += d * cos(headingRad);
    eastOffset  += d * sin(headingRad);
  }

  // ── 4. Compute current position from anchor + IMU offsets ───────────────────
  double metersPerDegLat = 111111.0;
  double metersPerDegLon = 111111.0 * cos(absLat * PI / 180.0);
  currentLat = absLat + (northOffset / metersPerDegLat);
  currentLon = absLon + (eastOffset  / metersPerDegLon);

  // ── 5. Check spacing — only log every spacingThreshold metres ───────────────
  double distFromLastLog = getDisplacement(lastLoggedLat, lastLoggedLon,
                                           currentLat, currentLon);

  if (!hasLoggedFirstPoint || distFromLastLog >= spacingThreshold) {

    // ── 6. Tilt check — skip reading if sensor is not level ─────────────────
    //    Read Euler angles fresh for the level check.
    //    Roll = euler.y(), Pitch = euler.z() on BNO055.
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float rollDeg  = euler.y();
    float pitchDeg = euler.z();
    bool  level    = (fabsf(rollDeg) < levelTolerance &&
                      fabsf(pitchDeg) < levelTolerance);

    if (!level) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("SENSOR NOT LEVEL");
      lcd.setCursor(0, 1);
      lcd.print("Roll:  ");
      lcd.print(rollDeg, 1);
      lcd.setCursor(0, 2);
      lcd.print("Pitch: ");
      lcd.print(pitchDeg, 1);
      lcd.setCursor(0, 3);
      lcd.print("Re-level and move");
      Serial.println(F("Tilt detected - level sensor before reading"));
      // Do NOT log, do NOT update lastLoggedLat/Lon — spacing check
      // will re-trigger next iteration until a level reading is taken.
      return;
    }

    // ── 7. Read light sensor ─────────────────────────────────────────────────
    uint32_t lum  = tsl.getFullLuminosity();
    uint16_t ir   = lum >> 16;
    uint16_t full = lum & 0xFFFF;
    lastLux = tsl.calculateLux(full, ir);
    float footcandles = lastLux / fc_conversion;

    // ── 8. Update LCD ────────────────────────────────────────────────────────
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sats: ");
    lcd.print(GPS.satellites);
    lcd.setCursor(0, 1);
    lcd.print("FC:   ");
    lcd.print(footcandles, 2);
    lcd.setCursor(0, 2);
    lcd.print("Lat:  ");
    lcd.print(currentLat, 5);
    lcd.setCursor(0, 3);
    lcd.print("Lon:  ");
    lcd.print(currentLon, 5);

    // ── 9. Build and write CSV row ───────────────────────────────────────────
    //   Format:  DD/MM/YY, HH:MM:SS, footcandles, lat, lon
    String dataString = "";
    dataString += String(GPS.day)    + "/" +
                  String(GPS.month)  + "/" +
                  String(GPS.year)   + ",";
    dataString += String(GPS.hour)   + ":" +
                  String(GPS.minute) + ":" +
                  String(GPS.seconds)+ ",";
    dataString += String(footcandles, 2) + ",";

    File dataFile = SD.open("datalog.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.print(dataString);
      dataFile.print(currentLat, 6);
      dataFile.print(",");
      dataFile.println(currentLon, 6);
      dataFile.close();

      lastLoggedLat      = currentLat;
      lastLoggedLon      = currentLon;
      hasLoggedFirstPoint = true;

      Serial.print(F("SD Write OK  fc="));
      Serial.print(footcandles, 2);
      Serial.print(F("  lat="));
      Serial.print(currentLat, 6);
      Serial.print(F("  lon="));
      Serial.println(currentLon, 6);
    } else {
      Serial.println(F("ERROR: could not open datalog.csv"));
    }
  }
}
