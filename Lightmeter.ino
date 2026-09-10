#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>
#include <math.h>

// ─────────────────────────────────────────────
// CONSTANTS
// ─────────────────────────────────────────────
const int   chipSelect       = 53;
const float fc_conversion    = 10.76391f;
const float spacingThreshold = 1.524f;    // 5 ft in metres
const float logTriggerDistance = 1.60f;   // small hysteresis above 5 ft
const float levelTolerance   = 5.0f;      // degrees
const int   MIN_SATS         = 4;

// LED pins
const int gps_red   = 2;
const int gps_green = 3;
const int lux_red   = 5;
const int lux_green = 6;
const int imu_red   = 8;
const int imu_green = 9;

// --- NEW BUTTON CONFIGURATION ---
const int BTN_SLEEP      = 10;
const int BTN_LOG_TOGGLE = 11;
const int BTN_FORCE_LOG  = 12;

bool isSleeping     = false;
bool loggingEnabled = true;
bool forceLog       = false;

unsigned long lastBtnSleepTime = 0;
unsigned long lastBtnLogTime   = 0;
unsigned long lastBtnForceTime = 0;
const unsigned long debounceDelay = 250;

// GPS speed conversion
// NMEA speed in knots -> metres/second
const float KNOTS_TO_MPS = 0.514444f;
const float MIN_GPS_VEL_FUSE_MPS = 0.80f;     // ignore low-speed GPS course jitter
const unsigned long GPS_FIX_FRESH_MS = 1500;  // require recent GPS anchor before logging

// GPS measurement noise
// Position in metres^2
const float R_GPS_POS = 4.0f;
// Velocity in (m/s)^2
const float R_GPS_VEL = 1.0f;

// Process noise heuristics (see kalmanPredict() for how they're used)
const float Q_POS = 0.05f;
const float Q_VEL = 0.20f;

unsigned long lastIMUTime        = 0; // 
unsigned long lastGoodGPSFixTime = 0; // 

unsigned long gpsSetupTime       = 0; 
unsigned long lastFixTime        = 0;

// ─────────────────────────────────────────────
// SENSOR OBJECTS
// ─────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_TSL2591  tsl = Adafruit_TSL2591(2591);
Adafruit_GPS      GPS(&Serial1);

// ─────────────────────────────────────────────
// BNO055 UART REGISTERS
// ─────────────────────────────────────────────
#define BNO_REG_CHIP_ID       0x00
#define BNO_REG_OPR_MODE      0x3D
#define BNO_REG_PWR_MODE      0x3E
#define BNO_REG_PAGE_ID       0x07
#define BNO_REG_SYS_TRIGGER   0x3F
#define BNO_REG_EULER_H_LSB   0x1A
#define BNO_REG_LIN_ACC_X_LSB 0x28

#define BNO_CHIP_ID         0xA0
#define BNO_OPR_MODE_CONFIG 0x00
#define BNO_OPR_MODE_NDOF   0x0C
#define BNO_PWR_MODE_NORMAL 0x00

// ─────────────────────────────────────────────
// EKF STATE
// State vector: [x_metres, y_metres, vx, vy]
// x = north offset from origin, y = east offset
// ─────────────────────────────────────────────
struct KalmanState {
  float x  = 0;   // north offset (metres)
  float y  = 0;   // east  offset (metres)
  float vx = 0;   // north velocity (m/s)
  float vy = 0;   // east  velocity (m/s)
};

KalmanState kf;

// 4×4 covariance matrix (row-major)
float P[4][4] = {
  {10,  0,  0,  0},
  { 0, 10,  0,  0},
  { 0,  0,  5,  0},
  { 0,  0,  0,  5}
};

double originLat = 0;
double originLon = 0;

bool          kfInitialized = false;

// ─────────────────────────────────────────────
// LOGGING STATE
// ─────────────────────────────────────────────
float lastLogX    = 0;
float lastLogY    = 0;

// ─────────────────────────────────────────────
// LED HELPER  —  RED / YELLOW / GREEN
// ─────────────────────────────────────────────
void setLED(int r, int g, const char* color) {
  if      (strcmp(color, "GREEN")  == 0) { digitalWrite(r, LOW); digitalWrite(g, HIGH);  }
  else if (strcmp(color, "RED") == 0) { digitalWrite(r, HIGH);  digitalWrite(g, LOW);  }
  else if (strcmp(color, "YELLOW") == 0) { digitalWrite(r, LOW); digitalWrite(g, LOW); }
  else                                   { digitalWrite(r, HIGH);  digitalWrite(g, HIGH); }
}

// ─────────────────────────────────────────────
// DISTANCE in local-frame metres
// ─────────────────────────────────────────────
float distanceMeters(float x1, float y1, float x2, float y2) {
  float dx = x2 - x1;
  float dy = y2 - y1;
  return sqrt(dx * dx + dy * dy);
}

// ─────────────────────────────────────────────
// BNO055 UART HELPERS
// ─────────────────────────────────────────────
bool bnoRead(uint8_t reg, uint8_t len, uint8_t* buf) {
  while (Serial2.available()) Serial2.read();

  Serial2.write(0xAA);
  Serial2.write(0x01);
  Serial2.write(reg);
  Serial2.write(len);

  unsigned long t = millis();
  while (Serial2.available() < (unsigned long)(len + 2)) {
    if (millis() - t > 100) return false;
  }

  uint8_t hdr = Serial2.read();
  uint8_t cnt = Serial2.read();
  if (hdr != 0xBB || cnt != len) return false;

  for (int i = 0; i < len; i++) buf[i] = Serial2.read();
  return true;
}

bool bnoWrite(uint8_t reg, uint8_t val) {
  while (Serial2.available()) Serial2.read();

  Serial2.write(0xAA);
  Serial2.write(0x00);
  Serial2.write(reg);
  Serial2.write(0x01);
  Serial2.write(val);

  unsigned long t = millis();
  while (Serial2.available() < 2) {
    if (millis() - t > 100) return false;
  }

  uint8_t hdr    = Serial2.read();
  uint8_t status = Serial2.read();
  return (hdr == 0xEE && status == 0x01);
}

bool bnoReadVector(uint8_t reg, float scale, float& x, float& y, float& z) {
  uint8_t buf[6];
  if (!bnoRead(reg, 6, buf)) return false;

  x = (int16_t)(buf[0] | (buf[1] << 8)) / scale;
  y = (int16_t)(buf[2] | (buf[3] << 8)) / scale;
  z = (int16_t)(buf[4] | (buf[5] << 8)) / scale;
  return true;
}

// ─────────────────────────────────────────────
// IMU INIT
// ─────────────────────────────────────────────
bool bnoBegin() {
  Serial2.begin(115200);
  delay(1000);

  uint8_t id;
  for (int i = 0; i < 5; i++) {
    if (bnoRead(BNO_REG_CHIP_ID, 1, &id) && id == BNO_CHIP_ID) break;
    if (i == 4) return false;
    delay(200);
  }

  bnoWrite(BNO_REG_OPR_MODE, BNO_OPR_MODE_CONFIG); delay(30);
  bnoWrite(BNO_REG_PWR_MODE, BNO_PWR_MODE_NORMAL);  delay(10);
  bnoWrite(BNO_REG_SYS_TRIGGER, 0x80);              delay(10);
  bnoWrite(BNO_REG_OPR_MODE, BNO_OPR_MODE_NDOF);    delay(50);

  return true;
}

// ─────────────────────────────────────────────
// LIGHT SENSOR CONFIG
// ─────────────────────────────────────────────
void configureSensor() {
  tsl.setGain(TSL2591_GAIN_LOW);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);

  Serial.println(F("------------------------------------"));
  Serial.print(F("Gain: "));
  switch (tsl.getGain()) {
    case TSL2591_GAIN_LOW:  Serial.println(F("1x (Low)"));    break;
    case TSL2591_GAIN_MED:  Serial.println(F("25x (Medium)")); break;
    case TSL2591_GAIN_HIGH: Serial.println(F("428x (High)"));  break;
    case TSL2591_GAIN_MAX:  Serial.println(F("9876x (Max)"));  break;
  }
}

// ─────────────────────────────────────────────
// KALMAN INIT  —  seeds origin and resets state
// ─────────────────────────────────────────────
void initKalman(double lat, double lon) {
  originLat = lat;
  originLon = lon;

  kf.x = 0;  kf.y = 0;
  kf.vx = 0; kf.vy = 0;

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      P[i][j] = (i == j) ? 1.0f : 0.0f;

  lastLogX = 0;
  lastLogY = 0;

  kfInitialized = true;
  lastIMUTime   = millis();

  setLED(gps_red, gps_green, "GREEN");
  Serial.println(F("KF origin set"));
}

// ─────────────────────────────────────────────
// Helpers for EKF update (scalar measurement)
// measIndex: 0=x, 1=y, 2=vx, 3=vy
// ─────────────────────────────────────────────
float getStateByIndex(int measIndex) {
  if (measIndex == 0) return kf.x;
  if (measIndex == 1) return kf.y;
  if (measIndex == 2) return kf.vx;
  return kf.vy;
}

void setStateByIndex(int measIndex, float value) {
  if (measIndex == 0) kf.x = value;
  else if (measIndex == 1) kf.y = value;
  else if (measIndex == 2) kf.vx = value;
  else kf.vy = value;
}

void kalmanUpdateScalar(int measIndex, float z, float R) {
  // Innovation
  float hx = getStateByIndex(measIndex);
  float y = z - hx;

  // S = HPH^T + R. For scalar measurement, S = P[ii] + R.
  float S = P[measIndex][measIndex] + R;
  if (S <= 0.0f) return;

  // Kalman gain vector: K_i = P_i,ii / S
  float K[4];
  for (int i = 0; i < 4; i++) {
    K[i] = P[i][measIndex] / S;
  }

  // State update: x = x + K*y
  kf.x  += K[0] * y;
  kf.y  += K[1] * y;
  kf.vx += K[2] * y;
  kf.vy += K[3] * y;

  // Covariance update: P = (I - K H) P
  // For scalar H picking state measIndex, this becomes:
  // P_ij = P_ij - K_i * P_measIndex,j
  float Pold[4][4];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      Pold[i][j] = P[i][j];

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      P[i][j] = Pold[i][j] - K[i] * Pold[measIndex][j];
    }
  }
}

// ─────────────────────────────────────────────
// EKF PREDICT using IMU acceleration + heading
// State transition:
// x = x + vx*dt + 0.5*ax*dt^2
// y = y + vy*dt + 0.5*ay*dt^2
// vx = vx + ax*dt
// vy = vy + ay*dt
// ─────────────────────────────────────────────
void kalmanPredict(float linXForward, float headingRad) {
  if (!kfInitialized) return;

  unsigned long now = millis();
  float dt = (now - lastIMUTime) / 1000.0f;
  lastIMUTime = now;

  if (dt <= 0.0f || dt > 0.5f) return;

  // Rotate forward acceleration into world-frame N/E components.
  float ax = linXForward * cos(headingRad);   // north
  float ay = linXForward * sin(headingRad);   // east

  // Stationary gating to limit bias drift.
  if (fabsf(linXForward) < 0.05f) {
    ax = 0.0f;
    ay = 0.0f;
    kf.vx *= 0.85f;
    kf.vy *= 0.85f;
  }

  // State propagation (as above)
  kf.x  += kf.vx * dt + 0.5f * ax * dt * dt;
  kf.y  += kf.vy * dt + 0.5f * ay * dt * dt;
  kf.vx += ax * dt;
  kf.vy += ay * dt;

  // Build F matrix (Jacobian wrt state)
  // [1,0,dt,0
  //  0,1,0, dt
  //  0,0,1, 0
  //  0,0,0, 1]
  float F[4][4] = {
    {1.0f, 0.0f, dt,   0.0f},
    {0.0f, 1.0f, 0.0f, dt  },
    {0.0f, 0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 1.0f}
  };

  // Process noise: heuristics for acceleration-driven motion.
  // Position noise grows with dt^2, velocity noise grows with dt.
  float Q[4][4] = {
    {Q_POS * dt * dt, 0, 0, 0},
    {0, Q_POS * dt * dt, 0, 0},
    {0, 0, Q_VEL * dt, 0},
    {0, 0, 0, Q_VEL * dt}
  };

  // P = F P F^T + Q
  float FP[4][4];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      FP[i][j] = 0.0f;
      for (int k = 0; k < 4; k++) {
        FP[i][j] += F[i][k] * P[k][j];
      }
    }
  }

  float Pnew[4][4];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      Pnew[i][j] = 0.0f;
      for (int k = 0; k < 4; k++) {
        // Multiply by F^T: (F^T)[k][j] = F[j][k]
        Pnew[i][j] += FP[i][k] * F[j][k];
      }
      Pnew[i][j] += Q[i][j];
    }
  }

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      P[i][j] = Pnew[i][j];
}

// ─────────────────────────────────────────────
// GPS UPDATE
// Always fuses position (lat/lon).
// If you pass useVel=true, it also fuses velocity from RMC (speed+course).
// ─────────────────────────────────────────────
void kalmanUpdateGPS(double lat, double lon, bool useVel, float speedKnots, float courseDeg) {
  if (!kfInitialized) {
    initKalman(lat, lon);
    lastGoodGPSFixTime = millis();
    return;
  }

  double mPerDegLat = 111111.0;
  double mPerDegLon = 111111.0 * cos(originLat * PI / 180.0);

  float zx = (float)((lat - originLat) * mPerDegLat);
  float zy = (float)((lon - originLon) * mPerDegLon);

  // Position updates (scalar measurements)
  kalmanUpdateScalar(0, zx, R_GPS_POS);
  kalmanUpdateScalar(1, zy, R_GPS_POS);
  lastGoodGPSFixTime = millis();

  if (!useVel) return;

  // NMEA RMC speed is in knots, course is degrees from North (0=N, 90=E).
  float speedMps = speedKnots * KNOTS_TO_MPS;
  if (speedMps < MIN_GPS_VEL_FUSE_MPS) return;
  float courseRad = courseDeg * PI / 180.0f;

  float z_vx = speedMps * cos(courseRad); // north velocity
  float z_vy = speedMps * sin(courseRad); // east velocity

  // Velocity updates (scalar measurements)
  kalmanUpdateScalar(2, z_vx, R_GPS_VEL);
  kalmanUpdateScalar(3, z_vy, R_GPS_VEL);
}

// ─────────────────────────────────────────────
// GET LAT/LON from local-frame state
// ─────────────────────────────────────────────
void getLatLon(double& lat, double& lon) {
  double mPerDegLat = 111111.0;
  double mPerDegLon = 111111.0 * cos(originLat * PI / 180.0);

  lat = originLat + (kf.x / mPerDegLat);
  lon = originLon + (kf.y / mPerDegLon);
}

// ─────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(1000);

  pinMode(gps_red,   OUTPUT);
  pinMode(gps_green, OUTPUT);
  pinMode(lux_red,   OUTPUT);
  pinMode(lux_green, OUTPUT);
  pinMode(imu_red,   OUTPUT);
  pinMode(imu_green, OUTPUT);
  setLED(gps_red, gps_green, "RED");
  setLED(lux_red, lux_green, "RED");
  setLED(imu_red, imu_green, "RED");

  // --- BUTTON PIN SETUP ---
  pinMode(BTN_SLEEP,      INPUT_PULLUP);
  pinMode(BTN_LOG_TOGGLE, INPUT_PULLUP);
  pinMode(BTN_FORCE_LOG,  INPUT_PULLUP);

// ── LCD STARTUP ─────────────────────────────────────────────────────
  Serial.println(F("LCD STARTUP"));
  lcd.init();
  lcd.backlight();
  lcd.display();
  lcd.home();

  // ── IMU ────────────────────────────────────────────────────────────
  lcd.setCursor(0, 0); lcd.print("IMU: ");
  Serial.println(F("IMU CHECK"));
  if (!bnoBegin()) { // [cite: 88]
    Serial.println(F("IMU FAIL - halting"));
    lcd.print("FAIL - HALTED");
    setLED(imu_red, imu_green, "RED");
    while (1) ; // [cite: 89]
  }
  Serial.println(F("IMU found"));
  lcd.print("PASS");
  setLED(imu_red, imu_green, "GREEN");

  // ── GPS ────────────────────────────────────────────────────────────
  lcd.setCursor(0, 1);
  lcd.print("GPS: ");
  Serial.println(F("GPS CHECK"));
  GPS.begin(9600); // [cite: 91]
  delay(500);
  GPS.sendCommand("$PMTK251,38400*27"); // [cite: 91]
  delay(500);                             
  GPS.begin(38400); // [cite: 92]
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // [cite: 92]
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // [cite: 92]
  GPS.sendCommand(PGCMD_ANTENNA); // [cite: 92]
  delay(1000);
  Serial.println(F("GPS OK"));
  lcd.print("PASS");
  setLED(gps_red, gps_green, "GREEN"); 
  gpsSetupTime = millis();
  // ── Light sensor ────────────────────────────────────────────────────
  lcd.setCursor(0, 2); lcd.print("LUX: ");
  Serial.println(F("LIGHT SENSOR CHECK"));
  if (!tsl.begin()) { // [cite: 93]
    Serial.println(F("TSL2591 not found - halting"));
    lcd.print("FAIL - HALTED");
    setLED(lux_red, lux_green, "RED");
    while (1) ; // [cite: 94]
  }
  Serial.println(F("TSL2591 found"));
  lcd.print("PASS");
  setLED(lux_red, lux_green, "GREEN"); // [cite: 94]
  configureSensor(); // [cite: 94]

  // ── SD ──────────────────────────────────────────────────────────────
  lcd.setCursor(0, 3); lcd.print("SD:  ");
  Serial.println(F("SD CHECK"));
  if (!SD.begin(chipSelect)) { // [cite: 95]
    Serial.println(F("SD FAIL - halting"));
    lcd.print("FAIL - HALTED");
    while (1) ; // [cite: 96]
  }
  Serial.println(F("SD OK"));
  lcd.print("PASS");
  
  delay(2500); // Hold the screen so the user can see the successful setup
  lcd.clear();
  Serial.println(F("STARTING LOOP...")); // [cite: 96]
}

// ─────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────
void loop() {
// ── 0. Check Buttons ─────────────────────────────────────────────────  
  // Sleep Button (Button 1)
  if (digitalRead(BTN_SLEEP) == LOW) { // Changed HIGH to LOW
      if (millis() - lastBtnSleepTime > debounceDelay) { // Changed < 1000 to > debounceDelay
        isSleeping = !isSleeping;
        if (isSleeping) {
          lcd.noBacklight();
          lcd.clear();
          setLED(gps_red, gps_green, "OFF");
          setLED(lux_red, lux_green, "OFF");
          setLED(imu_red, imu_green, "OFF");
          Serial.println(F("Sleep Mode: ON"));
        } else {
          lcd.backlight();
          lcd.clear();
          lcd.print("Waking up...");
          Serial.println(F("Sleep Mode: OFF"));
        }
        lastBtnSleepTime = millis();
      }
    }

  // If sleeping, keep GPS buffer clean but skip everything else
  if (isSleeping) {
    GPS.read();
    return;
  }

  // Logging Toggle Button (Button 2)
  if (digitalRead(BTN_LOG_TOGGLE) == LOW) {
    if (millis() - lastBtnLogTime > debounceDelay) {
      loggingEnabled = !loggingEnabled;
      lcd.setCursor(0, 3);
      if (loggingEnabled) lcd.print("Logging: ON         ");
      else                lcd.print("Logging: OFF        ");
      Serial.print(F("Data Logging: ")); 
      Serial.println(loggingEnabled ? "ON" : "OFF");
      lastBtnLogTime = millis();
    }
  }

  // Force Log Button (Button 3)
  if (digitalRead(BTN_FORCE_LOG) == LOW) {
    if (millis() - lastBtnForceTime > debounceDelay) {
      forceLog = true;
      Serial.println(F("Force Log Requested"));
      lastBtnForceTime = millis();
    }
  }
  // ── 1. Feed GPS parser ──────────────────────────────────────────────
  //GPS.read();
  GPS.read();

  if (GPS.newNMEAreceived()) {
    // Capture the sentence BEFORE parse() clears the buffer
    char* nmea = GPS.lastNMEA();
    bool gotRMC = (strstr(nmea, "RMC") != NULL);

    if (GPS.parse(nmea)) {
      if (GPS.fix) {
        lastFixTime = millis();
        Serial.print("Fix: ");
        Serial.print(GPS.fix);
        Serial.print("  Sats: ");
        Serial.println(GPS.satellites);
        if (GPS.satellites >= MIN_SATS) {
          kalmanUpdateGPS(
            GPS.latitudeDegrees,
            GPS.longitudeDegrees,
            gotRMC,
            GPS.speed,
            GPS.angle
          );
        } else {
          Serial.println(F("GPS fix rejected - low satellite count"));
        }
      } 
    }
  }
  bool recentFix = (lastFixTime > 0) && (millis() - lastFixTime <= 30000);
  bool recentlySetup = (millis() - gpsSetupTime <= 30000);

  if (recentFix) {
    // We had a fix within the last 30 seconds
    if (GPS.satellites < MIN_SATS) {
      setLED(gps_red, gps_green, "YELLOW"); // Satellite count is too low
    } else {
      setLED(gps_red, gps_green, "GREEN");  // Good fix!
    }
  } else if (recentlySetup) {
    // Passed setup < 30s ago, still searching for the initial fix
    setLED(gps_red, gps_green, "GREEN"); 
  } else {
    // Hasn't passed setup recently AND no fix within the last 30 seconds
    setLED(gps_red, gps_green, "RED");
  }
// ── 2. Wait for first good GPS fix ─────────────────────────────────
  if (!kfInitialized) { // 
    static unsigned long lastSearchUpdate = 0;
    // Update the screen every 1 second to avoid flickering
    if (millis() - lastSearchUpdate > 1000) {
      lcd.setCursor(0, 0); lcd.print("SEARCHING FOR ANCHOR");
      lcd.setCursor(0, 1); lcd.print("Sats Found: "); 
      lcd.print(GPS.satellites); 
      lcd.print("   "); // Spaces clear trailing numbers if sats drop
      lastSearchUpdate = millis();
    }
    return;
  }

  // ── 3. EKF predict using IMU ────────────────────────────────────────
  float linX, linY, linZ;
  float eulX, eulY, eulZ;


  if (bnoReadVector(BNO_REG_LIN_ACC_X_LSB, 100.0f, linX, linY, linZ) &&
      bnoReadVector(BNO_REG_EULER_H_LSB,   16.0f,  eulX, eulY, eulZ)) {
    // In your original code, eulX is treated as absolute heading.
    setLED(imu_red, imu_green, "GREEN");
    float headingRad = eulX * PI / 180.0f;
    kalmanPredict(linX, headingRad);
  } else {
    setLED(imu_red, imu_green, "RED");
  }

  // ── 4. Compute fused lat/lon ───────────────────────────────────────
  double lat, lon;
  getLatLon(lat, lon);

  // ── 5. Check 5-foot spacing ───────────────────────────────────────
  float dist = distanceMeters(lastLogX, lastLogY, kf.x, kf.y);
  float distFt = dist * 3.28084f;
  bool gpsFixFresh = (millis() - lastGoodGPSFixTime) <= GPS_FIX_FRESH_MS;
  if (dist >= logTriggerDistance || forceLog) {
    //if (!gpsFixFresh) {
      //Serial.println(F("Log delayed - waiting for fresh GPS anchor"));
      //return;40.43069149928217,-86.91537766489887
   // }

    // ── 6. Tilt check ───────────────────────────────────────────────
    if (!bnoReadVector(BNO_REG_EULER_H_LSB, 16.0f, eulX, eulY, eulZ)) {
      Serial.println(F("IMU read failed during tilt check"));
      return;
    }

    float rollDeg  = eulY;
    float pitchDeg = eulZ;
    bool  level    = (fabsf(rollDeg)  < levelTolerance &&
                       fabsf(pitchDeg) < levelTolerance);

    if (!level) {
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("SENSOR NOT LEVEL");
      lcd.setCursor(0, 1); lcd.print("Roll:  ");  lcd.print(rollDeg,  1);
      lcd.setCursor(0, 2); lcd.print("Pitch: "); lcd.print(pitchDeg, 1);
      lcd.setCursor(0, 3); lcd.print("Re-level and move");
      Serial.println(F("Tilt detected - waiting for level"));
      return; // lastLogX/Y unchanged
    }

    // ── 7. Read light sensor ─────────────────────────────────────────
    uint32_t lum  = tsl.getFullLuminosity();
    uint16_t ir   = lum >> 16;
    uint16_t full = lum & 0xFFFF;
    float    lux  = tsl.calculateLux(full, ir);
    float    fc   = lux / fc_conversion;

    // ── 8. Update LCD ───────────────────────────────────────────────
    lcd.clear();
    lcd.setCursor(0, 0); 
    lcd.print("Sats:"); 
    lcd.print(GPS.satellites);
    
    float kfVariance = P[0][0] + P[1][1];
    float kfstdev = sqrt(kfVariance);
    float confidence = 100.0f * (1.0f - (kfstdev / 15.24f));
    if (confidence < 0) confidence = 0;
    if (confidence > 100) confidence = 100;
    lcd.print(" Conf:");
    lcd.print((int)confidence);
    lcd.print("%");
    
    // Lines 2-4: FC, Lat, Lon
    lcd.setCursor(0, 1); lcd.print("FC: "); lcd.print(fc, 2);
    lcd.setCursor(0, 2); lcd.print("Lat: "); lcd.print(lat, 5);
    lcd.setCursor(0, 3); lcd.print("Lon:  "); lcd.print(lon, 5);
    Serial.println(fc, 2);
    Serial.println(lat, 5);
    Serial.println(lon, 5);

    // ── 9. Write CSV row ─────────────────────────────────────────────
    File f = SD.open("datalog.csv", FILE_WRITE);
    if (f && loggingEnabled) {
      f.print(GPS.day);   f.print("/");
      f.print(GPS.month); f.print("/");
      f.print(GPS.year);  f.print(",");

      f.print(GPS.hour);   f.print(":");
      if (GPS.minute  < 10) f.print("0");
      f.print(GPS.minute); f.print(":");
      if (GPS.seconds < 10) f.print("0");
      f.print(GPS.seconds); f.print(",");

      f.print(fc, 2);    f.print(",");
      f.print(lat, 6);   f.print(",");
      f.println(lon, 6);
      f.close();

      Serial.print(F("Logged  fc="));  Serial.print(fc, 2);
      Serial.print(F("  lat="));       Serial.print(lat, 6);
      Serial.print(F("  lon="));       Serial.println(lon, 6);
    } else {
      Serial.println(F("ERROR: could not open datalog.csv"));
    }

    // Advance the spacing reference only after a successful, GPS-anchored log.
    lastLogX = kf.x;
    lastLogY = kf.y;
    forceLog = false;
  }
}

