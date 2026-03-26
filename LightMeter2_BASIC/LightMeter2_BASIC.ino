#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <LiquidCrystal_I2C.h>

// constants

const int chipSelect = 53;
const int batteryPin = A0;
const float fc_conversion = 10.76391;
const float batteryMax = 4.35;
const float batteryMin = 3.30;


const int gps_red = 2;
const int gps_green = 3;
const int lux_red = 5;
const int lux_green = 6;


/* BUTTON CODE
const int BTN_POWER = 5;
const int BTN_PLOT = 6;
const int BTN_CAL = 7;
const unsigned long DEBOUNCE_MS = 60;
*/

LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // establishes a light sensor code, required if there are multiple sensors
SoftwareSerial mySerial(11, 10);
// SoftwareSerial mySerial(11, 10);    RX=11, TX=10 (to GPS TX/RX appropriately)

Adafruit_GPS GPS(&mySerial);

#define GPSECHO false // set true ONLY if you want raw NMEA spam

struct Button {
  int pin;
  bool lastStable;
  bool lastReading;
  unsigned long lastChangeMs;
};

/* BUTTON CODE
Button bPower {BTN_POWER, true, true, 0};
Button bPlot {BTN_PLOT, true, true,0};
Button bCal {BTN_CAL, true, true, 0};

bool deviceAwake = true;
bool loggingOn = true;
bool inCalMode = false;
*/

uint32_t timer = 0;

float luxSum = 0;     // sum of all lux reading values
unsigned long luxCount = 0;   // number of readings taken
float luxAvg = 0;     

// GPS outlier tracking
const int GPS_WINDOW = 10;           // rolling window size
const float GPS_OUTLIER_THRESH = 0.0005; // ~50m deviation in degrees
const int GPS_OUTLIER_LIMIT = 4;     // warn if this many outliers in window

float latHistory[GPS_WINDOW] = {0};
float lonHistory[GPS_WINDOW] = {0};
int gpsHistoryIndex = 0;
int gpsHistoryCount = 0;
int gpsOutlierCount = 0;
bool gpsWarning = false;

void configureSensor()
{
  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);

  Serial.println(F("------------------------------------"));
  Serial.print(F("Gain: "));
  tsl2591Gain_t gain = tsl.getGain();
  switch (gain)
  {
  case TSL2591_GAIN_LOW:
    Serial.println(F("1x (Low)"));
    break;
  case TSL2591_GAIN_MED:
    Serial.println(F("25x (Medium)"));
    break;
  case TSL2591_GAIN_HIGH:
    Serial.println(F("428x (High)"));
    break;
  case TSL2591_GAIN_MAX:
    Serial.println(F("9876x (Max)"));
    break;
  }
}
/* BUTTON CODE
bool buttonPressed(Button &b){
  bool reading = digitalRead(b.pin);

  if (reading != b.lastReading){
    b.lastChangeMs = millis();
    b.lastReading = reading;
  }

  if ((millis() - b.lastChangeMs) > DEBOUNCE_MS){
    if (reading != b.lastStable){
      b.lastStable = reading;
      if (b.lastStable == LOW){
        return true;
      }
    }
  }
  return false;
}
*/


void setLED(int rPin, int gPin, String color) {
  if (color == "RED") {
    digitalWrite(rPin, LOW);
    digitalWrite(gPin, HIGH);
  }
  else if (color == "GREEN") {
    digitalWrite(rPin, HIGH);
    digitalWrite(gPin, LOW);
  }
  else if (color == "YELLOW") {
    digitalWrite(rPin, LOW);
    digitalWrite(gPin, LOW);
  }
  else {
    digitalWrite(rPin, HIGH);
    digitalWrite(gPin, HIGH);
  }
}


void setup()
{
  Serial.begin(9600);
  Wire.begin();
  delay(500);

  pinMode(gps_red, OUTPUT);
  pinMode(gps_green, OUTPUT);
  pinMode(lux_red, OUTPUT);
  pinMode(lux_green, OUTPUT);

  setLED(gps_red, gps_green, "OFF");
  setLED(lux_red, lux_green, "OFF");


  /* BUTTON CODE
  pinMode(BTN_POWER, INPUT_PULLUP);
  pinMode(BTN_PLOT, INPUT_PULLUP);
  pinMode(BTN_CAL, INPUT_PULLUP);
  */

  Serial.println("LCD STARTUP");
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("LIGHT METER STARTING");

  Serial.println("GPS CHECK");

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);

  Serial.println("GPS.PASS");

  Serial.println("LIGHT.SENSOR.CHECK");
  if (tsl.begin())
  {
    Serial.println(F("Found a TSL2591 sensor"));
    setLED(lux_red, lux_green, "GREEN");
  }
  else
  {
    Serial.println(F("No sensor found ... check wiring"));
    setLED(lux_red, lux_green, "RED");
  }

  if (!SD.begin(chipSelect)){
    Serial.println("SD FAIL");
  }

  Serial.println("SENSOR.CONFIG.CHECK");
  configureSensor();
  Serial.println("SENSOR.CONFIG.PASS");

  Serial.println("STARTING LOOP...");
  timer = millis();
}

void loop()
{
/* BUTTON CODE
  if (buttonPressed(bPower)){
    deviceAwake = !deviceAwake;
      
    if (!deviceAwake){
    loggingOn = false;
    }

    Serial.println(deviceAwake ? "POWER: WAKE" : "POWER: SLEEP");
  }

  if (buttonPressed(bPlot)) {
    // Only allow plot toggle if awake
    if (deviceAwake) {
      loggingOn = !loggingOn;
      Serial.println(loggingOn ? "LOGGING: ON" : "LOGGING: OFF");
    }
  }

  if (buttonPressed(bCal)) {
    if (deviceAwake) {
      inCalMode = true;

      configureSensor();
      resetIMUPlaceholder();

      Serial.println("CAL: SENSOR CONFIG RESET");
    }
  }

  if (!deviceAwake) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("SLEEP MODE");
    display.println("Press POWER to wake");
    display.print("SD: "); display.println(sdOK ? "OK" : "FAIL");
    display.print("LOG: "); display.println(loggingOn ? "ON" : "OFF");
    display.display();
    }
  delay(100);
  return;
  }
  */  

  // ---- Read GPS characters continuously ----
  char c = GPS.read();
  if (GPSECHO && c)
    Serial.write(c);

  // If a full sentence arrived, parse it
  if (GPS.newNMEAreceived())
  {
    if (!GPS.parse(GPS.lastNMEA()))
    {
      return; // wait for another sentence
    }
  }

  // Light read
  
  uint32_t lum = tsl.getFullLuminosity();
  if (lum >= 0.01){
    setLED(lux_red, lux_green, "GREEN");
  }
  else{
    setLED(lux_red, lux_green, "RED");
    Serial.println("Light senor disconnected");
  }
  uint16_t ir = lum >> 16;
  uint16_t full = lum & 0xFFFF;
  float lux = tsl.calculateLux(full, ir);

  luxSum += lux;
  luxCount++;

  // ---- Print status every 2 seconds ----
  if (millis() - timer >= 1000)
  {

    luxAvg = luxSum / luxCount / fc_conversion;

    timer = millis(); 

    Serial.print("lux=");
    Serial.print(luxAvg);
    Serial.print("  sats=");
    Serial.print(GPS.satellites);
    Serial.print("  fix=");
    Serial.print(GPS.fix);
    Serial.print("\n");

    lcd.clear();

// --- GPS outlier check (only when we have a fix) ---
if (GPS.fix && gpsHistoryCount > 0) {
  // compute average of history window
  float latSum = 0, lonSum = 0;
  int validCount = min(gpsHistoryCount, GPS_WINDOW);
  for (int i = 0; i < validCount; i++) {
    latSum += latHistory[i];
    lonSum += lonHistory[i];
  }
  float latMean = latSum / validCount;
  float lonMean = lonSum / validCount;

  // is the current reading an outlier?
  bool isOutlier = (abs(GPS.latitudeDegrees - latMean) > GPS_OUTLIER_THRESH ||
                    abs(GPS.longitudeDegrees - lonMean) > GPS_OUTLIER_THRESH);

  // store current reading into rolling window
  latHistory[gpsHistoryIndex] = GPS.latitudeDegrees;
  lonHistory[gpsHistoryIndex] = GPS.longitudeDegrees;
  gpsHistoryIndex = (gpsHistoryIndex + 1) % GPS_WINDOW;
  if (gpsHistoryCount < GPS_WINDOW) gpsHistoryCount++;

  // rolling outlier count: recount from scratch each cycle
  gpsOutlierCount = 0;
  float latSum2 = 0, lonSum2 = 0;
  int n = min(gpsHistoryCount, GPS_WINDOW);
  for (int i = 0; i < n; i++) { latSum2 += latHistory[i]; lonSum2 += lonHistory[i]; }
  float lm2 = latSum2 / n, lnm2 = lonSum2 / n;
  for (int i = 0; i < n; i++) {
    if (abs(latHistory[i] - lm2) > GPS_OUTLIER_THRESH ||
        abs(lonHistory[i] - lnm2) > GPS_OUTLIER_THRESH) {
      gpsOutlierCount++;
    }
  }

  gpsWarning = (gpsOutlierCount >= GPS_OUTLIER_LIMIT);
} else if (GPS.fix) {
  // first fix reading — just seed the history
  latHistory[0] = GPS.latitudeDegrees;
  lonHistory[0] = GPS.longitudeDegrees;
  gpsHistoryIndex = 1;
  gpsHistoryCount = 1;
}

// --- LCD row 0: warning or blank ---
lcd.setCursor(0, 0);
if (gpsWarning) {
  lcd.print("! GPS INACCURATE    ");
} else {
  lcd.print("                    "); // clear row
}

    lcd.setCursor(0, 1); // luminosity display
    lcd.print("Footcandles:");
    lcd.print(luxAvg);

    if (GPS.fix /*&& BUTTON CODE loggingOn*/)
    {
      /*
      if (GPS.satellites >= 4); {
        setLED(gps_red, gps_green, "GREEN");
      }
      else {
        setLED(gps_red, gps_green, "YELLOW");
      }
      */
      lcd.setCursor(0, 2); // coordinate display
      lcd.print("X: ");
      lcd.print(GPS.latitude, 4);                
      lcd.setCursor(0, 3);
      lcd.print("Y:");
      lcd.print(GPS.longitude, 4);

      /*
      Serial.print("  ");
      Serial.print(GPS.month);
      Serial.print("/");
      Serial.print(GPS.day);
      Serial.print("/20");
      Serial.print(GPS.year);
      Serial.print("  ");

      Serial.print(GPS.hour);
      Serial.print(":");
      Serial.print(GPS.minute);
      Serial.print(":");
      Serial.print(GPS.seconds);

      Serial.print("  lat=");
      Serial.print(GPS.latitude, 4);
      Serial.print(GPS.lat);

      Serial.print("  lon=");
      Serial.print(GPS.longitude, 4);
      Serial.print(GPS.lon);
      */

      String dataString = "";

      // date input
      // the format for input to a particular string is type the string name.
      // type up to the (
      // type the varaible name OR any text in ""
      // to create a CSV, you must Separate Values by Commas, so end by adding each field with a comma

      dataString += String(GPS.day);
      dataString += String("/");
      dataString += String(GPS.month);
      dataString += String("/");
      dataString += String(GPS.year);
      dataString += ",";

      // time input

      dataString += String(GPS.hour);
      dataString += String(':');
      dataString += String(GPS.minute);
      dataString += String(':');
      dataString += String(GPS.seconds);
      dataString += ",";

      dataString += String(luxAvg/fc_conversion, 2); // references the function. I am not sure if it runs the function again to get this. Regardless, it works
      dataString += ",";

      File dataFile = SD.open("datalog.csv", FILE_WRITE); // this writes to a particular file on the SD card
      // there can be multiple files set up on this, so if you would like to make a USER HISTORY, another file could write the start time and then overwrite an end time until the unit shuts off.
      // then annother entry can start up when you begin.

      if (dataFile)
      {
        // print the string made above
        dataFile.print(dataString);
        // lat and lon must be printed directly to the SD file because of some dumb
        // way the machine stores the number of decimal points.
        dataFile.print(GPS.latitudeDegrees, 6);
        dataFile.print(",");
        // separated by a comma
        dataFile.println(GPS.longitudeDegrees, 6);
        // NOT separated by a comma, but added a new line (nl)
        // the 4 is for the number of decimal points
        dataFile.close();
        // print to the serial port too:
        Serial.print("SD Write OK");
      }
      // if the file isn't open or available, pop up an error:
      else
      {
        Serial.println("error opening DATALOG.csv");
      }
    }
    else
    {
      Serial.print("  NO FIX (go outside / near window)");
      setLED(gps_red, gps_green, "RED");
    }
    // we only want the meter to plot when there is a GPS fix, otherwise we can make it do something else by writing some code.
    // there are other GPS parameters, like the number of satilites termed "quality". you can find the return code for that in the GPS testing file.
  }
}