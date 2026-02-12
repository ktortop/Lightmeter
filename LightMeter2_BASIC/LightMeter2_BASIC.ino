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

  // constants

const int chipSelect = 53;
const int batteryPin = A0;
const float fc_conversion = 10.764;
const float batteryMax = 4.35;
const float batteryMin = 3.30;

// Distance tracking 

float distanceAccumulated = 0.0;
float lastLat, lastLon;
bool hasLastFix = false;

LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // establishes a light sensor code, required if there are multiple sensors
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // establishes IMU
SoftwareSerial mySerial(11, 10);
// SoftwareSerial mySerial(11, 10);    RX=11, TX=10 (to GPS TX/RX appropriately)

Adafruit_GPS GPS(&mySerial);

#define GPSECHO false // set true ONLY if you want raw NMEA spam

uint32_t timer = 0;

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

void setup()
{
  Serial.begin(9600);
  delay(500);

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

  Serial.println("IMU CHECK");
  if (!bno.begin()) {
      Serial.println("Failed to initialize IMU");
      while (1);
    }
  else
  {
      Serial.println("IMU found");
  }
  Serial.println("LIGHT.SENSOR.CHECK");
  if (tsl.begin())
  {
    Serial.println(F("Found a TSL2591 sensor"));
  }
  else
  {
    Serial.println(F("No light sensor found ... check wiring"));
    while (1)
      ;
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

//gets distance betweeen two points
float getDisplacement(float lat1, float lon1, float lat2, float lon2)
{
  const float R = 6371000.0; // Earth radius in meters

  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);

  lat1 = radians(lat1);
  lat2 = radians(lat2);

  float a = sin(dLat/2)*sin(dLat/2) +
            cos(lat1)*cos(lat2) *
            sin(dLon/2)*sin(dLon/2);

  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}
//Uses the IMU to determine if the cart is moving
bool isMoving()
{
  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  float mag = sqrt(
    linAccel.x()*linAccel.x() +
    linAccel.y()*linAccel.y() +
    linAccel.z()*linAccel.z()
  );

  return mag > 0.15;  // threshold (tune this)
}

void loop()
{
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
//Tracks distance using GPS & IMU
  if (GPS.fix)
{
  float lat = GPS.latitudeDegrees;
  float lon = GPS.longitudeDegrees;

  // First fix initialization
  if (!hasLastFix)
  {
    lastLat = lat;
    lastLon = lon;
    hasLastFix = true;
  }
  else
  {

    if (isMoving())   // walking threshold (tune later)
    {
      float d = getDisplacement(lastLat, lastLon, lat, lon);

      // Reject GPS glitches
      if (d > 0.05 && d < 10.0)
      {
        distanceAccumulated += d;
      }

      lastLat = lat;
      lastLon = lon;
    }
  }
  }

  // Print light sensor data every 5 meters
  if (distanceAccumulated >= 5.0)
  {
    distanceAccumulated -= 5.0;

    // Light read
    uint32_t lum = tsl.getFullLuminosity();
    uint16_t ir = lum >> 16;
    uint16_t full = lum & 0xFFFF;
    float lux = tsl.calculateLux(full, ir);

    Serial.print("lux=");
    Serial.print(lux);
    Serial.print("  sats=");
    Serial.print(GPS.satellites);
    Serial.print("  fix=");
    Serial.print(GPS.fix);

    lcd.clear();

    // battery read
    /*
    int raw = analogRead(A0);
    float batteryVoltage = (raw*5.0)/1023.0;
    float batteryPct = constrain((batteryVoltage-batteryMin)/(batteryMax-batteryMin)*100,0,100);
    lcd.setCursor(0,0);
    lcd.print("Batt: ");
    lcd.print((int)batteryPct);
    lcd.print("%");
    */

    lcd.setCursor(0, 1); // luminosity display
    lcd.print("Footcandles: ");
    lcd.print(lux/fc_conversion);

    if (GPS.fix)
    {
      lcd.setCursor(0, 2); // coordinate display
      lcd.print("Lat: ");
      lcd.print(GPS.latitude, 4);
      lcd.setCursor(0, 3);
      lcd.print("Long:");
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

      dataString += String(lux, 2); // references the function. I am not sure if it runs the function again to get this. Regardless, it works
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
        dataFile.print(GPS.latitude, 4);
        dataFile.print(",");
        // separated by a comma
        dataFile.println(GPS.longitude, 4);
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
    }
    // we only want the meter to plot when there is a GPS fix, otherwise we can make it do something else by writing some code.
    // there are other GPS parameters, like the number of satilites termed "quality". you can find the return code for that in the GPS testing file.
  }
}
