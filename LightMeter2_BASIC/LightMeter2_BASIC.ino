#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>

// constants

const int chipSelect = 53;
const int batteryPin = A0;
float batteryMax = 4.35;
float batteryMin = 3.30;

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // establishes a light sensor code, required if there are multiple sensors
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

  Serial.println("GPS + LIGHT METER (NO SD)");
  Serial.println("GPS.CHECK");

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
  }
  else
  {
    Serial.println(F("No sensor found ... check wiring"));
    while (1)
      ;
  }

  Serial.println("SENSOR.CONFIG.CHECK");
  configureSensor();
  Serial.println("SENSOR.CONFIG.PASS");

  Serial.println("STARTING LOOP...");
  timer = millis();
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

  // ---- Print status every 2 seconds ----
  if (millis() - timer >= 2000)
  {
    timer = millis();

    // Light read
    uint32_t lum = tsl.getFullLuminosity();
    uint16_t ir = lum >> 16;
    uint16_t full = lum & 0xFFFF;
    float lux = tsl.calculateLux(full, ir);

    Serial.print("lux=");
    Serial.print(lux, 2);
    Serial.print("  sats=");
    Serial.print(GPS.satellites);
    Serial.print("  fix=");
    Serial.print(GPS.fix);

    if (GPS.fix)
    {
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
    }
    else
    {
      Serial.print("  NO FIX (go outside / near window)");
    }

    if (millis() - timer > 2000)
    {
      timer = millis(); // reset the timer
      // we only want the meter to plot when there is a GPS fix, otherwise we can make it do something else by writing some code.
      // there are other GPS parameters, like the number of satilites termed "quality". you can find the return code for that in the GPS testing file.
      if (GPS.fix)
      {

        // light level read

        uint32_t lum = tsl.getFullLuminosity();
        uint16_t ir, full;
        ir = lum >> 16;
        full = lum & 0xFFFF;

        lcd.clear();

        // battery read
        /*
        int raw = analogRead(A0);
        float batteryVoltage = (raw*5.0)/1023.0;
        float batteryPct = constrain((batteryVoltage-batteryMin)/(batteryMax-batteryMin)*100,0,100);

        lcd.setCursor(0,0);
        lcd.print("%: ");
        lcd.print((int)batteryPct);
        */
        lcd.setCursor(0, 1); // luminosity display
        lcd.print("Lux: ");
        lcd.print(tsl.calculateLux(full, ir));

        lcd.setCursor(0, 2); // coordinate display
        lcd.print("Lat: ");
        lcd.print(GPS.latitude, 3);
        lcd.setCursor(0, 3);
        lcd.print(GPS.longitude);

        // establish a string of data to be put into CSV format. Strings are basically just excell files that the computer can reference.

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

        dataString += String(tsl.calculateLux(full, ir), 2); // references the function. I am not sure if it runs the function again to get this. Regardless, it works
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
          Serial.print(dataString);
          Serial.print(GPS.latitude, 4);
          Serial.print(",");
          Serial.println(GPS.longitude, 4);
        }
        // if the file isn't open or available, pop up an error:
        else
        {
          Serial.println("error opening DATALOG.csv");
        }
      }
      // if there is no FIX, we can make the code do something here, hint:lights or sounds would go here
    }
  }
