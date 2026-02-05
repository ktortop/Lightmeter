
// library setup

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
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
Adafruit_GPS GPS(&mySerial);

#define GPSECHO true // sends data to Serial Monitor

// setup code runs once
void setup()
{
  Serial.begin(115200);                 // set baud rate to 115200 
  lcd.init();                           // start up the lcd
  lcd.backlight();
  lcd.setCursor(0,0);
  delay(1000);                          // wait
  lcd.print("GPS SD LIGHT METER");
  Serial.println("GPS SD LIGHT METER"); // print startup and fun dots
  delay(200);
  Serial.print(".");
  delay(200);
  Serial.print(".");
  delay(200);
  Serial.println(".");

  /*
  Serial.println("SD.CHECK"); // begins check for SD card Reader

  if (!SD.begin(chipSelect))
  {
    Serial.println("Card failed, or not present");
    // dititalWrite(5) = HIGH; // if connnected at pin 5, an LED light turns on
    while (1)
      ; // don't do anything more:
  }
  // if an SD card is not present, the program halts here.
  // if an SD card is present, the program continues

  Serial.println("SD.PASS");
  */

  delay(5000);

  Serial.println("GPS.CHECK");

  GPS.begin(9600); // I honestly do not understand the complexity of the GPS parsing, but all of this is required
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);

  Serial.println("GPS.PASS"); // if all of this runs, it works!

  Serial.println("LIGHT.SENSOR.CHECK");

  if (tsl.begin())
  {
    Serial.println(F("Found a TSL2591 sensor"));
  }
  else
  {
    Serial.println(F("No sensor found ... check your wiring?"));
    while (1)
      ;
  }

  Serial.println("SENSOR.PASS");

  Serial.println("SENSOR.CONFIG.CHECK");
  configureSensor();
  Serial.println("SENSOR.CONFIG.PASS");

  Serial.println("AWAIT.FIX.FOR.LOOP.START"); // Prints to Serial, that Startup has ended
}

// This code is a few functions that the rest of the loop runs.

void simpleRead(void)
{
  // Simple data read example. Just read the infrared, fullspecrtrum diode
  // or 'visible' (difference between the two) channels.
  // This can take 100-600 milliseconds! Uncomment whichever of the following you want to read
  uint16_t x = tsl.getLuminosity(TSL2591_VISIBLE);
  // uint16_t x = tsl.getLuminosity(TSL2591_FULLSPECTRUM);
  // uint16_t x = tsl.getLuminosity(TSL2591_INFRARED);
}
void advancedRead(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  //  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  //  Serial.print(F("IR: ")); Serial.print(ir);  Serial.print(F("  "));
  //  Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  "));
  //  Serial.print(F("Visible: ")); Serial.print(full - ir); Serial.print(F("  "));
  //  Serial.print(F("Lux: ")); Serial.println(tsl.calculateLux(full, ir), 6);
}

void configureSensor(void)
{
  // tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl.setGain(TSL2591_GAIN_MED); // 25x gain
  // tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain

  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */
  Serial.println(F("------------------------------------"));
  Serial.print(F("Gain:         "));
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

uint32_t timer = millis();

void loop() // The meat and potatoes. This runs constantly after the above code has been processed.

{
  advancedRead();      // each loop we get a new light value.
  char c = GPS.read(); // read the gps status
  if ((c) && (GPSECHO))
    if (GPS.newNMEAreceived())
    {
      if (!GPS.parse(GPS.lastNMEA()))
        return;
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
      lcd.setCursor(0,1); //luminosity display
      lcd.print("Lux: ");
      lcd.print(tsl.calculateLux(full, ir));
      
      lcd.setCursor(0,2); //coordinate display
      lcd.print("Lat: ");
      lcd.print(GPS.latitude, 3);
      lcd.setCursor(0,3);
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
