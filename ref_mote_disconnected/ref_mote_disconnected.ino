//========================================================================
#define DEV_NOTES "Dev notes: "
//========================================================================

#include <Adafruit_SleepyDog.h>
#include <ArduinoECCX08.h>
#include <BMP388_DEV.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

//---------------------
#define HOSTNAME "wm27" // name of this mote
//---------------------

// SD card options
#define ACQ_FILE HOSTNAME ".txt"
#define SD_CS 3

// Watchdog settings:
#define WATCHDOG_MS 16000

// Create objects:
BMP388_DEV      bmpA(6); // connect CS to pin D6
BMP388_DEV      bmpB(7); // connect CS to pin D7
RTC_DS3231      rtc;

// Initialize variables:
unsigned long lastResetTime = 0;
unsigned long t_meas = 0;

File fid;

//=============================================================================================

void setup() {
  Serial.begin(500000);
  delay(10000);
  Serial.println(F("------------------------------------------------------------------"));
  Serial.print(HOSTNAME);
  Serial.println(F(" : 2022/12/14 Build - Reference, disconnected"));
  Serial.println(DEV_NOTES);
  Serial.println(F("------------------------------------------------------------------"));

  // Enable watchdog:
  Watchdog.enable(WATCHDOG_MS);
  Serial.println("Watchdog enabled");
  Watchdog.reset();

  // Initialize sensors:
  configBMP();

  // Begin RTC:
  if (!rtc.begin()) {
    Serial.println(F("ERROR! Couldn't find RTC"));
  }
  
  // Begin SD card:
  if (!SD.begin(SD_CS)) {
    Serial.println(F("Failed to initialize SD card"));
    while (1);
  } else {
    Serial.println(F("SD card initialized"));
  }

  fid = SD.open(ACQ_FILE, O_WRITE | O_APPEND | O_CREAT);
  if (!fid) {
    Serial.println(F("Failed to open measurements file"));
    while (1);
  } else {
    Serial.println(F("Measurement file opened"));
  }
}

//=============================================================================================

void loop() {
  Watchdog.reset();

  // Reset sensors every hour:
  if (rtc.now().unixtime() > lastResetTime + 3600) {
    bmpA.reset();
    bmpB.reset();
    configBMP();
  }

  // Take measurements:
  float Ta, Tb, Pa, Pb;
  if (rtc.now().unixtime() > t_meas) {
    Serial.println(F("Getting measurements..."));
    getReadings(Ta, Tb, Pa, Pb);
    t_meas = rtc.now().unixtime();
    writeReadings(Ta, Tb, Pa, Pb);
  }
}

//=============================================================================================
/*
   Configures the BMP388 sensors as desired.
*/
void configBMP() {
  bmpA.begin();
  bmpA.setTimeStandby(TIME_STANDBY_640MS);
  bmpA.setIIRFilter(IIR_FILTER_OFF);
  bmpA.setPresOversampling(OVERSAMPLING_X32);
  bmpA.setTempOversampling(OVERSAMPLING_X4);

  bmpB.begin();
  bmpB.setTimeStandby(TIME_STANDBY_640MS);
  bmpB.setIIRFilter(IIR_FILTER_OFF);
  bmpB.setPresOversampling(OVERSAMPLING_X32);
  bmpB.setTempOversampling(OVERSAMPLING_X4);

  bmpA.startNormalConversion();
  bmpB.startNormalConversion();
}

//=============================================================================================
/*
   Read the measurements from BMP388.
*/
void getReadings(float& Ta, float& Tb, float& Pa, float& Pb) {
  bool PAready = false;
  bool PBready = false;
  unsigned long startMillis = millis();
  Serial.println(F("Querying sensors..."));
  while (!PAready || !PBready) {
    if (!PAready && bmpA.getTempPres(Ta, Pa)) {
      PAready = true;
    }
    if (!PBready && bmpB.getTempPres(Tb, Pb)) {
      PBready = true;
    }
    if (millis() > startMillis + 1200) {
      Serial.println(F("Sensor timeout"));
      break;
    }
  }
}

//=============================================================================================
/*
   Publish the measurements to the MQTT broker.
*/
void writeReadings(float Ta, float Tb, float Pa, float Pb) {
  fid.print(t_meas);
  fid.print(", ");
  fid.print(Pa - 100000.0);
  fid.print(", ");
  fid.print(Pb - 100000.0);
  fid.print(", ");
  fid.print(Ta);
  fid.print(", ");
  fid.print(Tb);
  fid.print("\n");
  
  fid.flush();
  Serial.println(F("Measurement written"));
}
