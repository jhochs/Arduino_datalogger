#include <BMP388_DEV.h>
#include <RTClib.h>
#include <Wire.h>

BMP388_DEV      bmpA(7); // connect CS to pin D2
BMP388_DEV      bmpB(6); // connect CS to pin D3
//BMP388_DEV      bmpC(4); // connect CS to pin D4
RTC_DS3231      rtc;

unsigned long tMeas = 0;

void setup() {
  while (!Serial) {}

  // Initialize sensor:
  configBMP();

  // Initialize RTC:
  if (!rtc.begin()) {
    Serial.println(F("ERROR! Couldn't find RTC"));
  }
}

void loop() {
  // Take measurements:
  float Pa, Pb; //, Pc;
  while (!bmpA.getPressure(Pa)) {}
  while (!bmpB.getPressure(Pb)) {}
//  while (!bmpC.getPressure(Pc)) {}
//  bool PAready = false;
//  bool PBready = false;
//  bool PCready = false;
//  unsigned long startMillis = millis();
//  while (!PAready || !PBready || !PCready) {
//    if (!PAready && bmpA.getPressure(Pa)) {
//      PAready = true;
//    }
//    if (!PBready && bmpB.getPressure(Pb)) {
//      PBready = true;
//    }
//    if (!PCready && bmpC.getPressure(Pc)) {
//      PCready = true;
//    }
//
//    if (millis() > startMillis + 10000) {
//      Serial.println("Sensor timeout");
//      break;
//    }
//  }
  tMeas = rtc.now().unixtime();

  // Print
  Serial.print("A = ");
  Serial.print(Pa);
  Serial.print(" Pa | B = ");
  Serial.println(Pb);
//  Serial.print(" Pa | C = ");
//  Serial.print(Pc * 100);
//  Serial.println(" Pa");
}

//=============================================================================================
/*
   Configures the three BMP388 sensors as desired.
*/
void configBMP() {
  bmpA.begin();
  bmpA.setIIRFilter(IIR_FILTER_OFF);
  bmpA.setPresOversampling(OVERSAMPLING_X32);
  bmpA.setTempOversampling(OVERSAMPLING_X4);
  bmpA.setTimeStandby(TIME_STANDBY_640MS);

  bmpB.begin();
  bmpB.setIIRFilter(IIR_FILTER_OFF);
  bmpB.setPresOversampling(OVERSAMPLING_X32);
  bmpB.setTempOversampling(OVERSAMPLING_X4);
  bmpB.setTimeStandby(TIME_STANDBY_640MS);

//  bmpC.begin();
//  bmpC.setIIRFilter(IIR_FILTER_OFF);
//  bmpC.setPresOversampling(OVERSAMPLING_X16);
//  bmpC.setTempOversampling(OVERSAMPLING_X2);
//  bmpC.setTimeStandby(TIME_STANDBY_5120MS);

  bmpA.startNormalConversion();
  bmpB.startNormalConversion();
//  bmpC.startNormalConversion();
}
