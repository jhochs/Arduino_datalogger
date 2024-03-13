#include <MKRNB.h>
#include <RTClib.h>

#define PINNUMBER ""

RTC_DS3231 rtc;
GPRS       gprs;
NB         nbAccess;

bool NBconnection = false;

void setup() {
  Serial.begin(500000);
  while (!Serial) {}    // wait for Serial port to connect. Needed for native USB port only
  
  rtc.begin();
  Serial.print("RTC connected, time: ");
  Serial.println(rtc.now().unixtime());

  connectNB();

  setRTC();
}

void loop() {
  Serial.println(rtc.now().unixtime());
  delay(1000);
}


//=============================================================================================
/*
   Connects to narrowband (LTE) cellular network.
*/
void connectNB() {
  Serial.println(F("Attempting to connect to cellular network"));

  NBconnection = false;
  if ((nbAccess.begin(PINNUMBER) == NB_READY) && (gprs.attachGPRS() == GPRS_READY)) {
    Serial.println(F("NB success"));
    NBconnection = true;
  } else {
    Serial.println(F("NB connection failed"));
    delay(500);
  }
}

//=============================================================================================
/*
   Set the DS3231 by getting the time from the NB module.
*/
void setRTC() {
  Serial.println(F("Synchronizing RTC with time from NB module"));
  unsigned long initTime = nbAccess.getTime();
  while (nbAccess.getTime() == initTime) {}

  DateTime adjustedTime = DateTime(nbAccess.getTime());
  Serial.print(adjustedTime.year(), DEC);
  Serial.print('/');
  Serial.print(adjustedTime.month(), DEC);
  Serial.print('/');
  Serial.print(adjustedTime.day(), DEC);
  Serial.print(' ');
  Serial.print(adjustedTime.hour(), DEC);
  Serial.print(':');
  Serial.print(adjustedTime.minute(), DEC);
  Serial.print(':');
  Serial.println(adjustedTime.second(), DEC);
  
  rtc.adjust(adjustedTime);
  
  Serial.print(F("RTC set: "));
  Serial.println(rtc.now().unixtime());
  
}
