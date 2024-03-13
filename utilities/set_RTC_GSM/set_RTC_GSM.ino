#include <MKRGSM.h>
#include <RTClib.h>

#define PINNUMBER ""
#define GPRS_APN ""
#define GPRS_LOGIN ""
#define GPRS_PASSWORD ""

RTC_DS3231 rtc;
GPRS       gprs;
GSM        gsmAccess;

bool GSMconnection = false;

void setup() {
  Serial.begin(500000);
  while (!Serial) {}    // wait for Serial port to connect. Needed for native USB port only
  
  if (!rtc.begin()) {
    Serial.println("ERROR! Couldn't find RTC");
  } else {
    Serial.println("RTC connected");
  }

  connectGSM();

  setRTC();
}

void loop() {
  Serial.println(rtc.now().unixtime());
  delay(1000);
}


//=============================================================================================
/*
   Connects to GSM (3G) cellular network.
*/
void connectGSM() {
  Serial.println("Attempting to connect to cellular network");

  GSMconnection = false;
  if ((gsmAccess.begin(PINNUMBER) == GSM_READY) && (gprs.attachGPRS(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD) == GPRS_READY)) {
    Serial.println("GSM success");
    GSMconnection = true;
  } else {
    Serial.println("GSM connection failed");
    delay(500);
  }
}

//=============================================================================================
/*
   Set the DS3231 by getting the time from the NB module.
*/
void setRTC() {
  Serial.println(F("Synchronizing RTC with time from NB module"));
  unsigned long initTime = gsmAccess.getTime();
  while (gsmAccess.getTime() == initTime) {}

  DateTime adjustedTime = DateTime(gsmAccess.getTime());
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
