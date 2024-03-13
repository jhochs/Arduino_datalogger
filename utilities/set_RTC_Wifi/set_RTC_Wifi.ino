#include <WiFiNINA.h>
#include <ezTime.h>
#include "RTClib.h"

#define SSID "Stanford"
//#define SSID "SSID"
//#define PASS "Password"

RTC_DS3231 rtc;

void setup() {
  Serial.begin(500000);
  while (!Serial) {}    // wait for Serial port to connect. Needed for native USB port only
  
  if (!rtc.begin()) {
    Serial.println(F("ERROR! Couldn't find RTC"));
  } else {
    Serial.print("RTC connected, time: ");
    Serial.println(rtc.now().unixtime());
  }
  
  connectWiFi();

  setInterval(60);
  waitForSync();
  Serial.println("Time received from NTP server");
  while (ms() != 0) {} // wait until 0 milliseconds past the second to update
  rtc.adjust(DateTime(year(), month(), day(), hour(), minute(), second()));
  Serial.println("RTC successfully set");
}

void loop() {
  DateTime now = rtc.now();
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  delay(100);
}

void connectWiFi() {
  Serial.println("Attempting to connect to network");

  bool connection = false;
  while (!connection) {    
    #ifdef PASS 
      WiFi.begin(SSID, PASS);
    #else
      WiFi.begin(SSID);
    #endif
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("Connected to the network, status = ");
      Serial.println(WiFi.status());
      connection = true;
    }
    else {
      Serial.print("Connection failed, status = ");
      Serial.print(WiFi.status());
      Serial.println(". Retrying");
      WiFi.end();
      delay(5000);
    }
  }
}
