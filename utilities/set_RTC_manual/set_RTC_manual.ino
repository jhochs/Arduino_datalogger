#include "RTClib.h"

#define YEAR 2023
#define MONTH 3
#define DAY 31
#define HOUR 16
#define MINUTE 3
#define SECOND 0

RTC_DS3231 rtc;

void setup() {
  Serial.begin(500000);
  while (!Serial) {}    // wait for Serial port to connect. Needed for native USB port only
  
  rtc.begin();
  Serial.println("RTC connected");
  
  Serial.print("The time to be set is: ");
  Serial.print(YEAR);
  Serial.print("-");
  Serial.print(MONTH);
  Serial.print("-");
  Serial.print(DAY);
  Serial.print(" ");
  Serial.print(HOUR);
  Serial.print(":");
  Serial.print(MINUTE);
  Serial.print(":");
  Serial.print(SECOND);
  Serial.println(" UTC");
//  Serial.println("Press enter to set the clock to the above time: ");
//  readLine();
//
//  rtc.adjust(DateTime(YEAR, MONTH, DAY, HOUR, MINUTE, SECOND));
//  Serial.println("RTC successfully set");
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


String readLine() {
  String line;

  while (1) {
    if (Serial.available()) {
      char c = Serial.read();

      if (c == '\r') {
        // ignore
        continue;
      } else if (c == '\n') {
        break;
      }

      line += c;
    }
  }

  return line;
}
