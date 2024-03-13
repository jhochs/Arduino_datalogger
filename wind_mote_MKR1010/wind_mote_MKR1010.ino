//========================================================================
#define DEV_NOTES "Dev notes: "
//========================================================================

#include <Adafruit_SleepyDog.h>
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include <SDI12.h>

#include "certificate.h"

#define HOSTNAME "wm28" // must be unique for all motes on network
#define TOPIC_OUTGOING "wm28/outgoing" // measurements published on this topic
#define CERT WM28_CERTIFICATE

// WiFi and MQTT settings:
#define BROKER "XX.iot.us-east-2.amazonaws.com"
#define MQTT_TIMEOUT 30 * 1000
#define OVERALL_TIMEOUT 60 * 1000
#define SSID "SSID"
#define PASS "Password"

// Watchdog settings:
#define WATCHDOG_MS 16000

// Create objects:
WiFiClient      wifiClient;
BearSSLClient   sslClient(wifiClient);   // Used for SSL/TLS connection, integrates with ECC508
MqttClient      mqttClient(sslClient);

SDI12           sdi(7); // anemometer data in

// Initialize variables:
unsigned long lastResetTime = 0;
unsigned long t_meas = 0;
boolean sdiMsgReady = false;
String sdiMsgStr = "";

//=============================================================================================

void setup() {
  Serial.begin(500000);
  delay(5000);
  Serial.println(F("------------------------------------------------------------------"));
  Serial.print(HOSTNAME);
  Serial.println(F(" : 2023/04/07 Build - Wind, WiFi"));
  Serial.print(F("SSID: "));
  Serial.println(SSID);
  Serial.println(DEV_NOTES);
  Serial.println(F("------------------------------------------------------------------"));

  // Enable watchdog:
  Watchdog.enable(WATCHDOG_MS);
  Serial.println("Watchdog enabled");
  Watchdog.reset();

  // Configure SSL and MQTT:
  ArduinoBearSSL.onGetTime(getTime);
  sslClient.setEccSlot(0, CERT);
  mqttClient.setConnectionTimeout(MQTT_TIMEOUT);
  
  // Connect to NB and MQTT:
  verifyConnection();

  // Initialize SDI for reading from anemometer:
  Serial.println("Initializing SDI...");
  sdi.begin();
  delay(500);
  sdi.forceListen();
  Serial.println("Measuring");
}

//=============================================================================================

void loop() {
  Watchdog.reset();
  
  // Get measurement every second:
  if (WiFi.getTime() > t_meas) {
    t_meas = WiFi.getTime();
    
    // Send request:
    sdi.sendCommand("0R0!");

    // Read response:
    int avail = sdi.available();
    if (avail < 0) {
      sdi.flush(); // buffer is full so flush
    } else if (avail > 0) {
      for (int a = 0; a < avail; a++) {
        char inByte2 = sdi.read();
        if (inByte2 == '\n') {
          sdiMsgReady = true;
        }
        else if (inByte2 == '!') {
          sdiMsgStr += "!";
          sdiMsgReady = true;
        }
        else {
          sdiMsgStr += String(inByte2);
        }
      }
    }
    
    // If measurement was successful, publish it:
    if (sdiMsgReady) {
      verifyConnection();
      mqttClient.poll();
      
      Serial.println(sdiMsgStr);
      publishReading();
      
      // Reset String for next SDI-12 message
      sdiMsgReady = false;
      sdiMsgStr = "";
    }
  }
}


//=============================================================================================
/*
   Get UNIX time in seconds from cellular module.
*/
unsigned long getTime() {
  unsigned long t = WiFi.getTime();
  if (t == 0) {
    delay(50);
    return WiFi.getTime();
  }
  return t;
}


//=============================================================================================
/*
   If connection is not established, attempt to connect both NB and MQTT until
   successful.
*/
void verifyConnection() {
  byte mqttFailures = 0; // if WiFi repeatedly connects but MQTT repeatedly fails, hard reset
  wifiClient.setAbortMillis(millis() + OVERALL_TIMEOUT); // watchdog will no longer reset after this millis count. Note: requires edit to the WiFiClient class
  
  while (!mqttClient.connected()) {
    Watchdog.reset();
    byte attemptCount = 0;
    while (WiFi.status() != WL_CONNECTED) {
      connectWiFi();
    }

    // Connect to MQTT:
    delay(2000);
    Watchdog.reset();
    if (!connectMQTT()) {
      mqttFailures++;
    }

    if (mqttFailures > 3) {
      Serial.println(F("More than 4 failed attempts to connect MQTT, forcing WiFi reconnection"));
      Watchdog.reset();
      delay(1000);
      connectWiFi(); // force WiFi reconnect
      mqttFailures = 0;
    }
  }
}


//=============================================================================================
/*
   Connects to WiFi network.
*/
void connectWiFi() {
  Serial.println(F("Attempting to connect to WiFi"));
  WiFi.begin(SSID, PASS);
//  WiFi.begin(SSID);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("WiFi success"));
  } else {
    Serial.println(F("WiFi connection failed"));
    WiFi.end();
    delay(500);
  }
}


//=============================================================================================
/*
   Connects to MQTT broker (AWS in this case) to which measurements are streamed
   and shadow is held.
*/
bool connectMQTT() {
  Serial.println(F("Attempting to connect to MQTT broker"));
  if (mqttClient.connect(BROKER, 8883)) {
    Serial.println(F("MQTT connected"));
    return true;
  }
  Serial.println(F("MQTT failed to connect"));
  return false;
}


//=============================================================================================
/*
   Publish the measurements to the MQTT broker.
*/
void publishReading() {
  float WS = sdiMsgStr.substring(6,12).toFloat();
  int WDir = sdiMsgStr.substring(2,5).toInt();
  
  mqttClient.beginMessage(TOPIC_OUTGOING);
  mqttClient.print("{\n\t\"t\" : ");
  mqttClient.print(t_meas);
  mqttClient.print(",\n\t\"WS\" : ");
  mqttClient.print(WS);
  mqttClient.print(",\n\t\"WDir\" : ");
  mqttClient.print(WDir);
  mqttClient.print("\n}");
  mqttClient.endMessage();
  Serial.println("Measurement sent");
}
