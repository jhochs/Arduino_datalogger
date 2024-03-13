#include <Adafruit_SleepyDog.h>
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <ArduinoMqttClient.h>
#include <BMP388_DEV.h>
#include <WiFiNINA.h>

#include "certificate.h"

#define HOSTNAME "wm27" // must be unique for all motes on network
#define TOPIC_OUTGOING "wm27/outgoing" // measurements published on this topic
#define CERT WM27_CERTIFICATE

// WiFi and MQTT settings:
#define BROKER "XX.iot.us-east-2.amazonaws.com"
#define MQTT_TIMEOUT 30 * 1000UL
#define SSID "SSID"
#define PASS "Password"
//#define SSID "TeamMember"
//#define PASS "S0undV!ew"

// Watchdog settings:
#define WATCHDOG_MS 16000

// Create objects:
WiFiClient      wifiClient;
WiFiUDP         udp;
BearSSLClient   sslClient(wifiClient);   // Used for SSL/TLS connection, integrates with ECC508
MqttClient      mqttClient(sslClient);

BMP388_DEV      bmpA(6); // connect CS to pin D6
BMP388_DEV      bmpB(7); // connect CS to pin D7

// Initialize variables:
unsigned long lastResetTime = 0;
unsigned long t_meas = 0;

//=============================================================================================

void setup() {
  Serial.begin(500000);
  delay(10000);
  Serial.println(F("------------------------------------------------------------------"));
  Serial.print(HOSTNAME);
  Serial.println(F(" : 2022/11/04 Build - Reference (with watchdog)"));
  Serial.print(F("SSID: "));
  Serial.println(SSID);
  Serial.println(F("Dev notes:"));
  Serial.println(F("------------------------------------------------------------------"));

  // Enable watchdog:
  Watchdog.enable(WATCHDOG_MS);
  Serial.println("Watchdog enabled");
  Watchdog.reset();
  
  // Configure SSL and MQTT:
  ArduinoBearSSL.onGetTime(getTime);
  sslClient.setEccSlot(0, CERT);
  mqttClient.setConnectionTimeout(MQTT_TIMEOUT);

  // Connect to WiFi and MQTT:
  verifyConnection();

  // Initialize sensors:
  configBMP();
}

//=============================================================================================

void loop() {
  Watchdog.reset();

  // Reset sensors every hour:
  if (WiFi.getTime() > lastResetTime + 3600) {
    bmpA.reset();
    bmpB.reset();
    configBMP();
  }

  // Take measurements:
  float Ta, Tb, Pa, Pb;
  if (WiFi.getTime() > t_meas) {
    getReadings(Ta, Tb, Pa, Pb);
    t_meas = WiFi.getTime();
    verifyConnection();
    publishReadings(Ta, Tb, Pa, Pb);
    Serial.println("Measurement sent");
  }
}

//=============================================================================================
/*
   Get UNIX time in seconds from WiFi module. Disregards sub-second information.
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
   If connection is not established, attempt to connect both WiFi and MQTT until
   successful.
*/
void verifyConnection() {
  byte mqttFailures = 0; // if WiFi repeatedly connects but MQTT repeatedly fails, hard reset

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
   Read the measurements stored in the BMP388's FIFO memory.
*/
void getReadings(float& Ta, float& Tb, float& Pa, float& Pb) {
  bool PAready = false;
  bool PBready = false;
  unsigned long startMillis = millis();
  while (!PAready || !PBready) {
    if (!PAready && bmpA.getTempPres(Ta, Pa)) {
      PAready = true;
    }
    if (!PBready && bmpB.getTempPres(Tb, Pb)) {
      PBready = true;
    }
    if (millis() > startMillis + 1000) {
      //Serial.println(F("Sensor timeout"));
      break;
    }
  }
}

//=============================================================================================
/*
   Publish the measurements to the MQTT broker.
*/
void publishReadings(float Ta, float Tb, float Pa, float Pb) {
  mqttClient.beginMessage(TOPIC_OUTGOING);
  mqttClient.print("{\n\t\"t\" : ");
  mqttClient.print(t_meas);
  mqttClient.print(",\n\t\"Pa\" : ");
  mqttClient.print(Pa - 100000.0);
  mqttClient.print(",\n\t\"Pb\" : ");
  mqttClient.print(Pb - 100000.0);
  mqttClient.print(",\n\t\"Ta\" : ");
  mqttClient.print(Ta);
  mqttClient.print(",\n\t\"Tb\" : ");
  mqttClient.print(Tb);
  mqttClient.print("\n}");
  mqttClient.endMessage();
  //Serial.println("Measurement sent");
}
