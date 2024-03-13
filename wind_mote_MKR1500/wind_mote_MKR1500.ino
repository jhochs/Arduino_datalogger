//========================================================================
#define DEV_NOTES "Dev notes: -7 hour TZ offset hardcoded in NB.cpp"
//========================================================================

#include <Adafruit_SleepyDog.h>
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <ArduinoMqttClient.h>
#include <MKRNB.h>
#include <SDI12.h>

#include "certificate.h"

#define HOSTNAME "cm28" // must be unique for all motes on network
#define TOPIC_OUTGOING "cm28/outgoing" // measurements published on this topic
#define CERT CM28_CERTIFICATE

// NB and MQTT settings:
#define PINNUMBER ""
#define BROKER "XX.iot.us-east-2.amazonaws.com"
#define NB_TIMEOUT 60 * 1000UL
#define MQTT_TIMEOUT 30 * 1000UL
#define OVERALL_TIMEOUT 120 * 1000UL

// Watchdog settings:
#define WATCHDOG_MS 16000

// Create objects:
GPRS            gprs;
NB              nbAccess;
NBClient        client;
BearSSLClient   sslClient(client);   // Used for SSL/TLS connection, integrates with ECC508
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
  Serial.println(F(" : 2023/04/12 Build - Wind (with watchdog)"));
  Serial.println(DEV_NOTES);
  Serial.println(F("------------------------------------------------------------------"));

  // Reset modem:
  nbAccess.hardReset();
  delay(5000);
  softReset();

  // Enable watchdog:
  Watchdog.enable(WATCHDOG_MS);
  Serial.println("Watchdog enabled");
  Watchdog.reset();

  // Set timeouts:
  nbAccess.setTimeout(NB_TIMEOUT);
  client.setClientTimeout(MQTT_TIMEOUT); // this requires edits to the NBClient class
  gprs.setTimeout(MQTT_TIMEOUT);
  mqttClient.setConnectionTimeout(MQTT_TIMEOUT);

  // Configure SSL and MQTT:
  ArduinoBearSSL.onGetTime(getTime);
  sslClient.setEccSlot(0, CERT);
  
  // Connect to NB and MQTT:
  verifyConnection();
  Watchdog.reset();

  // Initialize SDI for reading from anemometer:
  sdi.begin();
  delay(500);
  sdi.forceListen();
}

//=============================================================================================

void loop() {
  Watchdog.reset();
  
  // Get measurement every second:
  if (nbAccess.getTime() > t_meas) {
    t_meas = nbAccess.getTime();
    
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
  unsigned long t = nbAccess.getTime();
  if (t == 0) {
    delay(50);
    return nbAccess.getTime();
  }
  return t;
}

//=============================================================================================
/*
   If connection is not established, attempt to connect both NB and MQTT until
   successful.
*/
void verifyConnection() {
  byte mqttFailures = 0; // if NB repeatedly connects but MQTT repeatedly fails, hard reset
  client.setAbortMillis(millis() + OVERALL_TIMEOUT); // watchdog will no longer reset after this millis count. Note: requires edit to the NBClient class
  
  while (!mqttClient.connected()) {
    Watchdog.reset();
    byte attemptCount = 0;
    while (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) {
      if (attemptCount > 4) {
        // Soft reset modem:
        softReset();
        attemptCount = 0;
      } else {
        attemptCount++;
      }
      connectNB();
    }

    // Connect to MQTT:
    delay(2000);
    Watchdog.reset();
    if (!connectMQTT()) {
      mqttFailures++;
    }

    if (mqttFailures > 3) {
      Serial.println(F("More than 4 failed attempts to connect MQTT, hard resetting"));
      Watchdog.reset();
      nbAccess.hardReset();
      delay(1000);
      connectNB(); // force NB reconnect
      mqttFailures = 0;
    }
  }
}

//=============================================================================================
/*
   Connects to narrowband (LTE) cellular network.
*/
void connectNB() {
  Serial.println(F("Attempting to connect to cellular network"));

  if ((nbAccess.begin(PINNUMBER) == NB_READY) && (gprs.attachGPRS() == GPRS_READY)) {
    Serial.println(F("NB success"));
  } else {
    Serial.println(F("NB connection failed"));
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
   Soft resets the SARA modem without having to create a modem class instance.
*/
void softReset() {
  SerialSARA.begin(115200);
  delay(10);
  SerialSARA.println(F("AT+CFUN=15"));
  delay(2000);
  Serial.println(F("Modem was soft reset"));
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
