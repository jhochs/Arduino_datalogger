**Tl;dr: ArduinoBearSSL can hang on BearSSLClient::connectSSL(). Has anyone found a way to prevent this or an alternate library to facilitate SSL that has the ability to set a timeout?**

I'm using a MKR 1500 to communicate with AWS using the MQTT protocol. Full code below but my connection object structure is:
```
NBClient        client;
BearSSLClient   sslClient(client);
MqttClient      mqttClient(sslClient);
```
I've verified/set timeouts for both the NB and ArduinoMqttClient libraries, so the program cannot hang due to these. However, there is evidently a loop without a timeout buried somewhere in the ArduinoBearSSL library that occasionally causes hanging. In BearSSLClient.cpp I enabled `#define DEBUGSERIAL Serial` (line 504) and placed some print statements to see where this occurs. I determined that the hang happens in the call to br_sslio_init() within BearSSLClient::connectSSL() on line 487 of the cpp file. br_sslio_init() is defined in a .c file so it is more difficult to pinpoint more than this. 

Here is my program:
```
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <ArduinoMqttClient.h>
#include <MKRNB.h>

#include "certificate.h"

// NB and MQTT settings:
#define PINNUMBER ""
#define BROKER "XXXX.iot.us-east-2.amazonaws.com"
#define NB_TIMEOUT 60 * 1000UL
#define MQTT_TIMEOUT 30 * 1000UL

// Create objects:
GPRS            gprs;
NB              nbAccess;
NBClient        client;
BearSSLClient   sslClient(client);
MqttClient      mqttClient(sslClient);

void setup() {
  Serial.begin(500000);
  delay(5000);
  Serial.println(F("------------------------------------------------------------------"));
  
  // Set timeouts:
  nbAccess.setTimeout(NB_TIMEOUT);
  gprs.setTimeout(NB_TIMEOUT);
  mqttClient.setConnectionTimeout(MQTT_TIMEOUT);
  
  // Configure SSL and MQTT:
  ArduinoBearSSL.onGetTime(getTime);
  sslClient.setEccSlot(0, CERT);

  // Connect
  while (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) {
    connectNB();
  }
  delay(2000); // SB delay(10000);
  while (!mqttClient.connected()) {
    connectMQTT();
  }
}

void loop() {
  
}

unsigned long getTime() {
  return nbAccess.getTime();
}

void connectNB() {
  Serial.println(F("Attempting to connect to cellular network"));

  if ((nbAccess.begin(PINNUMBER) == NB_READY) && (gprs.attachGPRS() == GPRS_READY)) {
    Serial.println(F("NB success"));
  } else {
    Serial.println(F("NB connection failed"));
    delay(500);
  }
}

bool connectMQTT() {
  Serial.println(F("Attempting to connect to MQTT broker"));
  if (mqttClient.connect(BROKER, 8883)) {
    Serial.println(F("MQTT connected"));
    return true;
  }
  Serial.println(F("MQTT failed to connect"));
  return false;
}
```

This is what the output looks like when it DOESN'T hang (I've truncated the hex outputs for readability):
```
14:00:09.998 -> ------------------------------------------------------------------
14:00:15.083 -> Attempting to connect to cellular network
14:00:20.823 -> NB success
14:00:22.815 -> Attempting to connect to MQTT broker
14:00:23.426 -> BearSSLClient::clientWrite - 250 - 16030100F50...
14:00:23.641 -> BearSSLClient::clientRead - 0 - 
14:00:23.641 -> 
14:00:23.679 -> BearSSLClient::clientRead - 0 - 
14:00:23.679 -> 
14:00:23.752 -> BearSSLClient::clientRead - 0 - 
14:00:23.752 -> 
14:00:23.789 -> BearSSLClient::clientRead - 5 - 
14:00:23.789 -> 160303005B
14:00:23.789 -> BearSSLClient::clientRead - 91 - 
14:00:23.789 -> 0200005703...
14:00:23.826 -> BearSSLClient::clientRead - 0 - 
14:00:23.826 -> 
14:00:23.895 -> BearSSLClient::clientRead - 0 - 
14:00:23.895 -> 
14:00:23.966 -> BearSSLClient::clientRead - 0 - 
14:00:23.966 -> 
14:00:24.003 -> BearSSLClient::clientRead - 0 - 
14:00:24.003 -> 
14:00:24.111 -> BearSSLClient::clientRead - 5 - 
14:00:24.111 -> 160303138B
14:00:24.111 -> BearSSLClient::clientRead - 507 - 
14:00:24.111 -> 0B001387001...
14:00:24.253 -> BearSSLClient::clientRead - 512 - 
14:00:24.253 -> 81B80E638A8...
14:00:24.431 -> BearSSLClient::clientRead - 512 - 
14:00:24.431 -> 15AD6769C0F8...
14:00:24.568 -> BearSSLClient::clientRead - 512 - 
14:00:24.568 -> 7F9A8BBC16...
14:00:24.714 -> BearSSLClient::clientRead - 512 - 
14:00:24.714 -> 4BB451E7020...
14:00:25.267 -> BearSSLClient::clientRead - 512 - 
14:00:25.267 -> 9F4D3985CA...
14:00:25.775 -> BearSSLClient::clientRead - 512 - 
14:00:25.775 -> 8823C73EEB...
14:00:25.923 -> BearSSLClient::clientRead - 512 - 
14:00:25.923 -> 300D06092A...
14:00:26.031 -> BearSSLClient::clientRead - 512 - 
14:00:26.031 -> 253023060...
14:00:26.167 -> BearSSLClient::clientRead - 400 - 
14:00:26.167 -> 301C06082B...
14:00:26.201 -> BearSSLClient::clientRead - 5 - 
14:00:26.201 -> 160303014D
14:00:26.201 -> BearSSLClient::clientRead - 107 - 
14:00:26.201 -> 0C0001490...
14:00:26.271 -> BearSSLClient::clientRead - 226 - 
14:00:26.271 -> 8757EED71...
14:00:26.662 -> BearSSLClient::clientRead - 5 - 
14:00:26.662 -> 1603030020
14:00:26.662 -> BearSSLClient::clientRead - 32 - 
14:00:26.662 -> 0D00001C0...
14:00:26.662 -> BearSSLClient::clientRead - 5 - 
14:00:26.662 -> 1603030004
14:00:26.662 -> BearSSLClient::clientRead - 4 - 
14:00:26.662 -> 0E000000
14:00:26.662 -> BearSSLClient::clientWrite - 517 - 160303020...
14:00:28.005 -> BearSSLClient::clientWrite - 353 - 16030301...
14:00:28.185 -> BearSSLClient::clientWrite - 6 - 140303000101...
14:00:28.253 -> BearSSLClient::clientWrite - 45 - 1603030028...
14:00:28.393 -> BearSSLClient::clientRead - 0 - 
14:00:28.393 -> 
14:00:28.429 -> BearSSLClient::clientRead - 0 - 
14:00:28.429 -> 
14:00:28.497 -> BearSSLClient::clientRead - 0 - 
14:00:28.497 -> 
14:00:28.532 -> BearSSLClient::clientRead - 0 - 
14:00:28.532 -> 
14:00:28.604 -> BearSSLClient::clientRead - 0 - 
14:00:28.604 -> 
14:00:28.641 -> BearSSLClient::clientRead - 0 - 
14:00:28.641 -> 
14:00:28.713 -> BearSSLClient::clientRead - 0 - 
14:00:28.713 -> 
14:00:28.749 -> BearSSLClient::clientRead - 5 - 
14:00:28.749 -> 1403030001
14:00:28.749 -> BearSSLClient::clientRead - 1 - 
14:00:28.749 -> 01
14:00:28.749 -> BearSSLClient::clientRead - 5 - 
14:00:28.749 -> 1603030028...
14:00:28.749 -> BearSSLClient::clientRead - 40 - 
14:00:28.749 -> 0000000000...
14:00:28.749 -> BearSSLClient::clientWrite - 59 - 170303003...
14:00:28.889 -> BearSSLClient::clientRead - 0 - 
14:00:28.889 -> 
14:00:28.993 -> BearSSLClient::clientRead - 0 - 
14:00:28.993 -> 
14:00:29.062 -> BearSSLClient::clientRead - 0 - 
14:00:29.062 -> 
14:00:29.133 -> BearSSLClient::clientRead - 5 - 
14:00:29.133 -> 170303001C...
14:00:29.133 -> BearSSLClient::clientRead - 28 - 
14:00:29.133 -> 000000...
14:00:29.166 -> BearSSLClient::clientRead - 0 - 
14:00:29.166 -> 
14:00:29.166 -> MQTT connected
```

Versus when it hangs:
```
14:01:56.524 -> ------------------------------------------------------------------
14:02:01.617 -> Attempting to connect to cellular network
14:02:12.582 -> NB success
14:02:14.595 -> Attempting to connect to MQTT broker
14:02:15.534 -> BearSSLClient::clientWrite - 250 - 16030100F50...
14:02:15.710 -> BearSSLClient::clientRead - 0 - 
14:02:15.710 -> 
14:02:15.777 -> BearSSLClient::clientRead - 0 - 
14:02:15.777 -> 
14:02:15.810 -> BearSSLClient::clientRead - 0 - 
14:02:15.810 -> 
14:02:15.883 -> BearSSLClient::clientRead - 0 - 
14:02:15.883 -> 
14:02:15.954 -> BearSSLClient::clientRead - 0 - 
14:02:15.954 -> 
14:02:15.992 -> BearSSLClient::clientRead - 0 - 
14:02:15.992 -> 
14:02:16.025 -> BearSSLClient::clientRead - 0 - 
14:02:16.025 -> 
14:02:16.098 -> BearSSLClient::clientRead - 0 - 
14:02:16.098 -> 
14:02:16.133 -> BearSSLClient::clientRead - 0 - 
14:02:16.133 -> 
14:02:16.201 -> BearSSLClient::clientRead - 0 - 
14:02:16.201 -> 
14:02:16.270 -> BearSSLClient::clientRead - 0 - 
14:02:16.270 -> 
14:02:16.314 -> BearSSLClient::clientRead - 0 - 
14:02:16.314 -> 
14:02:16.375 -> BearSSLClient::clientRead - 0 - 
14:02:16.375 -> 
14:02:16.413 -> BearSSLClient::clientRead - 0 - 
14:02:16.413 -> 
14:02:16.482 -> BearSSLClient::clientRead - 5 - 
14:02:16.482 -> 160303005B
14:02:16.482 -> BearSSLClient::clientRead - 91 - 
14:02:16.482 -> 02000057030...
14:02:16.554 -> BearSSLClient::clientRead - 0 - 
14:02:16.554 -> 
14:02:16.582 -> BearSSLClient::clientRead - 0 - 
14:02:16.582 -> 
14:02:16.655 -> BearSSLClient::clientRead - 0 - 
14:02:16.655 -> 
14:02:16.691 -> BearSSLClient::clientRead - 0 - 
14:02:16.691 -> 
14:02:16.762 -> BearSSLClient::clientRead - 0 - 
14:02:16.762 -> 
14:02:16.902 -> BearSSLClient::clientRead - 5 - 
14:02:16.902 -> 160303138B
14:02:16.902 -> BearSSLClient::clientRead - 507 - 
14:02:16.902 -> 0B00138700...
14:02:17.047 -> BearSSLClient::clientRead - 512 - 
14:02:17.047 -> 81B80E638A8...
14:02:17.186 -> BearSSLClient::clientRead - 372 - 
14:02:17.186 -> 15AD6769C0...
14:02:17.257 -> BearSSLClient::clientRead - 0 - 
14:02:17.257 -> 
14:02:17.290 -> BearSSLClient::clientRead - 0 - 
14:02:17.290 -> 
14:02:17.361 -> BearSSLClient::clientRead - 0 - 
14:02:17.361 -> 
14:02:17.429 -> BearSSLClient::clientRead - 0 - 
14:02:17.429 -> 
14:02:17.472 -> BearSSLClient::clientRead - 0 - 
14:02:17.472 -> 
14:02:17.534 -> BearSSLClient::clientRead - 0 - 
14:02:17.534 -> 
14:02:17.568 -> BearSSLClient::clientRead - 0 - 
14:02:17.568 -> 
14:02:17.635 -> BearSSLClient::clientRead - 0 - 
14:02:17.635 -> 
14:02:17.669 -> BearSSLClient::clientRead - 0 - 
14:02:17.669 -> 
14:02:17.737 -> BearSSLClient::clientRead - 0 - 
14:02:17.737 -> 
14:02:17.808 -> BearSSLClient::clientRead - 0 - 
14:02:17.808 -> 
14:02:17.842 -> BearSSLClient::clientRead - 0 - 
14:02:17.842 -> 
14:02:17.911 -> BearSSLClient::clientRead - 0 - 
14:02:17.911 -> 
14:02:17.983 -> BearSSLClient::clientRead - 0 - 
14:02:17.983 -> 
14:02:18.019 -> BearSSLClient::clientRead - 0 - 
14:02:18.019 -> 
14:02:18.089 -> BearSSLClient::clientRead - 0 - 
14:02:18.089 -> 
```
and then this repeats.