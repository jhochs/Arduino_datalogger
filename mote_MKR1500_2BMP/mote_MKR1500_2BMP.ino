//========================================================================
#define DEV_NOTES "Dev notes: -7 hour TZ offset hardcoded in NB.cpp"
//========================================================================
/*  March 2023 note: strangely, enabling NBUDP (line 66) causes hanging when
 *  trying to check shadow. Further testing showed this is only an issue if 
 *  the millisecond counter is enabled. Very perplexing. For now just using
 *  the modem to set the RTC.
 */

//#include <Arduino_PMIC.h>
#include <Adafruit_SleepyDog.h>
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include <ArduinoLowPower.h>
#include <ArduinoMqttClient.h>
#include <BMP388_DEV.h>
#include <MKRNB.h>
#include <RTClib.h>
#include <Wire.h>

#include "certificate.h"

#define HOSTNAME "cm17" // must be unique for all motes on network
#define TOPIC_OUTGOING "cm17/outgoing" // measurements published on this topic
#define TOPIC_SHADOW_UPDATE "$aws/things/Arduino_CM17/shadow/update" // used to report the updated shadow state
#define TOPIC_SHADOW_OUT "$aws/things/Arduino_CM17/shadow/get" // used for Arduino to ask AWS to publish shadow state, which is then published on:...
#define TOPIC_SHADOW_IN "$aws/things/Arduino_CM17/shadow/get/accepted" // ...this one
#define CERT CM17_CERTIFICATE

// Sensor connection options
#define A_PIN 5
#define B_PIN 7

// Measurement frequency in Hz and number of measurements to be cached by sensor:
#define FREQ 12.5
#define NUM_MEAS 6
#define MS_DT -2 // add this many milliseconds to the wait period (depends on NUM_MEAS, tune to get exactly 12.5 Hz measurements)

// Time to sleep or take measurements before re-checking desired state (in minutes):
#define WAKE_CHECK_PERIOD 60 * 60  // seconds
#define SLEEP_CHECK_PERIOD 20 * 60  // seconds
#define ONE_HOUR 60 * 60000  // milliseconds

// NB and MQTT settings:
#define PINNUMBER ""
#define BROKER "XX.iot.us-east-2.amazonaws.com"
#define OVERALL_TIMEOUT 120 * 1000  // max time to connect to NB and MQTT
#define NB_TIMEOUT 60 * 1000
#define MQTT_TIMEOUT 30 * 1000

// NTP settings:
//#define NTP_SERVER "pool.ntp.org"
//#define NTP_PACKET_SIZE 48
//#define NTP_LOCAL_PORT 2390
//#define NTP_TIMEOUT 5000

// Battery settings:
#define MIN_BATT_VOLTAGE 3.6   // do not acquire when below this voltage
// With solar charger:
#define BATT_PIN A0
#define BATT_FACTOR 6.6 / 4096.0
// Without solar charger:
//#define BATT_PIN ADC_BATTERY
//#define BATT_FACTOR 4.3 / 4096.0

// Watchdog settings:
#define WATCHDOG_MS 16000

// Create objects:
GPRS            gprs;
NB              nbAccess;
NBClient        client;
NBScanner       scannerNetworks;
//NBUDP           udp;
BearSSLClient   sslClient(client);   // Used for SSL/TLS connection, integrates with ECC508
MqttClient      mqttClient(sslClient);

BMP388_DEV      bmp1(A_PIN);
BMP388_DEV      bmp2(B_PIN);
RTC_DS3231      rtc;

// Initialize variables:
unsigned long lastResetTime = 0;
unsigned long lastRTCSetTime, lastMSCalibTime;
volatile float milliseconds = 0;
unsigned long t_s = 0;
int t_ms = 0;

byte sleeping = 2; // 0 = awake, 1 = asleep, 2 = unassigned
bool messagePending = true;
byte consecCheckFailures = 0; // tally consecutive failures to check the shadow state, automatically go to sleep after 24 checks (4 hours)

//=============================================================================================

void setup() {
  Serial.begin(500000);
  delay(5000);
  Serial.println(F("------------------------------------------------------------------"));
  Serial.print(HOSTNAME);
  Serial.println(F(" : 2023/04/05 Build - Pressure, NB"));
  Serial.print(F("Sensor pins: D"));
  Serial.print(A_PIN);
  Serial.print(", D");
  Serial.println(B_PIN);
  Serial.println(DEV_NOTES);
  Serial.println(F("------------------------------------------------------------------"));

  configureUnusedPins();
  
  // Set analogRead() resolution:
  analogReadResolution(12);

//  // Initialize charge chip:
//  if (!PMIC.begin()) {
//    Serial.println("Failed to initialize PMIC!");
//  }
//  PMIC.disableSafetyTimer();

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
  client.setClientTimeout(NB_TIMEOUT); // this requires edits to the NBClient class
  gprs.setTimeout(NB_TIMEOUT);
  mqttClient.setConnectionTimeout(MQTT_TIMEOUT);

  // Configure SSL and MQTT:
  ArduinoBearSSL.onGetTime(getTime);
  sslClient.setEccSlot(0, CERT);
  mqttClient.setTxPayloadSize(512); // should increase if NUM_MEAS > 6
  mqttClient.onMessage(messageReceived);

  // Connect to NB and MQTT:
  verifyConnection(false);

  // Begin RTC and set:
  if (!rtc.begin()) {
    Serial.println(F("ERROR! Couldn't find RTC"));
  }
//  lastRTCSetTime = setRTC();
//  if (!verifyTime()) {
//    // Failed, use the modem time instead:
//    lastRTCSetTime = setRTCwithSARA();
//  }
  lastRTCSetTime = setRTCwithSARA();
  Watchdog.reset();

  // Initialize sensors:
  configBMP();

  // Initialize milliseconds counter:
  Serial.println(F("Initializing ms counter..."));
  rtc.writeSqwPinMode(DS3231_SquareWave1kHz); // set a 1024 Hz square wave to be output on the SQW pin
  attachInterrupt(digitalPinToInterrupt(0), msCounter, FALLING); // the function 'msCounter' will run every time pin D2 falls (which happens 1024 times a second)
  lastMSCalibTime = calibMS();

  // Check desired state:
  Watchdog.reset();
  while (sleeping == 2) {
    checkShadow();
  }
}

//=============================================================================================

void loop() {  
  if (sleeping == 1) {
    // --------------------------------------- SLEEPING ---------------------------------------  
    // Disconnect and sleep:
    Serial.println(F("Disconnecting from MQTT..."));
    mqttClient.stop();
    Serial.println(F("Disconnecting from NB..."));
    nbAccess.shutdown();
    Serial.println(F("NB disconnected"));
    delay(100);
    Serial.end();
    configurePins(true);
    Watchdog.disable();

    LowPower.deepSleep(SLEEP_CHECK_PERIOD * 1000);
    
    Serial.begin(500000);
    delay(2000);
    Watchdog.enable(WATCHDOG_MS);
    Watchdog.reset();
    configurePins(false);

    // Connect to NB and MQTT:
    softReset();
    verifyConnection(true); // set true to check battery

    //Check shadow state:
    checkShadow();
    if (messagePending == 1) {
      // If failed, try again:
      checkShadow();
    }
  
  } else {
    // ----------------------------------------- AWAKE ----------------------------------------
    Watchdog.reset();

    // Reset sensors and check shadow:
    if (rtc.now().unixtime() > lastResetTime + WAKE_CHECK_PERIOD) {
      Serial.println(F("Checking shadow..."));
      checkShadow();

      bmp1.reset();
      bmp2.reset();
      configBMP();
      lastResetTime = rtc.now().unixtime();

      // Recalibrate RTC once every two days:
      if (rtc.now().unixtime() > lastRTCSetTime + 172800) {
//        lastRTCSetTime = setRTC();
//        if (!verifyTime()) {
//          lastRTCSetTime = setRTC();
//        }
        lastRTCSetTime = setRTCwithSARA();

        lastMSCalibTime = calibMS();
      }
    }

    // Recalibrate milliseconds counter every 2 minutes:
    if (rtc.now().unixtime() > lastMSCalibTime + 120 && milliseconds > 900 && milliseconds < 990) { //the below code waits for a new second to rollover, so only run it when it's about to happen anyway otherwise just wasting time
      lastMSCalibTime = calibMS();
      Serial.println(F("Millis recalibrated"));
    }

    // Take measurements:
    float Ta, Tb;
    float Pa[NUM_MEAS], Pb[NUM_MEAS];

    Serial.println(F("Getting measurements..."));
    getReadings(Pa, Ta, Pb, Tb);

    // Check connection before trying to poll/send:
    Serial.println(F("Verifying connection..."));
    verifyConnection(false);

    Serial.println(F("Polling..."));
    mqttClient.poll();

    // Upload the measurements:
    publishReadings(Pa, Ta, Pb, Tb);
  }
}

//=============================================================================================
/*
   Set unused pins to INPUT_PULLUP to minimize power consumption. Do not change
   D6 since the orange LED is tied to this pin.
*/
void configureUnusedPins() {
  pinMode(3, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(A6, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
}

//=============================================================================================
/*
   Set pins to INPUT_PULLUP to minimize power consumption when sleeping.
   Restore their functionality to OUTPUT when not sleeping.
*/
void configurePins(bool sleep) {
  if (sleep) {
    pinMode(7, INPUT_PULLUP);
    pinMode(5, INPUT_PULLUP);
    pinMode(4, INPUT_PULLUP);
    pinMode(A0, INPUT_PULLUP);
    pinMode(SCL, INPUT_PULLUP);
    pinMode(SDA, INPUT_PULLUP);
    pinMode(MISO, INPUT_PULLUP);
    pinMode(SCK, INPUT_PULLUP);
    pinMode(MOSI, INPUT_PULLUP);
  } else {
    pinMode(7, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(A0, INPUT);
    pinMode(SCL, OUTPUT);
    pinMode(SDA, OUTPUT);
    pinMode(MISO, OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(MOSI, OUTPUT);
  }
}

//=============================================================================================
/*
   Get UNIX time in seconds from cellular module. Disregards sub-second information.
   For more precise implementation, see syncClock below.
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
   Set the DS3231 by getting the time from the modem.
*/
unsigned long setRTCwithSARA() {
  Serial.println(F("Synchronizing RTC with NB time"));
  unsigned long initTime = nbAccess.getTime();
  unsigned long start = millis();
  while (nbAccess.getTime() == initTime && millis() < start + 5000) {}

  rtc.adjust(DateTime(nbAccess.getTime()));
  milliseconds = 0;
  Serial.print(F("RTC set: "));
  Serial.println(rtc.now().unixtime());
  return rtc.now().unixtime();
}

//=============================================================================================
/*
   Set the DS3231 by getting the time from the NTP server via the cellular
   network.
*/
/*
unsigned long setRTC() {
  Serial.println(F("Synchronizing RTC with NTP time"));

  unsigned long epoch;
  unsigned long measured_at;

  // Try maximum 5 times to get the time from NTP:
  byte attemptCount = 0;
  while (attemptCount < 5) {
    if (getNTPTime(epoch, measured_at)) {
      break;
    } else {
      Watchdog.reset();
      delay(500);
      attemptCount++;
    }
  }

  // If successful, update the RTC when a new second rolls over:
  if (attemptCount < 5) {
    float sinceEpoch;
    int t_millisecs;
    unsigned long t_secs;
    // Wait for milliseconds to equal 0:
    while (t_millisecs != 0) {
      sinceEpoch = (millis() - measured_at) / 1000.0;
      t_secs = epoch + (unsigned long)sinceEpoch;
      t_millisecs = (sinceEpoch - (unsigned long)sinceEpoch) * 1000;
    }

    // Verify the timestamp makes sense:
    if (t_secs > 1600000000UL && t_secs < 2000000000UL) {
      rtc.adjust(DateTime(t_secs));
      milliseconds = 0;
      Serial.print(F("RTC set: "));
      Serial.println(rtc.now().unixtime());
      return t_secs;
    } else {
      Serial.print(F("Bad time received, RTC not set"));
    }
  }

  // If not successful, try again in an hour
  return rtc.now().unixtime() - 86400;
}
*/

//=============================================================================================
/*
   This function is taken from the ezTime library. Queries the NTP server
   for the time.
*/
/*
bool getNTPTime(unsigned long &t_unix, unsigned long &measured_at) {
  byte buffer[NTP_PACKET_SIZE];
  memset(buffer, 0, NTP_PACKET_SIZE);
  buffer[0] = 0b11100011;   // LI, Version, Mode
  buffer[1] = 0;        // Stratum, or type of clock
  buffer[2] = 9;        // Polling Interval (9 = 2^9 secs = ~9 mins, close to our 10 min default)
  buffer[3] = 0xEC;     // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  buffer[12]  = 'X';      // "kiss code", see RFC5905
  buffer[13]  = 'E';      // (codes starting with 'X' are not interpreted)
  buffer[14]  = 'Z';
  buffer[15]  = 'T';

  Serial.println(F("Attempting to get time from NTP server"));
  udp.begin(NTP_LOCAL_PORT);
  unsigned long started = millis();

  udp.beginPacket(NTP_SERVER, 123); //NTP requests are to port 123
  udp.write(buffer, NTP_PACKET_SIZE);
  udp.endPacket();

  // Wait for packet or return false with timed out
  while (!udp.parsePacket()) {
    delay (1);
    if (millis() - started > NTP_TIMEOUT) {
      udp.stop();
      Serial.println(F("NTP request timed out"));
      return false;
    }
  }

  udp.read(buffer, NTP_PACKET_SIZE);
  udp.stop();
  Serial.print(F("Received NTP in "));
  Serial.print(millis() - started);
  Serial.println(F(" ms"));

  //prepare timestamps
  uint32_t highWord, lowWord;
  highWord = ( buffer[16] << 8 | buffer[17] ) & 0x0000FFFF;
  lowWord = ( buffer[18] << 8 | buffer[19] ) & 0x0000FFFF;
  uint32_t reftsSec = highWord << 16 | lowWord;       // reference timestamp seconds

  highWord = ( buffer[32] << 8 | buffer[33] ) & 0x0000FFFF;
  lowWord = ( buffer[34] << 8 | buffer[35] ) & 0x0000FFFF;
  uint32_t rcvtsSec = highWord << 16 | lowWord;       // receive timestamp seconds

  highWord = ( buffer[40] << 8 | buffer[41] ) & 0x0000FFFF;
  lowWord = ( buffer[42] << 8 | buffer[43] ) & 0x0000FFFF;
  uint32_t secsSince1900 = highWord << 16 | lowWord;      // transmit timestamp seconds

  highWord = ( buffer[44] << 8 | buffer[45] ) & 0x0000FFFF;
  lowWord = ( buffer[46] << 8 | buffer[47] ) & 0x0000FFFF;
  uint32_t fraction = highWord << 16 | lowWord;       // transmit timestamp fractions

  //check if received data makes sense
  //buffer[1] = stratum - should be 1..15 for valid reply
  //also checking that all timestamps are non-zero and receive timestamp seconds are <= transmit timestamp seconds
  if ((buffer[1] < 1) or (buffer[1] > 15) or (reftsSec == 0) or (rcvtsSec == 0) or (rcvtsSec > secsSince1900)) {
    Serial.println(F("Received invalid packet"));
    return false;
  }

  // Set the t and measured_at variables that were passed by reference
  uint32_t done = millis();
  t_unix = secsSince1900 - 2208988800UL;         // Subtract 70 years to get seconds since 1970
  uint16_t ms = fraction / 4294967UL;         // Turn 32 bit fraction into ms by dividing by 2^32 / 1000
  measured_at = done - ((done - started) / 2) - ms; // Assume symmetric network latency and return when we think the whole second was.

  Serial.print(F("Successfully received time from NTP server: "));
  Serial.println(t_unix);
  return true;
}
*/

//=============================================================================================
/*
   Checks that the time has been set correctly. Returns false if erroneous time.
*/
bool verifyTime() {
  unsigned long t_RTC = rtc.now().unixtime();
  unsigned long t_NB = nbAccess.getTime();

  Serial.print(F("Verify time - RTC time: "));
  Serial.print(t_RTC);
  Serial.print(F(". NB time: "));
  Serial.println(t_NB);

  if (t_RTC > 1600000000UL && t_RTC < 1900000000UL) {
    return true;
  }
//  if (t_RTC > t_NB + 2 || t_RTC < t_NB - 2) {
//    return false;
//  }
  return false;
}

//=============================================================================================
/*
   This interrupt subroutine is attached to a pin which is connected to
   the DS3231's 1024 Hz square wave output. Note that a 10 kOhm resistor
   needs to be placed between the VCC and SQW/INT pins on the DS3231 to
   enable the square wave output.
*/
void msCounter() {
  milliseconds = milliseconds + 0.97656;
  if (milliseconds > 1000) {
    milliseconds = milliseconds - 1000;
  }
}

//=============================================================================================
/*
   Synchronizes the millisecond counter interrupt sub-routine (see function
   'msCounter') with the DS3231's clock.
*/
unsigned long calibMS() {
  unsigned long initTime = rtc.now().unixtime();
  while (rtc.now().unixtime() == initTime) {} // wait until the second changes to reset the ms counter
  milliseconds = 0;

  return rtc.now().unixtime();
}

//=============================================================================================
/*
   If connection is not established, attempt to connect both NB and MQTT until
   successful.
*/
void verifyConnection(bool checkBatt) {
  byte mqttFailures = 0; // if NB repeatedly connects but MQTT repeatedly fails, hard reset
  client.setAbortMillis(millis() + OVERALL_TIMEOUT); // watchdog will no longer reset after this millis count. Note: requires edit to the NBClient class
  
  while (!mqttClient.connected()) {
    if (checkBatt) {
      checkBattery();
    }
    
    Watchdog.reset();
    byte attemptCount = 0;
    while (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) {
      if (attemptCount > 4) {
        // Reset modem:
        nbAccess.hardReset();
        delay(1000);
        Watchdog.reset();
        attemptCount = 0;
      } else {
        attemptCount++;
      }
      connectNB();
    }

    // Wait 10 seconds:
    for (byte i = 0; i<=5; i++) {
      delay(2000);
      Watchdog.reset();
    }

    // Connect to MQTT:
    if (!connectMQTT()) {
      mqttFailures++;
    }
    Watchdog.reset();

    if (mqttFailures > 3) {
      Serial.println(F("More than 4 failed attempts to connect MQTT, hard resetting"));
      Watchdog.reset();
      nbAccess.hardReset();
      client.setAbortMillis(millis() + OVERALL_TIMEOUT); // reset the abort millis
      Watchdog.reset();
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
    mqttClient.subscribe(TOPIC_SHADOW_IN, 0);
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
  delay(5000);
  Serial.println(F("Modem was soft reset"));
}

//=============================================================================================
/*
   Send a message prompting the publish of the mote shadow's current state.
   Since PubSubClient is set to listen to the topic on which the state will
   be published (TOPIC_SHADOW_IN), that incoming message will trigger the
   function messageReceived.
*/
void checkShadow() {
  verifyConnection(false);

  messagePending = 1;
  mqttClient.beginMessage(TOPIC_SHADOW_OUT);
  mqttClient.print(F(""));
  mqttClient.endMessage();
  Serial.println(F("Sent get message"));

  // Wait to receive response:
  unsigned long startMillis = millis();
  while (messagePending == 1 && millis() < startMillis + 2000) { // wait a maximum of 2 seconds
    mqttClient.poll();
    delay(10);
  }

  if (messagePending == 1) {
    Serial.println(F("No shadow received"));
    consecCheckFailures++;
    if (consecCheckFailures > 4) {
      // Force sleep
      Serial.println(F("Failed several times to get shadow, forcing sleep"));
//      bmp1.stopConversion();
//      bmp2.stopConversion();
      sleeping = 1;
    }
  }
}

//=============================================================================================
/*
   Take 10 measurements of battery voltage and return the mean in V.
*/
float readBattVoltage(int pin, float multiplier) {
  float result = 0;
  float totalResult = 0;
  byte count = 0;
  for (byte i = 0; i < 10; i++) {
    result = analogRead(pin) * multiplier;
    if (result > 2.5 && result < 4.5) {
      totalResult += result;
      count ++;
    }
    delay(5);
  }
  if (count == 0) {
    return 0;
  }
  Serial.print("Battery: ");
  Serial.print(totalResult / count);
  Serial.println(" V");
  
  return totalResult / count;
}

//=============================================================================================
/*
   This function is triggered when a message is received on the topic
   TOPIC_SHADOW_IN. The message received is the mote's shadow state. This
   function parses the shadow for desired state and (re-)enters sleep/wake
   mode as desired. It also publishes to the shadow the current battery
   voltage and cellular signal strength.
*/
void messageReceived(int messageSize) {
  // No longer waiting for message:
  messagePending = 0;

  // Read the message:
  char msgIn[55];
  byte i = 0;
  while (mqttClient.available()) {
    if (i < 55) {
      msgIn[i] = (char)mqttClient.read();
      i++;
    } else {
      mqttClient.read();
    }
  }

  Serial.println(F("Message received:"));
  Serial.println(msgIn);
  Serial.println(msgIn[52]);
  consecCheckFailures = 0; // reset counter

  if (msgIn[52] == '1') {
    sleeping = 1;
    Serial.println(F("Preparing to (re-)enter sleep mode"));
  } else if (msgIn[52] == '0') {
    sleeping = 0;
    Serial.println(F("Preparing to (re-)enter wake mode"));
  } else {
    sleeping = 1;
    Serial.println(F("Error parsing desired state, defaulting to sleep mode"));
  }

  // Get 10 measurements of battery voltages and publish the average to the shadow state:
  float battVoltage = readBattVoltage(BATT_PIN, BATT_FACTOR);

  // Measure cellular signal strength to publish as well:
  float signalStrength = scannerNetworks.getSignalStrength().toInt();

  // Update reported state:
  mqttClient.beginMessage(TOPIC_SHADOW_UPDATE);
  mqttClient.print(F("{\"state\" : {\"reported\": {\"sleeping\": "));
  mqttClient.print(sleeping);
  mqttClient.print(F(",\"vbatt\" : "));
  mqttClient.print(battVoltage);
  mqttClient.print(F(",\"signal\" : "));
  mqttClient.print(signalStrength);
  mqttClient.print(F(",\"resetcount\" : "));
  mqttClient.print(nbAccess.getResetCount());
  mqttClient.print(F(",\"SIM\" : "));
  mqttClient.print(nbAccess.getICCID());
  mqttClient.print(F("}}}"));
  bool pubResult = mqttClient.endMessage();
  if (pubResult) {
    Serial.println(F("Published reported state"));
  } else {
    Serial.println(F("Error publishing reported state"));
  }

  // Set Watchdog and sensor accordingly:
  if (sleeping == 1) {
//    bmp1.stopConversion();
//    bmp2.stopConversion();
  } else {
    configBMP();
  }
}

//=============================================================================================
/*
   Configures the two BMP388 sensors as desired.
*/
void configBMP() {
  bmp1.begin();
  bmp1.setTimeStandby(TIME_STANDBY_80MS);
  bmp1.setIIRFilter(IIR_FILTER_OFF);
  bmp1.setPresOversampling(OVERSAMPLING_X16);
  bmp1.setTempOversampling(OVERSAMPLING_X2);
  bmp1.enableFIFO();
  bmp1.setFIFONoOfMeasurements(NUM_MEAS);

  bmp2.begin();
  bmp2.setTimeStandby(TIME_STANDBY_80MS);
  bmp2.setIIRFilter(IIR_FILTER_OFF);
  bmp2.setPresOversampling(OVERSAMPLING_X16);
  bmp2.setTempOversampling(OVERSAMPLING_X2);
  bmp2.enableFIFO();
  bmp2.setFIFONoOfMeasurements(NUM_MEAS);

  bmp1.flushFIFO();
  bmp2.flushFIFO();

  bmp1.startNormalConversion();
  bmp2.startNormalConversion();
}

//=============================================================================================
/*
   Read the measurements stored in the BMP388's FIFO memory. These
   are stored in the variables T{a,b} and P{a,b}, passed in by
   reference. The time of the last measurement is stored in global
   variables t_s and t_ms.
*/
void getReadings(float *Pa, float &Ta, float *Pb, float &Tb) {

  // Wait until time to make a new measurement:
  Serial.println(F("Waiting for millis..."));
  int t_delta = milliseconds - t_ms;
  if (t_delta < 0) {
    t_delta += 1000;
  }
  while (t_delta < NUM_MEAS * 1000 / FREQ + MS_DT) {
    t_delta = milliseconds - t_ms;
    if (t_delta < 0) {
      t_delta += 1000;
    }
  }
  Serial.println(F("Querying sensors..."));

  // Read sensors
  bool PAready = false;
  bool PBready = false;
  unsigned long startMillis = millis();
  while (!PAready || !PBready) {
    if (!PAready && bmp1.getFIFOPresOneTemp(Pa, Ta)) {
      PAready = true;
    }
    if (!PBready && bmp2.getFIFOPresOneTemp(Pb, Tb)) {
      PBready = true;
    }

    if (millis() > startMillis + 1000) {
      Serial.println(F("Sensor timeout"));
      break;
    }
  }

  int delta = int(milliseconds) - t_ms;
  if (delta < 0) {
    delta += 1000;
  }
  Serial.print(F("Delta: "));
  Serial.println(delta);

  // Milliseconds can be slightly out of sync with RTC, handle the cases when it is:
  unsigned long t_s_new = rtc.now().unixtime();
  int t_ms_new = int(milliseconds);
  if (t_ms_new < t_ms && t_s_new == t_s) {
    t_s++;
  } else if (t_ms_new > 990 && t_ms_new > t_ms && t_s_new == t_s + 1) {
    // Don't update t_s
  } else {
    t_s = t_s_new;
  }
  t_ms = t_ms_new;
}

//=============================================================================================
/*
   Publish the pressure & temperature measurements to the MQTT broker.
*/
void publishReadings(float* Pa, float Ta, float* Pb, float Tb) {
  mqttClient.beginMessage(TOPIC_OUTGOING);
  mqttClient.print(F("{\n\t\"t\" : "));
  mqttClient.print(t_s);
  mqttClient.print(F("."));
  if (t_ms < 10) {
    mqttClient.print(F("00"));
  } else if (t_ms < 100) {
    mqttClient.print(F("0"));
  }
  mqttClient.print(t_ms);
  for (int idx = 0; idx < NUM_MEAS; idx++) {
    mqttClient.print(F(",\n\t\"Pa"));
    mqttClient.print(idx);
    mqttClient.print(F("\" : "));
    (Pa[idx] > 80000.0 && Pa[idx] < 120000.0) ? mqttClient.print(Pa[idx] - 100000.0) : mqttClient.print(-100000.0);
    mqttClient.print(F(",\n\t\"Pb"));
    mqttClient.print(idx);
    mqttClient.print(F("\" : "));
    (Pb[idx] > 80000.0 && Pb[idx] < 120000.0) ? mqttClient.print(Pb[idx] - 100000.0) : mqttClient.print(-100000.0);
    mqttClient.print(F(",\n\t\"Pc"));
    mqttClient.print(idx);
    mqttClient.print(F("\" : "));
    mqttClient.print(-100000.0);
  }
  mqttClient.print(F(",\n\t\"Ta"));
  mqttClient.print(F("\" : "));
  (Ta > -40.0 && Ta < 80.0) ? mqttClient.print(Ta) : mqttClient.print("-100.0");
  mqttClient.print(F(",\n\t\"Tb"));
  mqttClient.print(F("\" : "));
  (Tb > -40.0 && Tb < 80.0) ? mqttClient.print(Tb) : mqttClient.print("-100.0");
  mqttClient.print(F(",\n\t\"Tc"));
  mqttClient.print(F("\" : "));
  mqttClient.print("-100.0");
  mqttClient.print(F("\n}"));
  mqttClient.endMessage();
  Serial.println(F("Measurement sent"));
}

//=============================================================================================
/*
   If the battery capacity is < ~10%, deep sleep
*/
void checkBattery() {
  float battVoltage = readBattVoltage(BATT_PIN, BATT_FACTOR);
  while (battVoltage > 0.1 && battVoltage < MIN_BATT_VOLTAGE) { // if something goes wrong, voltage will read 0. Allow program to proceed in this case
    Serial.print(F("Sleeping because battery voltage is too low at "));
    Serial.print(battVoltage);
    Serial.println(F(" V"));
    
    Serial.println(F("If connected, disconnecting from MQTT..."));
    mqttClient.stop();
    Serial.println(F("If connected, disconnecting from NB..."));
    nbAccess.shutdown();
    
    Serial.println(F("Sleeping for one hour..."));
    Serial.end();
    configurePins(true);
    Watchdog.disable();

    LowPower.deepSleep(ONE_HOUR);
    
    Serial.begin(500000);
    delay(2000);
    Watchdog.enable(WATCHDOG_MS);
    Watchdog.reset();
    configurePins(false);
    
    battVoltage = readBattVoltage(BATT_PIN, BATT_FACTOR);
  }
}
