//========================================================================
#define DEV_NOTES "Dev notes: sensors are always on"
//========================================================================

#include <Arduino_PMIC.h>
#include <Adafruit_SleepyDog.h>
#include <ArduinoECCX08.h>
#include <ArduinoLowPower.h>
#include <BMP388_DEV.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

//---------------------
#define HOSTNAME "wm21" // name of this mote
//---------------------

// SD card options
#define ACQ_FILE HOSTNAME ".txt"
#define CALIB_FILE HOSTNAME "_calib.txt"
#define SD_CS 3

// Measurement frequency in Hz and number of measurements to be cached by sensor:
#define FREQ 12.5
#define NUM_MEAS 10
#define MS_DT -6 // add this many milliseconds to the wait period (depends on NUM_MEAS, tune to get exactly 12.5 Hz measurements)
#define CHECK_NUM_MEAS 30
#define PRMS_THRESH 6

// Time to take measurements before re-checking desired state (in minutes):
#define SLEEP_CHECK_PERIOD 20 * 60
#define WAKE_CHECK_PERIOD 60 * 60
#define ONE_HOUR 60 * 60000

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
BMP388_DEV      bmpA(4); // connect CS to pin D4
BMP388_DEV      bmpB(5); // connect CS to pin D5
BMP388_DEV      bmpC(7); // connect CS to pin D7
RTC_DS3231      rtc;

// Initialize variables:
unsigned long lastResetTime = 0;
unsigned long lastMSCalibTime = 0;
volatile float milliseconds = 0;
unsigned long t_s = 0;
int t_ms = 0;

byte sleeping = 2; // 0 = awake, 1 = asleep, 2 = unassigned

File fid;

//=============================================================================================

// Code in setup() runs only once:
void setup() {
  Serial.begin(500000);
  delay(5000);
  Serial.println(F("------------------------------------------------------------------"));
  Serial.print(HOSTNAME);
  Serial.println(F(" : 2022/12/16 Build - Cp, Disconnected"));
  Serial.println(DEV_NOTES);
  Serial.print("Prms thresh: ");
  Serial.print(PRMS_THRESH);
  Serial.println(" Pa");
  Serial.println(F("------------------------------------------------------------------"));

//  // Initialize charge chip:
//  if (!PMIC.begin()) {
//    Serial.println("Failed to initialize PMIC!");
//  }
//  PMIC.disableSafetyTimer();
  
  // To save power:
  configureUnusedPins();
  ECCX08.end();
  
  // Enable watchdog:
  Watchdog.enable(WATCHDOG_MS);
  Serial.println("Watchdog enabled");
  Watchdog.reset();

  // Set analogRead() resolution:
  analogReadResolution(12);

  // Initialize sensors:
  configBMP(NUM_MEAS);

  // Begin RTC:
  if (!rtc.begin()) {
    Serial.println(F("ERROR! Couldn't find RTC"));
  }

  // Begin SD card:
  if (!SD.begin(SD_CS)) {
    Serial.println(F("Failed to initialize SD card"));
    while (1);
  } else {
    Serial.println(F("SD card initialized"));
  }

  fid = SD.open(ACQ_FILE, O_WRITE | O_APPEND | O_CREAT);
  if (!fid) {
    Serial.println(F("Failed to open measurements file"));
    while (1);
  } else {
    Serial.println(F("Measurement file opened"));
  }

  // Initialize milliseconds counter:
  Serial.println(F("Initializing ms counter..."));
  rtc.writeSqwPinMode(DS3231_SquareWave1kHz); // set a 1024 Hz square wave to be output on the SQW pin
  attachInterrupt(digitalPinToInterrupt(0), msCounter, FALLING); // the function 'msCounter' will run every time pin D2 falls (which happens 1024 times a second)
  lastMSCalibTime = calibMS();

  // Check desired state:
  Watchdog.reset();
  while (sleeping == 2) {
    checkSensors();
  }
}

//=============================================================================================

// This code runs continuously:
void loop() {
  if (sleeping == 1) {
    // --------------------------------------- SLEEPING ---------------------------------------
    Serial.end();
    configurePins(true);
    Watchdog.disable();

    LowPower.deepSleep(SLEEP_CHECK_PERIOD * 1000);

    Serial.begin(500000);
    delay(2000);
    Watchdog.enable(WATCHDOG_MS);
    Watchdog.reset();
    configurePins(false);

    //Check desired state:
    checkSensors();

  } else {
    // ----------------------------------------- AWAKE ----------------------------------------
    Watchdog.reset();

    // Reset sensors and check state:
    if (rtc.now().unixtime() > lastResetTime + WAKE_CHECK_PERIOD) {
      Serial.println(F("Checking sensor Prms..."));
      checkSensors();

      bmpA.reset();
      bmpB.reset();
      bmpC.reset();
      configBMP(NUM_MEAS);
      lastResetTime = rtc.now().unixtime();
    }

    // Recalibrate milliseconds counter every 2 minutes:    
    if (rtc.now().unixtime() > lastMSCalibTime + 120 && milliseconds > 900 && milliseconds < 990) { //the below code waits for a new second to rollover, so only run it when it's about to happen anyway otherwise just wasting time
      lastMSCalibTime = calibMS();
      Serial.println(F("Millis recalibrated"));
    }

    // Take measurements:
    float Ta, Tb, Tc;
    float Pa[NUM_MEAS], Pb[NUM_MEAS], Pc[NUM_MEAS];

    Serial.println(F("Getting measurements..."));
    getReadings(Pa, Ta, Pb, Tb, Pc, Tc);

    // Write the measurements to SD card:
    writeReadings(Pa, Ta, Pb, Tb, Pc, Tc, NUM_MEAS);
  }
}

//=============================================================================================
/*
   Set unused pins to INPUT_PULLUP to minimize power consumption. Do not change
   D6 since the orange LED is tied to this pin.
*/
void configureUnusedPins() {
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
    pinMode(3, INPUT_PULLUP);
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
    pinMode(3, OUTPUT);
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
  Serial.println(initTime);
  while (rtc.now().unixtime() == initTime) {} // wait until the second changes to reset the ms counter
  milliseconds = 0;

  return rtc.now().unixtime();
}

//=============================================================================================
/*
   Checks Prms of the sensors to determine whether to acquire data
*/
void checkSensors() {
  // Take and save more measurements:
  configBMP(CHECK_NUM_MEAS);
  float Ta, Tb, Tc;
  float Pa[CHECK_NUM_MEAS], Pb[CHECK_NUM_MEAS], Pc[CHECK_NUM_MEAS];

/*
  Serial.println(F("Waiting 10 s..."));
  for (int i = 0; i<2; i++) {
    Watchdog.reset();
    delay(5000);
  }
  */

  Watchdog.reset();
  Serial.println(F("Getting measurements..."));
  getCheckReadings(Pa, Ta, Pb, Tb, Pc, Tc); // throw away first set
  getCheckReadings(Pa, Ta, Pb, Tb, Pc, Tc); 

  // Calculate standard deviation for each:
  float PrmsA = STDEV(Pa, CHECK_NUM_MEAS);
  float PrmsB = STDEV(Pb, CHECK_NUM_MEAS);
  float PrmsC = STDEV(Pc, CHECK_NUM_MEAS);
  Serial.print("RMS pressures: A: ");
  Serial.print(PrmsA);
  Serial.print(" Pa, B: ");
  Serial.print(PrmsB);
  Serial.print(" Pa, C: ");
  Serial.print(PrmsC);
  Serial.println(" Pa");

  // Reset the configuration back to normal:
  configBMP(NUM_MEAS);

  // Write the measurements to the calib log:
  fid.close(); // close main file
  fid = SD.open("calib.txt", O_WRITE | O_APPEND | O_CREAT);
  writeReadings(Pa, Ta, Pb, Tb, Pc, Tc, CHECK_NUM_MEAS);
  fid.close();

  // Write battery voltage to battery log:
  float vbatt = readBattVoltage(BATT_PIN, BATT_FACTOR);
  Serial.print("Battery: ");
  Serial.print(vbatt);
  Serial.println(" V");
  
  fid = SD.open("batt.txt", O_WRITE | O_APPEND | O_CREAT);
  fid.print(rtc.now().unixtime());
  fid.print(", ");
  fid.print(vbatt);
  fid.print(",\n");
  fid.close();

  fid = SD.open(ACQ_FILE, O_WRITE | O_APPEND | O_CREAT); // re-open the main file
  
  // Wake if any one of these exceeds threshold, but not erroneously so:
//  if (((PrmsA > PRMS_THRESH && PrmsA < 400) || 
//      (PrmsB > PRMS_THRESH && PrmsB < 400) ||
//      (PrmsC > PRMS_THRESH && PrmsC < 400)) && checkBattery()) {
//  if (rtc.now().hour() >= 19 && rtc.now().hour() < 22) {
  if (1) {
    sleeping = 0;
    Serial.println("(Re)-entering wake mode");
    lastResetTime = rtc.now().unixtime();
  } else {
    sleeping = 1;
//    bmpA.stopConversion();
//    bmpB.stopConversion();
//    bmpC.stopConversion();
    Serial.println("(Re)-entering sleep mode");
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
  return totalResult / count;
}

//=============================================================================================
/*
   Check battery voltage, returns true if high enough to take
   measurements, false otherwise. If something goes wrong, voltage 
   will read 0. Allow program to proceed in this case.
*/
bool checkBattery() {
  float battVoltage = readBattVoltage(BATT_PIN, BATT_FACTOR);
  if (battVoltage > 0.1 && battVoltage < MIN_BATT_VOLTAGE) {
    return false; 
  }
  return true;
}

//=============================================================================================
/*
   Configures the three BMP388 sensors as desired.
*/
void configBMP(int FIFO_num) {
  bmpA.begin();
  bmpA.setTimeStandby(TIME_STANDBY_80MS);
  bmpA.setIIRFilter(IIR_FILTER_OFF);
  bmpA.setPresOversampling(OVERSAMPLING_X16);
  bmpA.setTempOversampling(OVERSAMPLING_X2);
  bmpA.enableFIFO();
  bmpA.setFIFONoOfMeasurements(FIFO_num);

  bmpB.begin();
  bmpB.setTimeStandby(TIME_STANDBY_80MS);
  bmpB.setIIRFilter(IIR_FILTER_OFF);
  bmpB.setPresOversampling(OVERSAMPLING_X16);
  bmpB.setTempOversampling(OVERSAMPLING_X2);
  bmpB.enableFIFO();
  bmpB.setFIFONoOfMeasurements(FIFO_num);

  bmpC.begin();
  bmpC.setTimeStandby(TIME_STANDBY_80MS);
  bmpC.setIIRFilter(IIR_FILTER_OFF);
  bmpC.setPresOversampling(OVERSAMPLING_X16);
  bmpC.setTempOversampling(OVERSAMPLING_X2);
  bmpC.enableFIFO();
  bmpC.setFIFONoOfMeasurements(FIFO_num);

  bmpA.flushFIFO();
  bmpB.flushFIFO();
  bmpC.flushFIFO();

  bmpA.startNormalConversion();
  bmpB.startNormalConversion();
  bmpC.startNormalConversion();
}

//=============================================================================================
/*
   Read the measurements stored in the BMP388's FIFO memory. These
   are stored in floats T{a,b,c} and arrays P{a,b,c}. The time of 
   the last measurement is stored in global variables t_s and t_ms.
*/
void getReadings(float *Pa, float &Ta, float *Pb, float &Tb, float *Pc, float &Tc) {

  // Wait until time to make a new measurement:
  Serial.println(F("Waiting for millis..."));
  int t_delta = milliseconds - t_ms;
  if (t_delta < 0) {
    t_delta += 1000;
  }
  while (t_delta < NUM_MEAS * 1000.0 / FREQ + MS_DT) {
    t_delta = milliseconds - t_ms;
    if (t_delta < 0) {
      t_delta += 1000;
    }
  }
  Serial.println(F("Querying sensors..."));

  // Read sensors
  bool PAready = false;
  bool PBready = false;
  bool PCready = false;
  unsigned long startMillis = millis();
  while (!PAready || !PBready || !PCready) {
    if (!PAready && bmpA.getFIFOPresOneTemp(Pa, Ta)) {
      PAready = true;
    }
    if (!PBready && bmpB.getFIFOPresOneTemp(Pb, Tb)) {
      PBready = true;
    }
    if (!PCready && bmpC.getFIFOPresOneTemp(Pc, Tc)) {
      PCready = true;
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
   Write the pressure & temperature measurements to SD card.
*/
void writeReadings(float* Pa, float Ta, float* Pb, float Tb, float* Pc, float Tc, int num) {
  fid.print(t_s);
  fid.print(F("."));
  if (t_ms < 10) {
    fid.print(F("00"));
  } else if (t_ms < 100) {
    fid.print(F("0"));
  }
  fid.print(t_ms);
  fid.print(F(", "));
  for (int idx = 0; idx < num; idx++) {
    (Pa[idx] > 80000.0 && Pa[idx] < 120000.0) ? fid.print(Pa[idx] - 100000.0) : fid.print(-100000.0);
    fid.print(F(", "));
    (Pb[idx] > 80000.0 && Pb[idx] < 120000.0) ? fid.print(Pb[idx] - 100000.0) : fid.print(-100000.0);
    fid.print(F(", "));
    (Pc[idx] > 80000.0 && Pc[idx] < 120000.0) ? fid.print(Pc[idx] - 100000.0) : fid.print(-100000.0);
    fid.print(F(", "));
  }
  (Ta > -40.0 && Ta < 80.0) ? fid.print(Ta) : fid.print("-100.0");
  fid.print(F(", "));
  (Tb > -40.0 && Tb < 80.0) ? fid.print(Tb) : fid.print("-100.0");
  fid.print(F(", "));
  (Tc > -40.0 && Tc < 80.0) ? fid.print(Tc) : fid.print("-100.0");
  fid.print(F("\n"));

  fid.flush();
  Serial.println(F("Measurements written"));
}

//=============================================================================================
void printVector(float* vec, int sz) {
  Serial.print("[ ");
  for (int k = 0; k < sz; k++) {
    Serial.print(vec[k]);
    Serial.print(" ");
  }
  Serial.println("]");
}

//=============================================================================================
/*
   Read the measurements stored in the BMP388's FIFO memory. These
   are stored in the floats T{a,b,c} and arrays P{a,b,c}, passed in 
   by reference.
*/
void getCheckReadings(float *Pa, float &Ta, float *Pb, float &Tb, float *Pc, float &Tc) {
  delay(2000);
  
  // Read sensors
  bool PAready = false;
  bool PBready = false;
  bool PCready = false;
  unsigned long startMillis = millis();
  while (!PAready || !PBready || !PCready) {
    if (!PAready && bmpA.getFIFOPresOneTemp(Pa, Ta)) {
      PAready = true;
    }
    if (!PBready && bmpB.getFIFOPresOneTemp(Pb, Tb)) {
      PBready = true;
    }
    if (!PCready && bmpC.getFIFOPresOneTemp(Pc, Tc)) {
      PCready = true;
    }

    if (millis() > startMillis + 10000) {
      Serial.println(F("Sensor timeout"));
      break;
    }
  }

  t_s = rtc.now().unixtime();
  t_ms = 0;  // millisecond precision not necessary for calib measurements
}

//=============================================================================================
/*
   Calculate the standard deviation of elements of an array
*/
float STDEV(float* arr, int arr_size) {
  // First calculate the mean:
  float mean = 0;
  for (int i = 0; i < arr_size; i++) {
    mean += (arr[i] / arr_size);
  }

  // Now calculate the standard deviation:
  float sumsq = 0;
  for (int i = 0; i < arr_size; i++) {
    sumsq += sq(arr[i] - mean);
  }

  return sqrt(sumsq / arr_size);
}
