/////////////////////////////////////////////////////////////////////////////////////
// bmpC88_DEV - SPI Communications, Default Configuration, Normal Conversion, FIFO 
/////////////////////////////////////////////////////////////////////////////////////

#include <BMP388_DEV.h>                             // Include the bmpC88_DEV.h library

#define FREQ 12.5
#define NUM_MEAS 10
#define MS_DT -8

BMP388_DEV      bmpA(4); // connect CS to pin D4
BMP388_DEV      bmpB(5); // connect CS to pin D5
BMP388_DEV      bmpC(7); // connect CS to pin D7

unsigned long lastMillis;

void setup() 
{
  Serial.begin(500000);
  
  configBMP();
}

void loop() 
{  
  float _Ta[NUM_MEAS], _Tb[NUM_MEAS], _Tc[NUM_MEAS];
  float _Pa[NUM_MEAS], _Pb[NUM_MEAS], _Pc[NUM_MEAS];

  getReadings(_Ta, _Pa, _Tb, _Pb, _Tc, _Pc);

  Serial.print("Pa: ");
  printVector(_Pa);
  Serial.print(" Pb: ");
  printVector(_Pb);
  Serial.print("  Pc: ");
  printVector(_Pc);
  Serial.println("");

//  delay(150);
}

//=============================================================================================
void configBMP() {
  bmpA.begin();
  bmpA.setTimeStandby(TIME_STANDBY_80MS);
  bmpA.setIIRFilter(IIR_FILTER_OFF);
  bmpA.setPresOversampling(OVERSAMPLING_X16);
  bmpA.setTempOversampling(OVERSAMPLING_X2);
  bmpA.enableFIFO();
  bmpA.setFIFONoOfMeasurements(NUM_MEAS);

  bmpB.begin();
  bmpB.setTimeStandby(TIME_STANDBY_80MS);
  bmpB.setIIRFilter(IIR_FILTER_OFF);
  bmpB.setPresOversampling(OVERSAMPLING_X16);
  bmpB.setTempOversampling(OVERSAMPLING_X2);
  bmpB.enableFIFO();
  bmpB.setFIFONoOfMeasurements(NUM_MEAS);

  bmpC.begin();
  bmpC.setTimeStandby(TIME_STANDBY_80MS);
  bmpC.setIIRFilter(IIR_FILTER_OFF);
  bmpC.setPresOversampling(OVERSAMPLING_X16);
  bmpC.setTempOversampling(OVERSAMPLING_X2);
  bmpC.enableFIFO();
  bmpC.setFIFONoOfMeasurements(NUM_MEAS);

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
   are stored in the arrays T{a,b,c} and P{a,b,c}, passed in by
   reference. The time of the last measurement is stored in global
   variables t_s and t_ms.
*/
void getReadings(float* Ta, float* Pa, float* Tb, float* Pb, float* Tc, float* Pc) {

  // Wait until time to make a new measurement:
  while (millis() < lastMillis + NUM_MEAS * 1000 / FREQ + MS_DT) {}

  // Read sensors
  bool PAready = false;
  bool PBready = false;
  bool PCready = false;
  unsigned long startMillis = millis();
  while (!PAready || !PBready || !PCready) {
    if (!PAready && bmpA.getFIFOTempPres(Ta, Pa)) {
      PAready = true;
    }
    if (!PBready && bmpB.getFIFOTempPres(Tb, Pb)) {
      PBready = true;
    }
    if (!PCready && bmpC.getFIFOTempPres(Tc, Pc)) {
      PCready = true;
    }

    if (millis() > startMillis + 1000) {
      Serial.println(F("Sensor timeout"));
      break;
    }
  }

  Serial.print(F("Delta: "));
  Serial.println(millis() - lastMillis);
  lastMillis = millis();
}

//=============================================================================================
void printVector(float* vec) {
  Serial.print("[ ");
  for (int k = 0; k < NUM_MEAS; k++) {
    Serial.print(vec[k]);
    Serial.print(" ");
  }
  Serial.println("]");
}
