#include <Arduino.h>
#include <TEMT6000.h>

#include "Settings.h"
#include "LED.h"
#include "TEMT.h"
#include "DataBank.h"

float Sensor1_value = 0;
float Sensor2_value = 0;
float Sensor3_value = 0;

float TEMT_Data = 0;

TEMT6000 lux1(A0, 5, 1023);
TEMT6000 lux2(A1, 5, 1023);
TEMT6000 lux3(A2, 5, 1023);

void calc_TEMT_Data(){
  // Calculates the data from the pair that varies the least
  
  float dif1 = fabs(Sensor1_value - Sensor2_value);
  float dif2 = fabs(Sensor1_value - Sensor3_value);
  float dif3 = fabs(Sensor2_value - Sensor3_value);

  if (dif1 <= dif2 && dif1 <= dif3)
      TEMT_Data = (Sensor1_value + Sensor2_value) / 2.0f;
  else if (dif2 <= dif1 && dif2 <= dif3)
      TEMT_Data = (Sensor1_value + Sensor3_value) / 2.0f;
  else
      TEMT_Data = (Sensor2_value + Sensor3_value) / 2.0f;
}

void TEMT_init(){
  // no init for TEMT (yet)
  LOG("TEMT succesfully initialized!");
  //SHORT LED BEEP
  LED_beep(100);
  Status.temt = true;
}

void TEMT_run(){
  // returns the analog value in LUX

  Sensor1_value = lux1.readLUX();
  Sensor2_value = lux2.readLUX();
  Sensor3_value = lux3.readLUX();

  calc_TEMT_Data();
  MainBank.TEMT.Write_TEMT(TEMT_Data);
}
