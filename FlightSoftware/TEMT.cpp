#include <Arduino.h>
#include <TEMT6000.h>
#include "TEMT.h"


int TEMT_Data = 0;
TEMT6000 lux(A0, 5, 1023);
bool TEMT_initialized = false;

void TEMT_init(){
  // no init for TEMT (yet)
  Serial.println("TEMT succesfully initialized!");
  //SHORT LED BEEP
  digitalWrite(2, HIGH);
  delay(200);
  digitalWrite(2, LOW);
  TEMT_initialized = true;
}
void TEMT_run(){
  // returns the analog value in LUX
  TEMT_Data = lux.readLUX();
}
int get_TEMT_Data(){
  return TEMT_Data;
}