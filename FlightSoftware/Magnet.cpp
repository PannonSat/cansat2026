#include <Arduino.h>

#include "Magnet.h"
#include "DataBank.h"

void Magnet_init(){
  pinMode(8, OUTPUT);
}

void Magnet_ON(){
  digitalWrite(8, HIGH);
  MainBank.Write_Magnet_Mode(true);
}

void Magnet_OFF(){
  digitalWrite(8, LOW);
  MainBank.Write_Magnet_Mode(false);
}