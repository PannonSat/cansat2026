#include <Arduino.h>

#include "Magnet.h"
#include "DataBank.h"

void Magnet_init(){
  pinMode(20, OUTPUT);
}

void Magnet_ON(){
  pinMode(20, OUTPUT);
  digitalWrite(20, HIGH);
  MainBank.Write_Magnet_Mode(true);
}

void Magnet_OFF(){
  pinMode(20, OUTPUT);
  digitalWrite(20, LOW);
  MainBank.Write_Magnet_Mode(false);
}