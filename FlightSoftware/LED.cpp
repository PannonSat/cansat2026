#include <Arduino.h>

#include "Settings.h"
#include "LED.h"

void LED_init(){
  pinMode(2, OUTPUT);
  LOG("LED succesfully initialized! ");
}

void LED_beep(float length){
  digitalWrite(2, HIGH);
  delay(length);
  digitalWrite(2, LOW);
}