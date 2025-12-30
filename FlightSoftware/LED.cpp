#include <Arduino.h>

#include "USB_Serial.h"
#include "LED.h"

void LED_init(){
  pinMode(2, OUTPUT);
  LOG("LED succesfully initialized! ");
}

void LED_beep(float lenght){
  digitalWrite(2, HIGH);
  delay(lenght);
  digitalWrite(2, LOW);
}