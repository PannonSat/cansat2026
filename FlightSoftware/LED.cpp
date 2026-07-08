#include <Arduino.h>

#include "Settings.h"
#include "LED.h"

void LED_init(){
  pinMode(21, OUTPUT);
  pinMode(19, OUTPUT);
  LOG("LED succesfully initialized! ");
}
// Removed the pinModes might be needed back.

void LED_beep(float length, int LED){
  if(LED == 1){
    digitalWrite(21, HIGH);
    delay(length);
    digitalWrite(21, LOW);
  }else{
    digitalWrite(19, HIGH);
    delay(length);
    digitalWrite(13, LOW);
  }
}

void LED_status(){
  digitalWrite(19, HIGH);
  delay(500);
  digitalWrite(19, LOW);
}