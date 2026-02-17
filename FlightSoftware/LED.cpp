#include <Arduino.h>

#include "Settings.h"
#include "LED.h"

void LED_init(){
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  LOG("LED succesfully initialized! ");
}

void LED_beep(float length, int LED){
  if(LED == 1){
    digitalWrite(18, HIGH);
    delay(length);
    digitalWrite(18, LOW);
  }else{
    digitalWrite(19, HIGH);
    delay(length);
    digitalWrite(19, LOW);
  }
}

void LED_status(){
  digitalWrite(19, HIGH);
  delay(1000);
  digitalWrite(19, LOW);
}