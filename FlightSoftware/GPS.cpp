#include <Arduino.h>
#include "GPS.h"
#include <TinyGPS++.h>

TinyGPSPlus gps;
bool GPS_initialized = false;
// we use the library for formatting, very useful
static const uint32_t GPSBaud = 9600;
double GPS_Data[2] = {0};

void GPS_init(void){
  Serial1.begin(GPSBaud);
  //SHORT LED BEEP
  digitalWrite(2, HIGH);
  delay(200);
  digitalWrite(2, LOW);

  GPS_initialized = true;
}

void GPS_run(){
  while (Serial1.available() > 0){
    gps.encode(Serial1.read());
    // building the GPS data
  }
}

double* get_GPS_Data(){
  if (gps.location.isUpdated()){
    GPS_Data[0] = gps.location.lat();
    GPS_Data[1] = gps.location.lng();
  }
  else{
    GPS_Data[0] = gps.location.lat();
    GPS_Data[1] = gps.location.lng();
    // return the old data if not updated
  }

  return GPS_Data;
}