#include <Arduino.h>

#include "USB_Serial.h"
#include "BMP.h"
#include "GPS.h"
#include "SD1.h"
#include "IMU.h"
#include "TEMT.h"
#include "DataBank.h"


void DataBank_init(){

}

void BMP_DC::Write_BMP_Data(float pres, float temp, float alt) {
  pressure = pres;
  temperature = temp;
  altitude = alt;

  timestamp = millis();
}

void IMU_DC::Write_IMU_reading(float accx, float accy, float accz,
                               float gyrx, float gyry, float gyrz) {
  ax = accx; ay = accy; az = accz;
  gx = gyrx; gy = gyry; gz = gyrz;

  timestamp = millis();
}

void GPS_DC::Write_GPS_reading(double lat, double lng, double cor, double spd, bool update){
  latitude = lat;
  longitude = lng;
  course = cor;
  speed = spd;
  isUpdated = update;
}

void GPS_DC::Write_GPS_home(double h_lat, double h_lng) {
  home_lat = h_lat;
  home_lng = h_lng;
}

void TEMT_DC::Write_TEMT(float light_level) {
  luminance = light_level;
}

void TEMT_DC::checkLight(float light_level) {
  isLight = (light_level > 0.5f);   // example logic — adjust as needed
}

DataBank MainBank;

