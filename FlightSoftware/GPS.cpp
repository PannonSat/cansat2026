#include <Arduino.h>
#include <TinyGPS++.h>

#include "Settings.h"
#include "GPS.h"
#include "LED.h"
#include "DataBank.h"

TinyGPSPlus gps;

//Important bools for the GPS control
bool GPS_initialized = false;
bool home_set = false;
bool GPS_updated = false;

static const uint32_t GPSBaud = 9600;

double GPS_Data[4] = {0};
void GPS_sethome();

void GPS_init(void){
  Serial1.begin(GPSBaud);
  
  //SHORT LED BEEP    
  LED_beep(100);

  LOG("GPS initialized!");

  Status.gps = true;
}

void GPS_run(){
  // Read ONE BYTE and feed it directly to TinyGPS++
  int status = Serial1.available();

  if (status > 0){
    gps.encode(Serial1.read());
  }

  // If tested via USB Serial very handy
  if (gps.satellites.isUpdated()) {
    // Now you can print your data safely
    LOG("Sats: ");
    LOG(gps.satellites.value());
  }
  // Runs only once
  if (gps.location.isValid() && !home_set)
    GPS_sethome();
}

void GPS_sethome(){
  // Safety Gate: Only set Home if the GPS signal is strong (HDOP < 2.0 is good)
  // and we haven't set it yet.
  if (gps.hdop.hdop() < 2.0) {
    double home_lat = gps.location.lat();
    double home_lng = gps.location.lng();
     
    // Visual confirmation in your telemetry
    home_set = true;
    LOGln("Locked home location.");
    LOG("Lat: "); LOGln_multiple(home_lat, 6);
    LOG("Lng: "); LOGln_multiple(home_lng, 6);

    MainBank.GPS.Write_GPS_home(home_lat, home_lng);
  }
}

double* get_GPS_Data(){
  double course, speed, lat, lng;
  if (gps.location.isUpdated()){
  
    GPS_updated = true;
    lat = gps.location.lat();
    lng = gps.location.lng();

     // For the IMU logic to run
    if(gps.speed.isUpdated())
      speed = gps.speed.kmph();
      MainBank.GPS.Write_GPS_reading(lat, lng, course, speed, GPS_updated);

    if(gps.course.isUpdated())
      course = gps.course.deg();
      MainBank.GPS.Write_GPS_reading(lat, lng, course, speed, GPS_updated);
  }
  else{
    // return the old data if not updated
    GPS_updated = false;
    MainBank.GPS.Write_GPS_reading(lat, lng, course, speed, GPS_updated);
  }

  return GPS_Data;
}