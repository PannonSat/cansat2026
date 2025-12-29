#include <Arduino.h>
#include "GPS.h"
#include <TinyGPS++.h>

TinyGPSPlus gps;

//Important bools for the GPS controll
bool GPS_initialized = false;
bool home_set = false;
bool GPS_updated = false;

double home_lat, home_lng;
static const uint32_t GPSBaud = 9600;

double GPS_Data[4] = {0};
void GPS_sethome();

void GPS_init(void){
  Serial1.begin(GPSBaud);
  
  //SHORT LED BEEP    
  digitalWrite(2, HIGH);
  delay(100);
  digitalWrite(2, LOW);

  Serial.println("GPS initialized!");
  GPS_initialized = true;

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
    Serial.print("Sats: ");
    Serial.println(gps.satellites.value());
  }
  // Runs only once
  if (gps.location.isValid() && !home_set)
    GPS_sethome();
}

void GPS_sethome(){
  // Safety Gate: Only set Home if the GPS signal is strong (HDOP < 2.0 is good)
  // and we haven't set it yet.
  if (gps.hdop.hdop() < 2.0) {
    home_lat = gps.location.lat();
    home_lng = gps.location.lng();
     
    // Visual confirmation in your telemetry
    home_set = true;
    Serial.println("Locked home location.");
    Serial.print("Lat: "); Serial.println(home_lat, 6);
    Serial.print("Lng: "); Serial.println(home_lng, 6);
  }
}

double* get_Home_Location(){
  double Home_Loc[2] = {0};
  Home_Loc[0] = home_lat;
  Home_Loc[1] = home_lng;

  return Home_Loc;
}

double* get_GPS_Data(){
  if (gps.location.isUpdated()){
    GPS_updated = true;
    GPS_Data[0] = gps.location.lat();
    GPS_Data[1] = gps.location.lng();
    delay(30); // For the IMU logic to run
    if(gps.speed.isUpdated())
      GPS_Data[2] = gps.speed.kmph();

    if(gps.course.isUpdated())
      GPS_Data[3] = gps.course.deg();
  }
  else{
    // return the old data if not updated
  }

  return GPS_Data;
}