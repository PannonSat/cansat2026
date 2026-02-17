#include <Arduino.h>
#include <TinyGPS++.h>

#include "Settings.h"
#include "GPS.h"
#include "LED.h"
#include "LoRa.h"
#include "DataBank.h"

TinyGPSPlus gps;

//Important bools for the GPS control
bool GPS_initialized = false;
bool home_set = false;
bool GPS_updated = false;

SoftwareSerial GPS(D2, D3);
static const uint32_t GPSBaud = 9600; // or 9600 if this shi doesn't work

double GPS_Data[4] = {0};
void GPS_sethome();

void GPS_init(void){
  //SHORT LED BEEP    
  GPS.begin(GPSBaud);
  LED_beep(100, 1);

  LOG("GPS initialized!");
  MainBank.GPS.home_time.synced = false;
  Status.gps = true;
}

void GPS_run(){
  // Read ONE BYTE and feed it directly to TinyGPS++
  int status = GPS.available();
  double course, speed, lat, lng;

  if (status > 0){
    char c = GPS.read();
    gps.encode(c);
  }

  // If tested via USB Serial very handy
  if (gps.satellites.isUpdated()) {
    // Now you can print your data safely
    LOG("Sats: ");
    LOG(gps.satellites.value());
  }
  // Runs only once
  if (gps.location.isValid() && !home_set){
    LOGln("GPS COLD START COMPLETED IN: " + String(millis() / 60000));
    GPS_sethome();
  }

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
}

void GPS_sethome(){
  // Safety Gate: Only set Home if the GPS signal is strong (HDOP < 2.0 is good)
  // and we haven't set it yet.
  LOGln(gps.hdop.hdop());

  if (gps.hdop.hdop() > 2.0) {
    double home_lat = gps.location.lat();
    double home_lng = gps.location.lng();

    home_set = true;
     
    // Visual confirmation in your telemetry
    LOGln("Locked home location.");
    LOG("Lat: "); LOGln_multiple(home_lat, 6);
    LOG("Lng: "); LOGln_multiple(home_lng, 6);
    MainBank.GPS.Write_GPS_home(home_lat, home_lng);

    // Set initial time too
    // Uses a struct
    MainBank.GPS.home_time.synced = true;

    MainBank.GPS.home_time.seconds = gps.time.second();  MainBank.GPS.home_time.year = gps.date.year();
    MainBank.GPS.home_time.minutes = gps.time.minute();  MainBank.GPS.home_time.hour = gps.time.hour();
    MainBank.GPS.home_time.day = gps.date.day();         MainBank.GPS.home_time.month = gps.date.month();
  }
}