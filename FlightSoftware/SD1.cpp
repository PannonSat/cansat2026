#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#include "SD1.h"
#include "Settings.h"
#include "LED.h"
#include "DataBank.h"

const int CS_PIN = 5;
bool SD_initialized = false;
unsigned long start_time = 0;


String getCurrentTimeString();


void SD_init(){
  if (!SD.begin(CS_PIN)) {
    LOG("Card failed or not present");
  }else{
    //Short LED beep
    LED_beep(100);

    Status.sd = true;
    LOG("SD succesfully initialized!");
    start_time = millis();
  }
}

void Add_BMP_String(String& Data){
  String BMP_Data = " ";

  BMP_Data += String(MainBank.BMP.temperature);
  BMP_Data += ", ";
  BMP_Data += String(MainBank.BMP.pressure);
  BMP_Data += ", ";
  BMP_Data += String(MainBank.BMP.altitude);
  BMP_Data += ", ";

  Data += BMP_Data;
}

void Add_IMU_String(String& Data){
  String IMU_Data = " ";

  IMU_Data += String(MainBank.IMU.ax); IMU_Data += ", ";
  IMU_Data += String(MainBank.IMU.ay); IMU_Data += ", ";
  IMU_Data += String(MainBank.IMU.az); IMU_Data += ", ";

  IMU_Data += String(MainBank.IMU.gx); IMU_Data += ", ";
  IMU_Data += String(MainBank.IMU.gy); IMU_Data += ", ";
  IMU_Data += String(MainBank.IMU.gz); IMU_Data += ", ";

  Data += IMU_Data;
}

void Add_GPS_String(String& Data){
  String GPS_Data = " ";

  GPS_Data += String(MainBank.GPS.latitude);
  GPS_Data += ", ";
  GPS_Data += String(MainBank.GPS.longitude);
  GPS_Data += ", ";
  GPS_Data += String(MainBank.GPS.course);
  GPS_Data += ", ";
  GPS_Data += String(MainBank.GPS.speed);
  GPS_Data += ", ";

  Data += GPS_Data;
}

void Add_TEMT_String(String& Data){
  String TEMT_Data = " ";

  TEMT_Data += String(MainBank.TEMT.luminance);
  TEMT_Data += ", ";
  TEMT_Data += String(MainBank.TEMT.isLight);

  Data += TEMT_Data;
}

void SD_run(){
  // IMU, GPS, BMP, TEMT
  // CSV format
  
  String current_time = getCurrentTimeString();
  String timestamp = "[" + current_time + "]: ";
  String date = "12.30 "; // CHANGE MANUALLY BEFORE START

  String Data = " ";
  
  Data += timestamp;

  Add_BMP_String(Data);
  Add_IMU_String(Data);
  Add_GPS_String(Data);
  Add_TEMT_String(Data);

  File Log = SD.open(date + "Log.txt", FILE_WRITE);

  Log.println(Data);

  Log.close();
}

String getCurrentTimeString(){
  unsigned long ms = millis() - start_time;
  unsigned long totalSeconds = ms / 1000UL;
  unsigned long minutes = totalSeconds / 60UL;
  unsigned long seconds = totalSeconds % 60UL;
  unsigned long millisPart = ms % 1000UL;

  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%02lu:%02lu:%03lu",
           minutes, seconds, millisPart);

  return String(buffer);
  // NOTE: OVERFLOWS AFTER 49 days 
}