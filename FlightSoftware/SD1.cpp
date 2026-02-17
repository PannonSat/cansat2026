#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "SD1.h"
#include "Settings.h"
#include "LED.h"
#include "DataBank.h"
#include "SoftwareSerial.h"

#define CET_OFFSET_HOURS 1

bool timeSynced = false;
bool Log_SD = true;
unsigned long baseMillis = 0;

int baseDay, baseMonth;
int baseHour, baseMin, baseSec;

// ****** Timestamp Functions ******

void syncTime(){

  if (timeSynced) return ;

  auto &t = MainBank.GPS.home_time;

  baseDay   = t.day;
  baseMonth = t.month;

  baseHour = t.hour + CET_OFFSET_HOURS;
  baseMin  = t.minutes;
  baseSec  = t.seconds;


  baseMillis = millis();
  timeSynced = true;
}

String getTimestamp()
{
  if (!timeSynced)
    return "NO_TIME";

  unsigned long elapsed = (millis() - baseMillis) / 1000;
  int ms = (millis() - baseMillis) % 1000;

  int sec = baseSec  + elapsed;
  int min = baseMin  + sec / 60;   sec %= 60;
  int hr  = baseHour + min / 60;   min %= 60;

  char buf[32];
  sprintf(buf,
          "%02d-%02d %02d:%02d:%02d.%03d",
          baseMonth, baseDay,
          hr, min, sec, ms);

  return String(buf);
}


// ****** Log formater functions ******

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

void Add_Calc_String(String& Data){
  String Calc = " ";

  Calc += String(MainBank.IMU.spinrate); Calc += ", ";
  Calc += String(MainBank.IMU.tilt_x); Calc += ", ";
  Calc += String(MainBank.IMU.tilt_y); Calc += ", ";
  Calc += String(MainBank.IMU.V_vertical); Calc += ", ";
  Calc += String(MainBank.IMU.vel_x); Calc += ", ";
  Calc += String(MainBank.IMU.vel_y); Calc += ", ";

  Data += Calc;
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

// ****** Main Functions ******

void SD_init(){
  if(ESP.available()){
    LED_beep(100, 1);

    Status.sd = true;
    LOG("SD succesfully initialized!");
  }else{
    LOG("SD FAILED, check Serial");
  }
}

void SD_run(){
  // IMU, GPS, BMP, TEMT
  // CSV format
  if (MainBank.GPS.home_time.synced && !timeSynced)
    syncTime();

  String current_time = getTimestamp();

  String timestamp = "[" + current_time + "]: ";

  String Data = " ";
  
  Data += timestamp;

  Add_BMP_String(Data);
  Add_IMU_String(Data);
  Add_Calc_String(Data);
  Add_GPS_String(Data);
  Add_TEMT_String(Data);

  if(Log_SD){
    Status.esp = true;
    ESP.println(Data);
    delay(30);
    Status.esp = false;
  }

}


// ASSERTATION LOGS

void Log_Assert(String Content){
  Log_SD = false;
  String Data = "[" + getTimestamp() + "]: " + Content;
  ESP.print(Data);
}
void Log_Assert_ln(String Content){
  String Data = "[" + getTimestamp() + "]: " + Content;
  Status.esp = true;
  ESP.println(Data);
  delay(30);
  Log_SD = true;
  Status.esp = false;
}