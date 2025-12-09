#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "SD1.h"
#include <RTClib.h>
#include "DataBank.h"

RTC_PCF8523 RTC; // Loggingra
const int CS_PIN = 5;
File IMU_output;
bool SD_initialized = false;

void Write_DateStamp(File file, DateTime now);

void SD_init(){
  if (!SD.begin(CS_PIN)) {
    Serial.println("Card failed or not present");
  }else{
    //Short LED beep
    digitalWrite(2, HIGH);
    delay(200);
    digitalWrite(2, LOW);
    SD_initialized = true;
  }
  RTC.begin();
  SD.begin();
  Serial.println("SD card initialized.");

}

void SD_run(){
  //
}

void SD_Log_All(){
  // IMU, GPS, BMP, TEMT

  DateTime now = RTC.now();

  float* IMU_Acc = databank.AccData;
  float* IMU_Gyro = databank.GyroData;


  File IMU_file = SD.open("IMU_Data.txt", FILE_WRITE);

  Write_DateStamp(IMU_file, now);
  for(int i = 0;i <3; i++) {IMU_file.print(IMU_Acc[i]);}
  IMU_file.println(", ");
  for(int i = 0;i <3; i++) {IMU_file.print(IMU_Gyro[i]);}
  IMU_file.println(" ");
  IMU_file.close();

  // BMP

  File BMP_file = SD.open("BMP_Data.txt", FILE_WRITE);
  

  if (!BMP_file) {
    Serial.print("FAILED TO OPEN!");
  }else{
    Serial.print("SUCCESFULLY OPENED");
  }
  Write_DateStamp(BMP_file, now);
  BMP_file.print(databank.temperature);
  BMP_file.print(", ");
  BMP_file.print(databank.pressure);
  BMP_file.print(", ");
  BMP_file.print(databank.altitude);
  BMP_file.println(" ");
  BMP_file.close();

  // TEMT

  File TEMT_file = SD.open("TEMT_Data.txt", FILE_WRITE);

  Write_DateStamp(TEMT_file, now);
  TEMT_file.print(databank.light_level);
  TEMT_file.println(" ");
  TEMT_file.close();

  // GPS

  File GPS_file = SD.open("GPS_Data.txt", FILE_WRITE);

  Write_DateStamp(GPS_file, now);
  GPS_file.print(databank.latitude);
  GPS_file.print(", ");
  GPS_file.print(databank.longitude);
  GPS_file.println(" ");
  GPS_file.close();

}



void Write_DateStamp(File file, DateTime now){
  //

  file.print("[ ");
  file.print(now.year());
  file.print("/");
  file.print(now.month());
  file.print("/");
  file.print(now.day());
  file.print("/");
  file.print(now.hour());
  file.print("/");
  file.print(now.minute());
  file.print("/");
  file.print(now.second());
  file.print(" ]: ");
}