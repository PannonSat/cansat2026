#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "SD1.h"
#include "DataBank.h"

const int CS_PIN = 5;
bool SD_initialized = false;
unsigned long start_time = 0;


String getCurrentTimeString();


void SD_init(){
  if (!SD.begin(CS_PIN)) {
    Serial.println("Card failed or not present");
  }else{
    //Short LED beep

    digitalWrite(2, HIGH);
    delay(200);
    digitalWrite(2, LOW);

    SD_initialized = true;
    Serial.println("SD succesfully initialized!");
    start_time = millis();
  }
}

void SD_run(){
  // IMU, GPS, BMP, TEMT
  // CSV format

  float* IMU_Acc = databank.AccData;
  float* IMU_Gyro = databank.GyroData;

  String current_time = getCurrentTimeString();
  String timestamp = "[" + current_time + "]: ";


  File IMU_file = SD.open("IMU_Data.txt", FILE_WRITE);
  
  IMU_file.print(timestamp);
  for(int i = 0;i <3; i++) {IMU_file.print(IMU_Acc[i]);IMU_file.print(", ");}
  for(int i = 0;i <3; i++) {IMU_file.print(IMU_Gyro[i]);if (i != 2) IMU_file.print(", ");}
  IMU_file.println(" ");
  IMU_file.close();
  delay(50);

  // BMP

  File BMP_file = SD.open("BMP_Data.txt", FILE_WRITE);

  BMP_file.print(timestamp);
  BMP_file.print(databank.temperature);
  BMP_file.print(", ");
  BMP_file.print(databank.pressure);
  BMP_file.print(", ");
  BMP_file.print(databank.altitude);
  BMP_file.println(" ");
  BMP_file.close();

  // TEMT
  delay(50);

  File TEMT_file = SD.open("Light_D.txt", FILE_WRITE);
  
  TEMT_file.print(timestamp);
  TEMT_file.println(databank.light_level);
  TEMT_file.close();
  delay(50);
  // GPS

  File GPS_file = SD.open("GPS_Data.txt", FILE_WRITE);

  GPS_file.print(timestamp);
  GPS_file.print(databank.latitude, 5);
  GPS_file.print(", ");
  GPS_file.println(databank.longitude, 5);
  GPS_file.close();

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