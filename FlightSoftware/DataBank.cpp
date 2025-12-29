#include <Arduino.h>
#include "BMP.h"
#include "GPS.h"
#include "SD1.h"
#include "IMU.h"
#include "TEMT.h"
#include "DataBank.h"

TelemetryData databank;


void DataBank_init(){
  databank = {}; // clearing it
  Serial.println("DataBank succesfully initialized!");
}


void DataBank_run(){
  // getting ALLL the data
  
  double* GPS_Data = get_GPS_Data();
  double* Home_Loc = get_Home_Location();
  float* BMP_Data = get_BMP_Data();

  databank.AccData = get_Acceleration();
  databank.GyroData = get_Gyroscope();
  
  databank.GPS_updated = GPS_updated;
  databank.latitude = GPS_Data[0];
  databank.longitude = GPS_Data[1];
  databank.speed_kmh = GPS_Data[2];
  databank.course_deg = GPS_Data[3];
  databank.home_lat = Home_Loc[0];
  databank.home_lng = Home_Loc[1];

  databank.pressure = BMP_Data[0];
  databank.temperature = BMP_Data[1];
  databank.altitude = BMP_Data[2];

  databank.light_level = get_TEMT_Data();
}

TelemetryData snapshotDataBank(){
  return databank;
}