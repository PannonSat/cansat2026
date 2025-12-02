#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include "IMU.h"

static float AccData[3];
static float GyroData[3];
bool IMU_initialized = false;

//values - Acceleration, Gyroscope
void IMU_init(void){
  if (!IMU.begin()) {
    Serial.println("IMU Failed to initialize!");
  }else{
    //SHORT LED BEEP
    digitalWrite(2, HIGH);
    delay(200);
    digitalWrite(2, LOW);
    IMU_initialized = true;
  }
}

void IMU_run(){
  // Measuring the IMU

  if(IMU.accelerationAvailable()) {
    IMU.readAcceleration(AccData[0], AccData[1], AccData[2]);
  }

  if(IMU.gyroscopeAvailable()){
    IMU.readGyroscope(GyroData[0], GyroData[1], GyroData[2]);
  }

}
float* get_Acceleration(void){
  return AccData;
}

float* get_Gyroscope(void){
  return GyroData;
}