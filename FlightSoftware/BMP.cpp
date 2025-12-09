#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>

#include "BMP.h"


Adafruit_BMP280 bmp;
bool BMP_initialized = false;
const unsigned int BMP_size = 3;
float BMP_Data[BMP_size] = {0};
//GLOBALS

void BMP_run();
static float calc_Altitude(float pressure);


void BMP_init(){
  bool status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  if (!status) {
    Serial.print("csicska bmp nem jó"); // BMP not initialzed, error :/
  }else{
    //SHORT LED BEEP
    digitalWrite(2, HIGH);
    delay(200);
    digitalWrite(2, LOW);
    BMP_initialized = true;
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X4,   /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,  /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,    /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */


}

void BMP_run(){
  float pressure = bmp.readPressure();
  BMP_Data[0] = pressure;
  BMP_Data[1] = bmp.readTemperature();
  BMP_Data[2] = calc_Altitude(pressure);
  // storing the data in a static array
}

float* get_BMP_Data(){
  return BMP_Data;
}

int len_BMP_Data(){
  return BMP_size;
}

static float calc_Altitude(float pressure){
  float altitude;

  pressure /= 100; // important if you don't want to measure pressure twice
  altitude = 44330 * (1.0 - pow(pressure / 1013.25, 0.1903));

  return altitude;
}

