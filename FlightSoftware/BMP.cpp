#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>

#include "Settings.h"
#include "LED.h"
#include "BMP.h"
#include "DataBank.h"

//GLOBALS
Adafruit_BMP280 bmp;

static float calc_Altitude(float pressure);


void BMP_init(){
  bool status = bmp.begin();
  if (!status) {
    LOG("BMP failed to initalize! Check wiring!"); // BMP not initialzed, error :/
  }else{
    //SHORT LED BEEP
    LED_beep(100);
    LOG("BMP280 init succesful!");
    Status.bmp = true;
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X4,   /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,  /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,    /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */


}

void BMP_run(){
  float pressure = bmp.readPressure();

  MainBank.BMP.Write_BMP_Data(pressure, bmp.readTemperature(), calc_Altitude(pressure));
}

static float calc_Altitude(float pressure){
  float altitude;

  pressure /= 100; // important if you don't want to measure pressure twice
  altitude = 44330 * (1.0 - pow(pressure / 1013.25, 0.1903));

  return altitude;
}

