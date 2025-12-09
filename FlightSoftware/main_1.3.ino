#include <Arduino.h>
#include <USB/PluggableUSBSerial.h>
#include "BMP.h"
#include "SwTimer.h"
#include "GPS.h"
#include "SD1.h"
#include "IMU.h"
#include "DataBank.h"
#include "TEMT.h"


#define BMP_ch 0
#define IMU_ch 1
#define GPS_ch 2
#define Serial_ch 3
#define DataBank_ch 4

// Timer Channels

int main_Init(){
  SwTimer_Init(1);
  pinMode(2, OUTPUT); // for the LED

  BMP_init();
  delay(250);
  GPS_init();
  delay(250);
  SD_init();
  delay(250);
  IMU_init();
  delay(250);
  TEMT_init();
  delay(250);

  if (SD_initialized && BMP_initialized && IMU_initialized && TEMT_initialized && GPS_initialized){
    // BEEP LED long
    digitalWrite(2, HIGH);
    delay(2000);
    digitalWrite(2, LOW);
  }

  DataBank_init();
  
  //Initializing all of the modules

  SwTimer_Set_Continues(GPS_ch, 5, GPS_run);
  SwTimer_Set_Continues(IMU_ch, 100, IMU_run);
  SwTimer_Set_Continues(3, 600, SD_Log_All);
  SwTimer_Set_Continues(BMP_ch, 100, BMP_run);
  SwTimer_Set_Continues(DataBank_ch, 200, DataBank_run);

  return 0;
}


int main(){
  init(); // ARDUINO'S OWN init() - 2025.11.22. pls do not remove it
  PluggableUSBD().begin();
  _SerialUSB.begin(115200);
  main_Init();

  delay(800);
  Serial.begin(115200);
  delay(1000); // for good measure (just in case) some delay

  while(1){
    SwTimer_Run();
  }
}