#include <Arduino.h>
#include <USB/PluggableUSBSerial.h>

#include "LED.h"
#include "BMP.h"
#include "SwTimer.h"
#include "GPS.h"
#include "SD1.h"
#include "IMU.h"
#include "DataBank.h"
#include "TEMT.h"
#include "Settings.h"


// Timer Channels

#define BMP_ch 0
#define IMU_ch 1
#define IMU_Calc_ch 6
#define GPS_ch 2
#define SD_ch 3
#define TEMT_ch 5


int main_Init(){
  SwTimer_Init(1);
  delay(100);

  //DataBank_init();
  LED_init();
  BMP_init();
  delay(50);
  SD_init();
  delay(50);
  IMU_init();
  delay(50);
  TEMT_init();
  delay(50);
  GPS_init();
  delay(50);

  if (Status.ready()){
    // BEEP LED long
    LED_beep(2000);
  }

  SwTimer_Set_Continues(IMU_ch,  50, IMU_run);
  SwTimer_Set_Continues(IMU_Calc_ch, 50, IMU_main_logic);
  SwTimer_Set_Continues(SD_ch, 200, SD_run);
  SwTimer_Set_Continues(BMP_ch, 100, BMP_run);
  SwTimer_Set_Continues(GPS_ch, 5, GPS_run);
  SwTimer_Set_Continues(TEMT_ch, 20, TEMT_run);

  return 0;
}


int main(){
  init(); // ARDUINO'S OWN init() - 2025.11.22. pls do not remove it
  PluggableUSBD().begin();
  _SerialUSB.begin(115200);

  
  delay(300);

  if(PRINT)
    Serial.begin(115200);

  delay(100);

  main_Init(); 
  // for good measure (just in case) some delay
  delay(1500); 

  /*
  States
  **********************

  Mindig, idő a GPS, minden init, nincs még run()
  1: Init
  Egy adott idő után, run()
  2: In Rocket
  Ha van fény
  3: Flying
  Ha nem mozgunk
  4: Landed
  Akármikor meghívható
  5: Idle

  */

  /*
  int Operation_Mode = 0;
  float curr_time = millis();
  float start_time = curr_time;

  // Around 5 mins so the GPS, everything initializes

  while(curr_time-start_time < STARTUP_TIME_MIN*60000){
    curr_time = millis();
    Operation_Mode = 1;
    SwTimer_Run();
  }
  // Waiting to be placed inside
  while(MainBank.TEMT.isLight){
    Operation_Mode = 0;
    SwTimer_Run();
    delay(10);
  }

  // In rocket
  while(MainBank.TEMT.isLight){
    Operation_Mode = 2;
    SwTimer_Run();
  }

  //Descending
  curr_time = millis();
  start_time = curr_time;

  // Around 5 mins so the GPS, everything initializes

  while(curr_time-start_time < STARTUP_TIME_MIN*60000){
    curr_time = millis();
    Operation_Mode = 3;
    SwTimer_Run();
  }
  */
  // LANDED

  while(1){
    SwTimer_Run();
  }
}