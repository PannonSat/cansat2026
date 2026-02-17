#include <Arduino.h>

#include "Magnet.h"
#include "LED.h"
#include "BMP.h"
#include "SwTimer.h"
#include "GPS.h"
#include "SD1.h"
#include "IMU.h"
#include "DataBank.h"
#include "TEMT.h"
#include "LoRa.h"
#include "Settings.h"
#include <SerialPIO.h>


// Timer Channels

#define BMP_ch 0
#define IMU_ch 1
#define GPS_ch 2
#define SD_ch 3
#define TEMT_ch 4
#define IMU_Calc_ch 5
#define LoRa_ch 6
#define LED_ch 7
#define ESP_ch 1 


int main_Init(){
  SwTimer_Init(1);
  delay(100);

  Magnet_init();
  LED_init();

  LoRa_init();
  delay(50);
  IMU_init();
  delay(50);
  TEMT_init();
  delay(50);
  GPS_init();
  delay(50);
  SD_init();
  BMP_init();


  if (Status.ready()){
    // BEEP LED long
    // LED_beep(2000, 1);
    // LED_beep(2000, 2);
  }
  
  // Loop everything at about 20 Hz
  SwTimer_Set_Continues(IMU_ch,  5, IMU_run);
  SwTimer_Set_Continues(SD_ch, 500, SD_run);
  SwTimer_Set_Continues(BMP_ch, 50, BMP_run);
  SwTimer_Set_Continues(GPS_ch, 5, GPS_run); 
  SwTimer_Set_Continues(TEMT_ch, 200, TEMT_run);
  SwTimer_Set_Continues(ESP_ch, 1500, read_ESP);
  SwTimer_Set_Continues(LoRa_ch, 84, LoRa_run);
  SwTimer_Set_Continues(IMU_Calc_ch, 5, IMU_main_logic);
  SwTimer_Set_Continues(LED_ch, 1000, LED_status);

  return 0;
}


void setup(){
  // ARDUINO'S OWN init() - 2025.11.22. pls do not remove it
  

  
  delay(300);

  if(PRINT)
    Serial.begin(115200);

  delay(100);

  main_Init(); 
  delay(300);
  // for good measure (just in case) some delay

  /*
  States
  **********************

  Mindig, idő a GPS, minden init, nincs még run()
  0: Idle
  Akármikor meghívható
  1: Init
  Egy adott idő után, run()
  2: In Rocket
  Ha van fény
  3: Flying
  Ha nem mozgunk
  4: Flying Tumbling
  Ha forgunk (nagyon)
  5: Landed
  Akármikor meghívható
  */

  while(1){
    SwTimer_Run();
  }

  int Operation_Mode = 0;
  float curr_time = millis();
  float start_time = curr_time;
  // Around 5 mins so the GPS, everything initializes
  // Turn it on
  Magnet_ON();

  while(curr_time-start_time < STARTUP_TIME_MIN*60000){
    curr_time = millis();
    Operation_Mode = 1;
    MainBank.Write_Op_Mode(Operation_Mode);

    SwTimer_Run();
  }
  // Waiting to be placed inside
  Operation_Mode = 0;
  MainBank.Write_Op_Mode(Operation_Mode);
  int In_rocket_confirmations = 0;

  while(In_rocket_confirmations <= 5){
    SwTimer_Run();
    
    if(MainBank.TEMT.isLight)
      In_rocket_confirmations++;
    else 
      In_rocket_confirmations = 0;

    delay(200);// Will this break it? Hopefully not..
  }

  // In rocket
  Operation_Mode = 2;
  MainBank.Write_Op_Mode(Operation_Mode);

  while(!MainBank.TEMT.isLight){
    SwTimer_Run();
  }
  

  //Descending
  curr_time = millis();
  start_time = curr_time;
  // Start IMU calculations when out
  SwTimer_Set_Continues(IMU_Calc_ch, 25, IMU_main_logic);

  // Around 200 sec

  while(curr_time-start_time < 200000){
    curr_time = millis();

    if(MainBank.IMU.spinrate < TUMBLE_THRESHOLD){
      Operation_Mode = 4;
      MainBank.Write_Op_Mode(Operation_Mode);
    }else{
      Operation_Mode = 3;
      MainBank.Write_Op_Mode(Operation_Mode);
    }
    if(second_chute_deployed){
      Magnet_OFF();
    }
    SwTimer_Run();
  }
  // LANDED
  Operation_Mode = 5;
  MainBank.Write_Op_Mode(Operation_Mode);

  SwTimer_Stop(IMU_ch);
  SwTimer_Stop(IMU_Calc_ch);
  SwTimer_Stop(BMP_ch);

  while(1){
    SwTimer_Run();
  }
}

void loop(){

}