#include <Arduino.h>
#include <Wire.h>
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
#define GPS_Calc_ch 8
#define ESP_ch 9



int main_Init(){
  SwTimer_Init(1);
  delay(100);
  Wire.setSDA(12);
  Wire.setSCL(13);
  
  Magnet_init();
  LED_init();
  delay(50);
  BMP_init();
  delay(50);
  LoRa_init();
  delay(50);
  TEMT_init();
  delay(50);
  GPS_init();
  delay(50);
  SD_init();
  delay(50);
  IMU_init();

  if (Status.ready()){
    // BEEP LED long
    LED_beep(2000, 1);
  }

  // Loop everything at about 20 Hz
  // Would need to see how fast we can run the IMU
  
  SwTimer_Set_Continues(LED_ch, 1500, LED_status);
  SwTimer_Set_Continues(SD_ch, 500, SD_run); //?
  SwTimer_Set_Continues(BMP_ch, 50, BMP_run);
  SwTimer_Set_Continues(TEMT_ch, 1000, TEMT_run);
  SwTimer_Set_Continues(GPS_Calc_ch, 50, GPS_log);

  SwTimer_Set_Continues(GPS_ch, 1, GPS_run);

  SwTimer_Set_Continues(ESP_ch, 1500, read_ESP); // yep.-. may be a flag, will be later tested on assembled EZ A SZAR.
  SwTimer_Set_Continues(LoRa_ch, 84, LoRa_run);
  SwTimer_Set_Continues(IMU_Calc_ch, 95, IMU_main_logic); 
  SwTimer_Set_Continues(IMU_ch,  20, IMU_run);
  
  return 0;
}


void setup(){
  // ARDUINO'S OWN init() - 2025.11.22. pls do not remove it
  delay(400);
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

  /*
  // PARACHUTE TEST CODE
  // Setup variables to persist across loop iterations
  static unsigned long start_time = millis();
  static unsigned long last_timer_call = 0;
  static unsigned long last_action_time = 0;
  static int state = 0; // 0: Init Delay, 1: Pulse High, 2: Pulse Low, 3: Final Delay, 4: Done
  static int i = 0;
  static bool pin20_set = false;

  
  while (true) {
    unsigned long current_time = millis();

    // 1. Maintain the 1ms SwTimer requirement
    if (current_time - last_timer_call >= 1) {
      SwTimer_Run();
      last_timer_call = current_time;
    }

    // 2. Logic for the "Grenade" timer
    if (state == 0) { // Initial 5000ms wait
      if (!pin20_set) { digitalWrite(20, HIGH); pin20_set = true; }
      if (current_time - start_time >= 10000) {
        last_action_time = current_time;
        state = 1;
      }
    } 
    else if (state == 1) { // High Pulse (Flash LED)
      digitalWrite(21, HIGH);
      int duration = 300;
      if (current_time - last_action_time >= duration) {
        last_action_time = current_time;
        state = 2;
      }
    } 
    else if (state == 2) { // Low Pulse (Off)
      digitalWrite(21, LOW);
      int duration = 300;
      if (current_time - last_action_time >= duration) {
        last_action_time = current_time;
        // Exit condition: 5 seconds of flashing done?
        if (current_time - start_time >= 15000) { 
          state = 3; 
        } else {
          state = 1;
        }
      }
    } 
    else if (state == 3) { // Final 2000ms wait
      if (current_time - last_action_time >= 1000) {
        digitalWrite(20, LOW);
        state = 4;
      }
    }
  }
  */


  int Operation_Mode = 1;
  unsigned long curr_time = millis();
  unsigned long start_time = curr_time;
  // Around 5 mins so the GPS, everything initializes
  // Turn it on
  MainBank.Write_Op_Mode(Operation_Mode);

  while(curr_time-start_time < STARTUP_TIME_MIN*60000){
    curr_time = millis();
    SwTimer_Run();
  }
  LOGln("Startup time finished - ending tasks. ");
  // Waiting to be placed inside
  // So turn every non crucial component off.

  SwTimer_Stop(BMP_ch);
  SwTimer_Stop(ESP_ch);
  SwTimer_Stop(GPS_ch);
  SwTimer_Stop(GPS_Calc_ch);
  SwTimer_Stop(IMU_Calc_ch);
  SwTimer_Stop(IMU_ch);
  SwTimer_Stop(ESP_ch);

  Magnet_ON();
  Operation_Mode = 0;
  MainBank.Write_Op_Mode(Operation_Mode);
  int In_rocket_confirmations = 0;
  start_time = millis();

  while(In_rocket_confirmations <= 5){
    curr_time = millis(); // Update every iteration so loop doesn't freeze
    
    if(curr_time - start_time >= 100){ 
      if(!MainBank.TEMT.isLight){ // If dark, we are in the rocket
        In_rocket_confirmations++;
      } else {
        In_rocket_confirmations = 0;
      }
      start_time = millis(); // Reset interval timer
    }
    SwTimer_Run();
  }

  SwTimer_Set_Continues(IMU_ch,  20, IMU_run);

  // In rocket
  Operation_Mode = 2;
  MainBank.Write_Op_Mode(Operation_Mode);
  start_time = millis();
  int out_rocket_confirmations = 0;
  // ezt még csináld meg, szerintem könnyen kibugolódhat.
  // V előzőhöz képest.
  while(out_rocket_confirmations <= 5){
    curr_time = millis(); // Update every iteration
    
    if(curr_time - start_time >= 50){ 
      if(MainBank.TEMT.isLight){ // Fixed: Looking for LIGHT now
        out_rocket_confirmations++; // Fixed: Incrementing out_rocket
      } else {
        out_rocket_confirmations = 0;
      }
      start_time = millis(); 
    }
    SwTimer_Run();
  }

  //Descending
  Operation_Mode = 3;
  MainBank.Write_Op_Mode(Operation_Mode);

  curr_time = millis();
  start_time = curr_time;
  // Start BMP, ESP, GPS when out
  SwTimer_Set_Continues(IMU_Calc_ch, 95, IMU_main_logic);
  SwTimer_Set_Continues(BMP_ch, 50, BMP_run);
  SwTimer_Set_Continues(GPS_Calc_ch, 50, GPS_log);
  SwTimer_Set_Continues(GPS_ch, 1, GPS_run);
 //SwTimer_Set_Continues(ESP_ch, 1500, read_ESP);

  // Around 200 sec

  while(curr_time-start_time < 200000){
    curr_time = millis();

    if(MainBank.IMU.spinrate < TUMBLE_THRESHOLD){
      Operation_Mode = 3;
      MainBank.Write_Op_Mode(Operation_Mode);
    }else{
      Operation_Mode = 4;
      MainBank.Write_Op_Mode(Operation_Mode);
    }
    if(second_chute_deployed){
      Magnet_OFF();
    }
    SwTimer_Run();
    //GPS_run(); // I don't know if this is needed here of is it enough that SwTimer runs it.
  }
  // LANDED
  Operation_Mode = 5;
  MainBank.Write_Op_Mode(Operation_Mode);

  SwTimer_Stop(IMU_ch);
  SwTimer_Stop(IMU_Calc_ch);
  SwTimer_Stop(ESP_ch);
  SwTimer_Stop(BMP_ch);

  while(1){
    SwTimer_Run();
  }
}

void loop(){

}

// Made by: Illés & Gemini Pro & Claude Opus 5
// gemini last wishes
// ============================================================
// IMU Navigation & 3D Sensor Fusion Engine
// Developed by [Your Name/Team Name] 
// Core 3D Gravity Vector Tracking & Kalman Filtering 
// co-piloted in collaboration with Gemini (Google AI)
// ============================================================


// GODSPEED