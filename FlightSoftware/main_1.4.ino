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
#define IMU_Calc_ch 6
#define GPS_ch 2
#define SD_ch 3
#define DataBank_ch 4
#define TEMT_ch 5

// Timer Channels

int main_Init(){
  SwTimer_Init(1);
  delay(100);
  pinMode(2, OUTPUT);


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
  DataBank_init();
  delay(50);

  if (SD_initialized && BMP_initialized && IMU_initialized && TEMT_initialized && GPS_initialized){
    // BEEP LED long
    digitalWrite(2, HIGH);
    delay(2000);
    digitalWrite(2, LOW);
  }

  
  //Initializing all of the modules

  SwTimer_Set_Continues(IMU_ch, 50, IMU_run);
  SwTimer_Set_Continues(IMU_Calc_ch, 50, IMU_main_logic);
  SwTimer_Set_Continues(SD_ch, 200, SD_run);
  SwTimer_Set_Continues(BMP_ch, 100, BMP_run);
  SwTimer_Set_Continues(DataBank_ch, 100, DataBank_run);
  SwTimer_Set_Continues(GPS_ch, 5, GPS_run);
  SwTimer_Set_Continues(TEMT_ch, 20, TEMT_run);

  return 0;
}


int main(){
  init(); // ARDUINO'S OWN init() - 2025.11.22. pls do not remove it
  PluggableUSBD().begin();
  _SerialUSB.begin(115200);

  
  delay(300);
  Serial.begin(115200);
  delay(100);

  main_Init(); 
  // for good measure (just in case) some delay
  delay(2500);

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

  float curr_time = millis();
  float start_time = curr_time;

  // Around 5 mins so the GPS, everything initializes

  while(curr_time-start_time< 300000){
    curr_time = millis();
    SwTimer_Run();
  }

  /*
  Something like this; got no working stuff yet.

  while(inRocket){
    //ami még kell init
  }
  while(Light){
    // ha fény van kezdje el
  }
  while(mozog){
    SwTimer_Run();
  */
  // Landing protocol
  
  while(1){
    SwTimer_Run();
  }
}