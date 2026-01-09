#pragma once

#define PRINT 1

#if PRINT == 1 
  #define LOG(x) Serial.print(x)
  #define LOGln(x) Serial.println(x)
  #define LOGln_multiple(x, y) Serial.println(x, y)
#else  
  #define LOG(x) ;
  #define LOGln(x) ;
  #define LOGln_multiple(x, y) ;
#endif

#define CET_OFFSET_HOURS 1
#define STARTUP_TIME_MIN 5
// In LUX
#define LIGHT_BORDER 200

#define LoRa_RX 1
#define LoRa_TX 2


struct SystemStatus {
  bool sd   = false;
  bool bmp  = false;
  bool imu  = false;
  bool gps  = false;
  bool temt = false;
  // Checks if everything has initialized properly
  bool ready() const {
    return sd && bmp && imu && gps && temt;
  }
};

extern SystemStatus Status;
