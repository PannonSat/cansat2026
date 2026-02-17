#pragma once

#include "SD1.h"
#include "SoftwareSerial.h"

#define PRINT 0

#if PRINT == 1 
  #define LOG(x) Serial.print(x)
  #define LOGln(x) Serial.println(x)
  #define LOGln_multiple(x, y) Serial.println(x, y)
#else  
  #define LOG(x) ;
  #define LOGln(x) ;
  #define LOGln_multiple(x, y) ;
#endif

#define LOG_SD(x) Log_Assert(x)
#define LOG_SDln(x) Log_Assert_ln(x)

#define SAFE_ASSERT(condition, message) \
    if (!(condition)) { \
      LOG_SD("ASSERT FAILED: "); LOG_SD(message); LOG_SD(" at "); LOG_SD(__FILE__); LOG_SD(":"); LOG_SDln(String(__LINE__)); \
    }

#define CET_OFFSET_HOURS 1
#define STARTUP_TIME_MIN 5
// In LUX
#define LIGHT_BORDER 200
// Spin rate in case it fails
#define TUMBLE_THRESHOLD 70

#define LoRa_RX 3
#define LoRa_TX 4



struct SystemStatus {
  bool sd   = false;
  // For logging stuff
  bool esp = false;
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
extern SoftwareSerial ESP;
