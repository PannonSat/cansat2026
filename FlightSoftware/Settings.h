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
