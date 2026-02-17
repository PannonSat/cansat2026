#pragma once


class BMP_DC{
  public:
    float pressure;
    float temperature;
    float altitude;

    void Write_BMP_Data(float pres, float temp, float alt);
};

class IMU_DC{
  public:
    float ax, ay, az;
    float gx, gy, gz;

    float tilt_x, tilt_y;
    float vel_x, vel_y;
    float V_vertical;
    float spinrate;

    void Write_IMU_reading(float accx, float accy, float accz, float gyrx, float gyry, float gyrz);
    void Write_IMU_calculations(float t_x, float t_y, float v_x, float v_y, float v_vert, float spin);

};

class GPS_DC{
  public:
    double latitude;
    double longitude;

    double home_lat;
    double home_lng;
    
    // In UTC (!!)
    struct time{
      bool synced;
      int year;   int day;  int seconds; int minutes;
      int month;  int hour; int centiseconds; 
    };

    time home_time;
    // The direction of heading in degrees
    double course;
    // Speed in km/h
    double speed;
    // Calculated in the IMU.cpp
    float predicted_current_distance;
    // if gps has been updated
    bool isUpdated;

    unsigned long timestamp;
    // Writes to the DataBank: latitude, longitude, course
    void Write_GPS_reading(double lat, double lng, double cor, double spd, bool update);
    void Log_Pred_dist(float pred_dist);
    // Writes the base home location to the DataBank
    void Write_GPS_home(double h_lat, double h_lng);

};

class TEMT_DC{
  public:
    // In LUX, calculated in TEMT.cpp from 3 sensors
    float luminance;

    bool isLight;
    // Writes TEMT sensor readings, and calculates isLight
    void Write_TEMT(float light_level);
};

class DataBank{
  public:
    BMP_DC BMP;
    IMU_DC IMU;
    GPS_DC GPS;
    TEMT_DC TEMT;

    short Operation_Mode;
    bool Magnet_on;

    void Write_Op_Mode(short OP_mode);
    void Write_Magnet_Mode(bool mode);
};


extern DataBank MainBank;