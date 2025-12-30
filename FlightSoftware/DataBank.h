#pragma once


class BMP_DC{
  public:
    float pressure;
    float temperature;
    float altitude;

    long long int timestamp;

    void Write_BMP_Data(float pres, float temp, float alt);
};

class IMU_DC{
  public:
    float ax, ay, az;
    float gx, gy, gz;

    long long int timestamp;

    void Write_IMU_reading(float accx, float accy, float accz, float gyrx, float gyry, float gyrz);

};

class GPS_DC{
  public:
    double latitude;
    double longitude;

    double home_lat;
    double home_lng;
    // The direction of heading in degrees
    double course;
    // Speed in km/h
    double speed;
    // Calculated in the IMU.cpp
    float predicted_current_distance;
    // if gps has been updated
    bool isUpdated;
    // Writes to the DataBank: latitude, longitude, course
    void Write_GPS_reading(double lat, double lng, double cor, double spd, bool update);
    // Writes the base home location to the DataBank
    void Write_GPS_home(double h_lat, double h_lng);

};

class TEMT_DC{
  public:
    // In LUX, calculated in TEMT.cpp from 3 sensors
    float luminance;

    bool isLight;

    void Write_TEMT(float light_level);
    void checkLight(float light_level);
};

class DataBank{
  public:
    BMP_DC BMP;
    IMU_DC IMU;
    GPS_DC GPS;
    TEMT_DC TEMT;
};


extern DataBank MainBank;