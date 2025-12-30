#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>

#include "Settings.h"
#include "LED.h"
#include "DataBank.h"
#include "IMU.h"

#define G_IN_MS2 9.80665
#define KMH_TO_MS 0.277778
#define DEG_TO_RAD 0.0174533
#define METERS_PER_DEGREE 111320.0

// Calculation globals
float offset_ax, offset_ay;
float last_alt, v_x, v_y, pos_x, pos_y, v_vert;
float tilt_x, tilt_y;

// Max spinrate when IMU is the primary for the calculation
float TUMBLE_THRESHOLD = 200;

bool IMU_initialized = false;

// Helper functions, Callibration

float get_MetersFromStartLat(double current_lat) {
    return (current_lat - MainBank.GPS.home_lat) * METERS_PER_DEGREE;
}

float get_MetersFromStartLon(double current_lon, double current_lat) {
    float cos_lat = cos(current_lat * DEG_TO_RAD);
    return (current_lon - MainBank.GPS.home_lng) * METERS_PER_DEGREE * cos_lat;
}

void Calibrate_IMU(void) {
  LOG("Calibrating IMU... Keep the Sat still.");
  float sum_x = 0, sum_y = 0;
  int samples = 200;
  for (int i = 0; i < samples; i++) {
    float tx, ty, tz;
    if(IMU.accelerationAvailable()){
        IMU.readAcceleration(tx, ty, tz);
        sum_x += tx;
        sum_y += ty;
    }
    delay(5);
  }
  offset_ax = sum_x / samples;
  offset_ay = sum_y / samples;
  LOG("Calibration Done.");
}

void Predict_Landing(float current_alt){
  float time_to_impact = 0;
  float predicted_distance = 0;
  float current_dist = sqrt(pow(pos_x, 2) + pow(pos_y, 2));

  if (v_vert < -0.5) { 
    time_to_impact = current_alt / abs(v_vert);
    float v_horiz_total = sqrt(pow(v_x, 2) + pow(v_y, 2));
    predicted_distance = current_dist + (v_horiz_total * time_to_impact);
    MainBank.GPS.predicted_current_distance = predicted_distance;
  }
}

void Correct_with_GPS(float timestep){
  double gps_course = MainBank.GPS.course;
  double gps_speed = MainBank.GPS.speed;
  float speedMS = gps_speed * KMH_TO_MS;
  float courseRad = gps_course * DEG_TO_RAD;

  float gps_vx = speedMS * sin(courseRad); 
  float gps_vy = speedMS * cos(courseRad);

  v_x = (v_x * 0.90) + (gps_vx * 0.10);
  v_y = (v_y * 0.90) + (gps_vy * 0.10);

  if(MainBank.GPS.isUpdated){
    pos_x = get_MetersFromStartLon(MainBank.GPS.longitude, MainBank.GPS.latitude);
    pos_y = get_MetersFromStartLat(MainBank.GPS.latitude);
  }else{
    pos_x += v_x * timestep;
    pos_y += v_y * timestep;
  }
}

// --- Main Functions ---

void IMU_init(void){
  if (!IMU.begin()) {
    LOG("IMU Failed to initialize!");
  }else{
    LED_beep(100);
    IMU_initialized = true;
  }
  Calibrate_IMU();
  last_alt = MainBank.BMP.altitude;
}

void IMU_run(){
  if(IMU.accelerationAvailable()) {
    IMU.readAcceleration(MainBank.IMU.ax, MainBank.IMU.ay, MainBank.IMU.az);
  }
  if(IMU.gyroscopeAvailable()){
    IMU.readGyroscope(MainBank.IMU.gx, MainBank.IMU.gy, MainBank.IMU.gz);
  }
}

void IMU_main_logic(void){
  float timestep = 0.05; // 50ms looped on SwTimer (see main)
  float current_alt = MainBank.BMP.altitude;

  float acc_x_raw = MainBank.IMU.ax;
  float acc_y_raw = MainBank.IMU.ay;

  float gx = MainBank.IMU.gx;
  float gy = MainBank.IMU.gy;
  float gz = MainBank.IMU.gz;

  v_vert = (current_alt - last_alt) / timestep;
  last_alt = current_alt;

  tilt_x += gx * timestep;
  tilt_y += gy * timestep;

  float totalRotationRate = sqrt(gx*gx + gy*gy + gz*gz);
  
  if (totalRotationRate < TUMBLE_THRESHOLD) {
    float cleanAx = (acc_x_raw - offset_ax - sin(tilt_x * DEG_TO_RAD)) * G_IN_MS2;
    float cleanAy = (acc_y_raw - offset_ay - sin(tilt_y * DEG_TO_RAD)) * G_IN_MS2;

    v_x += cleanAx * timestep;
    v_y += cleanAy * timestep;
  }
  
  Correct_with_GPS(timestep);
  Predict_Landing(current_alt);
}