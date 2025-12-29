#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include "DataBank.h"
#include "IMU.h"

#define G_TO_MS2 9.80665
#define DEG_TO_RAD 0.0174533
#define METERS_PER_DEGREE 111320.0

static float AccData[3];
static float GyroData[3];

//Calculation vars
float offset_ax, offset_ay, gps_course, gps_speed;
float last_alt, v_x, v_y, pos_x, pos_y, v_vert;
float tilt_x, tilt_y;

float TUMBLE_THRESHOLD = 200;

bool IMU_initialized = false;

// Helper functions, Callibration

float get_MetersFromStartLat(double current_lat) {
    return (current_lat - databank.home_lat) * METERS_PER_DEGREE;
}

float get_MetersFromStartLon(double current_lon, double current_lat) {
    float cos_lat = cos(current_lat * DEG_TO_RAD);
    return (current_lon - databank.home_lng) * METERS_PER_DEGREE * cos_lat;
}

void Calibrate_IMU(void) {
  Serial.println("Calibrating IMU... Keep the Sat still.");
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
  Serial.println("Calibration Done.");
}

void Predict_Landing(float v_x, float v_y, float pos_x, float pos_y, float current_alt){
  float time_to_impact = 0;
  float predicted_distance = 0;
  float current_dist = sqrt(pow(pos_x, 2) + pow(pos_y, 2));

  if (v_vert < -0.5) { 
    time_to_impact = current_alt / abs(v_vert);
    float v_horiz_total = sqrt(pow(v_x, 2) + pow(v_y, 2));
    predicted_distance = current_dist + (v_horiz_total * time_to_impact);
    databank.predicted_current_distance = predicted_distance;
  }
}

void Correct_with_GPS(float timestep){
  gps_course = databank.course_deg;
  gps_speed = databank.speed_kmh;
  float speedMS = gps_speed * 0.277778;
  float courseRad = gps_course * DEG_TO_RAD;

  float gps_vx = speedMS * sin(courseRad); 
  float gps_vy = speedMS * cos(courseRad);

  v_x = (v_x * 0.90) + (gps_vx * 0.10);
  v_y = (v_y * 0.90) + (gps_vy * 0.10);

  if(databank.GPS_updated){
    pos_x = get_MetersFromStartLon(databank.longitude, databank.latitude);
    pos_y = get_MetersFromStartLat(databank.latitude);
  }else{
    pos_x += v_x * timestep;
    pos_y += v_y * timestep;
  }
}

// --- Main Functions ---

void IMU_init(void){
  if (!IMU.begin()) {
    Serial.println("IMU Failed to initialize!");
  }else{
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    IMU_initialized = true;
  }
  Calibrate_IMU();
  last_alt = databank.altitude;
}

void IMU_run(){
  if(IMU.accelerationAvailable()) {
    IMU.readAcceleration(AccData[0], AccData[1], AccData[2]);
  }
  if(IMU.gyroscopeAvailable()){
    IMU.readGyroscope(GyroData[0], GyroData[1], GyroData[2]);
  }
}

float* get_Acceleration(void){
  return AccData;
}

float* get_Gyroscope(void){
  return GyroData;
}

void IMU_main_logic(void){
  float timestep = 0.05; // 50ms looped on SwTimer (see main)
  float current_alt = databank.altitude;

  float acc_x_raw = databank.AccData[0];
  float acc_y_raw = databank.AccData[1];

  float gx = databank.GyroData[0];
  float gy = databank.GyroData[1];
  float gz = databank.GyroData[2];

  v_vert = (current_alt - last_alt) / timestep;
  last_alt = current_alt;

  tilt_x += gx * timestep;
  tilt_y += gy * timestep;

  float totalRotationRate = sqrt(gx*gx + gy*gy + gz*gz);
  
  if (totalRotationRate < TUMBLE_THRESHOLD) {
    float cleanAx = (acc_x_raw - offset_ax - sin(tilt_x * DEG_TO_RAD)) * G_TO_MS2;
    float cleanAy = (acc_y_raw - offset_ay - sin(tilt_y * DEG_TO_RAD)) * G_TO_MS2;

    v_x += cleanAx * timestep;
    v_y += cleanAy * timestep;
  }
  
  Correct_with_GPS(timestep);
  Predict_Landing(v_x, v_y, pos_x, pos_y, current_alt);
}