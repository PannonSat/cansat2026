#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>

#include "Settings.h"
#include "LED.h"
#include "DataBank.h"
#include "IMU.h"

// --- Constants ---
#define G_IN_MS2 9.80665
#define KMH_TO_MS 0.277778
#define DEG_TO_RAD 0.0174533
#define METERS_PER_DEGREE 111320.0

// Mission Profiles
const float V_SLOW_CHUTE = 6.0;      // m/s descent (Target for deployment)
const float DEPLOY_THRESHOLD = 30.0;  // Deploy if predicted landing is within 30m of pad
const float MIN_SAFE_ALT = 50.0;     // Hard floor to ensure inflation
const float ARMING_ALT = 150.0;      // Don't even check deployment until we are falling below this
const int CONFIRMATION_NEEDED = 3;   // Number of consecutive cycles to trigger

// --- Kalman Filter Structure ---
struct Kalman1D {
    float state = 0;       // The "Filtered" value (e.g., your best guess of current velocity)
    float uncertainty = 1.0; // The "Confidence" (How much do we trust the 'state' right now?)
    float Q = 0.02;        // Process Noise (How much error does our physics math add every step?)
    float R = 0.8;         // Measurement Noise (How "jittery" is the physical sensor?)

    void predict(float delta, float q_noise) {
        state += delta;             // Add the change 
        uncertainty += q_noise;     // Every time we use math to guess, our uncertainty grows
    }

    void update(float measurement, float r_noise) {
        // 1. Calculate the Kalman Gain (0.0 to 1.0)
        // High Gain (~1.0) = Trust the sensor more than the math
        // Low Gain (~0.0) = Trust the math more than the sensor
        float gain = uncertainty / (uncertainty + r_noise);

        // 2. Adjust the state based on the sensor
        // We take the difference between reality and our guess, multiply by gain, and add it
        state += gain * (measurement - state);

        // 3. Update uncertainty
        // Because we just got a "reality check" from a sensor, we are now more confident
        uncertainty *= (1.0 - gain);
    }
};

// State Vars
Kalman1D FilterVVert; 
Kalman1D FilterTiltX;

Kalman1D FilterTiltY;
// New Kalman objects for Horizontal Velocity to replace the 0.7/0.3 blend
Kalman1D FilterVX = {0, 1.0, 0.05, 0.1}; 
Kalman1D FilterVY = {0, 1.0, 0.05, 0.1};

float offset_ax, offset_ay;
float last_alt, v_x, v_y, pos_x, pos_y, v_vert;
float tilt_x, tilt_y;
float TUMBLE_THRESHOLD = 200;

int deployment_confirmations = 0;
bool second_chute_deployed = false;

// Helper Functions

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

// --- The Core Mission Logic ---

void runBreakpointDeployment(float current_alt) {
    if (second_chute_deployed || current_alt > ARMING_ALT) return;

    float time_to_impact = current_alt / V_SLOW_CHUTE;

    // pos_x/y are relative to HOME, home set in GPS.cpp
    float pred_land_x = pos_x + (v_x * time_to_impact);
    float pred_land_y = pos_y + (v_y * time_to_impact);
    float pred_dist_to_base = sqrt(sq(pred_land_x) + sq(pred_land_y));
    pred_dist_to_base = MainBank.GPS.predicted_current_distance;

    if (pred_dist_to_base <= DEPLOY_THRESHOLD || current_alt <= MIN_SAFE_ALT)
        // When it reaches 3, it means the signal is right
        deployment_confirmations++;
    else
        // if they are not followed by eachother, it means it was just a spike
        deployment_confirmations = 0;

    if (deployment_confirmations >= CONFIRMATION_NEEDED) {
        LOG("DEPLOYING SECOND CHUTE");
        second_chute_deployed = true;
    }
}

void updateTilt(float gx, float gy, float ax, float ay, float az, float rotation, float dt) {
    FilterTiltX.predict(gx * dt, 0.005);
    FilterTiltY.predict(gy * dt, 0.005);

    if (rotation < TUMBLE_THRESHOLD) {
        float accel_tilt_x = atan2(ay, az) * (180.0 / PI);
        float accel_tilt_y = atan2(-ax, az) * (180.0 / PI);
        FilterTiltX.update(accel_tilt_x, 1.5);
        FilterTiltY.update(accel_tilt_y, 1.5);
    }

    tilt_x = FilterTiltX.state;
    tilt_y = FilterTiltY.state;
}

void updateVerticalVelocity(float az, float current_alt, float rotation, float dt) {
    if (rotation < TUMBLE_THRESHOLD) {
        // az - 1.0 removes gravity. G_IN_MS2 converts 'g' to 'm/s2'
        float acc_z_earth = (az - 1.0) * G_IN_MS2; 
        FilterVVert.predict(acc_z_earth * dt, 0.02);
    } else {
        // We add uncertanty-IMU basically useless
        // This forces the filter to trust the Barometer more
        FilterVVert.uncertainty += 0.5; 
    }

    float v_baro = (current_alt - last_alt) / dt;
    FilterVVert.update(v_baro, 0.8);
    v_vert = FilterVVert.state;
    last_alt = current_alt;
}

void updateHorizontalPosition(float dt, float rotation) {
    // Only integrate Accelerometer if we are NOT spinning
    if (rotation < TUMBLE_THRESHOLD) {
        float cleanAx = (MainBank.IMU.ax - offset_ax - sin(tilt_x * DEG_TO_RAD)) * G_IN_MS2;
        float cleanAy = (MainBank.IMU.ay - offset_ay - sin(tilt_y * DEG_TO_RAD)) * G_IN_MS2;
        
        FilterVX.predict(cleanAx * dt, 0.05);
        FilterVY.predict(cleanAy * dt, 0.05);
    } else {
        // While spinning, we assume velocity stays the same (Inertia)
        // But we increase uncertainty because we aren't measuring changes
        FilterVX.uncertainty += 0.2;
        FilterVY.uncertainty += 0.2;
    }

    // 2. CORRECTION PHASE (GPS)
    if (MainBank.GPS.isUpdated) {
        float speedMS = MainBank.GPS.speed * KMH_TO_MS;
        float courseRad = MainBank.GPS.course * DEG_TO_RAD;
        float gps_vx = speedMS * sin(courseRad);
        float gps_vy = speedMS * cos(courseRad);

        // Using Kalman update instead of fixed (0.7/0.3) blend
        FilterVX.update(gps_vx, 0.1); 
        FilterVY.update(gps_vy, 0.1);

        pos_x = get_MetersFromStartLon(MainBank.GPS.longitude, MainBank.GPS.latitude);
        pos_y = get_MetersFromStartLat(MainBank.GPS.latitude);
    } else {
        // Dead Reckoning: Update position using our filtered velocity
        pos_x += FilterVX.state * dt;
        pos_y += FilterVY.state * dt;
    }

    v_x = FilterVX.state;
    v_y = FilterVY.state;
}

// --- Main Program Entry Points ---

void IMU_init(void) {
    if (!IMU.begin()) {
        LOG("IMU Fail");
    } else {
        Status.imu = true;
    }
    Calibrate_IMU();
    last_alt = MainBank.BMP.altitude;
    FilterVVert.state = 0; 
}

void IMU_run() {
    if(IMU.accelerationAvailable()) {
        IMU.readAcceleration(MainBank.IMU.ax, MainBank.IMU.ay, MainBank.IMU.az);
    }
    if(IMU.gyroscopeAvailable()){
        IMU.readGyroscope(MainBank.IMU.gx, MainBank.IMU.gy, MainBank.IMU.gz);
    }
}

void IMU_main_logic(void) {
    const float dt = 0.05; // Matches your SwTimer 50ms
    float current_alt = MainBank.BMP.altitude;

    float acc_x_raw = MainBank.IMU.ax;
    float acc_y_raw = MainBank.IMU.ay;
    float acc_z_raw = MainBank.IMU.az;

    float gx = MainBank.IMU.gx;
    float gy = MainBank.IMU.gy;
    float gz = MainBank.IMU.gz;

    float rotation = sqrt(sq(gx) + sq(gy) + sq(gz));

    // 1. Filter State
    updateTilt(gx, gy, acc_x_raw, acc_y_raw, acc_z_raw, rotation, dt);
    updateVerticalVelocity(acc_z_raw, current_alt, rotation, dt);
    updateHorizontalPosition(dt, rotation);
    
    // 2. Mission Control
    runBreakpointDeployment(current_alt);
}