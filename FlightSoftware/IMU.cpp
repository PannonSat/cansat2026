#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>

#include "Settings.h"
#include "LED.h"
#include "DataBank.h"
#include "IMU.h"

// --- Constants ---
#define G_IN_MS2 9.80665
#define KMH_TO_MS 0.277778
#define METERS_PER_DEGREE 111320.0

// Mission Profiles
const float V_SLOW_CHUTE = 6.0;      // m/s descent (Target for deployment)
const float DEPLOY_THRESHOLD = 50.0;  // Deploy if predicted landing is within 30m of pad
const float MIN_ALT = 100.0;
const float MIN_CALC_ALT = 50;     // Hard floor
const int CONFIRMATION_NEEDED = 3;   // Number of consecutive cycles to trigger

// GPS Reliability Thresholds
const float MIN_GPS_SPEED_FOR_COURSE = 2.0; // m/s - Below this, GPS course is unreliable
const float MIN_VALID_ALT = -20.0;
const float MAX_VALID_ALT = 1200.0;
const float MAX_DEAD_RECKON_TIME = 3.0; // seconds - Max time to use dead reckoning

float tilt_x_compl = 0.0f;
float tilt_y_compl = 0.0f;
const float alpha = 0.92f;

// --- Kalman Filter Structure ---
struct Kalman1D {
    float state = 0;       
    float uncertainty = 1.0; 
    float Q = 0.02; // Not used?       
    float R = 0.8;         

    void predict(float delta, float q_noise) {
        state += delta;             
        uncertainty += q_noise;     
    }

    void update(float measurement, float r_noise) { 
        float gain = uncertainty / (uncertainty + r_noise);
        state += gain * (measurement - state);
        uncertainty *= (1.0 - gain);
    }
};

// State Vars
Kalman1D FilterVVert; 
Kalman1D FilterTiltX;
Kalman1D FilterTiltY;
Kalman1D FilterVX = {0, 1.0, 0.05, 0.1}; 
Kalman1D FilterVY = {0, 1.0, 0.05, 0.1};

float offset_ax, offset_ay;
float last_alt, v_x, v_y, pos_x, pos_y, v_vert;
float lastCleanAx, lastCleanAy = 0.0f;
float tilt_x, tilt_y;

int deployment_confirmations = 0;
bool second_chute_deployed = false;
float gps_loss_timer = 0.0;  // Track how long GPS has been unavailable
float last_good_gps_time = 0.0;

// Helper Functions

float get_MetersFromStartLat(double current_lat) {
    return (current_lat - MainBank.GPS.home_lat) * METERS_PER_DEGREE;
}

float get_MetersFromStartLon(double current_lon, double current_lat) {
    float cos_lat = cos(current_lat * DEG_TO_RAD);
    return (current_lon - MainBank.GPS.home_lng) * METERS_PER_DEGREE * cos_lat;
}

void Calibrate_IMU(void) {
    LOG("WARNING: Calibrating IMU (Vertical Mount)... Keep the Sat perfectly level!");
    delay(2000);

    float sum_x = 0, sum_z = 0;
    int samples = 200;
    
    for (int i = 0; i < samples; i++) {
        float tx, ty, tz;
        if(IMU.accelerationAvailable()){
            IMU.readAcceleration(tx, ty, tz);
            sum_x += tx;  // Horizontal axis 1
            sum_z += tz;  // Horizontal axis 2
        }
        delay(5);
    }
    
    // Store offsets for horizontal axes (should be near zero when level)
    offset_ax = sum_x / samples;
    offset_ay = sum_z / samples;

    LOG("Calibration Done. X and Z offsets stored.");
    LOG("Offset X: " + String(offset_ax) + " | Offset Z: " + String(offset_ay));
}

// --- The Core Mission Logic ---

void runBreakpointDeployment(float current_alt) {
    // Don't calculate if current_alt is invalid, if parachute has been deployed, if we are below calculating altitude (alias we are on ground).
    if ((second_chute_deployed || current_alt < MIN_VALID_ALT || current_alt > MAX_VALID_ALT) || current_alt > MIN_CALC_ALT) return;

    float time_to_impact = current_alt / V_SLOW_CHUTE;

    // Calculate IMU-based prediction (unreliable if GPS was lost)
    float pred_land_x = pos_x + (v_x * time_to_impact);
    float pred_land_y = pos_y + (v_y * time_to_impact);
    float pred_dist_to_base = sqrt(sq(pred_land_x) + sq(pred_land_y));

    SAFE_ASSERT(pred_dist_to_base >= 0, "Predicted distance invalid!");
    
    // Log it
    MainBank.GPS.Log_Pred_dist(pred_dist_to_base);
    // Deployment decision
    if (pred_dist_to_base <= DEPLOY_THRESHOLD || current_alt <= MIN_ALT) {
        deployment_confirmations++;
    } else {
        deployment_confirmations = 0;
    }

    if (deployment_confirmations >= CONFIRMATION_NEEDED) {
        second_chute_deployed = true;
    }
}

void updateTilt(float gx, float gy, float ax, float ay, float az, float rotation, float dt) {
    // Predict tilt change from gyroscope

    if (rotation < TUMBLE_THRESHOLD) {
        float accel_tilt_x = atan2(ay, az) * (180.0f / PI);
        float accel_tilt_y = atan2(-ax, az) * (180.0f / PI);

        // Complementary filter on top (very smooth)
        tilt_x_compl = alpha * tilt_x_compl + (1.0f - alpha) * accel_tilt_x;
        tilt_y_compl = alpha * tilt_y_compl + (1.0f - alpha) * accel_tilt_y;
    }

    tilt_x = tilt_x_compl;
    tilt_y = tilt_y_compl;
    MainBank.IMU.Write_IMU_calculations(tilt_x, tilt_y, v_x, v_y, v_vert, rotation);
}

void updateVerticalVelocity(float az, float current_alt, float rotation, float dt) {
    if (rotation < TUMBLE_THRESHOLD) {
        // az is pointing UP when vertical, so az - 1.0 removes gravity
        // Positive acceleration = upward acceleration
        float acc_z_earth = (az - 1.0) * G_IN_MS2;
        SAFE_ASSERT(!isnan(acc_z_earth), "NaN in acc_z_earth calculation"); 
        FilterVVert.predict(acc_z_earth * dt, 0.02);
    } else {
        // Add uncertainty during tumble - IMU becomes less reliable
        FilterVVert.uncertainty += 0.5; 
    }

    // Barometer velocity (negative = descending)
    float v_baro = (current_alt - last_alt) / dt;
    FilterVVert.update(v_baro, 1);
    //v_vert = FilterVVert.state;
    v_vert = v_baro;
    SAFE_ASSERT(v_vert > -100.0f && v_vert < 100.0f, "v_vert out of physical bounds");
    last_alt = current_alt;
}

void correct_with_GPS(float dt, float rotation){
    if (MainBank.GPS.isUpdated) {
        float speedMS = MainBank.GPS.speed * KMH_TO_MS;
        
        // Only trust GPS course at reasonable speeds
        if (speedMS >= MIN_GPS_SPEED_FOR_COURSE) {
            float courseRad = MainBank.GPS.course * DEG_TO_RAD;
            
            // Convert Speed + Heading into X and Y velocities
            float gps_vx = speedMS * sin(courseRad);
            float gps_vy = speedMS * cos(courseRad);

            FilterVX.update(gps_vx, 0.1); 
            FilterVY.update(gps_vy, 0.1);
        } else {
            // Low speed - GPS course unreliable, just dampen velocity
            FilterVX.update(0.5, 0.5);  // Assume near-zero with higher uncertainty
            FilterVY.update(0.5, 0.5);
        }

        // Update ground-truth position from GPS
        pos_x = get_MetersFromStartLon(MainBank.GPS.longitude, MainBank.GPS.latitude);
        pos_y = get_MetersFromStartLat(MainBank.GPS.latitude);
        
        // Reset dead reckoning timer - we have good GPS
        gps_loss_timer = 0.0;
        last_good_gps_time = millis() / 1000.0;
        
    } else {
        // GPS LOSS HANDLING
        gps_loss_timer += dt;
        
        if (gps_loss_timer <= MAX_DEAD_RECKON_TIME) {
            // SHORT-TERM: Use dead reckoning with heavy damping
            pos_x += FilterVX.state * dt * 0.7;  // Only trust 70% of IMU velocity
            pos_y += FilterVY.state * dt * 0.7;
            
            // Dampen Velocity over time
            FilterVX.state *= 0.95;
            FilterVY.state *= 0.95;
        } else {
            // LONG TERM GPS LOSS
            
            // Assume we're drifting slowly, reduce velocity to near-zero
            FilterVX.state *= 0.90;
            FilterVY.state *= 0.90;
            
            // Mark high uncertainty
            FilterVX.uncertainty = 5.0;
            FilterVY.uncertainty = 5.0;
    }

    v_x = FilterVX.state;
    v_y = FilterVY.state;
    }
}

void updateHorizontalPosition(float dt, float rotation) {
    // Only integrate Accelerometer if we are NOT spinning (tumble)
    if (rotation < TUMBLE_THRESHOLD) {
        // When tilted, gravity component leaks into horizontal axes
        float gravity_leak_x = G_IN_MS2 * sin(tilt_x * DEG_TO_RAD);
        float gravity_leak_y = G_IN_MS2 * sin(tilt_y * DEG_TO_RAD);
        
        // Calculate true horizontal acceleration in Earth frame
        float currentAx = (MainBank.IMU.ax - offset_ax) * G_IN_MS2 - gravity_leak_x;
        float currentAy = (MainBank.IMU.az - offset_ay) * G_IN_MS2 - gravity_leak_y;
        
        // Adaptive deadband - gets rid of small jitter
        float deltaX = abs(currentAx - lastCleanAx);
        float deltaY = abs(currentAy - lastCleanAy);

        // Use adaptive threshold based on uncertainty
        float threshold = 0.15 + (FilterVX.uncertainty * 0.1);
        float activeAx = (deltaX > threshold) ? currentAx : lastCleanAx;
        float activeAy = (deltaY > threshold) ? currentAy : lastCleanAy;

        // Feed the Kalman Predictor
        FilterVX.predict(activeAx * dt, 0.05);
        FilterVY.predict(activeAy * dt, 0.05);

        // Update the last values
        lastCleanAx = activeAx;
        lastCleanAy = activeAy;

        // Dampener during stable periods (air resistance)
        if (deltaX <= threshold) FilterVX.state *= 0.99;
        if (deltaY <= threshold) FilterVY.state *= 0.99;

    } else {
        // High rotation/Tumbling: Trust the math less, wait for GPS
        FilterVX.uncertainty += 0.2;
        FilterVY.uncertainty += 0.2;
    }
    
    // CORRECTION PHASE
    correct_with_GPS(dt, rotation);
    SAFE_ASSERT(-50< v_x < 50, "V_X INVALID");
    SAFE_ASSERT(-50< v_y < 50, "V_Y INVALID");
}

// MAIN PROGRAM

void IMU_init() {
    if (!IMU.begin()) {
        LOG("IMU Fail");
    } else {
        LOG("Succesful IMU INIT!");
        LED_beep(100, 1);
        Status.imu = true;
    }
    Calibrate_IMU();
    last_alt = MainBank.BMP.altitude;
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
    const float dt = 0.025; // 5ms
    float current_alt = MainBank.BMP.altitude;
    SAFE_ASSERT(current_alt >= MIN_VALID_ALT && current_alt <= MAX_VALID_ALT, "Invalid current_alt range")
    
    // ============================================================
    // COORDINATE TRANSFORMATION - Vertical Mount
    // ============================================================
    // Physical IMU axes → Earth frame axes
    // X_imu (right) → X_earth (horizontal east/west)
    // Y_imu (up along board) → Z_earth (vertical up/down) 
    // Z_imu (out of board) → Y_earth (horizontal north/south)
    
    float acc_x_raw = MainBank.IMU.ax;   // Horizontal axis 1
    float acc_y_raw = MainBank.IMU.az;   // Horizontal axis 2
    float acc_z_raw = MainBank.IMU.ay;   // Vertical axis (gravity pulls negative)

    float gx = MainBank.IMU.gx;          // Roll rate
    float gy = MainBank.IMU.gz;          // Pitch rate
    float gz = -MainBank.IMU.gy;         // Yaw rate (inverted)

    LOGln(acc_x_raw);
    // Total rotation magnitude
    float rotation = sqrt(sq(gx) + sq(gy) + sq(gz));

    // 1. Update all filter states
    updateTilt(gx, gy, acc_x_raw, acc_y_raw, acc_z_raw, rotation, dt);
    updateVerticalVelocity(acc_z_raw, current_alt, rotation, dt);
    updateHorizontalPosition(dt, rotation);
    
    // 2. Mission Control Decision
    runBreakpointDeployment(current_alt);
    
    // 3. Write to DataBank for logging/
    MainBank.IMU.Write_IMU_calculations(tilt_x, tilt_y, v_x, v_y, v_vert, rotation);
}