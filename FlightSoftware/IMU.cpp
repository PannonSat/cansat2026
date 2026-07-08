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
const float DEPLOY_THRESHOLD = 50.0;  // Deploy if predicted landing is within 50m of pad
const float MIN_ALT = 500.0;
const float MIN_CALC_ALT = 50;     // Hard floor
const int CONFIRMATION_NEEDED = 5;   // Number of consecutive cycles to trigger

// GPS Reliability Thresholds
const float MIN_GPS_SPEED_FOR_COURSE = 2.0; // m/s - Below this, GPS course is unreliable
const float MIN_VALID_ALT = -20.0;
const float MAX_VALID_ALT = 2600.0;
const float MAX_DEAD_RECKON_TIME = 5.0; // seconds - Max time to use dead reckoning

// 3D Gravity Vector (Replaces the 1D complementary filters for physics)
float grav_x = 0.0f;
float grav_y = 0.0f;
float grav_z = 1.0f; // Assumes satellite boots mostly upright
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

float offset_ax, offset_ay, offset_az;
float last_alt, v_x, v_y, pos_x, pos_y, v_vert, v_baro;
float lastCleanAx, lastCleanAy = 0.0f;

// Visualization Variables
float tilt_x, tilt_y;
unsigned long start_time = millis();

int deployment_confirmations = 0;
bool second_chute_deployed = false;
float gps_loss_timer = 0.0;  // Track how long GPS has been unavailable
float last_good_gps_time = 0.0;

bool IMU_measure_step = false;

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
    delay(1000);

    float sum_x = 0, sum_z = 0, sum_y = 0;
    int samples = 20;

    for (int i = 0; i < samples; i++) {
        if(IMU.accelerationAvailable()){
            float tx, ty, tz;
            IMU.readAcceleration(tx, ty, tz);
            sum_x += tx;  // Horizontal axis 1
            sum_z += tz;  // Horizontal axis 2
            sum_y += ty;
        }
        delay(30);
    }
    // Store offsets for horizontal axes (should be near zero when level)
    offset_ax = sum_z / samples;
    offset_ay = sum_y / samples;
    offset_az = sum_x / samples;

    LOG("Calibration Done. X and Z offsets stored.");
    LOG("Offset X: " + String(offset_ax) + " | Offset Z: " + String(offset_az)+ " | Offset Y: " + String(offset_ay));
}


// --- The Core Mission Logic ---

void runBreakpointDeployment(float current_alt) {
    if ((second_chute_deployed || current_alt < MIN_VALID_ALT || current_alt > MAX_VALID_ALT) || current_alt < MIN_CALC_ALT) return;

    float time_to_impact = current_alt / V_SLOW_CHUTE;

    float pred_land_x = pos_x + (v_x * time_to_impact);
    float pred_land_y = pos_y + (v_y * time_to_impact);
    float pred_dist_to_base = sqrt(sq(pred_land_x) + sq(pred_land_y));

    SAFE_ASSERT(pred_dist_to_base >= 0, "Predicted distance invalid!");
    
    MainBank.GPS.Log_Pred_dist(pred_dist_to_base);
    
    unsigned long curr_time = millis();
    if(curr_time - start_time >= 100){
        if ((pred_dist_to_base <= DEPLOY_THRESHOLD || current_alt <= MIN_ALT) && (MainBank.Operation_Mode == 3 || MainBank.Operation_Mode == 4)) {
            deployment_confirmations++;
        }else{
            deployment_confirmations = 0;
        }
        start_time = millis();
    }

    if (deployment_confirmations >= CONFIRMATION_NEEDED) {
        second_chute_deployed = true;
    }

    LOG("Chute ");
    LOGln(second_chute_deployed);
}

void updateTilt(float gx, float gy, float gz, float ax, float ay, float az, float rotation, float dt) {
    // 1. Calculate raw accelerometer baseline angles FOR LOGGING ONLY
    float accel_tilt_x = atan2(ax, sqrt(ay * ay + az * az)) * (180.0f / PI);
    float accel_tilt_y = atan2(ay, az) * (180.0f / PI);
    
    LOGln("TEST");
    LOG("accel_tilt_x");
    LOGln(accel_tilt_x);
    LOG("accel_tilt_y");
    LOGln(accel_tilt_y);
    
    // 2. TRUE 3D GRAVITY VECTOR CALCULATION (Immune to Gimbal Lock)
    // Convert gyros to radians/sec for correct vector rotation
    float gx_rad = gx * DEG_TO_RAD;
    float gy_rad = gy * DEG_TO_RAD;
    float gz_rad = gz * DEG_TO_RAD;

    float prev_grav_x = grav_x;
    float prev_grav_y = grav_y;
    float prev_grav_z = grav_z;

    // Cross product rotation (Predict step based on Gyro)
    grav_x += (gy_rad * prev_grav_z - gz_rad * prev_grav_y) * dt;
    grav_y += (gz_rad * prev_grav_x - gx_rad * prev_grav_z) * dt;
    grav_z += (gx_rad * prev_grav_y - gy_rad * prev_grav_x) * dt;

    // Blend with Accelerometer to prevent drift (Update step)
    float accel_len = sqrt(ax*ax + ay*ay + az*az);
    if (accel_len > 0.01f) {
        float ax_norm = ax / accel_len;
        float ay_norm = ay / accel_len;
        float az_norm = az / accel_len;

        grav_x = alpha * grav_x + (1.0f - alpha) * ax_norm;
        grav_y = alpha * grav_y + (1.0f - alpha) * ay_norm;
        grav_z = alpha * grav_z + (1.0f - alpha) * az_norm;
    }

    // Normalize gravity vector to maintain exactly 1G length mathematically
    float grav_len = sqrt(grav_x*grav_x + grav_y*grav_y + grav_z*grav_z);
    if (grav_len > 0.01f) {
        grav_x /= grav_len;
        grav_y /= grav_len;
        grav_z /= grav_len;
    }

    // 3. Extract safe 1D Euler angles for Ground Station visualizer
    tilt_x = atan2(grav_x, sqrt(grav_y * grav_y + grav_z * grav_z)) * (180.0f / PI);
    tilt_y = atan2(grav_y, grav_z) * (180.0f / PI);

    LOG("tiltx ");
    LOGln(tilt_x);
    LOG("tilty ");
    LOGln(tilt_y);
    
    MainBank.IMU.Write_IMU_calculations(tilt_x, tilt_y, v_x, v_y, v_vert, rotation);
}
/*
void updateVerticalVelocity(float az, float current_alt, float rotation, float dt) {
    
    if (rotation < TUMBLE_THRESHOLD) {
        float acc_z_earth = (az - 1.0) * G_IN_MS2;
        SAFE_ASSERT(!isnan(acc_z_earth), "NaN in acc_z_earth calculation"); 
        FilterVVert.predict(acc_z_earth * dt, 0.02);
    } else {
        FilterVVert.uncertainty += 0.5; 
    }
    

    v_baro = (current_alt - last_alt) / dt;
    
    FilterVVert.update(v_baro, 1);
    v_vert = FilterVVert.state;
    v_vert = v_baro;
    Serial.print(current_alt);
    Serial.print(", ");
    Serial.println(last_alt);
    Serial.print("V_vert: ");
    Serial.println(v_vert);

    SAFE_ASSERT(v_vert > -100.0f && v_vert < 100.0f, "v_vert out of physical bounds");
    last_alt = current_alt;
}
*/
static unsigned long last_alt_update;
void updateVerticalVelocity(float ax, float ay, float az, float current_alt, float rotation, float dt) {
    /*
    // 1. IMU PREDICT: Still use the dot-product gravity projection
    float gravity_component = (ax * grav_x) + (ay * grav_y) + (az * grav_z);
    float linear_acc_vertical = (gravity_component - 1.0f) * G_IN_MS2;
    
    if (rotation < TUMBLE_THRESHOLD) {
        FilterVVert.predict(linear_acc_vertical * dt, 0.02);
    } else {
        FilterVVert.uncertainty += 0.5; // Tumble damping
    }
    

    // For now I don't want to waste time with this. 
    
    // 2. BARO UPDATE: Only update the filter if altitude has changed significantly
    // This ignores small noise-driven changes.
    static float last_stable_alt = current_alt;
    static unsigned long last_alt_update = millis();
    
    // Only trust the barometer if it has changed by more than 0.5 meters
    // (Adjust 0.5 based on your BMP noise floor)
    if (abs(current_alt - last_stable_alt) > 0.3f) {
        float alt_dt = (millis() - last_alt_update) / 1000.0f;
        
        // Calculate velocity using the actual time passed since the last SIGNIFICANT change
        float v_baro_smoothed = (current_alt - last_stable_alt) / alt_dt;
        
        // Update the Kalman filter with this "smoothed" velocity
        // Increase R (measurement noise) because Baro is jumpy
        FilterVVert.update(v_baro_smoothed, 3.0f); 
        
        last_stable_alt = current_alt;
        last_alt_update = millis();
    }
    
    v_vert = FilterVVert.state;
    
    Serial.print("V_vert: ");
    Serial.println(v_vert);
    */
    // 1. IMU PREDICT: Dot-product gravity projection
    float gravity_component = (ax * grav_x) + (ay * grav_y) + (az * grav_z);
    float linear_acc_vertical = (gravity_component - 1.0f) * G_IN_MS2;
    
    if (rotation < TUMBLE_THRESHOLD) {
        FilterVVert.predict(linear_acc_vertical * dt, 0.02f);
    } else {
        FilterVVert.uncertainty += 0.5f; // Tumble damping
    }
    
    // 2. BARO UPDATE: Trigger only when BMP provides a physically new reading
    float alt_dt = (millis() - last_alt_update) / 1000.0f;
    
    if (current_alt != last_alt) {
        last_alt_update = millis();
        
        // SAFEGUARD: Prevent divide-by-zero if sensor glitches and pushes 
        // two different values in the same millisecond loop.
        if (alt_dt > 0.4) { 
            // Calculate velocity based on actual time elapsed since last new value
            float v_baro = (current_alt - last_alt) / alt_dt;
            // Update the Kalman filter. 
            // Note: If v_vert still drifts, lower 3.0f to 1.0f to make it trust the baro more.
            FilterVVert.update(v_baro, 3.0f); 
            
            // Reset trackers for the next new value
            last_alt = current_alt;
            last_alt_update = millis();
        }
    }
    
    v_vert = FilterVVert.state;
    last_alt = current_alt;
    /*
    Serial.print("V_vert: ");
    Serial.println(v_vert);
    */
}

void correct_with_GPS(float dt, float rotation){
    if (MainBank.GPS.isUpdated) {
        float speedMS = MainBank.GPS.speed * KMH_TO_MS;
        
        if (speedMS >= MIN_GPS_SPEED_FOR_COURSE) {
            float courseRad = MainBank.GPS.course * DEG_TO_RAD;
            float gps_vx = speedMS * sin(courseRad);
            float gps_vy = speedMS * cos(courseRad);

            FilterVX.update(gps_vx, 0.1f); 
            FilterVY.update(gps_vy, 0.1f);
        } else {
            FilterVX.update(0.0f, 0.2f);  
            FilterVY.update(0.0f, 0.2f);
        }

        pos_x = get_MetersFromStartLon(MainBank.GPS.longitude, MainBank.GPS.latitude);
        pos_y = get_MetersFromStartLat(MainBank.GPS.latitude);
        
        gps_loss_timer = 0.0f;
        last_good_gps_time = millis();
        
    } else {
        gps_loss_timer += dt;
        
        if (gps_loss_timer <= MAX_DEAD_RECKON_TIME) {
            FilterVX.state *= 0.95f;
            FilterVY.state *= 0.95f;
        } else {
            FilterVX.update(0.0f, 6.0f); 
            FilterVY.update(0.0f, 6.0f);

            FilterVX.state = FilterVX.state*0.95;
            FilterVY.state = FilterVY.state*0.95;
            
            pos_x += FilterVX.state * dt;  
            pos_y += FilterVY.state * dt;
        }
    }

    v_x = FilterVX.state;
    v_y = FilterVY.state;
}

void updateHorizontalPosition(float dt, float rotation) {
    if (rotation < TUMBLE_THRESHOLD) {
        // Direct conversion of normalized gravity vector components to exact G leaks
        float gravity_leak_x = grav_x * G_IN_MS2;
        float gravity_leak_y = grav_y * G_IN_MS2;
        
        LOG("gravity leak y");
        LOGln(gravity_leak_y);
        LOG("gravity leak x");
        LOGln(gravity_leak_x);
        
        LOG("ax ");
        LOGln(MainBank.IMU.az);
        LOG("ay ");
        LOGln(MainBank.IMU.ay);
        
        float currentAx = (MainBank.IMU.az - offset_ax) * G_IN_MS2 - gravity_leak_x;
        float currentAy = (MainBank.IMU.ay - offset_ay) * G_IN_MS2 - gravity_leak_y; 
        
        float deltaX = abs(currentAx - lastCleanAx);
        float deltaY = abs(currentAy - lastCleanAy);

        float threshold = 0.15f + (FilterVX.uncertainty * 0.1f);
        float activeAx = (deltaX > threshold) ? currentAx : lastCleanAx;
        float activeAy = (deltaY > threshold) ? currentAy : lastCleanAy;

        FilterVX.predict(activeAx * dt, 0.02f);
        FilterVY.predict(activeAy * dt, 0.02f);
        
        lastCleanAx = activeAx;
        lastCleanAy = activeAy;

        if (deltaX <= threshold) FilterVX.state *= 0.99f;
        if (deltaY <= threshold) FilterVY.state *= 0.99f;

    } else {
        FilterVX.uncertainty += 0.2f;
        FilterVY.uncertainty += 0.2f;
    }
    
    correct_with_GPS(dt, rotation);

    SAFE_ASSERT((v_x > -50.0f) && (v_x < 50.0f), "V_X INVALID");
    SAFE_ASSERT((v_y > -50.0f) && (v_y < 50.0f), "V_Y INVALID");
}

// MAIN PROGRAM

void IMU_init() {
    if (!IMU.begin()) {
        LOG("IMU Fail");
    } else {
        LOG("Succesful IMU INIT!");
        LED_beep(1000, 1);
        Status.imu = true;
        LOG("Accelerometer sample rate = ");
        LOG(IMU.accelerationSampleRate());
        LOGln(" Hz");
        Calibrate_IMU();
        LOG("Accelerometer sample rate = ");
        LOG(IMU.accelerationSampleRate());
        LOGln(" Hz");
        last_alt = MainBank.BMP.altitude;
    }
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
    
    // FIXED: Properly scoped static timing variables so they update dynamically on every loop!
    static unsigned long last_loop_time = 0;
    unsigned long current_loop_time = micros();

    if (last_loop_time == 0) {
        last_loop_time = current_loop_time;
        return; // Skip this single frame to baseline the timer
    }

    float dt = (current_loop_time - last_loop_time) / 1000000.0f;
    last_loop_time = current_loop_time;

    if (dt > 0.20f || dt < 0.001f) {
        dt = 0.095f; 
    }
    

    float start_time = millis();
    float current_alt = MainBank.BMP.altitude;
    SAFE_ASSERT(current_alt >= MIN_VALID_ALT && current_alt <= MAX_VALID_ALT, "Invalid current_alt range")
    
    // ============================================================
    // COORDINATE TRANSFORMATION - Vertical Mount
    // ============================================================
    float acc_x_raw = MainBank.IMU.az;   // Horizontal axis 1
    float acc_y_raw = MainBank.IMU.ay;   // Horizontal axis 2
    float acc_z_raw = MainBank.IMU.ax;   // Vertical axis (gravity pulls negative)

    float gx = MainBank.IMU.gz;          // Roll rate
    float gy = MainBank.IMU.gy;          // Pitch rate
    float gz = -MainBank.IMU.gx;         // Yaw rate (inverted)

    float rotation = sqrt(sq(gx) + sq(gy) + sq(gz));

    // Note: Passed gz so the 3D gravity vector can process full spherical rotations
    updateTilt(gx, gy, gz, acc_x_raw, acc_y_raw, acc_z_raw, rotation, dt);
    updateVerticalVelocity(acc_x_raw, acc_y_raw, acc_z_raw, current_alt, rotation, dt);
    updateHorizontalPosition(dt, rotation);
    
    if(MainBank.Operation_Mode == 3 || MainBank.Operation_Mode == 4 || MainBank.Operation_Mode == 5)
        runBreakpointDeployment(current_alt);
    
    String output = "Pos_x: " + String(pos_x) +"Pos_y: " + String(pos_y);
    LOGln(output);
    String speed_output = "V_x" + String(v_x)+ "V_y" + String(v_y);
    LOGln(speed_output);
    
    MainBank.IMU.Write_IMU_calculations(tilt_x, tilt_y, v_x, v_y, v_vert, rotation);
}