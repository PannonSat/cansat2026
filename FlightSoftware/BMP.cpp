#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>

#include "Settings.h"
#include "LED.h"
#include "BMP.h"
#include "DataBank.h"

#define PIN_CS 5

static float calc_Altitude(float pressure);

Adafruit_BMP280 bmp(PIN_CS, &SPI);

float altitude_offset = 0.0f;

// ── EMA smoothing parameters ─────────────────────────────────────
float filtered_altitude = 0.0f;           // The smooth value we will use
const float EMA_ALPHA = 0.18f;            // 0.08–0.25 → smaller = smoother but slower
const float MAX_JUMP = 13.0f;              // meters — reject bigger single-frame changes
bool is_initialized = false;

void calibrate_BMP() {
    float sum = 0;
    float measurments = 150;
    for (int i = 0; i < measurments; i++) {
        sum += bmp.readPressure();
        delay(10);
    }
    float groundPressure = sum / measurments;
    altitude_offset = calc_Altitude(groundPressure);
    LOGln(altitude_offset);
}

void BMP_init() {
    SPI.setRX(4);
    SPI.setTX(7);
    SPI.setSCK(6);

    if (!bmp.begin(0x58)) {
        LOG("BMP failed to initialize! Check wiring!");
        LOGln("SensorID was: 0x"); 
        LOGln_multiple(bmp.sensorID(), HEX);
    } else {
        LED_beep(100, 1);
        LOG("BMP280 init successful!");
        Status.bmp = true;
        
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X2,
                        Adafruit_BMP280::SAMPLING_X16,
                        Adafruit_BMP280::FILTER_X16,
                        Adafruit_BMP280::STANDBY_MS_250);
                        
        delay(1000);
        calibrate_BMP();
    }
}

void BMP_run() {
    float start_time = millis();

    float pressure    = bmp.readPressure();
    float temperature = bmp.readTemperature();
    float raw_alt     = calc_Altitude(pressure);

    if (!is_initialized) {
        filtered_altitude = raw_alt;
        is_initialized = true;
    }

    // ── Simple outlier rejection ────────────────────────────────
    // Prevents spikes from vibration / noise / brief sensor glitch
    float delta = raw_alt - filtered_altitude;
    if (abs(delta) > MAX_JUMP) {
        // Questionable solution. We'll see
        filtered_altitude = (EMA_ALPHA * raw_alt) + (1.0f - EMA_ALPHA) * filtered_altitude;
    }else{
        filtered_altitude += constrain(delta, -0.5f, 0.5f);
    }
    //LOGln(temperature);
    MainBank.BMP.Write_BMP_Data(pressure, temperature, filtered_altitude);
}

static float calc_Altitude(float pressure) {
    pressure /= 100.0f;
    float altitude = 44330.0f * (1.0f - pow(pressure / 1013.25f, 0.1903f));
    return altitude - altitude_offset;
}