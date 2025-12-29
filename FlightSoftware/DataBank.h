#pragma once

struct TelemetryData {
  float temperature;
  float pressure;
  float altitude;

  float* AccData;
  float* GyroData;
  
  bool GPS_updated;
  double latitude;
  double longitude;
  float predicted_current_distance;
  double home_lat;
  double home_lng;
  
  float course_deg;
  float speed_kmh;

  float light_level;
};

extern TelemetryData databank;

extern void DataBank_init();

extern void DataBank_run();

// Returns a current snapshot of the DataBank
TelemetryData snapshotDataBank();