#pragma once

//
//

struct TelemetryData {
  float temperature;
  float pressure;
  float altitude;

  float* AccData;
  float* GyroData;

  double latitude;
  double longitude;

  int light_level;
};

extern TelemetryData databank;

extern void DataBank_init();
extern void DataBank_run();

TelemetryData snapshotDataBank();