#pragma once

extern void GPS_init();

extern void GPS_run();

extern double* get_GPS_Data();
extern double* get_Home_Location();

extern bool GPS_initialized;
extern bool GPS_updated;