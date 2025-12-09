#pragma once

extern float* get_Acceleration(void);
extern float* get_Gyroscope(void);
extern void IMU_run(void);
extern void IMU_init(void);
extern bool IMU_initialized;