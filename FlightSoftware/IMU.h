#pragma once

extern void IMU_init(void);

extern void IMU_run(void);

extern void IMU_main_logic(void);
extern float* get_Acceleration(void);
extern float* get_Gyroscope(void);

extern bool IMU_initialized;