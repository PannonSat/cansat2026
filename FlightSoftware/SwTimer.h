#pragma once

// Qpac CanSat's code, THANK  YOU!

//  This clever algorithm manages our tasks
//  Every task runed by SwTimer have a chanel and a time. If the time is over the modul calles that task.
//  The system can manage repetitive tasks to.


extern void SwTimer_Init(unsigned long ticktime);
extern void SwTimer_Run(void);

//  The given function runs only once after the given time
extern void SwTimer_Set_Single(unsigned int ch, unsigned int value_ms, void (*callback)(void));

//  The given function runs at specified time intervals
extern void SwTimer_Set_Continues(unsigned int ch, unsigned int value_ms, void (*callback)(void));

//  Stop a task
extern void SwTimer_Stop(unsigned int ch);

//  Returns the remaining time of a given Task (in miliseconds)
extern unsigned long SwTimer_Get_Remaining_Time(unsigned int ch);