#pragma once

extern void LED_init();
// Beeps the 1st or 2nd LED for given time, length in milliseconds.
extern void LED_beep(float length, int LED);

extern void LED_status();