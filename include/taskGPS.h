#include <Arduino.h>
#include <TinyGPS++.h>

extern TinyGPSPlus gpsParser;

[[noreturn]] void taskGPS(void *parameter);
