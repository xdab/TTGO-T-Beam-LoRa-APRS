#ifndef GPS_H
#define GPS_H

#include <TinyGPS++.h>
#include <taskWebServer.h>
#include <SparkFun_Ublox_Arduino_Library.h>
#include <pins.h>

#ifdef ENABLE_WIFI
#include "wifi_clients.h"
#define MAX_GPS_WIFI_CLIENTS 5
#endif

class GPS
{
private:
    SFE_UBLOX_GPS interface;
    TinyGPSPlus parser;
    bool initialized = false;
    bool state = true;

#ifdef ENABLE_WIFI
    WiFiClient *wifi_clients[MAX_GPS_WIFI_CLIENTS];
    String wifi_data_buf = "              ";
#endif

public:
    // Task API
    void _read_serial();

public:
    // GPS API
    void initialize();
    void set_state(bool state);
    bool enabled();
    bool fixed();
    uint32_t fix_age();
    uint32_t satellites();
    double latitude();
    double longitude();
    double course();
    double speed_knots();
    double speed_kmph();
    double altitude_meters();
};

extern GPS gps;

#endif // GPS_H