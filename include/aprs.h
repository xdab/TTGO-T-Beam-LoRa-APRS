#ifndef APRS_H
#define APRS_H

#include <Arduino.h>
#include <gps.h>

#define MAX_PATH_LEN 8

class APRSPacket
{
public:
    String source;
    String destination;
    String path;
    String info;

public:
    void encode_tnc2(String *str);
};

class APRSPositionReportFactory
{
private:
    GPS *gps;

    String callsign;
    String path;
    char symbol_table;
    char symbol_code;
    bool altitude;
    String fixed_latitude;
    String fixed_longitude;
    String comment;

public:
    APRSPositionReportFactory(GPS *gps);
    void set_callsign(String callsign);
    void set_path(String path);
    void set_symbol(char symbol_table, char symbol_code);
    void set_altitude(bool altitude);
    void set_fixed_position(String lat, String lon);
    void set_comment(String comment);
    void create(APRSPacket *packet);
};

#endif // APRS_H