#include <aprs.h>

/* Creates a Base-91 representation of the value in v in the string */
/* pointed to by s, n-characters long. String length should be n+1. */
char *ax25_base91enc(char *s, uint8_t n, uint32_t v);

void APRSPacket::encode_tnc2(String *str)
{
    *str = source + ">" + destination;
    if (!path.isEmpty())
        *str += "," + path;
    *str += ":" + info;
}

APRSPositionReportFactory::APRSPositionReportFactory(GPS *gps)
{
    this->gps = gps;
    this->callsign = "N0CALL";
    this->path = "";
    this->symbol_table = '/';
    this->symbol_code = '[';
    this->altitude = false;
    this->fixed_latitude = "0000.00N";
    this->fixed_longitude = "00000.00E";
    this->comment = "";
}

void APRSPositionReportFactory::set_callsign(String callsign)
{
    this->callsign = callsign;
}

void APRSPositionReportFactory::set_path(String path)
{
    this->path = path;
}

void APRSPositionReportFactory::set_symbol(char symbol_table, char symbol_code)
{
    this->symbol_table = symbol_table;
    this->symbol_code = symbol_code;
}

void APRSPositionReportFactory::set_altitude(bool altitude)
{
    this->altitude = altitude;
}

void APRSPositionReportFactory::set_fixed_position(String lat, String lon)
{
    this->fixed_latitude = lat;
    this->fixed_longitude = lon;
}

void APRSPositionReportFactory::set_comment(String comment)
{
    this->comment = comment;
}

void APRSPositionReportFactory::create(APRSPacket *packet)
{
    int i;

    packet->source = callsign;
    packet->destination = "APRS";
    packet->path = path;

    if (gps->fixed())
    {
        double lat = gps->latitude();
        double lon = gps->longitude();
        double course = gps->course();
        double speed = gps->speed_knots();

        // Position report indicator
        packet->info = "!";

        // Standard symbol table
        packet->info += symbol_table;

        // Compressed latitude and longitude
        uint32_t aprs_lat, aprs_lon;
        aprs_lat = 900000000 - lat * 10000000;
        aprs_lat = aprs_lat / 26 - aprs_lat / 2710 + aprs_lat / 15384615;
        aprs_lon = 900000000 + lon * 5000000;
        aprs_lon = aprs_lon / 26 - aprs_lon / 2710 + aprs_lon / 15384615;
        char helper_base91[] = {"0000\0"};
        ax25_base91enc(helper_base91, 4, aprs_lat);
        for (i = 0; i < 4; i++)
            packet->info += helper_base91[i];
        ax25_base91enc(helper_base91, 4, aprs_lon);
        for (i = 0; i < 4; i++)
            packet->info += helper_base91[i];

        // Standard symbol code
        packet->info += symbol_code;

        // Compressed course and speed
        ax25_base91enc(helper_base91, 1, (uint32_t)course / 4);
        packet->info += helper_base91[0];
        ax25_base91enc(helper_base91, 1, (uint32_t)(log1p(speed) / 0.07696));
        packet->info += helper_base91[0];
        packet->info += "H";

        if (altitude)
        {
            int Talt = gps->altitude_meters() * 3.28;
            String Altx = String(Talt);
            packet->info += "/A=";
            for (i = 0; i < (6 - Altx.length()); ++i)
                packet->info += "0";
            packet->info += Altx;
        }
    }
    else
    {
        // Fallback to fixed position if GPS is not available
        packet->info += fixed_latitude;
        packet->info += symbol_table;
        packet->info += fixed_longitude;
        packet->info += symbol_code;
    }

    if (!comment.isEmpty())
        packet->info += comment;
}

char *ax25_base91enc(char *s, uint8_t n, uint32_t v)
{
    for (s += n, *s = '\0'; n; n--)
    {
        *(--s) = v % 91 + 33;
        v /= 91;
    }
    return s;
}
