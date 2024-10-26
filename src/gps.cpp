#include <gps.h>
#include <HardwareSerial.h>

#define BAUD_GPS 9600

static HardwareSerial gpsSerial(1);
GPS gps;

void GPS::initialize()
{
    if (initialized)
        return;

    // Startup HW serial for GPS
    gpsSerial.begin(BAUD_GPS, SERIAL_8N1, PIN_GPS_TX, PIN_GPS_RX);

    if (!interface.begin(gpsSerial))
        return;

    interface.setUART1Output(COM_TYPE_NMEA); // Set the UART port to output NMEA only
    interface.enableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
    interface.enableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
    interface.enableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
    interface.enableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
    interface.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
    interface.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);

    initialized = true;
}

void GPS::_read_serial()
{
#ifdef ENABLE_WIFI
    check_for_new_clients(&gpsServer, wifi_clients, MAX_GPS_WIFI_CLIENTS);
#endif

    while (gpsSerial.available() > 0)
    {
        char gpsChar = (char)gpsSerial.read();
        parser.encode(gpsChar);

#ifdef ENABLE_WIFI
        if (gpsChar == '$')
        {
            wifi_data_buf = String(gpsChar);
        }
        else
        {
            wifi_data_buf += String(gpsChar);
            if (gpsChar == '\n')
            {
                auto sendToClient = [](WiFiClient *client, int clientIdx, const String *data)
                {
                    if (client->connected())
                    {
                        client->print(*data);
                        client->flush();
                    }
                };
                iterateWifiClients(sendToClient, &wifi_data_buf, wifi_clients, MAX_GPS_WIFI_CLIENTS);
                wifi_data_buf = "";
            }
        }
#endif
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void GPS::set_state(bool state)
{
    this->state = state;
}

bool GPS::enabled()
{
    return state;
}

bool GPS::fixed()
{
    return enabled() && parser.location.isValid();
}

uint32_t GPS::fix_age()
{
    if (!enabled())
        return (uint32_t)ULONG_MAX;
    return parser.location.age();
}

uint32_t GPS::satellites()
{
    return parser.satellites.value();
}

double GPS::latitude()
{
    return parser.location.lat();
}

double GPS::longitude()
{
    return parser.location.lng();
}

double GPS::course()
{
    return parser.course.deg();
}

double GPS::speed_knots()
{
    return parser.speed.knots();
}

double GPS::speed_kmph()
{
    return parser.speed.kmph();
}

double GPS::altitude_meters()
{
    return parser.altitude.meters();
}