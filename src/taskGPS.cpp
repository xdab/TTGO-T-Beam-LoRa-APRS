#include <taskGPS.h>
#include <taskWebServer.h>
#include <SparkFun_Ublox_Arduino_Library.h>
#include <pins.h>

#define BAUD_GPS 9600

HardwareSerial gpsSerial(1);
SFE_UBLOX_GPS gpsInterface;
TinyGPSPlus gpsParser;
bool gpsInitialized = false;

#ifdef ENABLE_WIFI
#include "wifi_clients.h"
#define MAX_GPS_WIFI_CLIENTS 3
WiFiClient *gps_clients[MAX_GPS_WIFI_CLIENTS];
#endif

[[noreturn]] void taskGPS(void *parameter)
{
  if (!gpsInitialized)
  {
    // Startup HW serial for GPS
    gpsSerial.begin(BAUD_GPS, SERIAL_8N1, PIN_GPS_TX, PIN_GPS_RX);
    gpsInitialized = true;

    // set GPS parameters on restart
    // Thanks Peter (https://github.com/peterus)
    // https://github.com/lora-aprs/TTGO-T-Beam_GPS-reset
    if (gpsInterface.begin(gpsSerial))
    {
      gpsInterface.setUART1Output(COM_TYPE_NMEA); // Set the UART port to output NMEA only
      // gpsInterface.saveConfiguration(); //Save the current settings to flash and BBR
      gpsInterface.enableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
      gpsInterface.enableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
      gpsInterface.enableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
      gpsInterface.enableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
      gpsInterface.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
      gpsInterface.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
      // gpsInterface.saveConfiguration(); //Save the current settings to flash and BBR
      delay(1000);
    }
  }

  String gpsDataBuffer = "              ";
  for (;;)
  {
#ifdef ENABLE_WIFI
    check_for_new_clients(&gpsServer, gps_clients, MAX_GPS_WIFI_CLIENTS);
#endif
    while (gpsSerial.available() > 0)
    {
      char gpsChar = (char)gpsSerial.read();
      gpsParser.encode(gpsChar);
#ifdef ENABLE_WIFI
      if (gpsChar == '$')
      {
        gpsDataBuffer = String(gpsChar);
      }
      else
      {
        gpsDataBuffer += String(gpsChar);

        if (gpsChar == '\n')
        {
          iterateWifiClients([](WiFiClient *client, int clientIdx, const String *data)
                             {
              if (client->connected()){
                client->print(*data);
                client->flush();
              } }, &gpsDataBuffer, gps_clients, MAX_GPS_WIFI_CLIENTS);
          gpsDataBuffer = "";
        }
      }
#endif
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
