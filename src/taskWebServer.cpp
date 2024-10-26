#include <list>
#include "taskWebServer.h"
#include "preference_storage.h"
#include "syslog_log.h"
#include "PSRAMJsonDocument.h"
#include <time.h>
#include <ArduinoJson.h>

/**
 * @see board_build.embed_txtfiles in platformio.ini
 */
extern const char web_index_html[] asm("_binary_data_embed_index_html_out_start");
extern const char web_index_html_end[] asm("_binary_data_embed_index_html_out_end");
extern const char web_style_css[] asm("_binary_data_embed_style_css_out_start");
extern const char web_style_css_end[] asm("_binary_data_embed_style_css_out_end");
extern const char web_js_js[] asm("_binary_data_embed_js_js_out_start");
extern const char web_js_js_end[] asm("_binary_data_embed_js_js_out_end");

// Variable needed to send beacon from html page
extern bool manBeacon;

// Variable to show AP status
extern bool apEnabled;
extern bool apConnected;
extern String infoApName;
extern String infoApPass;
extern String infoApAddr;

QueueHandle_t webListReceivedQueue = nullptr;
std::list<tReceivedPacketData *> receivedPackets;
const int MAX_RECEIVED_LIST_SIZE = 50;

String apSSID = "";
String apPassword;
String defApPassword = "xxxxxxxxxx";

WebServer server(80);
#ifdef KISS_PROTOCOL
WiFiServer tncServer(NETWORK_TNC_PORT);
#endif
WiFiServer gpsServer(NETWORK_GPS_PORT);

#ifdef ENABLE_SYSLOG
// A UDP instance to let us send and receive packets over UDP
WiFiUDP udpClient;

// Create a new empty syslog instance
Syslog syslog(udpClient, SYSLOG_PROTO_IETF);
#endif

#ifdef T_BEAM_V1_2
#include <XPowersLib.h>
extern XPowersAXP2101 PMU;
#elif T_BEAM_V1_0
#include <XPowersLib.h>
extern XPowersAXP192 PMU;
#endif

void sendCacheHeader() { server.sendHeader("Cache-Control", "max-age=3600"); }
void sendGzipHeader() { server.sendHeader("Content-Encoding", "gzip"); }

String jsonEscape(String s)
{
  s.replace("\\", "\\\\");
  s.replace("\"", "\\\"");
  s.replace("\x7f", "\\\x7f");
  for (char i = 0; i < 0x1f; i++)
  {
    s.replace(String(i), "\\" + String((char)i));
  }
  return s;
}

String jsonLineFromPreferenceString(const char *preferenceName, bool last = false)
{
  return String("\"") + preferenceName + "\":\"" + jsonEscape(preferences.getString(preferenceName)) + (last ? +R"(")" : +R"(",)");
}
String jsonLineFromPreferenceBool(const char *preferenceName, bool last = false)
{
  return String("\"") + preferenceName + "\":" + (preferences.getBool(preferenceName) ? "true" : "false") + (last ? +R"()" : +R"(,)");
}
String jsonLineFromPreferenceInt(const char *preferenceName, bool last = false)
{
  return String("\"") + preferenceName + "\":" + (preferences.getInt(preferenceName)) + (last ? +R"()" : +R"(,)");
}
String jsonLineFromPreferenceDouble(const char *preferenceName, bool last = false)
{
  return String("\"") + preferenceName + "\":" + String(preferences.getDouble(preferenceName), 3) + (last ? +R"()" : +R"(,)");
}
String jsonLineFromString(const char *name, const char *value, bool last = false)
{
  return String("\"") + name + "\":\"" + jsonEscape(value) + "\"" + (last ? +R"()" : +R"(,)");
}
String jsonLineFromInt(const char *name, const int value, bool last = false)
{
  return String("\"") + name + "\":" + String(value) + (last ? +R"()" : +R"(,)");
}

void handle_NotFound()
{
  sendCacheHeader();
  server.send(404, "text/plain", "Not found");
}

void handle_Index()
{
  sendGzipHeader();
  server.send_P(200, "text/html", web_index_html, web_index_html_end - web_index_html);
}

void handle_Style()
{
  sendCacheHeader();
  sendGzipHeader();
  server.send_P(200, "text/css", web_style_css, web_style_css_end - web_style_css);
}

void handle_Js()
{
  sendCacheHeader();
  sendGzipHeader();
  server.send_P(200, "text/javascript", web_js_js, web_js_js_end - web_js_js);
}

void handle_ScanWifi()
{
  String listResponse = R"(<label for="networks_found_list">Networks found:</label><select class="u-full-width" id="networks_found_list">)";
  int n = WiFi.scanNetworks();
  listResponse += "<option value=\"\">Select Network</option>";

  for (int i = 0; i < n; ++i)
  {
    listResponse += "<option value=\"" + WiFi.SSID(i) + "\">" + WiFi.SSID(i) + "</option>";
  }
  listResponse += "</select>";
  server.send(200, "text/html", listResponse);
}

void handle_SaveWifiCfg()
{
  if (!server.hasArg(PREF_WIFI_SSID) || !server.hasArg(PREF_WIFI_PASSWORD) || !server.hasArg(PREF_AP_PASSWORD))
  {
    server.send(500, "text/plain", "Invalid request, make sure all fields are set");
  }

  if (!server.arg(PREF_WIFI_SSID).length())
  {
    server.send(403, "text/plain", "Empty SSID");
  }
  else
  {
    // Update SSID
    preferences.putString(PREF_WIFI_SSID, server.arg(PREF_WIFI_SSID));
    Serial.println("Updated SSID: " + server.arg(PREF_WIFI_SSID));
  }

  if (server.arg(PREF_WIFI_PASSWORD) != "*" && server.arg(PREF_WIFI_PASSWORD).length() > 0 && server.arg(PREF_WIFI_PASSWORD).length() < 8)
  {
    server.send(403, "text/plain", "WiFi Password must be minimum 8 character");
  }
  else
  {
    if (server.arg(PREF_WIFI_PASSWORD) != "*")
    {
      // Update WiFi password
      preferences.putString(PREF_WIFI_PASSWORD, server.arg(PREF_WIFI_PASSWORD));
      Serial.println("Updated WiFi PASS: " + server.arg(PREF_WIFI_PASSWORD));
    }
  }

  if (server.arg(PREF_AP_PASSWORD) != "*" && server.arg(PREF_AP_PASSWORD).length() < 8)
  {
    server.send(403, "text/plain", "AP Password must be minimum 8 character");
  }
  else
  {
    if (server.arg(PREF_AP_PASSWORD) != "*")
    {
      // Update AP password
      preferences.putString(PREF_AP_PASSWORD, server.arg(PREF_AP_PASSWORD));
      Serial.println("Updated AP PASS: " + server.arg(PREF_AP_PASSWORD));
    }
  }

  server.sendHeader("Location", "/");
  server.send(302, "text/html", "");
}

void handle_Reboot()
{
  server.sendHeader("Location", "/");
  server.send(302, "text/html", "");
  server.close();
  ESP.restart();
}

void handle_Beacon()
{
  server.sendHeader("Location", "/");
  server.send(302, "text/html", "");
  manBeacon = true;
}

void handle_Shutdown()
{
#ifdef T_BEAM_V1_2
  server.send(200, "text/html", "Shutdown");
  PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
  PMU.shutdown();
#elif T_BEAM_V1_0
  server.send(200, "text/html", "Shutdown");
  PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
  PMU.shutdown();
#else
  server.send(404, "text/html", "Not supported");
#endif
}

void handle_Restore()
{
  server.sendHeader("Location", "/");
  server.send(302, "text/html", "");
  preferences.clear();
  preferences.end();
  ESP.restart();
}

void handle_Cfg()
{
  String jsonData = "{";
  jsonData += String("\"") + PREF_WIFI_PASSWORD + "\": \"" + jsonEscape((preferences.getString(PREF_WIFI_PASSWORD).isEmpty() ? String("") : "*")) + R"(",)";
  jsonData += String("\"") + PREF_AP_PASSWORD + "\": \"" + jsonEscape((preferences.getString(PREF_AP_PASSWORD).isEmpty() ? String("") : "*")) + R"(",)";
  jsonData += jsonLineFromPreferenceString(PREF_WIFI_SSID);
  jsonData += jsonLineFromPreferenceDouble(PREF_LORA_FREQ_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_LORA_SPEED_PRESET);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_CALLSIGN);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_RELAY_PATH);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_SYMBOL_TABLE);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_SYMBOL);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_COMMENT);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_LATITUDE_PRESET);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_LONGITUDE_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_APRS_SB_MIN_INTERVAL_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_APRS_SB_MAX_INTERVAL_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_APRS_SB_MIN_SPEED_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_APRS_SB_MAX_SPEED_PRESET);
  jsonData += jsonLineFromPreferenceDouble(PREF_APRS_SB_ANGLE_PRESET);
  jsonData += jsonLineFromPreferenceBool(PREF_APRS_SHOW_BATTERY);
  jsonData += jsonLineFromPreferenceBool(PREF_APRS_FIXED_BEACON_PRESET);
  jsonData += jsonLineFromPreferenceBool(PREF_APRS_SHOW_ALTITUDE);
  jsonData += jsonLineFromPreferenceBool(PREF_APRS_GPS_EN);
  jsonData += jsonLineFromPreferenceBool(PREF_ENABLE_TNC_SELF_TELEMETRY);
  jsonData += jsonLineFromPreferenceInt(PREF_TNC_SELF_TELEMETRY_INTERVAL);
  jsonData += jsonLineFromPreferenceInt(PREF_TNC_SELF_TELEMETRY_MIC);
  jsonData += jsonLineFromPreferenceString(PREF_TNC_SELF_TELEMETRY_PATH);
  jsonData += jsonLineFromPreferenceBool(PREF_DEV_OL_EN);
  jsonData += jsonLineFromPreferenceBool(PREF_APRS_SHOW_CMT);
  jsonData += jsonLineFromPreferenceBool(PREF_DEV_BT_EN);
  jsonData += jsonLineFromPreferenceInt(PREF_DEV_SHOW_RX_TIME);
  jsonData += jsonLineFromPreferenceBool(PREF_DEV_AUTO_SHUT);
  jsonData += jsonLineFromPreferenceInt(PREF_DEV_AUTO_SHUT_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_DEV_SHOW_OLED_TIME);
  jsonData += jsonLineFromInt("FreeHeap", ESP.getFreeHeap());
  jsonData += jsonLineFromInt("HeapSize", ESP.getHeapSize());
  jsonData += jsonLineFromInt("FreeSketchSpace", ESP.getFreeSketchSpace());
  jsonData += jsonLineFromInt("PSRAMSize", ESP.getPsramSize());
  jsonData += jsonLineFromInt("PSRAMFree", ESP.getFreePsram(), true);

  jsonData += "}";
  server.send(200, "application/json", jsonData);
}

void handle_ReceivedList()
{
  PSRAMJsonDocument doc(MAX_RECEIVED_LIST_SIZE * 1000);
  // DynamicJsonDocument doc(MAX_RECEIVED_LIST_SIZE * 500);
  JsonObject root = doc.to<JsonObject>();
  auto received = root.createNestedArray("received");
  for (auto element : receivedPackets)
  {
    char buf[64];
    strftime(buf, 64, "%Y.%m.%d %H:%M:%S", &element->rxTime);
    auto packet_data = received.createNestedObject();
    packet_data["time"] = String(buf);
    packet_data["packet"] = element->packet->c_str();
    packet_data["rssi"] = element->RSSI;
    packet_data["snr"] = element->SNR / 10.0f;
  }

  server.send(200, "application/json", doc.as<String>());
}

void handle_SaveAPRSCfg()
{
  // LoRa settings
  if (server.hasArg(PREF_LORA_FREQ_PRESET))
  {
    preferences.putDouble(PREF_LORA_FREQ_PRESET, server.arg(PREF_LORA_FREQ_PRESET).toDouble());
    Serial.printf("FREQ saved:\t%f\n", server.arg(PREF_LORA_FREQ_PRESET).toDouble());
  }
  if (server.hasArg(PREF_LORA_SPEED_PRESET))
  {
    preferences.putInt(PREF_LORA_SPEED_PRESET, server.arg(PREF_LORA_SPEED_PRESET).toInt());
  }
  // APRS station settings
  if (server.hasArg(PREF_APRS_CALLSIGN) && !server.arg(PREF_APRS_CALLSIGN).isEmpty())
  {
    preferences.putString(PREF_APRS_CALLSIGN, server.arg(PREF_APRS_CALLSIGN));
  }
  if (server.hasArg(PREF_APRS_SYMBOL_TABLE) && !server.arg(PREF_APRS_SYMBOL_TABLE).isEmpty())
  {
    preferences.putString(PREF_APRS_SYMBOL_TABLE, server.arg(PREF_APRS_SYMBOL_TABLE));
  }
  if (server.hasArg(PREF_APRS_SYMBOL) && !server.arg(PREF_APRS_SYMBOL).isEmpty())
  {
    preferences.putString(PREF_APRS_SYMBOL, server.arg(PREF_APRS_SYMBOL));
  }
  if (server.hasArg(PREF_APRS_RELAY_PATH))
  {
    preferences.putString(PREF_APRS_RELAY_PATH, server.arg(PREF_APRS_RELAY_PATH));
  }
  if (server.hasArg(PREF_APRS_COMMENT))
  {
    preferences.putString(PREF_APRS_COMMENT, server.arg(PREF_APRS_COMMENT));
  }
  if (server.hasArg(PREF_APRS_LATITUDE_PRESET))
  {
    preferences.putString(PREF_APRS_LATITUDE_PRESET, server.arg(PREF_APRS_LATITUDE_PRESET));
  }
  if (server.hasArg(PREF_APRS_LONGITUDE_PRESET))
  {
    preferences.putString(PREF_APRS_LONGITUDE_PRESET, server.arg(PREF_APRS_LONGITUDE_PRESET));
  }
  if (server.hasArg(PREF_TNC_SELF_TELEMETRY_INTERVAL))
  {
    preferences.putInt(PREF_TNC_SELF_TELEMETRY_INTERVAL, server.arg(PREF_TNC_SELF_TELEMETRY_INTERVAL).toInt());
  }
  if (server.hasArg(PREF_TNC_SELF_TELEMETRY_MIC))
  {
    preferences.putInt(PREF_TNC_SELF_TELEMETRY_MIC, server.arg(PREF_TNC_SELF_TELEMETRY_MIC).toInt());
  }
  if (server.hasArg(PREF_TNC_SELF_TELEMETRY_PATH))
  {
    preferences.putString(PREF_TNC_SELF_TELEMETRY_PATH, server.arg(PREF_TNC_SELF_TELEMETRY_PATH));
  }

  // Smart Beaconing settings
  if (server.hasArg(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET))
  {
    preferences.putInt(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET, server.arg(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET).toInt());
  }
  if (server.hasArg(PREF_APRS_SB_MIN_INTERVAL_PRESET))
  {
    preferences.putInt(PREF_APRS_SB_MIN_INTERVAL_PRESET, server.arg(PREF_APRS_SB_MIN_INTERVAL_PRESET).toInt());
  }
  if (server.hasArg(PREF_APRS_SB_MAX_INTERVAL_PRESET))
  {
    preferences.putInt(PREF_APRS_SB_MAX_INTERVAL_PRESET, server.arg(PREF_APRS_SB_MAX_INTERVAL_PRESET).toInt());
  }
  if (server.hasArg(PREF_APRS_SB_MIN_SPEED_PRESET))
  {
    preferences.putInt(PREF_APRS_SB_MIN_SPEED_PRESET, server.arg(PREF_APRS_SB_MIN_SPEED_PRESET).toInt());
  }
  if (server.hasArg(PREF_APRS_SB_MAX_SPEED_PRESET))
  {
    preferences.putInt(PREF_APRS_SB_MAX_SPEED_PRESET, server.arg(PREF_APRS_SB_MAX_SPEED_PRESET).toInt());
  }
  if (server.hasArg(PREF_APRS_SB_ANGLE_PRESET))
  {
    preferences.putDouble(PREF_APRS_SB_ANGLE_PRESET, server.arg(PREF_APRS_SB_ANGLE_PRESET).toDouble());
  }

  preferences.putBool(PREF_APRS_SHOW_BATTERY, server.hasArg(PREF_APRS_SHOW_BATTERY));
  preferences.putBool(PREF_ENABLE_TNC_SELF_TELEMETRY, server.hasArg(PREF_ENABLE_TNC_SELF_TELEMETRY));
  preferences.putBool(PREF_APRS_SHOW_ALTITUDE, server.hasArg(PREF_APRS_SHOW_ALTITUDE));
  preferences.putBool(PREF_APRS_FIXED_BEACON_PRESET, server.hasArg(PREF_APRS_FIXED_BEACON_PRESET));
  preferences.putBool(PREF_APRS_GPS_EN, server.hasArg(PREF_APRS_GPS_EN));
  preferences.putBool(PREF_APRS_SHOW_CMT, server.hasArg(PREF_APRS_SHOW_CMT));

  server.sendHeader("Location", "/");
  server.send(302, "text/html", "");
}

void handle_saveDeviceCfg()
{
  preferences.putBool(PREF_DEV_BT_EN, server.hasArg(PREF_DEV_BT_EN));
  preferences.putBool(PREF_DEV_OL_EN, server.hasArg(PREF_DEV_OL_EN));
  if (server.hasArg(PREF_DEV_SHOW_RX_TIME))
  {
    preferences.putInt(PREF_DEV_SHOW_RX_TIME, server.arg(PREF_DEV_SHOW_RX_TIME).toInt());
  }
  // Manage OLED Timeout
  if (server.hasArg(PREF_DEV_SHOW_OLED_TIME))
  {
    preferences.putInt(PREF_DEV_SHOW_OLED_TIME, server.arg(PREF_DEV_SHOW_OLED_TIME).toInt());
  }
  preferences.putBool(PREF_DEV_AUTO_SHUT, server.hasArg(PREF_DEV_AUTO_SHUT));
  if (server.hasArg(PREF_DEV_AUTO_SHUT_PRESET))
  {
    preferences.putInt(PREF_DEV_AUTO_SHUT_PRESET, server.arg(PREF_DEV_AUTO_SHUT_PRESET).toInt());
  }
  server.sendHeader("Location", "/");
  server.send(302, "text/html", "");
}

[[noreturn]] void taskWebServer(void *parameter)
{
  auto *webServerCfg = (tWebServerCfg *)parameter;
  apSSID = webServerCfg->callsign + " AP";

  server.on("/", handle_Index);
  server.on("/favicon.ico", handle_NotFound);
  server.on("/style.css", handle_Style);
  server.on("/js.js", handle_Js);
  server.on("/scan_wifi", handle_ScanWifi);
  server.on("/save_wifi_cfg", handle_SaveWifiCfg);
  server.on("/reboot", handle_Reboot);
  server.on("/beacon", handle_Beacon);
  server.on("/shutdown", handle_Shutdown);
  server.on("/cfg", handle_Cfg);
  server.on("/received_list", handle_ReceivedList);
  server.on("/save_aprs_cfg", handle_SaveAPRSCfg);
  server.on("/save_device_cfg", handle_saveDeviceCfg);
  server.on("/restore", handle_Restore);
  server.on("/update", HTTP_POST, []()
            {
    syslog_log(LOG_WARNING, String("Update finished. Status: ") + (!Update.hasError() ? "Ok" : "Error"));
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    delay(500);
    ESP.restart(); }, []()
            {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      rf95.sleep(); // disable rf95 before update
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        syslog_log(LOG_ERR, String("Update begin error: ") + Update.errorString());
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        syslog_log(LOG_ERR, String("Update error: ") + Update.errorString());
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        syslog_log(LOG_WARNING, String("Update Success: ") + String((int)upload.totalSize));
      } else {
        syslog_log(LOG_ERR, String("Update error: ") + Update.errorString());
        Update.printError(Serial);
      }
    } });
  server.onNotFound(handle_NotFound);

  String wifi_password = preferences.getString(PREF_WIFI_PASSWORD);
  String wifi_ssid = preferences.getString(PREF_WIFI_SSID);
  if (preferences.getString(PREF_AP_PASSWORD).length() > 8)
  {
    // 8 characters is requirements for WPA2
    apPassword = preferences.getString(PREF_AP_PASSWORD);
  }
  else
  {
    apPassword = defApPassword;
  }
  if (!wifi_ssid.length())
  {
    WiFi.softAP(apSSID.c_str(), apPassword.c_str());
  }
  else
  {
    int retryWifi = 0;
    WiFi.begin(wifi_ssid.c_str(), wifi_password.length() ? wifi_password.c_str() : nullptr);
    Serial.println("Connecting to " + wifi_ssid);
    // Set power to minimum (max 20)
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html
    esp_wifi_set_max_tx_power(8);
    while (WiFi.status() != WL_CONNECTED)
    {
      Serial.print("Not connected: ");
      Serial.println((int)WiFi.status());
      Serial.print("Retry: ");
      Serial.println(retryWifi);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      retryWifi += 1;
      if (retryWifi > 60)
      {
        WiFi.softAP(apSSID.c_str(), apPassword.c_str());
        // WiFi.softAP(apSSID.c_str(), "password");
        Serial.println("Unable to connect to to wifi. Starting AP");
        Serial.print("SSID: ");
        Serial.print(apSSID.c_str());
        Serial.print(" Password: ");
        Serial.println(apPassword.c_str());
        // Set power to minimum (max 20)
        esp_wifi_set_max_tx_power(8);
        break;
      }
    }

    // Serial.print("WiFi Mode: ");
    // Serial.println(WiFi.getMode());
    if (WiFi.getMode() == 3)
    {
      Serial.println("Running AP. IP: " + WiFi.softAPIP().toString());
      apEnabled = true;
      infoApName = apSSID.c_str();
      infoApPass = apSSID.c_str();
      infoApAddr = WiFi.softAPIP().toString();
    }
    else if (WiFi.getMode() == 1)
    {
      // Save some battery
      // WiFi.setSleep(true);
      esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
      Serial.println("Connected. IP: " + WiFi.localIP().toString());
      apConnected = true;
      infoApName = wifi_ssid.c_str();
      infoApPass = wifi_password.c_str();
      infoApAddr = WiFi.localIP().toString();
    }
    else
    {
      Serial.println("WiFi Mode: " + WiFi.getMode());
    }

#ifdef ENABLE_SYSLOG
    syslog.server(SYSLOG_IP, 514);
    syslog.deviceHostname(webServerCfg->callsign.c_str());
    syslog.appName("TTGO");
    syslog.defaultPriority(LOG_KERN);
    syslog_log(LOG_INFO, "Connected. IP: " + WiFi.localIP().toString());
#endif
    configTime(0, 0, "pool.ntp.org");
#ifdef ENABLE_SYSLOG
    struct tm timeinfo
    {
    };
    if (!getLocalTime(&timeinfo))
    {
      syslog_log(LOG_WARNING, "Failed to obtain time");
    }
    else
    {
      char buf[64];
      strftime(buf, 64, "%A, %B %d %Y %H:%M:%S", &timeinfo);
      syslog_log(LOG_INFO, String("Time: ") + String(buf));
    }
#endif
  }

  server.begin();
#ifdef KISS_PROTOCOL
  tncServer.begin();
#endif
  gpsServer.begin();
  if (MDNS.begin(webServerCfg->callsign.c_str()))
  {
    MDNS.setInstanceName(webServerCfg->callsign + " TTGO LoRa APRS TNC " + TXFREQ + "MHz");
    MDNS.addService("http", "tcp", 80);
#ifdef KISS_PROTOCOL
    MDNS.addService("kiss-tnc", "tcp", NETWORK_TNC_PORT);
#endif
  }

  webListReceivedQueue = xQueueCreate(4, sizeof(tReceivedPacketData *));

  tReceivedPacketData *receivedPacketData = nullptr;

  while (true)
  {
    server.handleClient();
    if (xQueueReceive(webListReceivedQueue, &receivedPacketData, (1 / portTICK_PERIOD_MS)) == pdPASS)
    {
      auto *receivedPacketToQueue = new tReceivedPacketData();
      receivedPacketToQueue->packet = new String();
      receivedPacketToQueue->packet->concat(*receivedPacketData->packet);
      receivedPacketToQueue->RSSI = receivedPacketData->RSSI;
      receivedPacketToQueue->SNR = receivedPacketData->SNR;
      receivedPacketToQueue->rxTime = receivedPacketData->rxTime;
      receivedPackets.push_back(receivedPacketToQueue);
      if (receivedPackets.size() > MAX_RECEIVED_LIST_SIZE)
      {
        auto *packetDataToDelete = receivedPackets.front();
        delete packetDataToDelete->packet;
        delete packetDataToDelete;
        receivedPackets.pop_front();
      }
      delete receivedPacketData->packet;
      delete receivedPacketData;
    }

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}
