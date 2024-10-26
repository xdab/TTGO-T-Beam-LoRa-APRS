// Tracker for LoRA APRS
// from OE1ACM and OE3CJB redesigned by SQ9MDD
// KISS ans Bluetooth by SQ5RWU
// TTGO T-Beam v1.0 only
//
// licensed under CC BY-NC-SA

#include <Arduino.h>
#include <SPI.h>
#include <BG_RF95.h> // library from OE1ACM
#include <math.h>
#include <driver/adc.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>
#include "version.h"
#include "preference_storage.h"
#include "syslog_log.h"
#include "XPowersLib.h"
#include <pins.h>
#include <gps.h>

// Tasks
#include "task_gps.h"
#ifdef KISS_PROTOCOL
#include "taskTNC.h"
#endif
#ifdef ENABLE_WIFI
#include "taskWebServer.h"
#endif

// Variables for LoRa settings
ulong lora_speed = 1200;
double lora_freq = 434.855;

// Variables for APRS packaging
String Tcall; // your Call Sign for normal position reports
String aprsSymbolTable = APRS_SYMBOL_TABLE;
String aprsSymbol = APRS_SYMBOL;
String relay_path;
String aprsComment = MY_COMMENT;
String aprsLatPreset = LATIDUDE_PRESET;
String aprsLonPreset = LONGITUDE_PRESET;
boolean key_up = true;
boolean t_lock = false;
boolean fixed_beacon_enabled = false;
boolean show_cmt = true;
// Telemetry sequence, current value
int tel_sequence;
// Telemetry path
String tel_path;

#ifdef SHOW_ALT
boolean showAltitude = true;
#else
boolean showAltitude = false;
#endif
#ifdef SHOW_BATT
boolean showBattery = true;
#else
boolean showBattery = false;
#endif
#ifdef ENABLE_TNC_SELF_TELEMETRY
boolean enable_tel = true;
#else
boolean enable_tel = false;
#endif
// Telemetry interval, seconds
#ifdef TNC_SELF_TELEMETRY_INTERVAL
int tel_interval = TNC_SELF_TELEMETRY_INTERVAL;
#else
int tel_interval = 3600;
#endif
#ifdef TNC_SELF_TELEMETRY_MIC
int tel_mic = 1; // telemetry as "T#MIC"
#else
int tel_mic = 0; // telemetry as "T#001"
#endif
#ifdef ENABLE_BLUETOOTH
boolean enable_bluetooth = true;
#else
boolean enable_bluetooth = false;
#endif
#ifdef ENABLE_OLED
boolean enabled_oled = true;
#else
boolean enabled_oled = false;
#endif

// Variables and Constants
String loraReceivedFrameString = ""; // data on buff is copied to this string
String Outputstring = "";
String outString = ""; // The new Output String with GPS Conversion RAW
String LongShown = "";
String LatShown = "";
String LongFixed = "";
String LatFixed = "";

#if defined(ENABLE_TNC_SELF_TELEMETRY) && defined(KISS_PROTOCOL)
time_t nextTelemetryFrame;
#endif

// byte arrays
byte lora_TXBUFF[BG_RF95_MAX_MESSAGE_LEN]; // buffer for packet to send
byte lora_RXBUFF[BG_RF95_MAX_MESSAGE_LEN]; // buffer for packet to send

// byte Variables
byte lora_TXStart;       // start of packet data in TXbuff
byte lora_TXEnd;         // end of packet data in TXbuff
byte lora_FTXOK;         // flag, set to 1 if TX OK
byte lora_TXPacketType;  // type number of packet to send
byte lora_TXDestination; // destination address of packet to send
byte lora_TXSource;      // source address of packet received
byte lora_FDeviceError;  // flag, set to 1 if RFM98 device error
byte lora_TXPacketL;     // length of packet to send, includes source, destination and packet type.

unsigned long lastTX = 0L;
float ConvertToV;
float BattVolts;
float InpVolts;

// variables for smart beaconing
ulong sb_min_interval = 60000L;
ulong sb_max_interval = 360000L;
float sb_min_speed = 0;
float sb_max_speed = 30;
float sb_angle = 30; // angle to send packet at smart beaconing

float average_speed[5] = {0, 0, 0, 0, 0}, average_speed_final = 0;
float old_course = 0, new_course = 0;
int point_avg_speed = 0, point_avg_course = 0;

ulong nextTX = 60000L; // preset time period between TX = 60000ms = 60secs = 1min

ulong time_to_refresh = 0;
ulong next_fixed_beacon = 0;
ulong fix_beacon_interval = FIX_BEACON_INTERVAL;
ulong showRXTime = SHOW_RX_TIME;
ulong time_delay = 0;
ulong shutdown_delay = 0;
ulong shutdown_delay_time = 10000;
ulong shutdown_countdown_timer = 0;
boolean shutdown_active = true;
boolean shutdown_countdown_timer_enable = false;
boolean shutdown_usb_status_bef = false;

// Variables required to Power Save OLED
// With "Display dimmer enabled" it will turn OLED off after some time
// if the checkbox is disabled the display stays OFF
uint oled_timeout = SHOW_OLED_TIME; // OLED Timeout
bool tempOled = true;               // Turn ON OLED at first startup
ulong oled_timer;

bool manBeacon = false;

// Variable to show AP settings on OLED
bool apEnabled = false;
bool apConnected = false;
String infoApName = "";
String infoApPass = "";
String infoApAddr = "";

#define ANGLE_AVGS 3 // angle averaging - x times
float average_course[ANGLE_AVGS];
float avg_c_y, avg_c_x;

uint8_t txPower = TXdbmW;

#ifdef ENABLE_WIFI
tWebServerCfg webServerCfg;
#endif

static const adc_atten_t atten = ADC_ATTEN_DB_6;
static const adc_unit_t unit = ADC_UNIT_1;
#ifdef T_BEAM_V1_0
bool pmu_flag = 0;
XPowersAXP192 PMU;

void setFlag(void)
{
  pmu_flag = true;
}
#elif T_BEAM_V1_2 // PMU TBEAM 1.2
bool pmu_flag = 0;
XPowersAXP2101 PMU;

void setFlag(void)
{
  pmu_flag = true;
}
#endif

// checkRX
uint8_t loraReceivedLength = sizeof(lora_RXBUFF);

// Singleton instance of the radio driver
BG_RF95 rf95(PIN_NSS, PIN_IO_INT);

// initialize OLED display
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

// + FUNCTIONS-----------------------------------------------------------+//

char *ax25_base91enc(char *s, uint8_t n, uint32_t v)
{
  /* Creates a Base-91 representation of the value in v in the string */
  /* pointed to by s, n-characters long. String length should be n+1. */
  for (s += n, *s = '\0'; n; n--)
  {
    *(--s) = v % 91 + 33;
    v /= 91;
  }
  return (s);
}

#if defined(KISS_PROTOCOL)
/**
 *
 * @param TNC2FormatedFrame
 */
void sendToTNC(const String &TNC2FormatedFrame)
{
  if (tncToSendQueue)
  {
    auto *buffer = new String();
    buffer->concat(TNC2FormatedFrame);
    if (xQueueSend(tncReceivedQueue, &buffer, (1000 / portTICK_PERIOD_MS)) != pdPASS)
    {
      // remove buffer on error
      delete buffer;
    }
  }
}
#endif
void enablepins()
{
#ifdef T_BEAM_V1_2
  PMU.enableBattDetection();
  PMU.enableVbusVoltageMeasure();
  PMU.enableBattVoltageMeasure();
  PMU.enableSystemVoltageMeasure();
#elif T_BEAM_V1_0
  PMU.enableBattDetection();
  PMU.enableVbusVoltageMeasure();
  PMU.enableBattVoltageMeasure();
  PMU.enableSystemVoltageMeasure();
#elif T_BEAM_V0_7 /*        \
  adcAttachPin(35);         \
  adcStart(35);             \
  analogReadResolution(10); \
  analogSetAttenuation(ADC_6db); */
  pinMode(35, INPUT);
  // adc1_config_width(ADC_WIDTH_BIT_12);
  // adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_11);
#endif
}
void prepareAPRSFrame()
{
  String helper;
  String Altx;
  String Speedx, Coursex;
  char helper_base91[] = {"0000\0"};
  double Tlat = 52.0000, Tlon = 20.0000;
  double Tspeed = 0, Tcourse = 0;
  uint32_t aprs_lat, aprs_lon;
  int i;
  int Talt;
  Tlat = gps.latitude();
  Tlon = gps.longitude();
  Tcourse = gps.course();
  Tspeed = gps.speed_knots();
  aprs_lat = 900000000 - Tlat * 10000000;
  aprs_lat = aprs_lat / 26 - aprs_lat / 2710 + aprs_lat / 15384615;
  aprs_lon = 900000000 + Tlon * 10000000 / 2;
  aprs_lon = aprs_lon / 26 - aprs_lon / 2710 + aprs_lon / 15384615;
  outString = "";
  outString += Tcall;

  if (relay_path.isEmpty())
  {
    outString += ">APRS:!";
  }
  else
  {
    outString += ">APRS," + relay_path + ":!";
  }

  if (gps.fixed())
  {
    outString += aprsSymbolTable;
    ax25_base91enc(helper_base91, 4, aprs_lat);
    for (i = 0; i < 4; i++)
    {
      outString += helper_base91[i];
    }
    ax25_base91enc(helper_base91, 4, aprs_lon);
    for (i = 0; i < 4; i++)
    {
      outString += helper_base91[i];
    }
    outString += aprsSymbol;
    ax25_base91enc(helper_base91, 1, (uint32_t)Tcourse / 4);
    outString += helper_base91[0];
    ax25_base91enc(helper_base91, 1, (uint32_t)(log1p(Tspeed) / 0.07696));
    outString += helper_base91[0];
    outString += "H";

    if (showAltitude)
    {
      Talt = gps.altitude_meters() * 3.28;
      Altx = Talt;
      outString += "/A=";
      for (i = 0; i < (6 - Altx.length()); ++i)
      {
        outString += "0";
      }
      outString += Talt;
    }
  }
  else
  { // fixed position not compresed
    outString += aprsLatPreset;
    outString += aprsSymbolTable;
    outString += aprsLonPreset;
    outString += aprsSymbol;
  }

  if (show_cmt)
  {
    outString += aprsComment;
  }

  if (showBattery)
  {
    outString += " ";
    outString += String(BattVolts, 2);
    outString += ("V");
  }

#ifdef KISS_PROTOCOL
  sendToTNC(outString);
#endif
}

#ifdef BUZZER
/**
 * Buzzer sound playback
 * @param melody - must be an array. Consisting of an even number of values. frequency and duration
 * @param array_size - number of elements in the array
 */
void buzzer(int *melody, int array_size)
{
  for (int i = 0; i < array_size; i += 2)
  {
    ledcWriteTone(0, *melody);
    melody++;
    delay(*melody);
    melody++;
  }
  ledcWriteTone(0, 0); // turn off buzzer
}
#endif

/**
 * Send message as APRS LoRa packet
 * @param lora_LTXPower
 * @param lora_FREQ
 * @param message
 */
void loraSend(byte lora_LTXPower, float lora_FREQ, const String &message)
{
#ifdef ENABLE_LED_SIGNALING
  digitalWrite(TXLED, LOW);
#endif
  lastTX = millis();

  int messageSize = min(message.length(), sizeof(lora_TXBUFF) - 1);
  message.toCharArray((char *)lora_TXBUFF, messageSize + 1, 0);
  if (lora_speed == 1200)
  {
    rf95.setModemConfig(BG_RF95::Bw125Cr47Sf512);
  }
  else if (lora_speed == 610)
  {
    rf95.setModemConfig(BG_RF95::Bw125Cr48Sf1024);
  }
  else if (lora_speed == 180)
  {
    rf95.setModemConfig(BG_RF95::Bw125Cr48Sf4096);
  }
  else if (lora_speed == 210)
  {
    rf95.setModemConfig(BG_RF95::Bw125Cr47Sf4096);
  }
  else if (lora_speed == 240)
  {
    rf95.setModemConfig(BG_RF95::Bw125Cr46Sf4096);
  }
  else
  {
    rf95.setModemConfig(BG_RF95::Bw125Cr45Sf4096);
  }
  rf95.setFrequency(lora_FREQ);
  rf95.setTxPower(lora_LTXPower);
  rf95.sendAPRS(lora_TXBUFF, messageSize);
  rf95.waitPacketSent();
#ifdef ENABLE_LED_SIGNALING
  digitalWrite(TXLED, HIGH);
#endif
}

void batt_read()
{
  ConvertToV = 1000;
#ifdef T_BEAM_V1_2
  BattVolts = PMU.getBattVoltage() / ConvertToV;
  InpVolts = PMU.getVbusVoltage() / ConvertToV;
#elif T_BEAM_V1_0
  BattVolts = PMU.getBattVoltage() / ConvertToV;
  InpVolts = PMU.getVbusVoltage() / ConvertToV;
#elif T_BEAM_V0_7
  BattVolts = (((float)analogRead(35) / 8192.0) * 2.0 * 3.3 * (1100.0 / 1000.0)) + 0.41; // fixed thanks to Luca IU2FRL
  // BattVolts = adc1_get_raw(ADC1_CHANNEL_7)/1000;
#else
  BattVolts = analogRead(35) * 7.221 / 4096;
#endif
  Serial.print(BattVolts);
}

void sendpacket()
{
#ifdef BUZZER
  int melody[] = {1000, 50, 800, 100};
  buzzer(melody, sizeof(melody) / sizeof(int));
#endif
  batt_read();
  prepareAPRSFrame();
  loraSend(txPower, lora_freq, outString); // send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
}

void writedisplaytext(String HeaderTxt, String Line1, String Line2, String Line3, String Line4, String Line5)
{
  batt_read();
  if (BattVolts < 3.5 && BattVolts > 3.3)
  {
#ifdef T_BEAM_V1_2
#ifdef ENABLE_LED_SIGNALING
    PMU.setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);
#endif
#elif T_BEAM_V1_0
#ifdef ENABLE_LED_SIGNALING
    PMU.setChargingLedMode(XPOWERS_CHG_LED_BLINK_1HZ);
#endif
#endif
  }
  else if (BattVolts <= 3.3)
  {
#ifdef T_BEAM_V1_2
    PMU.setChargingLedMode(XPOWERS_CHG_LED_BLINK_4HZ);
#elif T_BEAM_V1_0
    PMU.setChargingLedMode(XPOWERS_CHG_LED_BLINK_4HZ);
#endif
  }
  else if (BattVolts <= 3.0)
  {
#ifdef T_BEAM_V1_2
    // PMU.shutdown();
#elif T_BEAM_V1_0
    PMU.shutdown();
#endif
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(HeaderTxt);
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.println(Line1);
  display.setCursor(0, 26);
  display.println(Line2);
  display.setCursor(0, 36);
  display.println(Line3);
  display.setCursor(0, 46);
  display.println(Line4);
  display.setCursor(0, 56);
  display.println(Line5);
  if (!enabled_oled)
  { // disable oled
    display.dim(true);
  }
  display.display();
  time_to_refresh = millis() + showRXTime;
}

String getSatAndBatInfo()
{
  String line5;
  if (gps.enabled())
  {
    if (InpVolts > 4)
    {
      line5 = "SAT: " + String(gps.satellites()) + "  BAT: " + String(BattVolts, 1) + "V*";
    }
    else
    {
      line5 = "SAT: " + String(gps.satellites()) + "  BAT: " + String(BattVolts, 2) + "V";
    }
  }
  else
  {
    if (InpVolts > 4)
    {
      line5 = "SAT: X  BAT: " + String(BattVolts, 1) + "V*";
    }
    else
    {
      line5 = "SAT: X  BAT: " + String(BattVolts, 2) + "V";
    }
  }
#if defined(ENABLE_BLUETOOTH) && defined(KISS_PROTOCOL)
  if (SerialBT.hasClient())
  {
    line5 += "BT";
  }
#endif
  return line5;
}

void displayInvalidGPS()
{
  char *nextTxInfo;
  if (!gps.enabled())
  {
    nextTxInfo = (char *)"TX: GPS disabled";
  }
  else
  {
    nextTxInfo = (char *)"TX: at valid GPS";
  }
  writedisplaytext(Tcall, nextTxInfo, "LAT: not valid", "LON: not valid", "SPD: ---  CRS: ---", getSatAndBatInfo());
}

#if defined(ENABLE_WIFI)
/**
 *
 * @param TNC2FormatedFrame
 */
void sendToWebList(const String &TNC2FormatedFrame, const int RSSI, const int SNR)
{
  if (webListReceivedQueue)
  {
    auto *receivedPacketData = new tReceivedPacketData();
    receivedPacketData->packet = new String();
    receivedPacketData->packet->concat(TNC2FormatedFrame);
    receivedPacketData->RSSI = RSSI;
    receivedPacketData->SNR = SNR;
    getLocalTime(&receivedPacketData->rxTime);

    if (xQueueSend(webListReceivedQueue, &receivedPacketData, (1000 / portTICK_PERIOD_MS)) != pdPASS)
    {
      // remove buffer on error
      delete receivedPacketData->packet;
      delete receivedPacketData;
    }
  }
}
#endif

String prepareCallsign(const String &callsign)
{
  String tmpString = "";
  for (int i = 0; i < callsign.length(); ++i)
  { // remove unneeded "spaces" from callsign field
    if (callsign.charAt(i) != ' ')
    {
      tmpString += callsign.charAt(i);
    }
  }
  return tmpString;
}

#if defined(ENABLE_TNC_SELF_TELEMETRY) && defined(KISS_PROTOCOL)
void sendTelemetryFrame()
{
  if (enable_tel == true)
  {
#ifdef T_BEAM_V1_2
    uint8_t b_volt = (PMU.getBattVoltage() - 3000) / 5.1;
    uint8_t ac_volt = (PMU.getVbusVoltage() - 3000) / 28;
    // Pad telemetry message address to 9 characters
    char Tcall_message_char[9];
    sprintf_P(Tcall_message_char, "%-9s", Tcall);
    String Tcall_message = String(Tcall_message_char);
// Flash the light when telemetry is being sent
#ifdef ENABLE_LED_SIGNALING
    digitalWrite(TXLED, LOW);
#endif

    // Determine sequence number (or 'MIC')
    String tel_sequence_str;
    if (tel_mic == 1)
    {
      tel_sequence_str = "MIC";
    }
    else
    {
      // Get the current saved telemetry sequence
      tel_sequence = preferences.getUInt(PREF_TNC_SELF_TELEMETRY_SEQ, 0);
      // Pad to 3 digits
      char tel_sequence_char[3];
      sprintf_P(tel_sequence_char, "%03u", tel_sequence);
      tel_sequence_str = String(tel_sequence_char);
    }
    // Format telemetry path
    String tel_path_str;
    if (tel_path == "")
    {
      tel_path_str = tel_path;
    }
    else
    {
      tel_path_str = "," + tel_path;
    }

    String telemetryParamsNames = String(":") + Tcall_message + ":PARM.B Volt,AC V,";
    String telemetryUnitNames = String(":") + Tcall_message + ":UNIT.mV,mV,";
    String telemetryEquations = String(":") + Tcall_message + ":EQNS.0,5.1,3000,0,10,0,0,10,0,0,28,3000,0,10,0";
    String telemetryData = String("T#") + tel_sequence_str + "," + String(b_volt) + "," + String(ac_volt) + ",00000000";
    String telemetryBase = "";
    telemetryBase += Tcall + ">APRS" + tel_path_str + ":";
    Serial.print(telemetryBase);
    sendToTNC(telemetryBase + telemetryParamsNames);
    sendToTNC(telemetryBase + telemetryUnitNames);
    sendToTNC(telemetryBase + telemetryEquations);
    sendToTNC(telemetryBase + telemetryData);

    // Show when telemetry is being sent
    writedisplaytext("TEL TX", "", "", "", "", "");

// Flash the light when telemetry is being sent
#ifdef ENABLE_LED_SIGNALING
    digitalWrite(TXLED, HIGH);
#endif

    // Update the telemetry sequence number
    if (tel_sequence >= 999)
    {
      tel_sequence = 0;
    }
    else
    {
      tel_sequence = tel_sequence + 1;
    }
    preferences.putUInt(PREF_TNC_SELF_TELEMETRY_SEQ, tel_sequence);
#elif T_BEAM_V1_0
    uint8_t b_volt = (PMU.getBattVoltage() - 3000) / 5.1;
    uint8_t b_in_c = (PMU.getBatteryChargeCurrent()) / 10;
    uint8_t b_out_c = (PMU.getBattDischargeCurrent()) / 10;
    uint8_t ac_volt = (PMU.getVbusVoltage() - 3000) / 28;
    uint8_t ac_c = (PMU.getVbusCurrent()) / 10;
    // Pad telemetry message address to 9 characters
    char Tcall_message_char[9];
    sprintf_P(Tcall_message_char, "%-9s", Tcall);
    String Tcall_message = String(Tcall_message_char);
// Flash the light when telemetry is being sent
#ifdef ENABLE_LED_SIGNALING
    digitalWrite(TXLED, LOW);
#endif

    // Determine sequence number (or 'MIC')
    String tel_sequence_str;
    if (tel_mic == 1)
    {
      tel_sequence_str = "MIC";
    }
    else
    {
      // Get the current saved telemetry sequence
      tel_sequence = preferences.getUInt(PREF_TNC_SELF_TELEMETRY_SEQ, 0);
      // Pad to 3 digits
      char tel_sequence_char[3];
      sprintf_P(tel_sequence_char, "%03u", tel_sequence);
      tel_sequence_str = String(tel_sequence_char);
    }
    // Format telemetry path
    String tel_path_str;
    if (tel_path == "")
    {
      tel_path_str = tel_path;
    }
    else
    {
      tel_path_str = "," + tel_path;
    }

    String telemetryParamsNames = String(":") + Tcall_message + ":PARM.B Volt,B In,B Out,AC V,AC C";
    String telemetryUnitNames = String(":") + Tcall_message + ":UNIT.mV,mA,mA,mV,mA";
    String telemetryEquations = String(":") + Tcall_message + ":EQNS.0,5.1,3000,0,10,0,0,10,0,0,28,3000,0,10,0";
    String telemetryData = String("T#") + tel_sequence_str + "," + String(b_volt) + "," + String(b_in_c) + "," + String(b_out_c) + "," + String(ac_volt) + "," + String(ac_c) + ",00000000";
    String telemetryBase = "";
    telemetryBase += Tcall + ">APRS" + tel_path_str + ":";
    Serial.print(telemetryBase);
    sendToTNC(telemetryBase + telemetryParamsNames);
    sendToTNC(telemetryBase + telemetryUnitNames);
    sendToTNC(telemetryBase + telemetryEquations);
    sendToTNC(telemetryBase + telemetryData);

    // Show when telemetry is being sent
    writedisplaytext("TEL TX", "", "", "", "", "");

// Flash the light when telemetry is being sent
#ifdef ENABLE_LED_SIGNALING
    digitalWrite(TXLED, HIGH);
#endif

    // Update the telemetry sequence number
    if (tel_sequence >= 999)
    {
      tel_sequence = 0;
    }
    else
    {
      tel_sequence = tel_sequence + 1;
    }
    preferences.putUInt(PREF_TNC_SELF_TELEMETRY_SEQ, tel_sequence);
#endif
  }
}
#endif

// + SETUP --------------------------------------------------------------+//
void setup()
{

  SPI.begin(SPI_sck, SPI_miso, SPI_mosi, SPI_ss); // DO2JMG Heltec Patch
  Serial.begin(115200);

#ifdef BUZZER
  ledcSetup(0, 1E5, 12);
  ledcAttachPin(BUZZER, 0);
  ledcWriteTone(0, 0); // turn off buzzer on start
#endif

#ifdef DIGI_PATH
  relay_path = DIGI_PATH;
#else
  relay_path = "";
#endif

#ifdef FIXED_BEACON_EN
  fixed_beacon_enabled = true;
#endif

  // This section loads values from saved preferences,
  // if available.
  // https://randomnerdtutorials.com/esp32-save-data-permanently-preferences/

#ifdef ENABLE_PREFERENCES
  int clear_preferences = 0;
  if (digitalRead(BUTTON) == LOW)
  {
    clear_preferences = 1;
  }

  preferences.begin("cfg", false);

  // LoRa transmission settings

  if (!preferences.getBool(PREF_LORA_FREQ_PRESET_INIT))
  {
    preferences.putBool(PREF_LORA_FREQ_PRESET_INIT, true);
    preferences.putDouble(PREF_LORA_FREQ_PRESET, lora_freq);
  }
  lora_freq = preferences.getDouble(PREF_LORA_FREQ_PRESET);

  if (!preferences.getBool(PREF_LORA_SPEED_PRESET_INIT))
  {
    preferences.putBool(PREF_LORA_SPEED_PRESET_INIT, true);
    preferences.putInt(PREF_LORA_SPEED_PRESET, lora_speed);
  }
  lora_speed = preferences.getInt(PREF_LORA_SPEED_PRESET);

  // APRS station settings

  aprsSymbolTable = preferences.getString(PREF_APRS_SYMBOL_TABLE);
  if (aprsSymbolTable.isEmpty())
  {
    preferences.putString(PREF_APRS_SYMBOL_TABLE, APRS_SYMBOL_TABLE);
    aprsSymbolTable = preferences.getString(PREF_APRS_SYMBOL_TABLE);
  }

  aprsSymbol = preferences.getString(PREF_APRS_SYMBOL);
  if (aprsSymbol.isEmpty())
  {
    preferences.putString(PREF_APRS_SYMBOL, APRS_SYMBOL);
    aprsSymbol = preferences.getString(PREF_APRS_SYMBOL, APRS_SYMBOL);
  }

  if (!preferences.getBool(PREF_APRS_COMMENT_INIT))
  {
    preferences.putBool(PREF_APRS_COMMENT_INIT, true);
    preferences.putString(PREF_APRS_COMMENT, MY_COMMENT);
  }
  aprsComment = preferences.getString(PREF_APRS_COMMENT);

  if (!preferences.getBool(PREF_APRS_RELAY_PATH_INIT))
  {
    preferences.putBool(PREF_APRS_RELAY_PATH_INIT, true);
    preferences.putString(PREF_APRS_RELAY_PATH, DIGI_PATH);
  }
  relay_path = preferences.getString(PREF_APRS_RELAY_PATH);

  if (!preferences.getBool(PREF_APRS_SHOW_ALTITUDE_INIT))
  {
    preferences.putBool(PREF_APRS_SHOW_ALTITUDE_INIT, true);
    preferences.putBool(PREF_APRS_SHOW_ALTITUDE, showAltitude);
  }
  showAltitude = preferences.getBool(PREF_APRS_SHOW_ALTITUDE);

  if (!preferences.getBool(PREF_APRS_GPS_EN_INIT))
  {
    preferences.putBool(PREF_APRS_GPS_EN_INIT, true);
    preferences.putBool(PREF_APRS_GPS_EN, gps.enabled());
  }
  gps.set_state(preferences.getBool(PREF_APRS_GPS_EN));

  if (!preferences.getBool(PREF_APRS_SHOW_BATTERY_INIT))
  {
    preferences.putBool(PREF_APRS_SHOW_BATTERY_INIT, true);
    preferences.putBool(PREF_APRS_SHOW_BATTERY, showBattery);
  }
  showBattery = preferences.getBool(PREF_APRS_SHOW_BATTERY);

  if (!preferences.getBool(PREF_ENABLE_TNC_SELF_TELEMETRY_INIT))
  {
    preferences.putBool(PREF_ENABLE_TNC_SELF_TELEMETRY_INIT, true);
    preferences.putBool(PREF_ENABLE_TNC_SELF_TELEMETRY, enable_tel);
  }
  enable_tel = preferences.getBool(PREF_ENABLE_TNC_SELF_TELEMETRY);

  if (!preferences.getBool(PREF_TNC_SELF_TELEMETRY_INTERVAL_INIT))
  {
    preferences.putBool(PREF_TNC_SELF_TELEMETRY_INTERVAL_INIT, true);
    preferences.putInt(PREF_TNC_SELF_TELEMETRY_INTERVAL, tel_interval);
  }
  tel_interval = preferences.getInt(PREF_TNC_SELF_TELEMETRY_INTERVAL);

  if (!preferences.getBool(PREF_TNC_SELF_TELEMETRY_SEQ_INIT))
  {
    preferences.putBool(PREF_TNC_SELF_TELEMETRY_SEQ_INIT, true);
    preferences.putInt(PREF_TNC_SELF_TELEMETRY_SEQ, tel_sequence);
  }
  tel_sequence = preferences.getInt(PREF_TNC_SELF_TELEMETRY_SEQ);

  if (!preferences.getBool(PREF_TNC_SELF_TELEMETRY_MIC_INIT))
  {
    preferences.putBool(PREF_TNC_SELF_TELEMETRY_MIC_INIT, true);
    preferences.putInt(PREF_TNC_SELF_TELEMETRY_MIC, tel_mic);
  }
  tel_mic = preferences.getInt(PREF_TNC_SELF_TELEMETRY_MIC);

  if (!preferences.getBool(PREF_TNC_SELF_TELEMETRY_PATH_INIT))
  {
    preferences.putBool(PREF_TNC_SELF_TELEMETRY_PATH_INIT, true);
    preferences.putString(PREF_TNC_SELF_TELEMETRY_PATH, tel_path);
  }
  tel_path = preferences.getString(PREF_TNC_SELF_TELEMETRY_PATH);

  if (!preferences.getBool(PREF_APRS_LATITUDE_PRESET_INIT))
  {
    preferences.putBool(PREF_APRS_LATITUDE_PRESET_INIT, true);
    preferences.putString(PREF_APRS_LATITUDE_PRESET, LATIDUDE_PRESET);
  }
  aprsLatPreset = preferences.getString(PREF_APRS_LATITUDE_PRESET);

  if (!preferences.getBool(PREF_APRS_LONGITUDE_PRESET_INIT))
  {
    preferences.putBool(PREF_APRS_LONGITUDE_PRESET_INIT, true);
    preferences.putString(PREF_APRS_LONGITUDE_PRESET, LONGITUDE_PRESET);
  }
  aprsLonPreset = preferences.getString(PREF_APRS_LONGITUDE_PRESET);

  if (!preferences.getBool(PREF_APRS_FIXED_BEACON_PRESET_INIT))
  {
    preferences.putBool(PREF_APRS_FIXED_BEACON_PRESET_INIT, true);
    preferences.putBool(PREF_APRS_FIXED_BEACON_PRESET, fixed_beacon_enabled);
  }
  fixed_beacon_enabled = preferences.getBool(PREF_APRS_FIXED_BEACON_PRESET);

  if (!preferences.getBool(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET_INIT))
  {
    preferences.putBool(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET_INIT, true);
    preferences.putInt(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET, fix_beacon_interval / 1000);
  }
  fix_beacon_interval = preferences.getInt(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET) * 1000;

  // + SMART BEACONING

  if (!preferences.getBool(PREF_APRS_SB_MIN_INTERVAL_PRESET_INIT))
  {
    preferences.putBool(PREF_APRS_SB_MIN_INTERVAL_PRESET_INIT, true);
    preferences.putInt(PREF_APRS_SB_MIN_INTERVAL_PRESET, sb_min_interval / 1000);
  }
  sb_min_interval = preferences.getInt(PREF_APRS_SB_MIN_INTERVAL_PRESET) * 1000;

  if (!preferences.getBool(PREF_APRS_SB_MAX_INTERVAL_PRESET_INIT))
  {
    preferences.putBool(PREF_APRS_SB_MAX_INTERVAL_PRESET_INIT, true);
    preferences.putInt(PREF_APRS_SB_MAX_INTERVAL_PRESET, sb_max_interval / 1000);
  }
  sb_max_interval = preferences.getInt(PREF_APRS_SB_MAX_INTERVAL_PRESET) * 1000;

  if (!preferences.getBool(PREF_APRS_SB_MIN_SPEED_PRESET_INIT))
  {
    preferences.putBool(PREF_APRS_SB_MIN_SPEED_PRESET_INIT, true);
    preferences.putInt(PREF_APRS_SB_MIN_SPEED_PRESET, sb_min_speed);
  }
  sb_min_speed = preferences.getInt(PREF_APRS_SB_MIN_SPEED_PRESET);

  if (!preferences.getBool(PREF_APRS_SB_MAX_SPEED_PRESET_INIT))
  {
    preferences.putBool(PREF_APRS_SB_MAX_SPEED_PRESET_INIT, true);
    preferences.putInt(PREF_APRS_SB_MAX_SPEED_PRESET, sb_max_speed);
  }
  sb_max_speed = preferences.getInt(PREF_APRS_SB_MAX_SPEED_PRESET);

  if (!preferences.getBool(PREF_APRS_SB_ANGLE_PRESET_INIT))
  {
    preferences.putBool(PREF_APRS_SB_ANGLE_PRESET_INIT, true);
    preferences.putDouble(PREF_APRS_SB_ANGLE_PRESET, sb_angle);
  }
  sb_angle = preferences.getDouble(PREF_APRS_SB_ANGLE_PRESET);
  //

  if (!preferences.getBool(PREF_DEV_SHOW_RX_TIME_INIT))
  {
    preferences.putBool(PREF_DEV_SHOW_RX_TIME_INIT, true);
    preferences.putInt(PREF_DEV_SHOW_RX_TIME, showRXTime / 1000);
  }
  showRXTime = preferences.getInt(PREF_DEV_SHOW_RX_TIME) * 1000;

  // Read OLED RX Timer
  if (!preferences.getBool(PREF_DEV_SHOW_OLED_TIME_INIT))
  {
    preferences.putBool(PREF_DEV_SHOW_OLED_TIME_INIT, true);
    preferences.putInt(PREF_DEV_SHOW_OLED_TIME, oled_timeout / 1000);
  }
  oled_timeout = preferences.getInt(PREF_DEV_SHOW_OLED_TIME) * 1000;

  if (!preferences.getBool(PREF_DEV_AUTO_SHUT_PRESET_INIT))
  {
    preferences.putBool(PREF_DEV_AUTO_SHUT_PRESET_INIT, true);
    preferences.putInt(PREF_DEV_AUTO_SHUT_PRESET, shutdown_delay_time / 1000);
  }
  shutdown_delay_time = preferences.getInt(PREF_DEV_AUTO_SHUT_PRESET) * 1000;

  if (!preferences.getBool(PREF_DEV_AUTO_SHUT_INIT))
  {
    preferences.putBool(PREF_DEV_AUTO_SHUT_INIT, true);
    preferences.putBool(PREF_DEV_AUTO_SHUT, shutdown_active);
  }
  shutdown_active = preferences.getBool(PREF_DEV_AUTO_SHUT);

  if (clear_preferences)
  {
    delay(1000);
    if (digitalRead(BUTTON) == LOW)
    {
      clear_preferences = 2;
    }
  }

  if (!preferences.getBool(PREF_APRS_SHOW_CMT_INIT))
  {
    preferences.putBool(PREF_APRS_SHOW_CMT_INIT, true);
    preferences.putBool(PREF_APRS_SHOW_CMT, show_cmt);
  }
  show_cmt = preferences.getBool(PREF_APRS_SHOW_CMT);

  if (!preferences.getBool(PREF_DEV_BT_EN_INIT))
  {
    preferences.putBool(PREF_DEV_BT_EN_INIT, true);
    preferences.putBool(PREF_DEV_BT_EN, enable_bluetooth);
  }
  enable_bluetooth = preferences.getBool(PREF_DEV_BT_EN);

  if (!preferences.getBool(PREF_DEV_OL_EN_INIT))
  {
    preferences.putBool(PREF_DEV_OL_EN_INIT, true);
    preferences.putBool(PREF_DEV_OL_EN, enabled_oled);
  }
  enabled_oled = preferences.getBool(PREF_DEV_OL_EN);
#endif

  for (int i = 0; i < ANGLE_AVGS; i++)
  { // set average_course to "0"
    average_course[i] = 0;
  }

  pinMode(TXLED, OUTPUT);
#ifdef T_BEAM_V1_2
  pinMode(BUTTON, INPUT);
#elif T_BEAM_V1_0
  pinMode(BUTTON, INPUT);
#elif T_BEAM_V0_7
  pinMode(BUTTON, INPUT);
#else
  pinMode(BUTTON, INPUT_PULLUP);
#endif
  digitalWrite(TXLED, LOW); // turn blue LED off

  Wire.begin(I2C_SDA, I2C_SCL);

#ifdef T_BEAM_V1_2
  if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL))
  {
  }
  // axp.setLowTemp(0xFF);                                                 //SP6VWX Set low charging temperature need to convert
  PMU.setALDO2Voltage(3300);
  PMU.enableALDO2(); // LoRa
  gps.set_state(gps.enabled());
  if (gps.enabled())
  {
    PMU.enableALDO3(); // switch on GPS
  }
  else
  {
    PMU.disableALDO3(); // switch off GPS
  }
  PMU.enableDC2();
  // axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON); NC
  PMU.setDC1Voltage(3300);
  // Enable ADC to measure battery current, USB voltage etc.
  // axp.adc1Enable(0xfe, true);
  // axp.adc2Enable(0x80, true);
  PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
  PMU.enableDC1(); // oled do not turn off
#elif T_BEAM_V1_0
  if (!PMU.begin(Wire, AXP192_SLAVE_ADDRESS, I2C_SDA, I2C_SCL))
  {
  }
  // axp.setLowTemp(0xFF);                                                 //SP6VWX Set low charging temperature need to convert
  PMU.setLDO2Voltage(3300);
  PMU.enableLDO2(); // LoRa
  if (gps.enabled())
  {
    PMU.enableLDO3(); // switch on GPS
  }
  else
  {
    PMU.disableLDO3(); // switch off GPS
  }
  PMU.enableDC2();
  // axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON); NC
  PMU.setDC1Voltage(3300);
  // Enable ADC to measure battery current, USB voltage etc.
  // axp.adc1Enable(0xfe, true);
  // axp.adc2Enable(0x80, true);
  PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
  PMU.enableDC1(); // oled do not turn off
#endif

  if (!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_ADDRESS))
  {
    for (;;)
      ; // Don't proceed, loop forever
  }

#ifdef ENABLE_PREFERENCES
  if (clear_preferences == 2)
  {
    writedisplaytext("LoRa-APRS", "", "", "Factory reset", "", "");
    delay(1000);
    // #ifdef T_BEAM_V1_0
    if (digitalRead(BUTTON) == LOW)
    {
      clear_preferences = 3;
      preferences.clear();
      preferences.end();
      writedisplaytext("LoRa-APRS", "", "Factory reset", "Done!", "", "");
      delay(2000);
      ESP.restart();
    }
    else
    {
      writedisplaytext("LoRa-APRS", "", "Factory reset", "Cancel", "", "");
      delay(2000);
    }
    // #endif
  }
#endif
  writedisplaytext("LoRa-APRS", "", "Init:", "Display OK!", "", "");

  Tcall = prepareCallsign(String(CALLSIGN));
#ifdef ENABLE_PREFERENCES
  Tcall = preferences.getString(PREF_APRS_CALLSIGN);
  if (Tcall.isEmpty())
  {
    preferences.putString(PREF_APRS_CALLSIGN, String(CALLSIGN));
    Tcall = preferences.getString(PREF_APRS_CALLSIGN);
  }
#endif

  if (!rf95.init())
  {
    writedisplaytext("LoRa-APRS", "", "Init:", "RF95 FAILED!", ":-(", "");
    for (;;)
      ; // Don't proceed, loop forever
  }

  if (sb_max_interval < nextTX)
  {
    sb_max_interval = nextTX;
  }
  writedisplaytext("LoRa-APRS", "", "Init:", "RF95 OK!", "", "");
  writedisplaytext(" " + Tcall, "", "Init:", "Waiting for GPS", "", "");
  xTaskCreate(task_gps, "task_gps", 5000, nullptr, 1, nullptr);
  writedisplaytext(" " + Tcall, "", "Init:", "GPS Task Created!", "", "");
#ifndef T_BEAM_V1_0
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_6);
#endif
  batt_read();
  writedisplaytext("LoRa-APRS", "", "Init:", "ADC OK!", "BAT: " + String(BattVolts, 2), "");

  if (lora_speed == 1200)
  {
    rf95.setModemConfig(BG_RF95::Bw125Cr47Sf512);
  }
  else if (lora_speed == 610)
  {
    rf95.setModemConfig(BG_RF95::Bw125Cr48Sf1024);
  }
  else if (lora_speed == 180)
  {
    rf95.setModemConfig(BG_RF95::Bw125Cr48Sf4096);
  }
  else if (lora_speed == 210)
  {
    rf95.setModemConfig(BG_RF95::Bw125Cr47Sf4096);
  }
  else if (lora_speed == 240)
  {
    rf95.setModemConfig(BG_RF95::Bw125Cr46Sf4096);
  }
  else
  {
    rf95.setModemConfig(BG_RF95::Bw125Cr45Sf4096);
  }
  rf95.setFrequency(lora_freq);
  rf95.setTxPower(txPower);
  delay(250);

#ifdef KISS_PROTOCOL
  xTaskCreatePinnedToCore(taskTNC, "taskTNC", 10000, nullptr, 1, nullptr, xPortGetCoreID());
#endif

#if defined(KISS_PROTOCOL)
  if (enable_bluetooth)
  {
#ifdef BLUETOOTH_PIN
    SerialBT.setPin(BLUETOOTH_PIN);
#endif
#ifdef ENABLE_BLUETOOTH
    SerialBT.begin(String("TTGO LORA APRS ") + Tcall);
    writedisplaytext("LoRa-APRS", "", "Init:", "BT OK!", "", "");
#endif
  }
#endif

#ifdef ENABLE_WIFI
  webServerCfg = {.callsign = Tcall};
  xTaskCreate(taskWebServer, "taskWebServer", 12000, (void *)(&webServerCfg), 1, nullptr);
  writedisplaytext("LoRa-APRS", "", "Init:", "WiFi task started", "   =:-)   ", "");
#endif

  writedisplaytext("LoRa-APRS", "", "Init:", "FINISHED OK!", "   =:-)   ", "");
  writedisplaytext("", "", "", "", "", "");
  time_to_refresh = millis() + showRXTime;
  displayInvalidGPS();
  digitalWrite(TXLED, HIGH);

  // Hold the OLED ON at first boot
  oled_timer = millis() + oled_timeout;
}

void enableOled()
{
  // This function enables OLED display after pressing a button
  tempOled = true;
  oled_timer = millis() + oled_timeout;
}

// +---------------------------------------------------------------------+//
// + MAINLOOP -----------------------------------------------------------+//
// +---------------------------------------------------------------------+//
void loop()
{
  if (digitalRead(BUTTON) == LOW && key_up == true)
  {
    key_up = false;
    delay(50);
    if (digitalRead(BUTTON) == LOW)
    {
      delay(300);
      time_delay = millis() + 1500;
      if (digitalRead(BUTTON) == HIGH)
      {
        if (!tempOled && enabled_oled)
        {
          enableOled(); // turn ON OLED temporary
        }
        else
        {
          if (gps.fixed())
          {
            writedisplaytext("MAN TX", "", "", "", "", "");
            sendpacket();
          }
          else
          {
            writedisplaytext("FIX TX", "", "", "", "", "");
            sendpacket();
          }
        }
        key_up = true;
      }
    }
  }

  // Show informations on WiFi Status
  if (apConnected)
  {
    enableOled(); // turn ON OLED temporary
    writedisplaytext("WiFi", "Client Mode", "SSID: " + infoApName, "Pass: ********", "IP: " + infoApAddr, getSatAndBatInfo());
    apConnected = false;
  }
  else if (apEnabled)
  {
    enableOled(); // turn ON OLED temporary
    writedisplaytext("WiFi", "AP Mode", "SSID: " + infoApName, "Pass: " + infoApPass, "IP: " + infoApAddr, getSatAndBatInfo());
    apEnabled = false;
  }

  if (manBeacon)
  {
    // Manually sending beacon from html page
    enableOled();
    writedisplaytext("WEB TX", "", "", "", "", "");
    sendpacket();
    manBeacon = false;
  }
  // Only wake up OLED when necessary, note that DIM is to turn OFF the backlight
  if (enabled_oled)
  {
    if (oled_timeout > 0)
    {
      display.dim(!tempOled);
    }
    else
    {
      // If timeout is 0 keep OLED awake
      display.dim(false);
    }
  }

  if (tempOled && millis() >= oled_timer)
  {
    tempOled = false; // After some time reset backlight
  }

  if (digitalRead(BUTTON) == LOW && key_up == false && millis() >= time_delay && t_lock == false)
  {
    // enable OLED
    enableOled();
    //---------------
    t_lock = true;
    if (gps.enabled())
    {
      gps.set_state(false);
#ifdef T_BEAM_V1_2
      PMU.disableALDO3(); // GPS OFF
#elif T_BEAM_V1_0
      PMU.disableLDO3(); // GPS OFF
#endif
      writedisplaytext("GPS OFF", "", "", "", "", "");
      next_fixed_beacon = millis() + fix_beacon_interval;
#ifdef ENABLE_PREFERENCES
      preferences.putBool(PREF_APRS_GPS_EN, false);
#endif
    }
    else
    {
      gps.set_state(true);
#ifdef T_BEAM_v1_2
      PMU.enableALDO3(); // GPS ON
#elif T_BEAM_V1_0
      PMU.enableLDO3();
#endif
      writedisplaytext("GPS ON", "", "", "", "", ""); // GPS ON
#ifdef ENABLE_PREFERENCES
      preferences.putBool(PREF_APRS_GPS_EN, true);
#endif
    }
  }

  if (digitalRead(BUTTON) == HIGH && !key_up)
  {
    key_up = true;
    t_lock = false;
  }

  if (fixed_beacon_enabled)
  {
    if (millis() >= next_fixed_beacon && !gps.enabled())
    {
      enableOled(); // enable OLED
      next_fixed_beacon = millis() + fix_beacon_interval;
      writedisplaytext("TX BCN", "", "", "", "", "");
      sendpacket();
    }
  }

#ifdef T_BEAM_V1_2
  if (shutdown_active)
  {
    if (InpVolts > 4)
    {
      shutdown_usb_status_bef = true;
      shutdown_countdown_timer_enable = false;
    }

    if (InpVolts < 4 && shutdown_usb_status_bef == true)
    {
      shutdown_usb_status_bef = false;
      shutdown_countdown_timer_enable = true;
      shutdown_countdown_timer = millis() + shutdown_delay_time;
    }

    if (shutdown_countdown_timer_enable)
    {
      if (millis() >= shutdown_countdown_timer)
      {
        PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
        PMU.shutdown();
      }
    }
  }
#elif T_BEAM_V1_0
  if (shutdown_active)
  {
    if (InpVolts > 4)
    {
      shutdown_usb_status_bef = true;
      shutdown_countdown_timer_enable = false;
    }

    if (InpVolts < 4 && shutdown_usb_status_bef == true)
    {
      shutdown_usb_status_bef = false;
      shutdown_countdown_timer_enable = true;
      shutdown_countdown_timer = millis() + shutdown_delay_time;
    }

    if (shutdown_countdown_timer_enable)
    {
      if (millis() >= shutdown_countdown_timer)
      {
        PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
        PMU.shutdown();
      }
    }
  }
#endif

#ifdef KISS_PROTOCOL
  String *TNC2DataFrame = nullptr;
  if (tncToSendQueue)
  {
    if (xQueueReceive(tncToSendQueue, &TNC2DataFrame, (1 / portTICK_PERIOD_MS)) == pdPASS)
    {
      enableOled(); // enable OLED
      writedisplaytext("TX KISS", "", "", "", "", "");
      time_to_refresh = millis() + showRXTime;
      loraSend(txPower, lora_freq, *TNC2DataFrame);
      delete TNC2DataFrame;
    }
  }
#endif

  if (rf95.waitAvailableTimeout(100))
  {
#ifdef T_BEAM_V1_2
#ifdef ENABLE_LED_SIGNALING
    PMU.setChargingLedMode(XPOWERS_CHG_LED_ON);
#endif
#elif T_BEAM_V1_0
#ifdef ENABLE_LED_SIGNALING
    PMU.setChargingLedMode(XPOWERS_CHG_LED_ON);
#endif
#endif
#ifdef BUZZER
    int melody[] = {300, 50, 500, 100};
    buzzer(melody, sizeof(melody) / sizeof(int));
#endif
#ifdef SHOW_RX_PACKET                         // only show RX packets when activitated in config
    loraReceivedLength = sizeof(lora_RXBUFF); // reset max length before receiving!
    if (rf95.recvAPRS(lora_RXBUFF, &loraReceivedLength))
    {
      loraReceivedFrameString = "";
      // int rssi = rf95.lastSNR();
      // Serial.println(rssi);
      enableOled(); // enable OLED
      for (int i = 0; i < loraReceivedLength; i++)
      {
        loraReceivedFrameString += (char)lora_RXBUFF[i];
      }
      writedisplaytext("RX", "", loraReceivedFrameString, "", "", "");
#ifdef KISS_PROTOCOL
      sendToTNC(loraReceivedFrameString);
#endif
#ifdef ENABLE_WIFI
      sendToWebList(loraReceivedFrameString, rf95.lastRssi(), rf95.lastSNR());
#endif
      syslog_log(LOG_INFO, String("Received LoRa: '") + loraReceivedFrameString + "', RSSI:" + rf95.lastRssi() + ", SNR: " + rf95.lastSNR());
    }
#endif
#ifdef T_BEAM_V1_2
#ifdef ENABLE_LED_SIGNALING
    PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
#endif
#elif T_BEAM_V1_0
#ifdef ENABLE_LED_SIGNALING
    PMU.setChargingLedMode(XPOWERS_CHG_LED_OFF);
#endif
#endif
  }

  LatShown = String(gps.latitude(), 5);
  LongShown = String(gps.longitude(), 5);
  average_speed[point_avg_speed] = gps.speed_kmph(); // calculate smart beaconing
  ++point_avg_speed;
  if (point_avg_speed > 4)
  {
    point_avg_speed = 0;
  }
  average_speed_final = (average_speed[0] + average_speed[1] + average_speed[2] + average_speed[3] + average_speed[4]) / 5;
  nextTX = (sb_max_interval - sb_min_interval) / (sb_max_speed - sb_min_speed) * (sb_max_speed - average_speed_final) + sb_min_interval;
  if (nextTX < sb_min_interval)
  {
    nextTX = sb_min_interval;
  }
  if (nextTX > sb_max_interval)
  {
    nextTX = sb_max_interval;
  }
  average_course[point_avg_course] = gps.course(); // calculate smart beaconing course
  ++point_avg_course;
  if (point_avg_course > (ANGLE_AVGS - 1))
  {
    point_avg_course = 0;
    avg_c_y = 0;
    avg_c_x = 0;
    for (int i = 0; i < ANGLE_AVGS; i++)
    {
      avg_c_y += sin(average_course[i] / 180 * 3.1415);
      avg_c_x += cos(average_course[i] / 180 * 3.1415);
    }
    new_course = atan2f(avg_c_y, avg_c_x) * 180 / 3.1415;
    if (new_course < 0)
    {
      new_course = 360 + new_course;
    }
    if ((old_course < sb_angle) && (new_course > (360 - sb_angle)))
    {
      if (abs(new_course - old_course - 360) >= sb_angle)
      {
        nextTX = 1; // give one second for turn to finish and then TX
      }
    }
    else
    {
      if ((old_course > (360 - sb_angle)) && (new_course < sb_angle))
      {
        if (abs(new_course - old_course + 360) >= sb_angle)
        {
          nextTX = 1;
        }
      }
      else
      {
        if (abs(new_course - old_course) >= sb_angle)
        {
          nextTX = 1;
        }
      }
    }
    old_course = new_course;
  }
  if ((millis() < sb_max_interval) && (lastTX == 0))
  {
    nextTX = 0;
  }
  if ((lastTX + nextTX) <= millis())
  {
    if (gps.fix_age() < 2000)
    {
      enableOled(); // enable OLED
      writedisplaytext("TX GPS", "",
                       "LAT: " + LatShown, "LON: " + LongShown,
                       "SPD: " + String(gps.speed_kmph(), 1) + "  CRS: " + String(gps.course(), 1),
                       getSatAndBatInfo());
      sendpacket();
    }
    else
    {
      if (millis() > time_to_refresh)
      {
        displayInvalidGPS();
      }
    }
  }
  else
  {
    if (millis() > time_to_refresh)
    {
      if (gps.fix_age() < 2000)
      {
        writedisplaytext(Tcall,
                         "TX in: " + String(((lastTX + nextTX) - millis()) / 1000) + " s",
                         "LAT: " + LatShown, "LON: " + LongShown,
                         "SPD: " + String(gps.speed_kmph()) + "  CRS: " + String(gps.course(), 1),
                         getSatAndBatInfo());
      }
      else
      {
        displayInvalidGPS();
      }
    }
  }
#if defined(ENABLE_TNC_SELF_TELEMETRY) && defined(KISS_PROTOCOL)
  if (nextTelemetryFrame < millis())
  {
    // Schedule the next telemetry frame
    nextTelemetryFrame = millis() + (tel_interval * 1000);
    sendTelemetryFrame();
  }
#endif
#ifdef KISS_PROTOCOL
#ifdef KISS_DEBUG
  static auto last_debug_send_time = millis();
  if (millis() - last_debug_send_time > 1000 * 5)
  {
    last_debug_send_time = millis();
    String debug_message = "";
#ifdef T_BEAM_V1_2
    debug_message += "Bat V: " + String(PMU.getBattVoltage());
    debug_message += ", ";
    debug_message += "USB Plugged: " + String(PMU.isVbusInsertOnSource());
    debug_message += ", ";
    debug_message += "USB V: " + String(PMU.getVbusVoltage());
    debug_message += ", ";
    debug_message += "Temp C: " + String(PMU.getTemperature());
#elif T_BEAM_V1_0
    debug_message += "Bat V: " + String(PMU.getBattVoltage());
    debug_message += ", ";
    debug_message += "Bat IN A: " + String(PMU.getBatteryChargeCurrent());
    debug_message += ", ";
    debug_message += "Bat OUT A: " + String(PMU.getBattDischargeCurrent());
    debug_message += ", ";
    debug_message += "USB Plugged: " + String(PMU.isVBUSin());
    debug_message += ", ";
    debug_message += "USB V: " + String(PMU.getVbusVoltage());
    debug_message += ", ";
    debug_message += "USB A: " + String(PMU.getVbusCurrent());
    debug_message += ", ";
    debug_message += "Temp C: " + String(PMU.getTemperature());
#else
    debug_message += "Bat V: " + String(BattVolts);
#endif

    Serial.print(encapsulateKISS(debug_message, CMD_HARDWARE));
#ifdef ENABLE_BLUETOOTH
    SerialBT.print(encapsulateKISS(debug_message, CMD_HARDWARE));
#endif
  }
#endif
#endif
  vTaskDelay(1);
}
