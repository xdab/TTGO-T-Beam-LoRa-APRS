#ifndef PINS_H
#define PINS_H

// Pins for GPS
#ifdef T_BEAM_V1_2
#define PIN_GPS_RX 12
#define PIN_GPS_TX 34
#elif T_BEAM_V1_0
#define PIN_GPS_RX 12
#define PIN_GPS_TX 34
#else
#define PIN_GPS_RX 15
#define PIN_GPS_TX 12
#endif

// SPI config
#define SPI_sck 5
#define SPI_miso 19
#define SPI_mosi 27
#define SPI_ss 18

// IO config
#ifdef T_BEAM_V1_2
#define XPOWERS_CHIP_AXP2101
#define I2C_SDA 21
#define I2C_SCL 22
#define BUTTON 38 // pin number for Button on TTGO T-Beam
#define BUZZER 15 // enter your buzzer pin gpio
#define TXLED 4
#elif T_BEAM_V1_0
#define XPOWERS_CHIP_AXP192
#define I2C_SDA 21
#define I2C_SCL 22
#define BUTTON 38 // pin number for Button on TTGO T-Beam
#define BUZZER 15 // enter your buzzer pin gpio
#define TXLED 4   // pin number for LED on TX Tracker
#elif T_BEAM_V0_7
#define I2C_SDA 21
#define I2C_SCL 22
#define BUTTON 39 // pin number for Button on TTGO T-Beam
#define BUZZER 15 // enter your buzzer pin gpio
#define TXLED 4   // pin number for LED on TX Tracker
#elif LORA32_21   // Modified as in #47
#define I2C_SDA 21
#define I2C_SCL 22
#define BUTTON 2  // pin number for BUTTO
#define BUZZER 13 // enter your buzzer pin gpio
#define TXLED 4   // pin number for LED on TX Tracker
#elif LORA32_2
#define I2C_SDA 21
#define I2C_SCL 22
#define BUTTON 2  // pin number for BUTTO
#define BUZZER 13 // enter your buzzer pin gpio
#define TXLED 4   // pin number for LED on TX Tracker
#elif LORA32_1
#define I2C_SDA 21
#define I2C_SCL 22
#define BUTTON 2  // pin number for BUTTO
#define BUZZER 13 // enter your buzzer pin gpio
#define TXLED 4   // pin number for LED on TX Tracker
#elif HELTEC_V1
#define I2C_SDA 4
#define I2C_SCL 15
#define BUTTON 2  // pin number for BUTTO
#define BUZZER 13 // enter your buzzer pin gpio
#define TXLED 4   // pin number for LED on TX Tracker
#elif HELTEC_V2
#define I2C_SDA 4
#define I2C_SCL 15
#define BUTTON 2  // pin number for BUTTO
#define BUZZER 13 // enter your buzzer pin gpio
#define TXLED 4   // pin number for LED on TX Tracker
#endif

// Pins and details for OLED
#define OLED_RESET 16 // not used
#define SSD1306_ADDRESS 0x3C

// For radio driver
#define PIN_NSS 18
#define PIN_IO_INT 26

#endif // PINS_H