#ifndef IMAGES_H
#define IMAGES_H

#include <Arduino.h>

#define SATELLITE_IMAGE_WIDTH 16
#define SATELLITE_IMAGE_HEIGHT 15
const uint8_t SATELLITE_IMAGE[] PROGMEM = {0x00, 0x08, 0x00, 0x1C, 0x00, 0x0E, 0x20, 0x07, 0x70, 0x02,
                                           0xF8, 0x00, 0xF0, 0x01, 0xE0, 0x03, 0xC8, 0x01, 0x9C, 0x54,
                                           0x0E, 0x52, 0x07, 0x48, 0x02, 0x26, 0x00, 0x10, 0x00, 0x0E};

const uint8_t imgSatellite[] PROGMEM = {0x70, 0x71, 0x22, 0xFA, 0xFA, 0x22, 0x71, 0x70};
const uint8_t imgUSB[] PROGMEM = {0x60, 0x60, 0x30, 0x18, 0x18, 0x18, 0x24, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x24, 0x24, 0x24, 0x3C};
const uint8_t imgPower[] PROGMEM = {0x40, 0x40, 0x40, 0x58, 0x48, 0x08, 0x08, 0x08, 0x1C, 0x22, 0x22, 0x41, 0x7F, 0x22, 0x22, 0x22};
const uint8_t imgUser[] PROGMEM = {0x3C, 0x42, 0x99, 0xA5, 0xA5, 0x99, 0x42, 0x3C};
const uint8_t imgPositionEmpty[] PROGMEM = {0x20, 0x30, 0x28, 0x24, 0x42, 0xFF};
const uint8_t imgPositionSolid[] PROGMEM = {0x20, 0x30, 0x38, 0x3C, 0x7E, 0xFF};
const uint8_t imgInfo[] PROGMEM = {0xFF, 0x81, 0x81, 0xB5, 0xB5, 0x81, 0x81, 0xFF};

// We now programmatically draw our compass
#if 0
const
#include "img/compass.xbm"
#endif

#if 0
const uint8_t activeSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00011000,
    B00100100,
    B01000010,
    B01000010,
    B00100100,
    B00011000
};

const uint8_t inactiveSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B00000000,
    B00000000
};
#endif

#endif // IMAGES_H