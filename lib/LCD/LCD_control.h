#ifndef LCDc
#define LCDc

#if ARDUINO >= 100
#include "Arduino.h"
#endif

#include "globals.h"
#include <LiquidCrystal_I2C.h>

class LCD_control {
public:
    LCD_control();
    void begin();
    void display();
    String Units[6] = {" cm  ", " U/min  ", " L/min  ", " L/h  ", " C  ", " %  "};
    char SendStr[17];
    uint16_t lastSP[6] = {0,0,0,0,0,0};
    uint16_t lastVal[6] = {0,0,0,0,0,0};
    boolean init = true;
    boolean alarmprinted[6] = {false,false,false,false,false,false};

private:
    LiquidCrystal_I2C LCD[6] = { LiquidCrystal_I2C(DISPLAY0, 16, 2), LiquidCrystal_I2C(DISPLAY1, 16, 2), LiquidCrystal_I2C(DISPLAY2, 16, 2), LiquidCrystal_I2C(DISPLAY3, 16, 2), LiquidCrystal_I2C(DISPLAY4, 16, 2), LiquidCrystal_I2C(DISPLAY5, 16, 2) };
    uint16_t convert_value(uint16_t val, uint8_t idx);
};

#endif