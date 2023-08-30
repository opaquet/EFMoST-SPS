#ifndef EM_IO
#define EM_IO

#if ARDUINO >= 100
#include "Arduino.h"
#endif

#include "globals.h"
#include <MCP_DAC.h>

class IO {
public:
    IO();
    void begin();
    void digital_out(uint16_t values);
    void LED_out(uint32_t led_state);
    void analog_out(uint16_t * values);
    uint16_t * analog_in();
    uint16_t buttons_pressed_top, buttons_pressed_bottom;
    uint16_t buttons_pressed_top_last, buttons_pressed_bottom_last;
    uint16_t buttons_pressed_temp;

private:
    MCP4921 DAC[4] = {};

};

#endif