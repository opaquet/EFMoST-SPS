#if ARDUINO >= 100
#include "Arduino.h"
#endif

#ifndef GLOBAL
#define GLOBAL

// ********* Define statements *********
#define SERIAL_BAUD_RATE 115200
#define I2C_CLOCK_RATE 400000
#define LOOP_PERIOD 100

// i2c adress definitions
#define DISPLAY0 0x27
#define DISPLAY1 0x28
#define DISPLAY2 0x29
#define DISPLAY3 0x2A
#define DISPLAY4 0x2B
#define DISPLAY5 0x2C

// pin definitions
#define SR_DATA 15
#define SR_CLOCK 16
#define SR_LATCH 17

/*
Um Arbeitspeicher zu sparen benutzen wir Bitshifting und Bitfields wo immer möglich:
In 1 Byte kann ich so z.B. 8 Booleans oder 2 4-bit Ints unterbringen.
RAM ist mit 8192 Bytes recht knapp, und Rechenzeit, um die Bitfields auseinander zu frickeln ist bei 20 MHz eigentlich genug da.
*/

// global Variables
extern uint32_t g_buttons_LEDs; // pressed button bit set to 1 in timer interrupt handler
extern uint8_t g_analog_control_out[5];
extern uint16_t g_digital_control_out;
extern uint16_t g_analog_values[16]; // 6 poti values and 10 anaolg inputs or sensor readings

struct t_auto_state {
    boolean filling : 1;
    boolean rotation : 1;
    boolean airation : 1;
    boolean feeding : 1;
    boolean cooling : 1;
    boolean concentrating : 1;
};
extern t_auto_state g_auto_state; // 6 control outputs that can be set either manually or automatically (0 -> manual, 1 -> auto)

struct t_manual_control {
    boolean filling : 1;
    boolean rotation : 1;
    boolean rotation_reverse : 1;
    boolean airation : 1;
    boolean feeding : 1;
    boolean cooling : 1;
    boolean concentrating : 1;
};
extern t_manual_control g_manual_control; // 7 selected/desired manual control outputs

#endif