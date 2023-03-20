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
#define DISPLAY0 0x20
#define DISPLAY1 0x21
#define DISPLAY2 0x22
#define DISPLAY3 0x23
#define DISPLAY4 0x24
#define DISPLAY5 0x25

// pin definitions
#define SR_DATA 15
#define SR_CLOCK 16
#define SR_LATCH 17

// ********* global Variables *********
extern uint16_t g_analog_control_out[3];
extern uint16_t g_digital_control_out;
extern uint16_t g_analog_values[5]; // 6 poti values and 10 anaolg inputs or sensor readings
extern boolean g_auto_state[6];
extern boolean g_control[7];
extern boolean g_alarm[6];
extern boolean g_direct_control;

struct Measurements {
    float Temp1;
    float Temp2;
    float pH;
    float Conduct;
    float Ox1;
    float Ox2;
    float H2S;
    uint16_t Press1;
    uint16_t Press2;
    uint16_t FluidLevel;
    uint16_t Speed_FeedPump;
    uint16_t Speed_Filter;
    uint16_t Pos_AirationValve;
    uint16_t Concentration_Fraction;
};
extern Measurements g_ProcessState;

struct SetPoints {
    uint16_t FluidLevel;
    uint16_t Speed_Filter;
    uint16_t Rate_Airation;
    uint16_t Speed_FeedPump;
    uint16_t Temp;
    uint16_t Concentration_Fraction;
};
extern SetPoints g_Setpoints;

#endif