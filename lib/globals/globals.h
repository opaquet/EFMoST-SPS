#if ARDUINO >= 100
#include "Arduino.h"
#endif

#ifndef GLOBAL
#define GLOBAL

// general definition
#define DEVICE   "EFMoST SPS"
#define VERSION  "1.0.2a"

// variable ranges
#define MinLevel    0       // cm
#define MaxLevel    150     // cm
#define MinRPM      0       // U/min
#define MaxRPM      384     // U/min
#define MinAiration 0       // L/min
#define MaxAiration 900     // L/min
#define MinFeed     0       // L/h
#define MaxFeed     378     // L/h
#define MinTemp     0       // 0,1 °C -> 0 °C
#define MaxTemp     500     // 0,1 °C -> 50 °C
#define MinConc     100     // 0,1 % -> 10 %
#define MaxConc     1000    // 0,1 % -> 100 %

// i2c adress definitions
#define DISPLAY0 0x20
#define DISPLAY1 0x21
#define DISPLAY2 0x22
#define DISPLAY3 0x23
#define DISPLAY4 0x24
#define DISPLAY5 0x27
#define i2c_Clock   400000L // 400 kHz should work? 

// serial connection
#define SERIAL0_BAUD_RATE 115200
#define SERIAL1_BAUD_RATE 19200


enum  {FluidLevel, FilterSpeed, Airation, FeedRate, Temp, ConcentrationFraction, Press1, Press2, Temp2, Temp3, Ox, H2S, pH, Conduct, Distance};

// ********* global Variables *********
extern uint16_t     g_analog_control_out[3];
extern uint16_t     g_digital_control_out;
extern boolean      g_auto_state[6];
extern boolean      g_control[7];
extern boolean      g_alarm[6];
extern boolean      g_direct_control;
extern uint16_t     g_ProcessState[15];
extern uint16_t     g_Setpoints[6];

#endif