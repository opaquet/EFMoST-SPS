#if ARDUINO >= 100
#include "Arduino.h"
#endif

#ifndef GLOBAL
#define GLOBAL

// general definition
#define DEVICE   "EFMoST SPS"
#define VERSION  "1.0.9a"

// variable ranges
#define MinLevel    0       // cm
#define MaxLevel    120     // cm
#define MinTemp     0       // 0,1 °C -> 0 °C
#define MaxTemp     500     // 0,1 °C -> 50 °C
#define MinConc     100     // 0,1 % -> 10 %
#define MaxConc     1000    // 0,1 % -> 100 %

// i2c adress definitions
#define DISPLAY0 0x20
#define DISPLAY1 0x21
#define DISPLAY2 0x22
#define DISPLAY3 0x27
#define DISPLAY4 0x24
#define DISPLAY5 0x25
#define i2c_Clock   400000L // 400 kHz should work? 

#define BYTESWAP32(z) ((uint32_t)((z&0xFF)<<24|((z>>8)&0xFF)<<16|((z>>16)&0xFF)<<8|((z>>24)&0xFF)))
#define BYTESWAP16(z) ((uint16_t)((z&0xFF)<<8|((z>>8)&0xFF)))


// serial connection
#define SERIAL0_BAUD_RATE 115200
#define SERIAL1_BAUD_RATE 19200

#define FIFO_BUFFER_LENGHT 5


enum  {     FluidLevel, 
            FilterSpeed, 
            Airation, 
            FeedRate, 
            Temp, 
            ConcentrationFraction, 
            Press1, 
            Press2, 
            Temp2, 
            Temp3, 
            Ox, 
            H2S, 
            pH, 
            Conduct, 
            Distance,
            pOx,
            Foam
            };

// ********* global Variables *********
extern uint16_t     g_analog_control_out[3];
extern uint16_t     g_digital_control_out;
extern boolean      g_auto_state[6];
extern boolean      g_control[7];
extern boolean      g_alarm[6];
extern boolean      g_alarm_ignore[6];
extern boolean      g_direct_control;
extern boolean      g_foam;
extern boolean      g_state_changed;
extern uint16_t     g_foam_trigger_counter;
extern uint16_t     g_foam_trigger_counter_long;
extern uint16_t     g_ProcessState[17];
extern uint16_t     g_Setpoints[6];
extern uint16_t     g_errorCode;

/* error codes:
bit 0: measurement string recieved over serial 1 exceeded maximum buffer lenght and was discarded
bit 1: number of elements in measurement string not equal to expected lenght (1-23)
bit 2: measurement information too old (> 30 s) and therefore invalid 
bit 3: unknown Name/ID in measurement string or unable to interpret
bit 4: unknown serial command or unable to interpret
bit 5:
bit 6:
bit 7: pid saturation (0)
bit 8: pid saturation (max)
bit 9:
bit 10: Value conversion (measurement or setpoint) error -> index out of bounds.
bit 11:
bit 12:
bit 13:
bit 14:
bit 15:
*/

#endif