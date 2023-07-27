#ifndef S_COM
#define S_COM

#if ARDUINO >= 100
#include "Arduino.h"
#endif

#include "globals.h"

/*
All serial commands must start with "!"

Command List:
!SM(int ch, int mode)                -> sets the channel 1-6 into either manual (0) or automatic (1) mode
!SA(int ch, int active)              -> sets the channel 1-6 control to active(1) or not(0) (only if channel is in auto mode, otherwise command is ignored)
!AI(int ch)                          -> ignores an active alarm on the given channel or on all channels if no argument is given
!SP(int ch, int value)               -> Sets the setpoint value for a given channel to the given value (only if channel is in auto mode)
!DC(int active)                      -> activates (1) or deactivates(0) the direct control mode. In direct control mode any local control funtions are disabled. all outputs can now only be switched on or off by serial command. (press any "manual" button on the control panel to override)
!DS(int type, int index, int value)  -> directly sets the outputs (type 0 -> digital, type 1 -> Y analog, index can be 0-2 for analog outputs and 0-16 for digital outputs, values can be anything, in case of digital control anything greater 0 is considered true)
!Rs                                  -> resets the MCU
!CN                                  -> turns autosend (JSON) on
!DC                                  -> turns autosend off
!GD                                  -> returns device and version string
!Rl(int value)                       -> relay any serial data coming from serial1 to serial0 (1) or not (0)
!SF(int value)                       -> number of loop runs in between the autosends (smaller number -> faster send intervals, 10 is default)
*/

// structure to hold parsed command -> 66 bytes
struct serial_cmd_t {
    char* command;
    uint32_t name;
    uint8_t nargs;
    uint32_t args[8];
};

// type of callback method (should return 0 for valid command)
typedef void (*cb_data)();

class Serial_Com {
public:
    Serial_Com();
    void begin(cb_data DCB);
    bool validBaud(uint32_t BaudRate);
    void reinit0(uint32_t baud);
    void reinit1(uint32_t baud);
    void CmdOK();
    void CmdFAIL();
    void SendDev();
    void SendStateJSON();
    void SendBtn(uint16_t top_btn, uint16_t bottom_btn);
    void parse_command0();
    void parse_command1();
    void read_serial0();
    void read_serial1();
    cb_data DataCallback = nullptr; 
    boolean relay_serial = true;
    boolean autosend = false;
    boolean serial_ready = true;
    uint16_t autosend_delay = 10000;
    uint32_t lastsend = 0;
    uint32_t time_since_last_measurement = 0;
    uint32_t baud0 = SERIAL0_BAUD_RATE;
    uint32_t baud1 = SERIAL1_BAUD_RATE;
    uint16_t FiFoBuffer[17][FIFO_BUFFER_LENGHT];
    uint8_t FiFoBufferPos[17];


private:
    char SendBuffer[64];
    char serial_buf0[32];
    char serial_buf1[128];
    uint8_t buf_pos0 = 0;
    uint8_t buf_pos1 = 0;
    boolean SerialCmdValid0 = false;
    boolean SerialCmdValid1 = false;
    uint16_t fMedian(uint16_t array[]);
};

#endif
