#ifndef S_COM
#define S_COM

#if ARDUINO >= 100
#include "Arduino.h"
#endif

#include "globals.h"

/*
Serial commands must start with "!"
They start with a keyword and end wit a list of up to 8 interger parameters in parenteses (comma separated)
--> !mode(3,0)
--> !set(0,1,53)
--> !get() or !get
*/

// structure to hold parsed command -> 66 bytes
struct serial_cmd_t {
    char* command;
    uint8_t nargs;
    uint32_t args[8];
};

// type of callback method (should return 0 for valid command)
typedef void (*cb_fun)(serial_cmd_t);

class Serial_Com {
public:
    Serial_Com();
    void begin(uint32_t baud, cb_fun CB);
    void CmdOK();
    void CmdFAIL();
    void SendAll();
    void SendState(uint16_t btnt, uint16_t btnd);
    void parse_command0();
    void parse_command1();
    void read_serial0();
    void read_serial1();
    cb_fun Callback = nullptr;

private:
    char send_string_buf[64];
    char serial_buf0[64];
    char serial_buf1[64];
    uint8_t buf_pos0 = 0;
    uint8_t buf_pos1 = 0;
    boolean SerialCmdValid0 = false;
    boolean SerialCmdValid1 = false;
    char* SerialCommand;
    char* SerialArguments;
    char* SerialToken;
    const char SerialCommandDelim[5] = " ,;-";
};

#endif
