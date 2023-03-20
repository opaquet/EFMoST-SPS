#ifndef S_COM
#define S_COM

#if ARDUINO >= 100
#include "Arduino.h"
#endif

#include "globals.h"

/*
Serial commands must start with "!"
They start with a keyword and end wit a list of up to 8 interger parameters in parenteses (comma separated)
--> !set_mode(3,0)
--> !get_panel() or !get_panel
*/

// structure to hold parsed command -> 66 bytes
struct serial_cmd_t {
    char* command;
    uint8_t nargs;
    uint32_t args[8];
};

// type of callback method (should return 0 for valid command)
typedef void (*cb_cmd)(serial_cmd_t);
typedef void (*cb_data)(Measurements);

class Serial_Com {
public:
    Serial_Com();
    void begin(uint32_t baud, cb_cmd CCB, cb_data DCB);
    void CmdOK();
    void CmdFAIL();
    void OK();
    void SendDebug();
    void SendBtn(uint16_t top_btn, uint16_t bottom_btn);
    void SendST();
    void SendSP();
    void SendCtrl();
    void SendOut();
    void parse_command0();
    void parse_command1();
    void read_serial0();
    void read_serial1();
    cb_cmd CmdCallback = nullptr;
    cb_data DataCallback = nullptr; 

private:
    char send_string_buf[64];
    char serial_buf0[64];
    char serial_buf1[64];
    uint8_t buf_pos0 = 0;
    uint8_t buf_pos1 = 0;
    boolean SerialCmdValid0 = false;
    boolean SerialCmdValid1 = false;
    const char SerialCommandDelim[5] = " ,;-";
    const char SerialDataDelim[4] = " ;-";
};

#endif
