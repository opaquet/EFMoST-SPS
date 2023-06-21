#include "serial_com.h"

Serial_Com::Serial_Com() { }

void Serial_Com::begin(cb_cmd CCB, cb_data DCB) {
    Serial.begin(baud0);
    SendDev();
    Serial1.begin(baud1);
    CmdCallback = CCB;
    DataCallback = DCB;
}

bool Serial_Com::validBaud(uint32_t BaudRate) {
    if ((BaudRate == 300)|| 
        (BaudRate == 1200)||
        (BaudRate == 2400)||
        (BaudRate == 4800)||
        (BaudRate == 9600)|| 
        (BaudRate == 19200)||
        (BaudRate == 38400)||
        (BaudRate == 57600)||
        (BaudRate == 115200)||
        (BaudRate == 230400)||
        (BaudRate == 250000)||
        (BaudRate == 500000)||
        (BaudRate == 1000000)||
        (BaudRate == 2000000)) 
        return true;
    else
        return false;
}

void Serial_Com::reinit0(uint32_t Baud) {
    baud0 = Baud;
    Serial.begin(Baud);
    if (autosend_delay < 5760000/Baud)
        autosend_delay = 5760000/Baud;
    SendDev();
}

void Serial_Com::reinit1(uint32_t Baud) {
    baud1 = Baud;
    Serial1.begin(Baud);
}

void Serial_Com::SendDev() {
    sprintf(SendBuffer,"{\"D\":\"%s\",\"V\":\"%s\",\"Speed\":[%lu,%lu]}", DEVICE, VERSION, baud0,baud1);
    Serial.println(SendBuffer);
}

// send information about a button presses as JSON
void Serial_Com::SendBtn(uint16_t top_btn, uint16_t bottom_btn){
    Serial.print(F("{\"btn\":["));
    boolean sep = false;
    for (uint8_t i = 0; i < 12; i++) {
        if ( (top_btn >> i) & 1) {
            if (sep) {
                Serial.print(",");
                sep = false;
            }
            Serial.print(i+1);
            sep = true;
        }
    }
    for (uint8_t i = 0; i < 13; i++) {
        if ( (bottom_btn >> i) & 1) {
            if (sep) {
                Serial.print(",");
                sep = false;
            }
            Serial.print(i+13);
            sep = true;
        }
    }
    Serial.println("]}");
}

// send complete system state and all measurement information in JSON form
void Serial_Com::SendStateJSON() {
    lastsend = millis();
    Serial.print("{\"M\":[");

    // Send Measurements (15x2 -> 34 Bytes)
    for (uint8_t i = 0; i < 16; i++) {
        Serial.print(g_ProcessState[i]);
        Serial.print(',');
    }
    Serial.print(millis() - time_since_last_measurement); 
    Serial.print("],\"Ctrl\":{\"Auto\":[");
    for (uint8_t i = 0; i < 5; i++) {
        Serial.print(g_auto_state[i]?"1":"0");
        Serial.print(',');
    }
    Serial.print(g_auto_state[5]?"1":"0");
    Serial.print("],\"Act\":[");
    for (uint8_t i = 0; i < 6; i++) {
        Serial.print(g_control[i]?"1":"0");
        Serial.print(',');
    }
    Serial.print(g_control[6]?"1":"0");
    Serial.print("],\"Alarm\":[");
    for (uint8_t i = 0; i < 5; i++) {
        Serial.print(g_alarm[i]?"1":"0");
        Serial.print(',');
    }
    Serial.print(g_alarm[5]?"1":"0");
    Serial.print("],\"SP\":[");

    // Send actual Setpoints (6x2 -> 12 Bytes)
    for (uint8_t i = 0; i < 5; i++) {
        Serial.print(g_Setpoints[i]);
        Serial.print(',');
    }
    Serial.print(g_Setpoints[5]);
    Serial.print("],\"Out\":{\"D\":[");
    for (uint8_t i = 0; i < 11; i++) {
        Serial.print(((g_digital_control_out >> i) & 1)?"1,":"0," );
    }
    Serial.print(((g_digital_control_out >> 11) & 1)?"1":"0" );
    Serial.print("],\"A\":[");
    Serial.print(g_analog_control_out[0]);
    Serial.print(",");
    Serial.print(g_analog_control_out[1]);
    Serial.print(",");
    Serial.print(g_analog_control_out[2]);
    Serial.print("],\"dctrl\":");
    Serial.print(g_direct_control?"1":"0" );
    Serial.print("}},\"Error\":");
    Serial.print(g_errorCode);
    Serial.println("}");
}

// reply for unrecognized/illegal command!
void Serial_Com::CmdFAIL() {
    Serial.println(F("{\"Cmd\":0}"));
}

// reply for recognized command!
void Serial_Com::CmdOK() {
    Serial.println(F("{\"Cmd\":1}"));
}

// parse command string coming from Serial0
void Serial_Com::parse_command0() {
    serial_cmd_t cmd;
    char* SerialCommand = strtok(serial_buf0, "(");
    char* SerialArguments = strtok(0, ")");
    char* SerialToken = strtok(SerialArguments, ",");
    cmd.command = SerialCommand;
    cmd.nargs = 0;
    while ((SerialToken != NULL) & (cmd.nargs < 8)) {
        cmd.args[cmd.nargs++] = atol(SerialToken);
        SerialToken = strtok(NULL, ",");
    }
    // execute Command
    CmdCallback(cmd);
}

// parse measurement string coming from Serial1
void Serial_Com::parse_command1() {
    if (relay_serial) {
        Serial.print(F("{\"Relay\":\""));
        Serial.print(serial_buf1);
        Serial.println(F("\"}"));
    }
    char* SerialToken = strtok(serial_buf1, ",");
    uint8_t nvals = 0;
    uint16_t vals[14]; // one longer than necessary
    while ((SerialToken != NULL)) {
        if (nvals == 14) break;
        vals[nvals++] = atoi(SerialToken);
        SerialToken = strtok(NULL, ",");
    }
    // call the handling callback routine
    if (nvals == 13) { // only if 13 values are recieved, call the callback function
        DataCallback(vals);
        time_since_last_measurement = millis();
    } else {
        g_errorCode &= _BV(1);
    }
}

// read method for serial0 uart (command interface)
void Serial_Com::read_serial0() {
    while (Serial.available()) {
        char C = (char)Serial.read(); // Zeichen einlesen
        switch (C) {
        case '!': // start char
            if (not SerialCmdValid0) {
                SerialCmdValid0 = true;
            }
            break;
        case '\r': // end char
        case '\n': // end char
            if (SerialCmdValid0) {
                serial_buf0[buf_pos0++] = 0;
                SerialCmdValid0 = false;
                parse_command0();
                buf_pos0 = 0;
            }
            break;
        default:
            if ((buf_pos0 < 60) & SerialCmdValid0) {
                serial_buf0[buf_pos0++] = C;
            }
            break;
        }
        if (buf_pos0 >= 60) {
            SerialCmdValid0 = false;
            buf_pos0 = 0;
        }
    }
}


// read method for serial1 uart (measurement interface)
void Serial_Com::read_serial1() {
    while (Serial1.available()) {
        char C = (char)Serial1.read();
        switch (C) {
        case '!': // start char
            if (not SerialCmdValid1) {
                SerialCmdValid1 = true;
            }
            break;
        case '?': // end char
        case '\r': // end char
        case '\n': // end char
            if (SerialCmdValid1) {
                serial_buf1[buf_pos1++] = 0;
                SerialCmdValid1 = false;
                buf_pos1 = 0;
                parse_command1();
            }
            break;
        default:
            if ((buf_pos1 < 120) & SerialCmdValid1) {
                serial_buf1[buf_pos1++] = C;
            }
            break;
        }
        if (buf_pos1 >= 120) {
            SerialCmdValid1 = false;
            buf_pos1 = 0;
            g_errorCode &= _BV(0);
        }
    }
}