#include "serial_com.h"

Serial_Com::Serial_Com() { }

void Serial_Com::begin(uint32_t baud, cb_cmd CCB, cb_data DCB) {
    Serial.begin(baud);
    Serial.println(F("*** EFMoST SPS ***"));
    Serial1.begin(19200);
    CmdCallback = CCB;
    DataCallback = DCB;
}

void Serial_Com::SendDebug() {
    // Serial.print("state:");
    // Serial.print(g_auto_state);
    Serial.print(" out:");
    Serial.print(g_digital_control_out);
    Serial.print(',');
    Serial.print(g_analog_control_out[0]);
    Serial.print(',');
    Serial.print(g_analog_control_out[1]);
    Serial.print(',');
    Serial.print(g_analog_control_out[2]);
    Serial.print(" in:");
    Serial.print(g_analog_values[0]);
    for (uint8_t i = 1; i < 5; i++) {
        Serial.print(',');
        Serial.print(g_analog_values[i]);
    }
    Serial.println();
}

// send information about a button press
void Serial_Com::SendBtn(uint16_t top_btn, uint16_t bottom_btn){
    Serial.print(F(" btn: "));
    Serial.print(top_btn);
    Serial.print(", ");
    Serial.println(bottom_btn);
}

// send system state (measurement values)
void Serial_Com::SendST() {
    Serial.print(F(" state:"));
    Serial.print(g_ProcessState.Temp1);
    Serial.print(',');
    Serial.print(g_ProcessState.Temp2);
    Serial.print(',');
    Serial.print(g_ProcessState.Press1);
    Serial.print(',');
    Serial.print(g_ProcessState.Press2);
    Serial.print(',');
    Serial.print(g_ProcessState.FluidLevel);
    Serial.print(',');
    Serial.print(g_ProcessState.H2S);
    Serial.print(',');
    Serial.print(g_ProcessState.Ox1);
    Serial.print(',');
    Serial.print(g_ProcessState.Ox2);
    Serial.print(',');
    Serial.print(g_ProcessState.pH);
    Serial.print(',');
    Serial.print(g_ProcessState.Speed_FeedPump);
    Serial.print(',');
    Serial.print(g_ProcessState.Speed_Filter);
    Serial.print(',');
    Serial.print(g_ProcessState.Conduct);
    Serial.print(',');
    Serial.print(g_ProcessState.Pos_AirationValve);
    Serial.print(',');
    Serial.println(g_ProcessState.Concentration_Fraction);
}

// send stored stetpoint values
void Serial_Com::SendSP() {
    Serial.print(F(" setpoint:"));
    Serial.print(g_Setpoints.FluidLevel);
    Serial.print(',');
    Serial.print(g_Setpoints.Speed_Filter);
    Serial.print(',');
    Serial.print(g_Setpoints.Rate_Airation);
    Serial.print(',');
    Serial.print(g_Setpoints.Speed_FeedPump);
    Serial.print(',');
    Serial.print(g_Setpoints.Temp);
    Serial.print(',');
    Serial.println(g_Setpoints.Concentration_Fraction);
}

// send control panel input state
void Serial_Com::SendCtrl() {
    Serial.print(F(" auto:"));
    for (uint8_t i=0; i<5; i++) {
        Serial.print(g_auto_state[i]);
        Serial.print(',');
    }
    Serial.print(g_auto_state[5]);
    Serial.print(F(" active:"));
    for (uint8_t i=0; i<5; i++) {
        Serial.print(g_control[i]);
        Serial.print(',');
    }
    Serial.print(g_control[5]);
    Serial.print(F(" alarm:"));
    for (uint8_t i=0; i<5; i++) {
        Serial.print(g_alarm[i]);
        Serial.print(',');
    }
    Serial.print(g_alarm[5]);
    Serial.print(F(" direct:"));
    Serial.println(g_direct_control);

}

// send actual control output / actuator values
void Serial_Com::SendOut() {
    Serial.print(F(" digital:"));
    Serial.print(g_digital_control_out, BIN);
    Serial.print(F(" analog:"));
    Serial.print(g_analog_control_out[0]);
    Serial.print(',');
    Serial.print(g_analog_control_out[1]);
    Serial.print(',');
    Serial.println(g_analog_control_out[2]);
}

// reply for unrecognized/illegal command!
void Serial_Com::CmdFAIL() {
    Serial.println(F("?"));
}

// reply for recognized command!
void Serial_Com::CmdOK() {
    Serial.println(F("!"));
}

// send "OK" 
void Serial_Com::OK() {
    Serial.println(F(" OK"));
}

// parse command string coming from Serial0
void Serial_Com::parse_command0() {
    serial_cmd_t cmd;
    char* SerialCommand = strtok(serial_buf0, "(");
    char* SerialArguments = strtok(0, ")");
    char* SerialToken = strtok(SerialArguments, SerialCommandDelim);
    cmd.command = SerialCommand;
    cmd.nargs = 0;
    while ((SerialToken != NULL) & (cmd.nargs < 8)) {
        cmd.args[cmd.nargs++] = atoi(SerialToken);
        SerialToken = strtok(0, SerialCommandDelim);
    }
    // execute Command
    CmdCallback(cmd);
}

// Todo!!!
// parse measurement string coming from Serial1
void Serial_Com::parse_command1() {
    Measurements M;
    char* SerialToken = strtok(serial_buf0, SerialDataDelim);
    uint8_t nvals = 0;
    uint16_t vals[14];
    while ((SerialToken != NULL) & (nvals < 14)) {
        vals[nvals++] = atoi(SerialToken);
        SerialToken = strtok(0, SerialCommandDelim);
    }
    M.Temp1 = vals[0];
    M.Temp2 = vals[1];
    M.pH = vals[2];
    M.Conduct = vals[3];
    M.Ox1 = vals[4];
    M.Ox2 = vals[5];
    M.H2S = vals[6];
    M.Press1 = vals[7];
    M.Press2 = vals[8];
    M.FluidLevel = vals[9];
    M.Speed_FeedPump = vals[10];
    M.Speed_Filter = vals[11];
    M.Pos_AirationValve = vals[12];
    M.Concentration_Fraction = vals[13];
    DataCallback(M);
}

// read method for serial0 uart (command interface)
void Serial_Com::read_serial0() {
    while (Serial.available()) {
        char C = (char)Serial.read(); // Zeichen einlesen
        Serial.print(C);
        switch (C) {
        case '!':
            if (not SerialCmdValid0) {
                SerialCmdValid0 = true;
            }
            break;
        case '\r':
        case '\n':
            if (SerialCmdValid0) {
                serial_buf0[buf_pos0++] = 0;
                SerialCmdValid0 = false;
                buf_pos0 = 0;
                parse_command0();
            }
            break;
        default:
            if ((buf_pos0 < 50) & SerialCmdValid0) {
                serial_buf0[buf_pos0++] = C;
            }
            break;
        }
        if (buf_pos0 >= 50) {
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
        case '!':
            if (not SerialCmdValid1) {
                SerialCmdValid1 = true;
            }
            break;
        case '\r':
        case '\n':
            if (SerialCmdValid1) {
                serial_buf1[buf_pos1++] = 0;
                SerialCmdValid1 = false;
                buf_pos1 = 0;
                parse_command1();
            }
            break;
        default:
            if ((buf_pos1 < 50) & SerialCmdValid1) {
                serial_buf1[buf_pos1++] = C;
            }
            break;
        }
        if (buf_pos1 >= 50) {
            SerialCmdValid1 = false;
            buf_pos1 = 0;
        }
    }
}