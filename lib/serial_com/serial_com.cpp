#include "serial_com.h"

Serial_Com::Serial_Com() {}

void Serial_Com::begin(uint32_t baud, cb_fun CB) {
  Serial.begin(baud);
  Serial.println(F("EFMoST SPS"));
  Serial1.begin(baud);
  Callback = CB;
}

void Serial_Com::CmdOK() {
  Serial.println(F("!"));
}

void Serial_Com::SendAll() {
  Serial.print("state:");
  Serial.print(g_auto_state);
  Serial.print(" out:");
  Serial.print(g_digital_control_out);
  Serial.print(',');
  Serial.print(g_analog_control_out[0]);
  Serial.print(',');
  Serial.print(g_analog_control_out[1]);
  Serial.print(',');
  Serial.print(g_analog_control_out[2]);
  Serial.print(',');
  Serial.print(g_analog_control_out[3]);
  Serial.print(',');
  Serial.print(g_analog_control_out[4]);
  Serial.print(" in:");
  Serial.print(g_analog_values[0]);
  for (uint8_t i = 1; i < 16; i++) {
    Serial.print(',');
    Serial.print(g_analog_values[i]);
  }
  Serial.println();  
}

void Serial_Com::CmdFAIL() {
  Serial.println(F("?"));
}

void Serial_Com::parse_command0() {
    serial_cmd_t cmd;
    SerialCommand = strtok(serial_buf0, "(");
    SerialArguments = strtok(0, ")");
    SerialToken = strtok(SerialArguments, SerialCommandDelim);
    cmd.command = SerialCommand;
    cmd.nargs = 0;
    while ((SerialToken != NULL) & (cmd.nargs < 8)) {
        cmd.args[cmd.nargs++] = atoi(SerialToken);
        SerialToken = strtok(0, SerialCommandDelim);
    }
    //execute Command
    Callback(cmd);
}

void Serial_Com::parse_command1() {
    // Todo!!! Still no definition here...
}

void Serial_Com::read_serial0() {
    while (Serial.available()) {
        char C = (char)Serial.read(); // Zeichen einlesen
        Serial.print(C);
        switch (C){
        case '!':
            if (not SerialCmdValid0){ 
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
            if ((buf_pos0 < 50) & SerialCmdValid0){
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

void Serial_Com::read_serial1() {
    while (Serial1.available()) {
        char C = (char)Serial1.read();
        switch (C){
        case '!':
            if (not SerialCmdValid1){ 
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
            if ((buf_pos1 < 50) & SerialCmdValid1){
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