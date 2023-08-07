#include "serial_com.h"

Serial_Com::Serial_Com() { }

// Schnelle und schutzige Median Berechnug, ausgehend von fester Array Länge (5) für FiFo Puffer
uint16_t Serial_Com::fMedian(uint16_t array[]) {
    uint16_t median = 0;
    uint8_t idx1 = 0;
    uint8_t idx2 = 0;

    for (uint8_t i = 0; i < FIFO_BUFFER_LENGHT; i++) 
        if (array[i] >= median) {
            median = array[i];
            idx1 = i;
        }       
    median = 0;
    for (uint8_t i = 0; i < FIFO_BUFFER_LENGHT; i++) 
        if ((array[i] >= median) & (idx1 != i)) {
            median = array[i];
            idx2 = i;
        }       
    median = 0;
    for (uint8_t i = 0; i < FIFO_BUFFER_LENGHT; i++) 
        if ((array[i] >= median) & (idx1 != i) & (idx2 != i)) 
            median = array[i];      
    return median; 
}

void Serial_Com::begin(cb_data DCB) {
    Serial.begin(baud0);
    SendDev();
    Serial1.begin(baud1);
    DataCallback = DCB;

    // initialize Fifo Buffer for calculation of running median of all incoming measurement values
    for (uint8_t i = 0; i < 17; i++) {
        for (uint8_t j = 0; j < FIFO_BUFFER_LENGHT; j++) {
            FiFoBuffer[i][j] = 0;
        }
        FiFoBufferPos[i] = 0;
    }
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
    else {
        g_errorCode |= _BV(4); // invalid baud rate
        return false;
    }
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
    Serial.print(F("{\"M\":["));

    // Send Measurements (18)
    for (uint8_t i = 0; i < 17; i++) {
        Serial.print(g_ProcessState[i]);
        Serial.print(',');
    }
    Serial.print(millis() - time_since_last_measurement); 
    Serial.print(F("],\"Ctrl\":{\"Auto\":["));
    for (uint8_t i = 0; i < 5; i++) {
        Serial.print(g_auto_state[i]?"1":"0");
        Serial.print(',');
    }
    Serial.print(g_auto_state[5]?"1":"0");
    Serial.print(F("],\"Act\":["));
    for (uint8_t i = 0; i < 6; i++) {
        Serial.print(g_control[i]?"1":"0");
        Serial.print(',');
    }
    Serial.print(g_control[6]?"1":"0");
    Serial.print(F("],\"Alarm\":["));
    for (uint8_t i = 0; i < 5; i++) {
        Serial.print(g_alarm[i]?"1":"0");
        Serial.print(',');
    }
    Serial.print(g_alarm[5]?"1":"0");
    Serial.print(F("],\"SP\":["));

    // Send actual Setpoints (6x2 -> 12 Bytes)
    for (uint8_t i = 0; i < 5; i++) {
        Serial.print(g_Setpoints[i]);
        Serial.print(',');
    }
    Serial.print(g_Setpoints[5]);
    Serial.print(F("],\"Out\":{\"D\":["));
    for (uint8_t i = 0; i < 11; i++) {
        Serial.print(((g_digital_control_out >> i) & 1)?"1,":"0," );
    }
    Serial.print(((g_digital_control_out >> 11) & 1)?"1":"0" );
    Serial.print(F("],\"A\":["));
    Serial.print(g_analog_control_out[0]);
    Serial.print(",");
    Serial.print(g_analog_control_out[1]);
    Serial.print(",");
    Serial.print(g_analog_control_out[2]);
    Serial.print(F("],\"dctrl\":"));
    Serial.print(g_direct_control?"1":"0" );
    Serial.print(F("}},\"Error\":"));
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

    char* SerialCommand = strtok(serial_buf0, "(");
    uint16_t name = * (uint32_t *) & SerialCommand[0];
    uint32_t args[8] = {0,0,0,0,0,0,0,0};

    char* SerialArguments = strtok(0, ")");
    char* SerialToken = strtok(SerialArguments, ",");
    uint8_t nargs = 0;
    while ((SerialToken != NULL) & (nargs < 8)) {
        args[nargs++] = atol(SerialToken);
        SerialToken = strtok(NULL, ",");
    }

    // execute Command
    switch (BYTESWAP16(name)) {
        case 'SP':
            if ((nargs == 2) && (args[0] < 6) && (g_auto_state[args[0]])) {
                g_Setpoints[args[0]] = args[1];   
            } 
            CmdOK();
            lastsend = 0;
            break;
        case 'AI':
            if ((nargs == 1) && (args[0] < 6)) {
                g_alarm_ignore[args[0]] = true;
            } 
            if (nargs == 0) {
                for (uint8_t i=0; i<6; i++) {
                    g_alarm_ignore[i] = true;
                }
            } 
            CmdOK();
            break;
        case 'DR':
            if (nargs == 1) g_direct_control = args[0];
            if (g_direct_control){
                for (uint8_t i=0; i<6; i++) {
                    g_alarm_ignore[i] = true;
                    g_control[i] = false;
                }
            } else {
                for (uint8_t i=0; i<6; i++) {
                    g_alarm_ignore[i] = false;
                }
            }
            lastsend = 0;
            CmdOK();
            break;
        case 'DS':
            if (nargs == 3) {
                switch (args[0]) {
                case 0:
                    if (args[1] > 16) break;
                    if (args[2]) {
                        g_digital_control_out |= _BV(args[1]);
                    } else {
                        g_digital_control_out &= ~_BV(args[1]);
                    }
                    lastsend = 0;
                    break;
                case 1:
                    if (args[1] > 3) break;
                    g_analog_control_out[args[1]] = args[2];
                    lastsend = 0;
                    break;
                default:
                    break;
                }
            } 
            CmdOK();
            break;
        case 'SM':
            if ((nargs == 2) && (args[0] < 6)) {
                g_auto_state[args[0]] = args[1];
                lastsend = 0;
            } 
            CmdOK();
            lastsend = 0;
            break;
        case 'BM':
            if ((nargs == 1) && validBaud(args[0])) {
                reinit1(args[0]);
            } 
            break;
        case 'BS':
            if ((nargs == 1) && validBaud(args[0])) {
                reinit0(args[0]);
            } 
            break; 
        case 'SA':
            if ((nargs == 2) && (args[0] < 6) ) {
                g_control[args[0]] = args[1];
                lastsend = 0;
            } 
            CmdOK();
            lastsend = 0;
            break;
        case 'Rs':
            delay(100);
            asm volatile ("jmp 0");  //inline assembly: jump to adress 0 -> jump to first interrupt vector (reset/reinitialize on most Atmel MCUs)
            break;; //will (or should) never be reached...
        case 'Rl':
            if (nargs == 1) {
                relay_serial = args[0];
            }
            CmdOK();
            break;
        case 'SF':
            if (nargs == 1) {
                if (args[0] < 5760000/baud0)  // make sure that send frequency is not too fast so that all data can still be trasmitted... on very low baud rates reduce send frequency if necessary
                    autosend_delay = 5760000/baud0;
                else
                    autosend_delay = args[0];
            }
            CmdOK();
            break;
        case 'GS':
            SendStateJSON();
            break;
        case 'CN':
            autosend = true;
            lastsend = 0;
            CmdOK();
            break;
        case 'GD':
            SendDev();
            break;
        case 'DC':
            autosend = false;
            CmdOK();
            break;
        default:
            g_errorCode |= _BV(4);
            CmdFAIL();
            break;
    }
}

// parse measurement string coming from Serial1
void Serial_Com::parse_command1() {
    // relay anything from measurement arduin to control PC if "relay_serial" is set true
    if (relay_serial) {
        Serial.print(F("{\"Relay\":\""));
        Serial.print(serial_buf1);
        Serial.println(F("\"}"));
    }

    // split message int blocks separated by commas
    uint8_t nvals = 0;
    char* SerialToken = strtok(serial_buf1, ",");
    uint16_t Name;
    uint16_t value;
    while ((SerialToken != NULL)) {
        uint8_t IDX = 255;
        if (nvals == 24) break;
        nvals++;
        Name = * (int*) &  SerialToken[0]; // cast first two chars (the name) into a single 16 bit unsigned integer
        value = atoi(SerialToken+3); // convert everything from the 4th char on (the value) into a 16 bit signed integer
        // interpret messageblock
        switch (BYTESWAP16(Name)) {
            case 'TP':
                IDX = Temp;
                break;
            case 'TA':
                IDX = Temp2;
                break;
            case 'TD':
                IDX = Temp3;
                break;
            case 'PH':
                IDX = pH;
                break;
            case 'EC':
                IDX = Conduct;
                break;
            case 'DO':
                IDX = Ox;
                break;
            case 'SG':
                IDX = pOx;
                break;
            case 'AB':
                IDX = Distance;
                break;
            case 'PU':
                IDX = FeedRate;
                break;
            case 'PV':
                IDX = Airation;
                break;
            case 'RT':
                IDX = FilterSpeed;
                break;
            case 'DB':
                IDX = Press1;
                break;
            case 'DK':
                IDX = Press2;
                break;
            case 'FD':
                IDX = 255;
                break;
            case 'SD':
                IDX = Foam;
                break;
            default:
                g_errorCode |= _BV(3); // set error code 4: one of the message block could not be interpreted
        }
        

        // add value to fifo buffer overwriting the oldest value
        if (IDX < 17) {
            g_state_changed |= (g_ProcessState[IDX] != value);

            FiFoBuffer[IDX][FiFoBufferPos[IDX]++] = value;

            // resetting the buffer position back to zero if out of bounds (>FIFO_BUFFER_LENGHT) 
            if (FiFoBufferPos[IDX] >= FIFO_BUFFER_LENGHT) 
                FiFoBufferPos[IDX] = 0;

            // take the median value of the selected fifo buffer as the according measurement value
            g_ProcessState[IDX] = fMedian(FiFoBuffer[IDX]);
        }

        SerialToken = strtok(NULL, ","); // get next message block
    }

    // set error code 2: number of message blocks is not 1-23
    if ((nvals == 0) | (nvals >= 24)) 
        g_errorCode |= _BV(1);

    DataCallback(); // call the handling callback routine
    time_since_last_measurement = millis();
}

// read method for serial0 uart (command interface)
void Serial_Com::read_serial0() {
    while (Serial.available()) {
        char C = (char)Serial.read(); // Zeichen einlesen
        switch (C) {
        case '!': // start char
            SerialCmdValid0 = true;
            break;
        case '\r': // end char
        case '\n': // end char
            if (SerialCmdValid0) {
                serial_buf0[buf_pos0] = 0;
                SerialCmdValid0 = false;
                buf_pos0 = 0;
                parse_command0();
            }
            break;
        default:
            if ((buf_pos0 < 30) & SerialCmdValid0) {
                serial_buf0[buf_pos0++] = C;
            }
            break;
        }
        if (buf_pos0 >= 30) {
            SerialCmdValid0 = false;
            buf_pos0 = 0;
            g_errorCode |= _BV(7);
        }
    }
}

// read method for serial1 uart (measurement interface)
void Serial_Com::read_serial1() {
    while (Serial1.available()) {
        char C = (char)Serial1.read();
        switch (C) {
        case '!': // start char
            SerialCmdValid1 = true;
            break;
        case '?': // end char
        case '\r': // end char
        case '\n': // end char
            if (SerialCmdValid1) {
                serial_buf1[buf_pos1] = 0;
                SerialCmdValid1 = false;
                buf_pos1 = 0;
                parse_command1();
            }
            break;
        case '{': // ignore quotation marks, brackets and other characters that might corrupt JSON and store a space instead
        case '}':
        case ':':
        case '"':  
            if ((buf_pos1 < 120) & SerialCmdValid1) {
                serial_buf1[buf_pos1++] = ' ';
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
            g_errorCode |= _BV(0);
        }
    }
}