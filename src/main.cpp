#include "globals.h"
#include "serial_com.h"
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <MCP_DAC.h>
#include <SPI.h>

// global variables
uint32_t g_buttons_LEDs = 0;
uint8_t g_manual_button_state = 0;
uint8_t g_analog_control_out[5] = { 0, 0, 0, 0, 0 };
uint16_t g_digital_control_out = 0;
uint16_t g_analog_values[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
t_auto_state g_auto_state = { false, false, false, false, false, false };
t_manual_control g_manual_control = { false, false, false, false, false, false, false };

// local variables
uint16_t buttons_pressed_top = 0; // pressed button bit set to 1 in timer interrupt handler
uint16_t buttons_pressed_bottom = 0; // pressed button bit set to 1 in timer interrupt handler
Serial_Com scom;
uint8_t i = 0;
LiquidCrystal_I2C LCD[6] = { LiquidCrystal_I2C(DISPLAY0, 20, 4), LiquidCrystal_I2C(DISPLAY1, 20, 4), LiquidCrystal_I2C(DISPLAY2, 20, 4), LiquidCrystal_I2C(DISPLAY3, 20, 4), LiquidCrystal_I2C(DISPLAY4, 20, 4), LiquidCrystal_I2C(DISPLAY5, 20, 4) };
MCP4921 DAC[6] = {};

// interpret and execute commands that were recieved via serial0
void serial_cmd_exec(serial_cmd_t cmd) {
    if (strcmp(cmd.command, "set") == 0) {
        // set output values
        // Todo!!! -> check maual override
        scom.CmdOK();
        if (cmd.nargs == 3) {
            if (cmd.args[0] == 0) {
                if (cmd.args[1] < 5) {
                    g_analog_control_out[cmd.args[1]] = (uint8_t)cmd.args[2];
                } else if (cmd.args[1] == 255) {
                    for (i = 0; i < 5; i++) {
                        g_analog_control_out[i] = (uint8_t)cmd.args[2];
                    }
                }
            } else if (cmd.args[0] == 1) {
                if (cmd.args[1] < 12) {
                    if (cmd.args[2]) {
                        g_digital_control_out |= 1 << cmd.args[1];
                    } else {
                        g_digital_control_out &= ~(1 << cmd.args[1]);
                    }
                } else if (cmd.args[1] == 255) {
                    g_digital_control_out = cmd.args[2];
                }
            }
        }
    } else if (strcmp(cmd.command, "mode") == 0) {
        // change auto/manual mode
        scom.CmdOK();
    } else if (strcmp(cmd.command, "get") == 0) {
        // get all information about the system
        scom.CmdOK();
        if (cmd.nargs == 0) {
            scom.SendAll();
        }
    } else {
        scom.CmdFAIL();
    }
}

// use port/register manipulation to quickly shift out 25 bits to set the shift registers for LED output
void shiftOut_Fast() {
    // Latch pin (17) LOW
    PORTH &= B11111110;
    delayMicroseconds(5);
    for (i = 0; i < 25; i++) {
        // data pin (15) HIGH or LOW
        if (g_buttons_LEDs & (uint32_t(1) << (24 - i))) {
            PORTJ |= B00000001;
        } else {
            PORTJ &= B11111110;
        }
        delayMicroseconds(5);
        // pulse clock pin (16)
        PORTH |= B00000010;
        PORTH &= B11111101;
        delayMicroseconds(5);
    }
    delayMicroseconds(20);
    PORTH |= B00000001; // Latch pin (17) HIGH
}

// read all 16 analog values into array
void readAnalogValues() {
    for (i = 0; i < 16; i++) {
        g_analog_values[i] = analogRead(54 + i); // dummy read for adc to settle
        g_analog_values[i] = analogRead(54 + i);
    }
}

// set digtial control out using pins 2 - 13
// using port/register manipulation and loops were unrolled to speed execution up
void setDigitalControlOut() {
    // set pin 2
    if (g_digital_control_out & 1) {
        PORTE |= B00010000;
    } else {
        PORTE &= B11101111;
    }
    // set pin 3
    if (g_digital_control_out & 2) {
        PORTE |= B00100000;
    } else {
        PORTE &= B11011111;
    }
    // set pin 4
    if (g_digital_control_out & 4) {
        PORTG |= B00100000;
    } else {
        PORTG &= B11011111;
    }
    // set pin 5
    if (g_digital_control_out & 8) {
        PORTE |= B00001000;
    } else {
        PORTE &= B11110111;
    }
    // set pin 6
    if (g_digital_control_out & 16) {
        PORTH |= B00001000;
    } else {
        PORTH &= B11110111;
    }
    // set pin 7
    if (g_digital_control_out & 32) {
        PORTH |= B00010000;
    } else {
        PORTH &= B11101111;
    }
    // set pin 8
    if (g_digital_control_out & 64) {
        PORTH |= B00100000;
    } else {
        PORTH &= B11011111;
    }
    // set pin 9
    if (g_digital_control_out & 128) {
        PORTH |= B01000000;
    } else {
        PORTH &= B10111111;
    }
    // set pin 10
    if (g_digital_control_out & 256) {
        PORTB |= B00010000;
    } else {
        PORTB &= B11101111;
    }
    // set pin 11
    if (g_digital_control_out & 512) {
        PORTB |= B00100000;
    } else {
        PORTB &= B11011111;
    }
    // set pin 12
    if (g_digital_control_out & 1024) {
        PORTB |= B01000000;
    } else {
        PORTB &= B10111111;
    }
    // set pin 13
    if (g_digital_control_out & 2048) {
        PORTB |= B10000000;
    } else {
        PORTB &= B01111111;
    }
}

// set analog control outputs using external DACs connected via SPI
void setAnalogControlOut() {
}

// helper routine to flip the bit order of a byte
uint8_t flipByte(uint8_t c) {
    uint8_t r = 0;
    for (i = 0; i < 8; i++) {
        r <<= 1;
        r |= c & 1;
        c >>= 1;
    }
    return r;
}

// state change logic -> when any button has been pressed, change accordingly
void state_change() {
    if (buttons_pressed_top > 0) {
        switch (buttons_pressed_top) {
        case (uint32_t)1 << 0: // Befüllung Hand
            g_auto_state.filling = false;
            g_buttons_LEDs |= 1; // 1 on
            g_buttons_LEDs &= ~2; // 2 off
            break;
        case (uint32_t)1 << 1: // Befüllung Auto
            g_auto_state.filling = true;
            g_buttons_LEDs &= ~1; // 1 off
            g_buttons_LEDs |= 2; // 2 on
            break;
        case (uint32_t)1 << 2: // RotFilter Hand
            g_auto_state.rotation = false;
            g_buttons_LEDs |= 4; // 3 on
            g_buttons_LEDs &= ~8; // 4 off
            break;
        case (uint32_t)1 << 3: // RotFilter Auto
            g_auto_state.rotation = true;
            g_buttons_LEDs &= ~4; // 3 off
            g_buttons_LEDs |= 8; // 4 on
            break;
        case (uint32_t)1 << 4: // Belüftung Hand
            g_auto_state.airation = false;
            g_buttons_LEDs |= 16; // 5 on
            g_buttons_LEDs &= ~32; // 6 off
            break;
        case (uint32_t)1 << 5: // Belüftung Auto
            g_auto_state.airation = true;
            g_buttons_LEDs &= ~16; // 5 off
            g_buttons_LEDs |= 32; // 6 on
            break;
        case (uint32_t)1 << 6: // Dosierung Hand
            g_auto_state.feeding = false;
            g_buttons_LEDs |= 64; // 7 on
            g_buttons_LEDs &= ~128; // 8 off
            break;
        case (uint32_t)1 << 7: // Dosierung Auto
            g_auto_state.feeding = true;
            g_buttons_LEDs &= ~64; // 7 off
            g_buttons_LEDs |= 128; // 8 on
            break;
        case (uint32_t)1 << 8: // Kühlung Hand
            g_auto_state.cooling = false;
            g_buttons_LEDs |= 256; // 9 on
            g_buttons_LEDs &= ~512; // 10 off
            break;
        case (uint32_t)1 << 9: // Kühlung Auto
            g_auto_state.cooling = true;
            g_buttons_LEDs &= ~256; // 9 off
            g_buttons_LEDs |= 512; // 10 on
            break;
        case (uint32_t)1 << 10: // Filtration Hand
            g_auto_state.concentrating = false;
            g_buttons_LEDs |= 1024; // 11 on
            g_buttons_LEDs &= ~2048; // 12 off
            break;
        case (uint32_t)1 << 11: // Filtration Auto
            g_auto_state.concentrating = true;
            g_buttons_LEDs &= ~1024; // 11 off
            g_buttons_LEDs |= 2048; // 12 on
            break;
        }
    }
    if (buttons_pressed_bottom > 0) {
        switch (buttons_pressed_bottom) {
        case (uint32_t)1 << 0:
            if (!g_auto_state.filling) {
                g_manual_control.filling = true;
                g_buttons_LEDs |= 4096; // 13 on
                g_buttons_LEDs &= ~8192; // 14 off
            }
            break;
        case (uint32_t)1 << 1:
            if (!g_auto_state.filling) {
                g_manual_control.filling = false;
                g_buttons_LEDs &= ~4096; // 13 off
                g_buttons_LEDs |= 8192; // 14 on
            }
            break;
        case (uint32_t)1 << 2:
            if (!g_auto_state.rotation) {
                g_manual_control.rotation = true;
                g_manual_control.rotation_reverse = false;
                g_buttons_LEDs |= 16384; // 15 on
                g_buttons_LEDs &= ~32768; // 16 off
                g_buttons_LEDs &= ~65536; // 17 off
            }
            break;
        case (uint32_t)1 << 3:
            if (!g_auto_state.rotation) {
                g_manual_control.rotation = false;
                g_manual_control.rotation_reverse = false;
                g_buttons_LEDs &= ~16384; // 15 off
                g_buttons_LEDs |= 32768; // 16 on
                g_buttons_LEDs &= ~65536; // 17 off
            }
            break;
        case (uint32_t)1 << 4:
            if (!g_auto_state.rotation) {
                g_manual_control.rotation = true;
                g_manual_control.rotation_reverse = true;
                g_buttons_LEDs &= ~16384; // 15 off
                g_buttons_LEDs &= ~32768; // 16 off
                g_buttons_LEDs |= 65536; // 17 on
            }
            break;
        case (uint32_t)1 << 5:
            if (!g_auto_state.airation) {
                g_manual_control.airation = true;
                g_buttons_LEDs |= 131072; // 18 on
                g_buttons_LEDs &= ~262144; // 19 off
            }
            break;
        case (uint32_t)1 << 6:
            if (!g_auto_state.airation) {
                g_manual_control.airation = false;
                g_buttons_LEDs &= ~131072; // 18 off
                g_buttons_LEDs |= 262144; // 19 on
            }
            break;
        case (uint32_t)1 << 7:
            if (!g_auto_state.feeding) {
                g_manual_control.feeding = true;
                g_buttons_LEDs |= 524288; // 20 on
                g_buttons_LEDs &= ~1048576; // 21 off
            }
            break;
        case (uint32_t)1 << 8:
            if (!g_auto_state.feeding) {
                g_manual_control.feeding = false;
                g_buttons_LEDs &= ~524288; // 20 off
                g_buttons_LEDs |= 1048576; // 21 on
            }
            break;
        case (uint32_t)1 << 9:
            if (!g_auto_state.cooling) {
                g_manual_control.cooling = true;
                g_buttons_LEDs |= 2097152; // 22 on
                g_buttons_LEDs &= ~4194304; // 23 off
            }
            break;
        case (uint32_t)1 << 10:
            if (!g_auto_state.cooling) {
                g_manual_control.cooling = false;
                g_buttons_LEDs &= ~2097152; // 22 off
                g_buttons_LEDs |= 4194304; // 23 on
            }
            break;
        case (uint32_t)1 << 11:
            if (!g_auto_state.concentrating) {
                g_manual_control.concentrating = true;
                g_buttons_LEDs |= 8388608; // 24 on
                g_buttons_LEDs &= ~16777216; // 25 off
            }
            break;
        case (uint32_t)1 << 12:
            if (!g_auto_state.concentrating) {
                g_manual_control.concentrating = false;
                g_buttons_LEDs &= ~8388608; // 24 on
                g_buttons_LEDs |= 16777216; // 25 off
            }
            break;
        default:
            break;
        }
    }
}

void compAnalogOut() {
}

// ISR for button polling. Executed every 50 ms
ISR(TIMER5_COMPA_vect) {
    buttons_pressed_top = uint16_t(PINA) + ((uint16_t(flipByte(PINC & 0xF0)) << 8));
    buttons_pressed_bottom = (flipByte(PINC) >> 4) + (flipByte(PINL) << 4) + ((PINJ & 0x02) << 11);
}

void setup() {
    // ****** setup serial communication ******
    scom.begin(SERIAL_BAUD_RATE, serial_cmd_exec);
    Serial.println("init displays...");
    // ****** setup LC displays ******
    for (i = 0; i < 6; i++) {
        LCD[i].init();
        LCD[i].backlight();
    }
    Serial.println("init pins...");
    // ****** setup pinmodes ******
    // all frontpanel buttons -> input
    DDRC = 0x00; // set entire port as input
    DDRA = 0x00; // set entire port as input
    DDRL = 0x00; // set entire port as input
    DDRJ &= B11111101; // pin 14

    // shift registers for LEDs -> output
    DDRJ |= B00000001; // pin 15
    DDRH |= B00000011; // pin 16 & 17

    // digital control -> output
    DDRE |= B00111000; // pin 5, 2 & 3
    DDRG |= B00100000; // pin 4
    DDRH |= B01111000; // pin 6, 7, 8 & 9
    DDRB |= B11110000; // pin 10, 11, 12 & 13

    Serial.println("init button polling interrupts...");
    // ****** setup timer interrupt for 50 ms button polling ******
    cli(); // stop all interrupts
    TCCR5A = 0;
    TCCR5B = 0;
    TCNT5 = 0;
    OCR5A = 781;
    TCCR5B |= (1 << WGM52);
    TCCR5B |= (1 << CS52) | (1 << CS50);
    TIMSK5 |= (1 << OCIE5A);
    sei(); // enable interrupts

    Serial.println("init shift registers...");
    for (int i = 0; i < 25; i++) {
        g_buttons_LEDs = uint32_t(1) << i;
        shiftOut_Fast();
        delay(100);
    }

    Serial.println("ready!");
    g_buttons_LEDs = B01010101 + (uint32_t(B10100101) << 8) + (uint32_t(B01010101) << 16) + (uint32_t(B00000000) << 24);
}

void loop() {
    // state change - react on button inputs if there were any
    state_change();

    // input - read Sensor states
    readAnalogValues();

    // compute control outputs for manual control
    compAnalogOut();

    // output - control
    setDigitalControlOut();
    setAnalogControlOut();

    // output - button LEDs
    shiftOut_Fast();

    // update LC displays
    for (i = 0; i < 6; i++) {
        LCD[i].setCursor(0, 0);
        LCD[i].print("      ");
        LCD[i].setCursor(0, 0);
        LCD[i].print(g_analog_values[i]); // preliminary test
    }

    // execute Serial Comms events
    scom.read_serial0();
    scom.read_serial1();
    // scom.SendState(buttons_pressed_top,buttons_pressed_bottom);
    scom.SendAll();
}