#include "globals.h"
#include "serial_com.h"
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <MCP_DAC.h>
#include <SPI.h>

// global variables
uint8_t g_manual_button_state = 0;
uint16_t g_analog_control_out[3] = { 0, 0, 0 };
uint16_t g_digital_control_out = 0;
uint16_t g_analog_values[5] = { 0, 0, 0, 0, 0 };
boolean g_auto_state[6] = {false, false, false, false, false, false};
const uint8_t m2a_idx[13] = {0,0,1,1,1,2,2,3,3,4,4,5,5};
boolean g_control[7] = {false, false, false, false, false, false, false};
const uint8_t m2l_idx[7] = {0,2,4,5,7,9,11};
boolean g_alarm[6] = {false, false, false, false, false, false};
boolean g_alarm_ignore[6] = {false, false, false, false, false, false};
const uint8_t alarm_idx[6] = {1,3,6,8,10,12};
Measurements g_ProcessState = {0,0,0,0,0,0,0,0,0,0,0,0,0};
SetPoints g_Setpoints = {0,0,0,0,0,0};
boolean g_direct_control = false;


// local variables
uint16_t buttons_pressed_top = 0; // pressed button bit set to 1 in timer interrupt handler
uint16_t buttons_pressed_bottom = 0; // pressed button bit set to 1 in timer interrupt handler
uint16_t maxV = 0; // maximum fill level ever recoded
uint8_t FL_offset = 0; // offset/hysteresis for fluid level control (0 or +5 L)
Serial_Com scom;
uint8_t i = 0;
LiquidCrystal_I2C LCD[6] = { LiquidCrystal_I2C(DISPLAY0, 16, 2), LiquidCrystal_I2C(DISPLAY1, 16, 2), LiquidCrystal_I2C(DISPLAY2, 16, 2), LiquidCrystal_I2C(DISPLAY3, 16, 2), LiquidCrystal_I2C(DISPLAY4, 16, 2), LiquidCrystal_I2C(DISPLAY5, 16, 2) };
MCP4921 DAC[3] = {};
uint16_t lbtn_t, lbtn_b;


// interpret and execute commands that were recieved via serial0
void serial_cmd_exec(serial_cmd_t cmd) {
    if        (strcmp(cmd.command, "set_point") == 0) {  // set Setpoints (only when in automatic mode)
        if (cmd.nargs == 2) {
            if (cmd.args[0] < 6) {
                switch (cmd.args[0]) {
                case 0:
                    if (g_auto_state[0]) g_Setpoints.FluidLevel = cmd.args[1];
                    break;
                case 1:
                    if (g_auto_state[1]) g_Setpoints.Speed_Filter = cmd.args[1];
                    break;
                case 2:
                    if (g_auto_state[2]) g_Setpoints.Rate_Airation = cmd.args[1];
                    break;
                case 3:
                    if (g_auto_state[3]) g_Setpoints.Speed_FeedPump = cmd.args[1];
                    break;
                case 4:
                    if (g_auto_state[4]) g_Setpoints.Temp = cmd.args[1];
                    break;
                case 5:
                    if (g_auto_state[5]) g_Setpoints.Concentration_Fraction = cmd.args[1];
                    break;
                }   
            }
        }
        scom.CmdOK();     
    } else if (strcmp(cmd.command, "alarm_ignore") == 0) { // ignore alarm
        if (cmd.nargs == 1) {
            if (cmd.args[0] < 6) g_alarm_ignore[cmd.args[0]] = true;
        }
        if (cmd.nargs == 0) {
            for (i=0; i<6; i++) g_alarm_ignore[i] = true;
        }
        scom.CmdOK();
    } else if (strcmp(cmd.command, "dctrl") == 0) { // enable or disable direct control mode
        if (cmd.nargs == 1) g_direct_control = cmd.args[0];
        scom.CmdOK();
    } else if (strcmp(cmd.command, "dctrl_set") == 0) { // directly set outputs/actators
        // Todo!!!
        scom.CmdOK();  
    } else if (strcmp(cmd.command, "set_mode") == 0) { // change auto/manual mode
        if (cmd.nargs == 2) {
            if (cmd.args[0] < 6) g_auto_state[cmd.args[0]] = cmd.args[1];
        }
        scom.CmdOK();
    } else if (strcmp(cmd.command, "get_point") == 0) { // get Setpoints
        scom.CmdOK();
        scom.SendSP();
    } else if (strcmp(cmd.command, "get_state") == 0) { // get Systemstate
        scom.CmdOK();
        scom.SendST();
    } else if (strcmp(cmd.command, "get_panel") == 0) { // get Panel/Button state
        scom.CmdOK();
        scom.SendCtrl();
    } else if (strcmp(cmd.command, "get_out") == 0) { // get actual output values
        scom.CmdOK();
        scom.SendOut();
    } else {
        scom.CmdFAIL();
    }
}

// copy and evaluate new data from measurement MCU recieved via serial1
void serial_data_read(Measurements M) {
    memcpy(&M,&g_ProcessState,sizeof(M));
}

// use port/register manipulation to quickly shift out 25 bits to set the shift registers for LED output
inline void shiftOut_Fast() {
    uint32_t g_buttons_LEDs = 0;
    // top row
    for (i = 0; i < 12; i+=2) {
        if (g_auto_state[i/2]) {
            g_buttons_LEDs |= _BV(i+1);
        } else {
            g_buttons_LEDs |= _BV(i);
        }
    }
    // bottom row
    for (i = 0; i < 7; i++) {
        if (g_control[i]) {
            g_buttons_LEDs |= uint32_t(1)<<(m2l_idx[i]+12);
            if (i == 2) {
                g_buttons_LEDs &= ~(uint32_t(1)<<(2+12));
            }
        }
    }
    // bottom row - off --> alarm (blink every 500 ms)
    if (millis()%1000 > 500) {
        for (i = 0; i < 6; i++) {
            if (g_alarm[i] & !g_alarm_ignore[i]) {
                g_buttons_LEDs |= uint32_t(1)<<(alarm_idx[i]+12);
            }
        }
    }
    
    cli(); // stop all interrupts
    // Latch pin (17) LOW
    PORTH &= B11111110;
    delayMicroseconds(2);
    for (i = 0; i < 25; i++) {
        // data pin (15) HIGH or LOW
        if (g_buttons_LEDs & (uint32_t(1) << (24 - i))) {
            PORTJ |= B00000001;
        } else {
            PORTJ &= B11111110;
        }
        delayMicroseconds(2);
        // pulse clock pin (16)
        PORTH |= B00000010;
        PORTH &= B11111101;
        delayMicroseconds(2);
    }
    delayMicroseconds(5);
    PORTH |= B00000001; // Latch pin (17) HIGH
    sei(); // enable all interrupts
}

// read all 6 analog potentiometer values into array
inline void readAnalogValues() {
    // read poti values
    for (i = 0; i < 5; i++) {
        g_analog_values[i] = analogRead(A0 + i); // dummy read for adc to settle
        g_analog_values[i] = analogRead(A0 + i);
    }
    // calculate according setpoints if in maual mode (otherwise dont change setpoints)
    if (!g_auto_state[0])
        g_Setpoints.FluidLevel = g_analog_values[0];                        // cm oder L
    if (!g_auto_state[1])
        g_Setpoints.Speed_Filter = map(g_analog_values[1],0, 1024, 0, 360); // RPM
    if (!g_auto_state[2])
        g_Setpoints.Rate_Airation = map(g_analog_values[2],0, 1024, 0, 500); // L/min
    if (!g_auto_state[3])
        g_Setpoints.Speed_FeedPump = map(g_analog_values[3],0, 1024, 0, 500); // ml/min
    if (!g_auto_state[4])
        g_Setpoints.Temp = map(g_analog_values[4],0, 1024, 50, 500);         // °C (zehntel)
    if (!g_auto_state[5])
        g_Setpoints.Concentration_Fraction = map(g_analog_values[5],0, 1024, 0, 1000); // % (zehntel)
}

// set digtial control out using pins 2 - 13
// using port/register manipulation and loops were unrolled to speed execution up
inline void setDigitalControlOut() {
    // set pin 2
    if (g_digital_control_out & _BV(0) ) {
        PORTE |= _BV(PE4); 
    } else {
        PORTE &= ~_BV(PE4);
    }
    // set pin 3
    if (g_digital_control_out & _BV(1)) {
        PORTE |= _BV(PE5);
    } else {
        PORTE &= ~_BV(PE5);
    }
    // set pin 4
    if (g_digital_control_out & _BV(2)) {
        PORTG |= _BV(PG5);
    } else {
        PORTG &= ~_BV(PG5);
    }
    // set pin 5
    if (g_digital_control_out & _BV(3)) {
        PORTE |= _BV(PE3);
    } else {
        PORTE &= ~_BV(PE3);
    }
    // set pin 6
    if (g_digital_control_out & _BV(4)) {
        PORTH |= _BV(PH3);
    } else {
        PORTH &= ~_BV(PH3);
    }
    // set pin 7
    if (g_digital_control_out & _BV(5)) {
        PORTH |= _BV(PH4);
    } else {
        PORTH &= ~_BV(PH4);
    }
    // set pin 8
    if (g_digital_control_out & _BV(6)) {
        PORTH |= _BV(PH5);
    } else {
        PORTH &= ~_BV(PH5);
    }
    // set pin 9
    if (g_digital_control_out & _BV(7)) {
        PORTH |= _BV(PH6);
    } else {
        PORTH &= ~_BV(PH6);
    }
    // set pin 10
    if (g_digital_control_out & _BV(8)) {
        PORTB |= _BV(PB4);
    } else {
        PORTB &= ~_BV(PB4);
    }
    // set pin 11
    if (g_digital_control_out & _BV(9)) {
        PORTB |= _BV(PB5);
    } else {
        PORTB &= ~_BV(PB5);
    }
    // set pin 12
    if (g_digital_control_out & _BV(10)) {
        PORTB |= _BV(PB6);
    } else {
        PORTB &= ~_BV(PB6);
    }
    // set pin 13
    if (g_digital_control_out & _BV(11)) {
        PORTB |= _BV(PB7);
    } else {
        PORTB &= ~_BV(PB7);
    }
}

// set analog control outputs using external DACs connected via SPI
inline void setAnalogControlOut() {
    DAC[0].analogWrite(g_analog_control_out[0]);
    DAC[1].analogWrite(g_analog_control_out[1]);
    DAC[2].analogWrite(g_analog_control_out[2]);
}

// compute and set the actual control outputs
inline void ControlOut() {

    // if in direct control mode ignore everything! actuator values are set by the control PC directly
    // Otherwise use normal Set-Point control scheme
    if (!g_direct_control) {
        // reset every output
        g_digital_control_out = 0;
        g_analog_control_out[0] = 0;
        g_analog_control_out[1] = 0;
        g_analog_control_out[2] = 0;

        // Befüllung
        //   Ventil (digital) auf in Abhängigkeit vom Füllstand
        //   SollFüllstand vom Poti
        if (g_control[0]) {
            if ((g_ProcessState.FluidLevel + FL_offset) < g_Setpoints.FluidLevel) {
                g_digital_control_out |= _BV(4); // Ventil auf
                FL_offset = 0;
            } else {
                FL_offset = 5;
            }
        } else {
            FL_offset = 0;
        }

        // Rotationsfilter
        //   FU Frequenz (Analog) und Richtung ausgeben 
        //   Sollwert vom Poti oder PC
        //   Rechts oder Linkslauf?
        if (g_control[1]) {
            g_analog_control_out[0] = map(g_Setpoints.Speed_Filter,0,360,0,4095);
            if (g_control[2]) { // right or left rotation
                g_digital_control_out |= _BV(8);
            }
        }

        // Belüftung
        //   Proportionalventil (analog) stellen abhängig vom Sollwert
        //   Sollwert vom Poti oder PC
        //   Vorschaltventil (digital) auf oder zu
        if (g_control[3]) {
            g_analog_control_out[2] = map(g_Setpoints.Rate_Airation,0,500,0,4095);
            g_digital_control_out |= _BV(7);  // Vorschaltventil auf
        }

        // Dosierpumpe
        //   Pumprate (analog) stellen abhängig vom Sollwert
        //   Sollwert vom Poti oder PC
        if (g_control[4]) {
            g_analog_control_out[1] = map(g_Setpoints.Speed_FeedPump,0,500,0,4095);
        }

        // Kühlung
        //   Kühlung (digital) ein oder ausschalten, abhängig von SollTemp und IstTemp
        //   SollTemp vom Poti oder PC
        if (g_control[5]) {
            if ((g_ProcessState.Temp1) > g_Setpoints.Temp) {
                g_digital_control_out |= _BV(5); // Kühlboden Ventil auf
                g_digital_control_out |= _BV(6); // Kühlmantel Ventil auf
            }
        }

        // Konzentrierung
        //   Maximalen Füllstand erfassen -> Aufkonzentrierung ist Quotient aus aktuellem und maximalem Füllstand
        //   Abluft (digital) schließen
        //   Druckregler / Druckluft (digital) einschalten um Überdruck zu erzeugen
        //   Auslassventil (digital) auf
        if (g_ProcessState.FluidLevel > maxV) maxV = g_ProcessState.FluidLevel;
        if (g_control[6] & !g_control[0] & !g_control[3]) { // filtration can only happen if airation and filling are both off.
            // Todo!!!
    

        }
    }

    // write the actual output values out to the hardware registers
    setDigitalControlOut();
    setAnalogControlOut();
}

// helper routine to flip the bit order of a byte
inline uint8_t flipByte(uint8_t c) {
    uint8_t r = 0;
    for (i = 0; i < 8; i++) {
        r <<= 1;
        r |= c & 1;
        c >>= 1;
    }
    return r;
}

// state change logic -> when any button has been pressed, change accordingly
inline void state_change() {
    if ((buttons_pressed_top == lbtn_t) & (buttons_pressed_bottom == lbtn_b)) return;
    lbtn_t = buttons_pressed_top;
    lbtn_b = buttons_pressed_bottom;
    // top row
    if (buttons_pressed_top) {
        scom.SendBtn(buttons_pressed_top,buttons_pressed_bottom);
        for (i = 0; i < 12; i++) {
            if ((buttons_pressed_top >> i) & 1 ) {  // bit test for pressed button
                if (i%2) {
                    g_auto_state[i/2] = true;
                } else  {
                    g_auto_state[i/2] = false;
                }
            }
        }
    }
    // bottom row
    if (buttons_pressed_bottom) {
        scom.SendBtn(buttons_pressed_top,buttons_pressed_bottom);
        for (i = 0; i < 13; i++) {
            if ((buttons_pressed_bottom >> i) & 1) {  // bit test for pressed button
                if (!g_auto_state[m2a_idx[i]]) { // if not in manual mode -> ignore button press
                    switch (i) {
                    case 0:
                        g_control[0] = true;
                        break;
                    case 1:
                        g_control[0] = false;
                        if (g_alarm[0]) g_alarm_ignore[0] = true;
                        break;
                    case 2:
                        g_control[1] = true;
                        g_control[2] = false;
                        break;
                    case 3:
                        g_control[1] = false;
                        g_control[2] = false;
                        if (g_alarm[1]) g_alarm_ignore[1] = true;
                        break;
                    case 4:
                        g_control[1] = true;
                        g_control[2] = true;
                        break;
                    case 5:
                        g_control[3] = true;
                        break;
                    case 6:
                        g_control[3] = false;
                        if (g_alarm[2]) g_alarm_ignore[2] = true;
                        break;
                    case 7:
                        g_control[4] = true;
                        break;
                    case 8:
                        g_control[4] = false;
                        if (g_alarm[3]) g_alarm_ignore[3] = true;
                        break;
                    case 9:
                        g_control[5] = true;
                        break;
                    case 10:
                        g_control[5] = false;
                        if (g_alarm[4]) g_alarm_ignore[4] = true;
                        break;
                    case 11:
                        g_control[6] = true;
                        break;
                    case 12:
                        g_control[6] = false;
                        if (g_alarm[5]) g_alarm_ignore[5] = true;
                        break;
                    }
                }
            }
        }
    }
}

// sets or resets the alarms based on deviations between set-points and system-states
inline void resetAlarm() {

    // set alarm if Fluidlevel is 10 above setpoint 
    if (g_Setpoints.FluidLevel < g_ProcessState.FluidLevel - 10) {
        g_alarm[0] = true;
    } else {
        g_alarm[0] = false;
    }

    // set alarm if rotationspeed is off by more than 10 % 
    if ( (g_Setpoints.Speed_Filter < g_ProcessState.Speed_Filter * .9) | (g_Setpoints.Speed_Filter > g_ProcessState.Speed_Filter * 1.1) ) {
        g_alarm[1] = true;
    } else {
        g_alarm[1] = false;
    }

    // set alarm if airation is off by more than 10 % // TODO
    //if ( (g_Setpoints.Rate_Airation < g_ProcessState.Pos_AirationValve * .9) | (g_Setpoints.Rate_Airation > g_ProcessState.Pos_AirationValve * 1.1) ) {
    //    g_alarm[2] = true;
    //} else {
    //    g_alarm[2] = false;
    //}

    // set alarm if feedrate is off by more than 10 % 
    if ( (g_Setpoints.Speed_FeedPump < g_ProcessState.Speed_FeedPump * .9) | (g_Setpoints.Speed_FeedPump > g_ProcessState.Speed_FeedPump * 1.1) ) {
        g_alarm[3] = true;
    } else {
        g_alarm[3] = false;
    }

    // set alarm if Temp is off by more than 5 °C
    if ( (g_Setpoints.Temp < g_ProcessState.Temp1 - 5) | (g_Setpoints.Temp > g_ProcessState.Temp1 + 5) ) {
        g_alarm[4] = true;
    } else {
        g_alarm[4] = false;
    }

    // no alarm for concentrating yet


    // reset "alarm ignore" if actual alarm condition is no longer true
    for (i = 0; i < 6; i++) {
        if (!g_alarm[0]) g_alarm_ignore[0] = false;
    }
}

// ISR for button polling. Executed every 50 ms
ISR(TIMER5_COMPA_vect) {
    buttons_pressed_top = uint16_t(PINA) + ((uint16_t(flipByte(PINC & 0xF0)) << 8));
    buttons_pressed_bottom = (flipByte(PINC) >> 4) + (flipByte(PINL) << 4) + ((PINJ & 0x02) << 11);
}

// initializes the 6 LCD screens
inline void initLCD() {
    for (i = 0; i < 6; i++) {
        LCD[i].init();
        LCD[i].backlight();
        Serial.print("*");
    }
    LCD[0].setCursor(0, 0);
    LCD[0].print("Soll:      L");
    LCD[0].setCursor(0, 1);
    LCD[0].print("Ist :      L");
    LCD[1].setCursor(0, 0);
    LCD[1].print("Soll:      U/min");
    LCD[1].setCursor(0, 1);
    LCD[1].print("Ist :      U/min");
    LCD[2].setCursor(0, 0);
    LCD[2].print("Soll:      L/min");
    LCD[2].setCursor(0, 1);
    LCD[2].print("Ist :      L/min");
    LCD[3].setCursor(0, 0);
    LCD[3].print("Soll:      L/h");
    LCD[3].setCursor(0, 1);
    LCD[3].print("Ist :      L/h");
    LCD[4].setCursor(0, 0);
    LCD[4].print("Soll:      C");
    LCD[4].setCursor(0, 1);
    LCD[4].print("Ist :      C");
    LCD[5].setCursor(0, 0);
    LCD[5].print("Soll:      %");
    LCD[5].setCursor(0, 1);
    LCD[5].print("Ist :      %");
}

// updates the LCD screen content based on actual values
inline void updateLCD() {
    // LCD 1 -> fill level
    LCD[0].setCursor(6, 0);
    LCD[0].print(g_Setpoints.FluidLevel);
    LCD[0].setCursor(6, 1);
    LCD[0].print(g_ProcessState.FluidLevel);
    // LCD 2 -> Rotation speed of filter disks
    LCD[1].setCursor(6, 0);
    LCD[1].print(g_Setpoints.Speed_Filter);
    LCD[1].setCursor(6, 1);
    LCD[1].print(g_ProcessState.Speed_Filter);
    // LCD 3 -> Airation rate
    LCD[2].setCursor(6, 0);
    LCD[2].print(g_Setpoints.Rate_Airation);
    LCD[2].setCursor(6, 1);
    LCD[2].print(g_ProcessState.Pos_AirationValve);
    // LCD 4 -> Feed pump speed / feed rate
    LCD[3].setCursor(6, 0);
    LCD[3].print(g_Setpoints.Speed_FeedPump);
    LCD[3].setCursor(6, 1);
    LCD[3].print(g_ProcessState.Speed_FeedPump);
    // LCD 5 -> Temperature
    LCD[4].setCursor(6, 0);
    LCD[4].print(g_Setpoints.Temp);
    LCD[4].setCursor(6, 1);
    LCD[4].print(g_ProcessState.Temp1);
    // LCD 6 -> Concentrating
    LCD[5].setCursor(6, 0);
    LCD[5].print(g_Setpoints.Concentration_Fraction);
    LCD[5].setCursor(6, 1);
    g_ProcessState.Concentration_Fraction = g_ProcessState.FluidLevel * 100 / maxV;
    LCD[5].print(g_ProcessState.Concentration_Fraction);
}

void setup() {
    // ****** setup serial communication ******
    scom.begin(SERIAL_BAUD_RATE, serial_cmd_exec, serial_data_read);

    // ****** setup LC displays ******
    Serial.print(F("init displays: "));
    initLCD();
    scom.OK();
    
    // ****** setup DAC ******
    Serial.print(F("init DACs:"));
    DAC[0].begin(38);
    DAC[1].begin(39);
    DAC[2].begin(40);
    scom.OK();

    // ****** setup pinmodes ******
    Serial.print(F("init pins:"));
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

    // SPI/DAC Chipselect -> output
    DDRG |= B00000111; // pin 39, 40 & 41
    DDRD |= B01000000; // pin 38
    scom.OK();
    
    // ****** setup timer interrupt for 50 ms button polling ******
    Serial.print(F("init button polling interrupts:"));
    cli(); // stop all interrupts
    TCCR5A = 0;
    TCCR5B = 0;
    TCNT5 = 0;
    OCR5A = 781;
    TCCR5B |= (1 << WGM52);
    TCCR5B |= (1 << CS52) | (1 << CS50);
    TIMSK5 |= (1 << OCIE5A);
    sei(); // enable interrupts
    scom.OK();
}

void loop() {
    // state change - react on button inputs if there were any and set or reset any alarms if applicable
    state_change();
    resetAlarm();

    // input - read Sensor states
    readAnalogValues();

    // output - button LEDs
    shiftOut_Fast();

    // output - compute control output and perform actions accordingly
    ControlOut();

    // update LC displays
    updateLCD();

    // execute Serial Comms events
    scom.read_serial0(); // read commands from control PC if there are any
    scom.read_serial1(); // read sensor data from measure_arduino if there is any available
}