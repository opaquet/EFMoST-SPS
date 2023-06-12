#include "globals.h"
#include "serial_com.h"
#include "EFMoST_IO.h"
#include "LCD_control.h"
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>

// *** global variables ***
uint16_t        g_analog_control_out[3] = { 0, 0, 0 };
uint16_t        g_digital_control_out   = 0;
boolean         g_auto_state[6]         = {false, false, false, false, false, false};
boolean         g_alarm[6]              = {false, false, false, false, false, false};
boolean         g_alarm_ignore[6]       = {false, false, false, false, false, false};
boolean         g_control[7]            = {false, false, false, false, false, false, false};
uint16_t        g_ProcessState[16]      = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t        g_Setpoints[6]          = {0,0,0,0,0,0};
boolean         g_direct_control        = false;
uint16_t        g_errorCode             = 0;  // 16 error bits that can be set as seen fit. Will be sent with each data packet, so that pc knows if certain errors occoured


// *** local variables ***
const uint8_t   m2a_idx[13]             = {0,0,1,1,1,2,2,3,3,4,4,5,5};
const uint8_t   m2l_idx[7]              = {0,2,4,5,7,9,11};
const uint8_t   alarm_idx[6]            = {1,3,6,8,10,12};
uint16_t        maxV                    = 0; // maximum fill level ever recoded
uint16_t        lbtn_t, lbtn_b;   //last button state to avoid double reaction
Serial_Com      scom;
IO              IO_handler;
LCD_control     LCDs;

// helper routine to flip the bit order of a byte
inline uint8_t flipByte(uint8_t c) {
    uint8_t r = 0;
    for (uint8_t i = 0; i < 8; i++) {
        r <<= 1;
        r |= c & 1;
        c >>= 1;
    }
    return r;
}

// ISR for button polling. Executed every 50 ms
ISR(TIMER5_COMPA_vect) {
    // store value temporarily in variable and check during next excution if button is still pressed by 
    // ANDing the temp state and the actual button state. So a Button press is only detected if a button is pressed for at least 50 ms
    // (or twice with 50 ms delay, which is highly unlikely)

    //read top buttons state
    IO_handler.buttons_pressed_temp = uint16_t(PINA) + ((uint16_t(flipByte(PINC & 0xF0)) << 8));
    IO_handler.buttons_pressed_top = IO_handler.buttons_pressed_temp & IO_handler.buttons_pressed_top_last;
    IO_handler.buttons_pressed_top_last = IO_handler.buttons_pressed_temp;

    //read bottom buttons state
    IO_handler.buttons_pressed_temp = (flipByte(PINC) >> 4) + (flipByte(PINL) << 4) + ((PINJ & 0x02) << 11);
    IO_handler.buttons_pressed_bottom = IO_handler.buttons_pressed_temp & IO_handler.buttons_pressed_bottom_last;
    IO_handler.buttons_pressed_bottom_last = IO_handler.buttons_pressed_temp;
}

// interpret and execute commands that were recieved via serial0
void serial_cmd_exec(serial_cmd_t cmd) {
    boolean cmdOK = false;
    if (strcmp(cmd.command, "set_point") == 0) {  // set Setpoints (only when in automatic mode)
        if ((cmd.nargs == 2) && (cmd.args[0] < 6) && (g_auto_state[cmd.args[0]])) {
            g_Setpoints[cmd.args[0]] = cmd.args[1]; 
            cmdOK = true;     
        } 
    } else if (strcmp(cmd.command, "alarm_ignore") == 0) { // ignore alarm
        if ((cmd.nargs == 1) && (cmd.args[0] < 6)) {
            g_alarm_ignore[cmd.args[0]] = true;
            cmdOK = true; 
        } 
        if (cmd.nargs == 0) {
            for (uint8_t i=0; i<6; i++) {
                g_alarm_ignore[i] = true;
            }
            cmdOK = true; 
        } 
    } else if (strcmp(cmd.command, "dctrl") == 0) { // enable or disable direct control mode
        if (cmd.nargs == 1) g_direct_control = cmd.args[0];
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
        cmdOK = true; 
        scom.lastsend = 0;
    } else if (strcmp(cmd.command, "dctrl_set") == 0) { // directly set outputs/actators
         if (cmd.nargs == 3) {
            cmdOK = true; 
            switch (cmd.args[0]) {
            case 0:
                if (cmd.args[1] > 16) break;
                if (cmd.args[2]) {
                    g_digital_control_out |= _BV(cmd.args[1]);
                } else {
                    g_digital_control_out &= ~_BV(cmd.args[1]);
                }
                scom.lastsend = 0;
                break;
            case 1:
                if (cmd.args[1] > 3) break;
                g_analog_control_out[cmd.args[1]] = cmd.args[2];
                scom.lastsend = 0;
                break;
            default:
                cmdOK = false; 
                break;
            }
        } 
    } else if (strcmp(cmd.command, "set_mode") == 0) { // change auto/manual mode
        if ((cmd.nargs == 2) && (cmd.args[0] < 6)) {
            g_auto_state[cmd.args[0]] = cmd.args[1];
            cmdOK = true; 
            scom.lastsend = 0;
        } 
    } else if (strcmp(cmd.command, "mmcu_baud") == 0) { // change auto/manual mode
        if ((cmd.nargs == 1) && scom.validBaud(cmd.args[0])) {
            scom.reinit1(cmd.args[0]);
            cmdOK = true; 
        } 
    } else if (strcmp(cmd.command, "sps_baud") == 0) { // change auto/manual mode
        if ((cmd.nargs == 1) && scom.validBaud(cmd.args[0])) {
            scom.reinit0(cmd.args[0]);
            cmdOK = true; 
        } 
    } else if (strcmp(cmd.command, "set_active") == 0) { // /(de)activate control outputs (if mode is set to auto)
        if ((cmd.nargs == 2) && (cmd.args[0] < 6) && (g_auto_state[cmd.args[0]])) {
            g_control[cmd.args[0]] = cmd.args[1];
            cmdOK = true; 
            scom.lastsend = 0;
        } 
    } else if (strcmp(cmd.command, "reset") == 0) { // reset mcu
        scom.CmdOK();
        delay(100);
        asm volatile ("jmp 0");  //inline assembly: jump to adress 0 -> jump to first interrupt vector (reset/reinitialize on most Atmel MCUs)
        return; //will (or should) never be reached...
    } else if (strcmp(cmd.command, "relay_serial") == 0) { // enable or disable the relaying of any serial data recieved from serial1 out to serial0
        if (cmd.nargs == 1) {
            scom.relay_serial = cmd.args[0];
            cmdOK = true; 
        }
    } else if (strcmp(cmd.command, "send_freq") == 0) { // change send frequency
        if (cmd.nargs == 1) {
            if (cmd.args[0] < 5760000/scom.baud0)  // make sure that send frequency is not too fast so that all data can still be trasmitted... on very low baud rates reduce send frequency if necessary
                scom.autosend_delay = 5760000/scom.baud0;
            else
                scom.autosend_delay = cmd.args[0];
            cmdOK = true; 
        }
    } else if (strcmp(cmd.command, "state") == 0) { // get Systemstate
        scom.SendStateJSON();
        return; // send no other confirmation
    } else if (strcmp(cmd.command, "connect") == 0) { // send device information and start autosend
        scom.autosend = true;
        cmdOK = true; 
    } else if (strcmp(cmd.command, "dev") == 0) { // send device information and start autosend
        scom.SendDev();
        return; // send no other confirmation
    } else if (strcmp(cmd.command, "disconnect") == 0) { // send device information and start autosend
        cmdOK = true; 
        scom.autosend = false;
    }
    if (cmdOK) scom.CmdOK();
    else scom.CmdFAIL();
}

// copy and evaluate new data from measurement MCU recieved via serial1
// M is 12 long -> temp1, pH, temp2, conductivity, oxygen, temp3, distance, p1, p2, H2S, feedrate, airation_valve_pos
void serial_data_read(uint16_t * M) {
    if ((M[0] > 100) & (M[0] < 5000)) g_ProcessState[Temp] = M[0]; //valide temperatur zwischen 1 und 50 °C
    g_ProcessState[pH] = M[1];
    if ((M[2] > 100) & (M[2] < 5000)) g_ProcessState[Temp2] = M[2];
    //g_ProcessState[Conduct] = M[3];
    g_ProcessState[Ox]= M[3];
    if ((M[5] > 100) & (M[4] < 5000)) g_ProcessState[Temp3] = M[4];
    g_ProcessState[pOx] = M[5];
    g_ProcessState[Distance] = M[6];
    if ((M[7] < 4000)) g_ProcessState[Press1] = M[7]; //valider druck zwischen 0,8 und 4 bar
    if ((M[8] < 4000)) g_ProcessState[Press2] = M[8];
    //g_ProcessState[H2S] = M[9];
    g_ProcessState[FeedRate] = map(M[9], 0, 1023, MinFeed, MaxFeed);
    g_ProcessState[Airation] = M[10];
    g_ProcessState[FilterSpeed] = map(M[11],0,1023,MinRPM,MaxRPM);

    // Fluidlevel ist druckdifferenz zwischen boden udn deckel druck. Sollte diese differenz negative sein, setzte level auf 0
    if (g_ProcessState[Press1] > g_ProcessState[Press2])
        g_ProcessState[FluidLevel] = (g_ProcessState[Press1] - g_ProcessState[Press2]) ;
    else
        g_ProcessState[FluidLevel] = 0;


    // tracke maximalen Fluidlevel für die Konzentrierung/Filterung am Ende, aber nur wenn fluid level control oder Feed aktiv ist
    if ((g_control[0] & g_control[4]) & (g_ProcessState[FluidLevel] > maxV)) maxV = g_ProcessState[FluidLevel];

    // Concentration fraction berechnen - wieviel volumen habe ich noch verglichen zum maximalvolumen (in 1000)
    if (maxV > 0) {
        g_ProcessState[ConcentrationFraction] = (g_ProcessState[FluidLevel] * 1000) / maxV;
    } else {
        g_ProcessState[ConcentrationFraction] = 1000;
    }
}

// Set LED states according to system state and shift values out to shif registers
inline void Set_LED() {
    uint32_t g_buttons_LEDs = 0;
    // blink all auto leds when in direct control mode
    if (g_direct_control) {
        if (millis()%900 > 450) {
            for (uint8_t i = 0; i < 6; i++) {
                g_buttons_LEDs |= _BV(i*2+1);
            }
        }
    } else {
        // top row
        for (uint8_t i = 0; i < 12; i+=2) {
            if (g_auto_state[i/2]) {
                g_buttons_LEDs |= _BV(i+1);
            } else {
                g_buttons_LEDs |= _BV(i);
            }
        }
        // bottom row
        for (uint8_t i = 0; i < 7; i++) {
            if (g_control[i]) {
                g_buttons_LEDs |= uint32_t(1)<<(m2l_idx[i]+12);
                if (i == 2) {
                    g_buttons_LEDs &= ~(uint32_t(1)<<(2+12));
                }
            }
        }
    }

    // bottom row - off --> alarm (blink every 500 ms)
    if (millis()%1000 > 500) {
        for (uint8_t i = 0; i < 6; i++) {
            if (g_alarm[i] & !g_alarm_ignore[i]) {
                g_buttons_LEDs |= uint32_t(1)<<(alarm_idx[i]+12);
            }
        }
    }
    IO_handler.LED_out(g_buttons_LEDs);
}

// read all 6 analog potentiometer values into array and map to appropriate range
inline void GetSetpointValues() {
    // read poti values
    uint16_t * analog_Val = IO_handler.analog_in();

    // calculate according setpoints if in maual mode (otherwise dont change setpoints)
    if (!g_auto_state[0])
        g_Setpoints[0] = map(analog_Val[FluidLevel],            0, 1023, MinLevel, MaxLevel);       // cm oder L
    if (!g_auto_state[1])
        g_Setpoints[1] = map(analog_Val[FilterSpeed],           0, 1023, MinRPM, MaxRPM);           // RPM
    if (!g_auto_state[2])
        g_Setpoints[2] = map(analog_Val[Airation],              0, 1023, MinAiration, MaxAiration); // L/min
    if (!g_auto_state[3])
        g_Setpoints[3] = map(analog_Val[FeedRate],              0, 1023, MinFeed, MaxFeed);         // L/h
    if (!g_auto_state[4])
        g_Setpoints[4] = map(analog_Val[Temp],                  0, 1023, MinTemp, MaxTemp);         // °C (zehntel)
    if (!g_auto_state[5])
        g_Setpoints[5] = map(analog_Val[ConcentrationFraction], 0, 1023, MinConc, MaxConc);         // % (zehntel)
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
            if ((g_ProcessState[FluidLevel]) < g_Setpoints[FluidLevel]) {
                g_digital_control_out |= _BV(4); // Ventil auf
            } else {
                //g_control[0] = false; // Steuerung aus
            }
        }

        // Rotationsfilter
        //   FU Frequenz (Analog) und Richtung ausgeben 
        //   Sollwert vom Poti oder PC
        //   Rechts oder Linkslauf?
        if (g_control[1]) {
            g_analog_control_out[0] = map(g_Setpoints[FilterSpeed], MinRPM, MaxRPM, 0, 4095);
            if (g_control[2]) { // right or left rotation
                g_digital_control_out |= _BV(8);
            } else {
                g_digital_control_out |= _BV(9);
            }
        }

        // Belüftung
        //   Proportionalventil (analog) stellen abhängig vom Sollwert
        //   Sollwert vom Poti oder PC
        //   Vorschaltventil (digital) auf oder zu
        if (g_control[3]) {
            g_analog_control_out[2] = map(g_Setpoints[Airation], MinAiration, MaxAiration, 0, 4095);
            g_digital_control_out |= _BV(1);  // Vorschaltventil auf
            g_digital_control_out |= _BV(3);  // Freigabe Propventil
        }

        // Dosierpumpe
        //   Pumprate (analog) stellen abhängig vom Sollwert
        //   Sollwert vom Poti oder PC
        if (g_control[4]) {
            g_analog_control_out[1] = map(g_Setpoints[FeedRate], MinFeed, MaxFeed, 0, 4095);
        }

        // Kühlung
        //   Kühlung (digital) ein oder ausschalten, abhängig von SollTemp und IstTemp
        //   SollTemp vom Poti oder PC
        if (g_control[5]) {
            if ((g_ProcessState[Temp]) > g_Setpoints[Temp]) {
                g_digital_control_out |= _BV(5); // Kühlboden Ventil auf         
            }
            if ((g_ProcessState[Temp]) > g_Setpoints[Temp]+1) {
                g_digital_control_out |= _BV(6); // Kühlmantel Ventil auf
            }

        }

        // Konzentrierung
        //   Maximalen Füllstand erfassen -> Aufkonzentrierung ist Quotient aus aktuellem und maximalem Füllstand
        //   Abluft (digital) schließen
        //   Druckregler / Druckluft (digital) einschalten um Überdruck zu erzeugen
        //   Auslassventil (digital) auf
        if (g_control[6]) {
        //if (g_control[6] & !g_control[0] & !g_control[3]) { // filtration can only happen if airation and filling are both off.
        //    if ((g_ProcessState[ConcentrationFraction]) > g_Setpoints[ConcentrationFraction]) {
                g_digital_control_out |= _BV(0); // Motorventil auf
                g_digital_control_out |= _BV(2); // Konzentreirung Druckluft auf
        //    } else {
        //        g_control[6] = false; //Konzentrierung/Filterung stoppen
        //    }
        }
    }

    // write the actual output values out to the hardware registers
    IO_handler.digital_out(g_digital_control_out);
    IO_handler.analog_out(g_analog_control_out);
}

// state change logic -> when any button has been pressed, change accordingly
inline void state_change() {
    if ((IO_handler.buttons_pressed_top == lbtn_t) & (IO_handler.buttons_pressed_bottom == lbtn_b)) return;
    lbtn_t = IO_handler.buttons_pressed_top;
    lbtn_b = IO_handler.buttons_pressed_bottom;
    // top row
    if (IO_handler.buttons_pressed_top) {
        scom.SendBtn(IO_handler.buttons_pressed_top,IO_handler.buttons_pressed_bottom);
        for (uint8_t i = 0; i < 12; i++) {
            if ((IO_handler.buttons_pressed_top >> i) & 1 ) {  // bit test for pressed button
                if (i%2) {
                    g_auto_state[i/2] = true;
                } else  {
                    g_auto_state[i/2] = false;
                    g_direct_control = false;
                }
            }
        }
    }
    // bottom row
    if (IO_handler.buttons_pressed_bottom) {
        scom.SendBtn(IO_handler.buttons_pressed_top,IO_handler.buttons_pressed_bottom);
        for (uint8_t i = 0; i < 13; i++) {
            if ((IO_handler.buttons_pressed_bottom >> i) & 1) {  // bit test for pressed button
                if (!g_auto_state[m2a_idx[i]]) { // if not in manual mode -> ignore button press
                    switch (i) {
                    case 0:
                        g_control[0] = true;
                        break;
                    case 1:
                        if (g_alarm[0]) g_alarm_ignore[0] = true;
                        else g_control[0] = false;
                        break;
                    case 2:
                        g_control[1] = true;
                        g_control[2] = false;
                        break;
                    case 3:
                        if (g_alarm[1]) g_alarm_ignore[1] = true;
                        else {
                            g_control[1] = false;
                            g_control[2] = false;
                        }
                        break;
                    case 4:
                        g_control[1] = true;
                        g_control[2] = true;
                        break;
                    case 5:
                        g_control[3] = true;
                        break;
                    case 6:
                        if (g_alarm[2]) g_alarm_ignore[2] = true;
                        else g_control[3] = false;
                        break;
                    case 7:
                        g_control[4] = true;
                        break;
                    case 8:
                        if (g_alarm[3]) g_alarm_ignore[3] = true;
                        else g_control[4] = false;
                        break;
                    case 9:
                        g_control[5] = true;
                        break;
                    case 10:
                        if (g_alarm[4]) g_alarm_ignore[4] = true;
                        else g_control[5] = false;
                        break;
                    case 11:
                        g_control[6] = true;
                        break;
                    case 12:
                        if (g_alarm[5]) g_alarm_ignore[5] = true;
                        else g_control[6] = false;
                        break;
                    }
                }
            }
        }
    }
}

// sets or resets the alarms based on deviations between set-points and system-states
inline void resetAlarm() {
    uint16_t Threshold = 0;
    // set alarm if Fluidlevel is 10 above setpoint 
    Threshold               = 10;
    g_alarm[FluidLevel]     = (g_Setpoints[FluidLevel] > (g_ProcessState[FluidLevel] + Threshold));

    // set alarm if rotationspeed is off by more than 10 %    
    Threshold               = g_Setpoints[FilterSpeed] * .9;
    g_alarm[FilterSpeed]    = ((g_Setpoints[FilterSpeed] < (g_ProcessState[FilterSpeed] - Threshold))   | (g_Setpoints[FilterSpeed] > (g_ProcessState[FilterSpeed] + Threshold)));

    // set alarm if airation is off by more than 10 % 
    Threshold               = g_Setpoints[Airation] * .9;
    g_alarm[Airation]       = ((g_Setpoints[Airation] < (g_ProcessState[Airation] - Threshold))         | (g_Setpoints[Airation] > (g_ProcessState[Airation] + Threshold )));

    // set alarm if feedrate is off by more than 10 % 
    Threshold               = g_Setpoints[FeedRate] * .9;
    g_alarm[FeedRate]       = ((g_Setpoints[FeedRate] < (g_ProcessState[FeedRate] - Threshold))         | (g_Setpoints[FeedRate] > (g_ProcessState[FeedRate] + Threshold )));

    // set alarm if Temp is off by more than 5 °C
    Threshold               = 5;
    g_alarm[Temp]           = ((g_Setpoints[Temp] < (g_ProcessState[Temp] - Threshold))                 | (g_Setpoints[Temp] > (g_ProcessState[Temp] + Threshold)));

    // no alarm for concentrating, instead set alarm if last measurement is older than 60 seconds
    g_alarm[5]              = ((millis() - scom.time_since_last_measurement) > 60000);
    
    if (g_alarm[5])
        g_errorCode &= _BV(2);

    // reset "alarm ignore" if actual alarm condition is no longer true
    for (uint8_t i = 0; i < 5; i++) {
       if (!g_alarm[i]) g_alarm_ignore[i] = false;
    }
}

void setup() {
    // ****** setup serial communication ******
    scom.begin(serial_cmd_exec, serial_data_read);

    // ****** setup LC displays ******
    Serial.print(F("{\"init\":{\"displays\":["));
    LCDs.begin();
    Serial.print(F("],\"io\":"));
    // ****** setup IO handling ******
    IO_handler.begin();
    Serial.println(F("1}}"));
}

void loop() {
    g_errorCode = 0; // reset error codes

    // state change - react on button inputs if there were any and set or reset any alarms if applicable
    state_change();
    resetAlarm();

    // input - read Sensor states
    GetSetpointValues();

    // update LC displays
    LCDs.display();

    // execute Serial Comms events
    scom.read_serial0(); // read commands from control PC if there are any
    scom.read_serial1(); // read sensor data from measure_arduino if there is any available

    // output - compute control output and perform actions accordingly
    ControlOut();

    // output - button LEDs
    Set_LED();

    // send out system state and error codes over serial in regular (more or less) time interval as set in serial_com
    if ((scom.autosend) & (millis() - scom.lastsend >= scom.autosend_delay)) {
        scom.SendStateJSON();
    }
    
}