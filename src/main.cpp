#include "globals.h"
#include "serial_com.h"
#include "EFMoST_IO.h"
#include "LCD_control.h"
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>

#pragma region global variables
// *** global variables ***
uint16_t        g_analog_control_out[4] = { 0, 0, 0, 0 };
uint16_t        g_digital_control_out   = 0;
boolean         g_auto_state[6]         = {false, false, false, false, false, false};
boolean         g_alarm[6]              = {false, false, false, false, false, false};
boolean         g_alarm_ignore[6]       = {false, false, false, false, false, false};
boolean         g_control[7]            = {false, false, false, false, false, false, false};
boolean         g_foam                  = false;
boolean         g_state_changed         = true;
boolean         g_pump_ctrl_pwm         = false;
boolean         g_pump_ctrl_pwm_state   = false;
uint16_t        g_pump_ctrl_pwm_speed   = 128;
uint32_t        g_pump_ctrl_pwm_interval= 10000;
uint32_t        g_pump_ctrl_pwm_ontime  = 0;
uint32_t        g_pump_ctrl_pwm_offtime = 0;
uint16_t        g_foam_trigger_counter  = 0;
uint16_t        g_foam_trigger_counter_long = 0;
uint16_t        g_ProcessState[17]      = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t        g_Setpoints[6]          = {0,0,0,0,0,0};
boolean         g_direct_control        = false;
uint16_t        g_errorCode             = 0;  // 16 error bits that can be set as seen fit. Will be sent with each data packet, so that controller pc knows if certain errors occoured
#pragma endregion

#pragma region local variables
// *** local variables ***
const uint8_t   m2a_idx[13]             = {0,0,1,1,1,2,2,3,3,4,4,5,5};
const uint8_t   m2l_idx[7]              = {0,2,4,5,7,9,11};
const uint8_t   alarm_idx[6]            = {1,3,6,8,10,12};
uint16_t        maxV                    = 0; // maximum fill level ever recoded
uint16_t        lbtn_t, lbtn_b;   //last button state to avoid double reaction
uint16_t        foam_cleaning_duration  = 0;
Serial_Com      scom;
IO              IO_handler;
LCD_control     LCDs;
#pragma endregion

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

// Method to run after new measurement data has been recieved and interpreted
void process_serial_data() {
    // Fluidlevel ist druckdifferenz zwischen boden und deckel druck. Sollte diese differenz negativ sein, setzte level auf 0
    if (g_ProcessState[Press1] > g_ProcessState[Press2])
        g_ProcessState[FluidLevel] = (g_ProcessState[Press1] - g_ProcessState[Press2]) ;
    else
        g_ProcessState[FluidLevel] = 0;

    // tracke maximalen Fluidlevel für die Konzentrierung/Filterung am Ende, aber nur wenn fluid level control oder Feed aktiv ist
    if ((g_control[0] && g_control[4]) && (g_ProcessState[FluidLevel] > maxV)) maxV = g_ProcessState[FluidLevel];

    // Concentration fraction berechnen - wieviel volumen habe ich noch, verglichen zum MaximalVolumen (in 1000)
    if (maxV > 0) {
        g_ProcessState[ConcentrationFraction] = (g_ProcessState[FluidLevel] * 1000) / maxV;
    } else {
        g_ProcessState[ConcentrationFraction] = 1000;
    }

    // reset failsafe foam trigger counter since valid date has been recieved
    g_foam_trigger_counter_long = 0; 
    
    // Foam detection and destruction
    if (g_ProcessState[Foam] == 1) { 
        g_foam_trigger_counter++;
    } else {
        g_foam_trigger_counter = 0;
    }

    // Todo!!! --> cleaning_duration wird immer wieder zurück gesetzt, wenn das Schaumsignal weiterhin anliegt
    // Dadurch läuft die Pumpe ohne Unterbrechung, bis der Schaum weg ist...
    // So lassen !?
    // Evtl löst sich der Schaumtrigger so endlos von selbst aus?

    if ((g_foam_trigger_counter == 5)) { // 5 consecutive foam signals needed to trigger cleaning procedure
        g_foam = true;
        foam_cleaning_duration = 0;
    }
}

// Set LED states according to system state and shift values out to shift registers
inline void Set_LED() {
    uint32_t g_buttons_LEDs = 0;
    // blink all auto leds when in direct control mode
    if (g_direct_control) {
        if (millis()%600 > 400) {
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
    // bottom row - off --> alarm (blink every 1 s for 200 ms)
    if (millis()%1000 > 800) {
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
    int16_t v;
    // calculate according setpoints if in manual mode (otherwise dont change setpoints)
    if (!g_auto_state[FluidLevel]) {
        v = map(analog_Val[FluidLevel],            0, 1023, MinLevel, MaxLevel);       // cm oder L
        g_state_changed |= abs(v - (int16_t)g_Setpoints[FluidLevel]) > 2;
        g_Setpoints[FluidLevel] = v;
    }
    if (!g_auto_state[FilterSpeed]) {
        v = analog_Val[FilterSpeed];
        g_state_changed |= abs(v - (int16_t)g_Setpoints[FilterSpeed]) > 2;
        g_Setpoints[FilterSpeed] = v;
    }
    if (!g_auto_state[Airation]) {
        v = analog_Val[Airation];
        g_state_changed |= abs(v - (int16_t)g_Setpoints[Airation]) > 2;
        g_Setpoints[Airation] = v;
    }
    if (!g_auto_state[FeedRate]) {
        v = analog_Val[FeedRate];
        g_state_changed |= abs(v - (int16_t)g_Setpoints[FeedRate]) > 2;
        g_Setpoints[FeedRate] = v;
    }
    if (!g_auto_state[Temp]) {
        v = map(analog_Val[Temp],                  0, 1023, MinTemp, MaxTemp);         // °C (zehntel)
        g_state_changed |= abs(v - (int16_t)g_Setpoints[Temp]) > 2;
        g_Setpoints[Temp] = v;
    }
    if (!g_auto_state[ConcentrationFraction]) {
        v = map(analog_Val[ConcentrationFraction], 0, 1023, MinConc, MaxConc);         // % (zehntel)
        g_state_changed |= abs(v - (int16_t)g_Setpoints[ConcentrationFraction]) > 2;
        g_Setpoints[ConcentrationFraction] = v;
    }
}



// compute and set the actual control outputs
inline void ControlOut() {

    // if in direct control mode ignore everything! actuator values are set by the control PC directly
    // Otherwise use normal Set-Point control scheme
    if (!g_direct_control) {
        // reset every output
        g_digital_control_out = 0;
        g_analog_control_out[0] = 0;
        if (!g_pump_ctrl_pwm) g_analog_control_out[1] = 0;
        g_analog_control_out[2] = 0;
        g_analog_control_out[3] = 0;

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
            g_analog_control_out[0] = g_Setpoints[FilterSpeed] << 2; // multiply by 4 (shift left by 2 bits) to get from 10 to 12 bit range
            if (g_control[2]) { // right or left rotation
                g_digital_control_out |= _BV(8);
            } else {
                g_digital_control_out |= _BV(9);
            }
        }

        // Belüftung
        //   Proportionalventil (analog) stellen abhängig vom Sollwert
        //   Sollwert vom Poti oder PC
        //   Vorschaltventil (digital) auf oder zu (wenn Luftstrom größer als 0, sonst Vprschaltentil immer zu)
        if (g_control[3]) {
            g_analog_control_out[2] = g_Setpoints[Airation] << 2; // multiply by 4 (shift left by 2 bits) to get from 10 to 12 bit range
            if (g_Setpoints[Airation] > 0) {
                g_digital_control_out |= _BV(1);  // Vorschaltventil auf
            }
            g_digital_control_out |= _BV(3);  // Freigabe Propventil
        }

        // Schaumzerstörung (an Belüftung gekoppelt)
        //    Wenn Schaum erkannt wurde, Schaumzerstörung einschalten für 20 s
        //if (g_control[3]) {
            if (g_foam) {
                g_digital_control_out |= _BV(10);  // Relais K20 (Reserve 1) ein
            }
        //}

        // Dosierpumpe
        // PWM Modus, wenn die Pumpe mit weniger 75% (--> 96) von der Pumpgeschwindigkeit im PWM Modus (128) betrieben werden soll.
        g_pump_ctrl_pwm = (g_Setpoints[FeedRate] < 96) ? true : false;// 75 % von 128 --> 75 % ist der maximale duty cycle

        // Wenn in PWM Modus, schalte Pumpe ein und aus in Intervallen
        if (g_pump_ctrl_pwm) {
            if (g_control[4] && !g_pump_ctrl_pwm_state && millis() > g_pump_ctrl_pwm_ontime + g_pump_ctrl_pwm_interval) {
                // pumpe an und berechne, wann pumpe ausgeschaltet werden muss
                g_pump_ctrl_pwm_ontime = millis();
                g_pump_ctrl_pwm_state = true;
                g_analog_control_out[1] = g_pump_ctrl_pwm_speed << 2; // 1/8 of max speed
                uint32_t delta_t = (g_Setpoints[FeedRate] * g_pump_ctrl_pwm_interval) / g_pump_ctrl_pwm_speed;
                g_pump_ctrl_pwm_offtime = delta_t + g_pump_ctrl_pwm_ontime;
                 Serial.print("An: ");
                Serial.println(g_analog_control_out[1] );
            }
            // pumpe aus
            if (g_pump_ctrl_pwm_state && millis() > g_pump_ctrl_pwm_offtime) {
                g_analog_control_out[1] = 0;
                g_pump_ctrl_pwm_state = false;
                Serial.print("Aus: ");
                Serial.println(g_analog_control_out[1]);
            }
        } else
            //   Pumprate (analog) stellen abhängig vom Sollwert
            //   Sollwert vom Poti oder PC
            if (g_control[4]) {
                g_analog_control_out[1] = g_Setpoints[FeedRate] << 2; // multiply by 4 (shift left by 2 bits) to get from 10 to 12 bit range
            }

        // Kühlung
        //   Kühlung (digital) ein oder ausschalten, abhängig von SollTemp und IstTemp
        //   SollTemp vom Poti oder PC
        uint8_t temperature_deadband = 5; // Setpoint +/- Deadband ist OK. Deadband 5 = 0.5 °C
        if (g_control[5]) {
            if ((g_ProcessState[Temp]) < g_Setpoints[Temp] - temperature_deadband) {
                g_digital_control_out |= _BV(6); // Kühlboden Ventil auf         
            }
            if ((g_ProcessState[Temp]) > g_Setpoints[Temp] + temperature_deadband) {
                g_digital_control_out |= _BV(5); // Kühlmantel Ventil auf
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
    // if the button is still pressed or the buuton "state" did not change, exit right away
    if ((IO_handler.buttons_pressed_top == lbtn_t) & (IO_handler.buttons_pressed_bottom == lbtn_b)) return;
    lbtn_t = IO_handler.buttons_pressed_top;
    lbtn_b = IO_handler.buttons_pressed_bottom;
    // top row
    if (IO_handler.buttons_pressed_top) {
        scom.SendBtn(IO_handler.buttons_pressed_top,IO_handler.buttons_pressed_bottom);
        g_state_changed = true;
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
        g_state_changed = true;
        for (uint8_t i = 0; i < 13; i++) {
            if ((IO_handler.buttons_pressed_bottom >> i) & 1) {  // bit test for pressed button
                if (!g_auto_state[m2a_idx[i]]) { // if not in manual mode -> ignore button press
                    switch (i) {
                    case 0:
                        g_alarm_ignore[FluidLevel] = true;
                        g_control[0] = true;
                        break;
                    case 1:
                        g_alarm_ignore[FluidLevel] = true;
                        g_control[0] = false;
                        break;
                    case 2:
                        g_alarm_ignore[FilterSpeed] = true;
                        g_control[1] = true;
                        g_control[2] = false;
                        break;
                    case 3:
                        g_alarm_ignore[FilterSpeed] = true;
                        g_control[1] = false;
                        g_control[2] = false;
                        break;
                    case 4:
                        g_alarm_ignore[FilterSpeed] = true;
                        g_control[1] = true;
                        g_control[2] = true;
                        break;
                    case 5:
                        g_alarm_ignore[Airation] = true;
                        g_control[3] = true;
                        break;
                    case 6:
                        g_alarm_ignore[Airation] = true;
                        g_control[3] = false;
                        break;
                    case 7:
                        g_alarm_ignore[FeedRate] = true;
                        g_control[4] = true;
                        break;
                    case 8:
                        g_alarm_ignore[FeedRate] = true;
                        g_control[4] = false;
                        break;
                    case 9:
                        g_alarm_ignore[Temp] = true;
                        g_control[5] = true;
                        break;
                    case 10:
                        g_alarm_ignore[Temp] = true;
                        g_control[5] = false;
                        break;
                    case 11:
                        g_alarm_ignore[ConcentrationFraction] = true;
                        g_control[6] = true;
                        break;
                    case 12:
                        g_alarm_ignore[ConcentrationFraction] = true;
                        g_control[6] = false;
                        break;
                    default:
                        g_errorCode |= _BV(6);
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
    Threshold               = min(g_Setpoints[FilterSpeed] * .8, 25);
    g_alarm[FilterSpeed]    = ((g_Setpoints[FilterSpeed] < (g_ProcessState[FilterSpeed] - Threshold))   || (g_Setpoints[FilterSpeed] > (g_ProcessState[FilterSpeed] + Threshold)));

    // set alarm if airation is off by more than 10 % 
    Threshold               = min(g_Setpoints[Airation] * .8, 25);
    g_alarm[Airation]       = ((g_Setpoints[Airation] < (g_ProcessState[Airation] - Threshold))         || (g_Setpoints[Airation] > (g_ProcessState[Airation] + Threshold )));

    // set alarm if feedrate is off by more than 10 % 
    Threshold               = min(g_Setpoints[FeedRate] * .8, 5);
    g_alarm[FeedRate]       = ((g_Setpoints[FeedRate] < (g_ProcessState[FeedRate] - Threshold))         || (g_Setpoints[FeedRate] > (g_ProcessState[FeedRate] + Threshold )));

    // set alarm if Temp is off by more than 1 °C
    Threshold               = 10;
    g_alarm[Temp]           = ((g_Setpoints[Temp] < (g_ProcessState[Temp] - Threshold))                 || (g_Setpoints[Temp] > (g_ProcessState[Temp] + Threshold)));

    // no alarm for concentrating, instead set alarm if last measurement is older than 60 seconds
    g_alarm[5]              = ((millis() - scom.time_since_last_measurement) > 60000);
    
    if (g_alarm[5])
        g_errorCode |= _BV(2);

    // reset "alarm ignore" if actual alarm condition is no longer true
    for (uint8_t i = 0; i < 5; i++)
       if (!g_alarm[i]) g_alarm_ignore[i] = false;
}

// cleansing routine called in each loop cycle
// as long as "g_foam" is set, the according output will be HIGH)
void foam_clean() {
    // increment failsafe foam trigger counter. Idea: if no valid measurement data is available, perform cleaning in regular intervals (like every 10 minutes)
    // if failsafe counter is high enough (roughly 12 minutes in this case), trigger cleaning
    // the counter is reset, whenever valid data is recieved from measurement arduino.
    g_foam_trigger_counter_long++;
    if (g_foam_trigger_counter_long == 32768) 
        foam_cleaning_duration = 0;

    // count cycles while cleaning (much faster than using actual time value) -> 1 cycle ~ 22 ms
    if (g_foam) 
        foam_cleaning_duration++; 

    // after 910 cycles (-> roughly 40 Seconds) turn off cleaning and reset counters
    if (foam_cleaning_duration >= 1920) { 
        g_foam_trigger_counter_long = 0;
        g_foam_trigger_counter = 0;
        g_foam = false;
        foam_cleaning_duration = 0;
    }
}

#pragma region Arduino
// normal Arduino style setup function, called once for initialization
void setup() {
    // ****** setup serial communication ******
    scom.begin(process_serial_data);

    // ****** setup LC displays ******
    Serial.print(F("{\"init\":{\"displays\":["));
    LCDs.begin();
    Serial.print(F("],\"io\":"));
    // ****** setup IO handling ******
    IO_handler.begin();
    Serial.println(F("1}}"));
}

// normal Arduino style loop function, called over and over again

void loop() {
    
    g_state_changed = false;

    // state change - react on button inputs if there were any and set or reset any alarms if applicable
    state_change();
    resetAlarm();
    foam_clean();

    // input - read potentiometer values
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
    if (scom.autosend) {
        if (g_state_changed) {
            if (((millis() - scom.lastsend) >= 100)) {
                scom.SendStateJSON();
                g_errorCode = 0; // reset error codes
            }
        }
        if (((millis() - scom.lastsend) >= scom.autosend_delay)) {
            scom.SendStateJSON();
            g_errorCode = 0; // reset error codes
        }
    }

}
#pragma endregion