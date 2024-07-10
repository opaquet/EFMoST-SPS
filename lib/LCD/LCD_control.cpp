#include "LCD_control.h"

LCD_control::LCD_control() {

}

// initialize the 6 screens
void LCD_control::begin() {
    for (uint8_t i = 0; i < 6; i++) {
        LCD[i].init();
        LCD[i].backlight();
        LCD[i].setCursor(0, 0);
        LCD[i].print("Soll:       ");
        LCD[i].setCursor(0, 1);
        LCD[i].print("Ist :       ");

        // print 1 to serial to indicate initialized display
        if (i == 5) 
            Serial.print(F("1"));
        else
            Serial.print(F("1,"));
    }
    Wire.setClock(i2c_Clock);
}

uint16_t LCD_control::convert_value(uint16_t val, uint8_t idx) {
    float x, x2, y;
    switch (idx) {
        case 0:
            if (val > 0) {
                y = val * 8.04 + 125.8;
            } else {
                y = 0;
            }
            break;
        case 1: //filter speed
            y = val * 0.375; // same as (val / 10.24) * 3.84
            break;
        case 2: //airation
            y = val * 0.439453125; // same as (val / 10.24) * 4.5
            break;
        case 3: //feed pump -> non linear conversion
            x = (val / 10.24);
            x2 = x*x;
            y = x*3 - x2*0.0076116 - x2*x*1.8243e-5;
            break;
        default:
            y = val;
            break;
    }
    return uint16_t(y);
}

// updates the LCD screen content based on actual values. print new values only if necessary / when changed
void LCD_control::display() {
    uint16_t DisplayValue = 0;
    for (uint8_t i = 0; i < 6; i++) {
        // print setpoint (only if value changed or display is initializing)
        if ((g_Setpoints[i] != lastSP[i]) | init) {
            DisplayValue = convert_value(g_Setpoints[i],i);
            LCD[i].setCursor(6, 0);
            if (i < 4) {
                if (i==0) { // Füllstand auf 10 L genau runden bzw flooren
                    DisplayValue /=10;
                    DisplayValue *=10;
                }

                LCD[i].print(DisplayValue);
            } else {
                LCD[i].print(DisplayValue/10);
                LCD[i].print(".");
                LCD[i].print(DisplayValue%10);
            }
            LCD[i].print(Units[i]);
            if (i==3) { // Feed PWM Modus
                if (g_pump_ctrl_pwm)
                    LCD[i].print("PWM");
                else
                    LCD[i].print("     ");
            } 
        }


        // print measured value (only if value changed, display is initializing or error occoured for fist time)
        if ( (g_ProcessState[i] != lastVal[i]) || init || (g_alarm[5] && !alarmprinted[i]) || (alarmprinted[i] && !(g_alarm[5])) ) {
            DisplayValue = convert_value(g_ProcessState[i],i);
            LCD[i].setCursor(6, 1);

            //if alarm 6 is set (last valid measurement older than 60 seconds) display "Fehler" instead of measurement value
            if (g_alarm[5] & !alarmprinted[i]){
                LCD[i].print(F("Fehler!    "));
                alarmprinted[i] = true;
            } else {
                if (i < 4) {
                    if (i==0) { // Füllstand auf 10 L genau runden bzw flooren
                        DisplayValue /=10;
                        DisplayValue *=10;
                    }
                    LCD[i].print(DisplayValue);
                } else {
                    LCD[i].print(DisplayValue/10);
                    LCD[i].print(".");
                    LCD[i].print(DisplayValue%10);
                }
                LCD[i].print(Units[i]);
                alarmprinted[i] = false;
            }
        }
    }

    //remember last value to deterime if the value has changed (can be done slightly more efficiently, but this is simpe and it works)
    for (uint8_t i = 0; i < 6; i++) {
        lastSP[i] = g_Setpoints[i];
        lastVal[i] = g_ProcessState[i];
    }
    init = false;

    LCD[0].setCursor(15, 0);
    if (g_foam) {
        LCD[0].print("F");
    } else {
        LCD[0].print(" ");
    }

}