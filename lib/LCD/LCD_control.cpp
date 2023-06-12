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

// updates the LCD screen content based on actual values. print new values only if necessary / when changed
void LCD_control::display() {
    for (uint8_t i = 0; i < 6; i++) {
        // print setpoint (only if value changed or display is initializing)
        if ((g_Setpoints[i] != lastSP[i]) | init) {
            LCD[i].setCursor(6, 0);
            if (i < 4) {
                LCD[i].print(g_Setpoints[i]);
            } else {
                LCD[i].print(g_Setpoints[i]/10);
                LCD[i].print(".");
                LCD[i].print(g_Setpoints[i]%10);
            }
            LCD[i].print(Units[i]);
        }
        // print measured value (only if value changed, display is initializing or error occoured for fist time)
        if ((g_ProcessState[i] != lastVal[i]) | init | (g_alarm[5] & !alarmprinted[i])) {
            LCD[i].setCursor(6, 1);

            //if alarm 6 is set (last valid measurement older than 30 seconds) display "Fehler" instead of measurement value
            if (g_alarm[5]){
                LCD[i].print(F("Fehler!    "));
                alarmprinted[i] = true;
            } else {
                if (i < 4) {
                    LCD[i].print(g_ProcessState[i]);
                } else {
                    LCD[i].print(g_ProcessState[i]/10);
                    LCD[i].print(".");
                    LCD[i].print(g_ProcessState[i]%10);
                }
                LCD[i].print(Units[i]);
                alarmprinted[i] = false;
            }
        }
    }

    //remember last value to deterime if the value has changed (can be done slightly mor efficient, but this si simpe and it works)
    for (uint8_t i = 0; i < 6; i++) {
        lastSP[i] = g_Setpoints[i];
        lastVal[i] = g_ProcessState[i];
    }
    init = false;
}