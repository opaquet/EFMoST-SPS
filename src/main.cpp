#include <Arduino.h>
#include <SPI.h>
#include "globals.h"
#include "serial_com.h"
#include <LiquidCrystal_I2C.h>
#include <MCP_DAC.h>


// global variables
uint32_t  g_buttons_pressed = 0;          // pressed button bit set to 1 in timer interrupt handler
uint8_t   g_auto_state = 0;               // 6 control outputs that can be set either manually or automatically (0 -> manual, 1 -> auto)
uint32_t  g_buttons_LEDs = 0;             // pressed button bit set to 1 in timer interrupt handler
uint8_t   g_manual_button_state = 0;
uint8_t   g_analog_control_out[5] = {0,0,0,0,0};
uint16_t  g_digital_control_out = 0;
uint16_t  g_analog_values[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// local variables
Serial_Com scom;
uint8_t   i = 0;
LiquidCrystal_I2C LCD[6] = {LiquidCrystal_I2C(DISPLAY0,20,4),LiquidCrystal_I2C(DISPLAY1,20,4),LiquidCrystal_I2C(DISPLAY2,20,4),LiquidCrystal_I2C(DISPLAY3,20,4),LiquidCrystal_I2C(DISPLAY4,20,4),LiquidCrystal_I2C(DISPLAY5,20,4)};
MCP4921 DAC[6] = {};


// interpret and execute commands that were recieved via serial0
void serial_cmd_exec(serial_cmd_t cmd) {
  if (strcmp(cmd.command,"set")==0) {
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
        } else if (cmd.args[1] == 255) {g_digital_control_out = cmd.args[2];}
      }
    }
  } else if (strcmp(cmd.command,"mode")==0) {
    // change auto/manual mode 
    scom.CmdOK();

  } else if (strcmp(cmd.command,"get")==0) {
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
void shiftOut_Fast(){
  // Latch pin (17) LOW
  PORTH &= B11111110; 
  for (i = 0; i < 25; i++)  {
    // data pin (15) HIGH or LOW
    if (g_buttons_LEDs & (1 << i)) {
      PORTJ |= B00000001;
    } else {
      PORTJ &= B11111110;
    }
    // pulse clock pin (16)
    PORTH |= B00000010;
    PORTH &= B11111101; 	
  }
  PORTH |= B00000001; // Latch pin (17) HIGH
}

// read all 16 analog values into array
void readAnalogValues() {
  for (i = 0; i < 16; i++)  {
    g_analog_values[i] = analogRead(54+i); //dummy read for adc to settle
    g_analog_values[i] = analogRead(54+i);
  }
}

// set digtial control out using pins 2 - 13
// using port/register manipulation and loops were unrolled to speed execution up
void setDigitalControlOut() {
  //set pin 2
  if (g_digital_control_out & 1) {
    PORTE |= B00010000;
  } else {
    PORTE &= B11101111;
  }
  //set pin 3
  if (g_digital_control_out & 2) {
    PORTE |= B00100000;
  } else {
    PORTE &= B11011111;
  }
  //set pin 4
  if (g_digital_control_out & 4) {
    PORTG |= B00100000;
  } else {
    PORTG &= B11011111;
  }
  //set pin 5
  if (g_digital_control_out & 8) {
    PORTE |= B00001000;
  } else {
    PORTE &= B11110111;
  }
  //set pin 6
  if (g_digital_control_out & 16) {
    PORTH |= B00001000;
  } else {
    PORTH &= B11110111;
  }
  //set pin 7
  if (g_digital_control_out & 32) {
    PORTH |= B00010000;
  } else {
    PORTH &= B11101111;
  }
  //set pin 8
  if (g_digital_control_out & 64) {
    PORTH |= B00100000;
  } else {
    PORTH &= B11011111;
  }
  //set pin 9
  if (g_digital_control_out & 128) {
    PORTH |= B01000000;
  } else {
    PORTH &= B10111111;
  }
  //set pin 10
  if (g_digital_control_out & 256) {
    PORTB |= B00010000;
  } else {
    PORTB &= B11101111;
  }
  //set pin 11
  if (g_digital_control_out & 512) {
    PORTB |= B00100000;
  } else {
    PORTB &= B11011111;
  }
  //set pin 12
  if (g_digital_control_out & 1024) {
    PORTB |= B01000000;
  } else {
    PORTB &= B10111111;
  }
  //set pin 13
  if (g_digital_control_out & 2048) {
    PORTB |= B10000000;
  } else {
    PORTB &= B01111111;
  }
}

// set analog control outputs using external DACs connected via SPI
void setAnalogControlOut() {

}

// ISR for button polling. Executed every 50 ms
ISR(TIMER5_COMPA_vect){
  g_buttons_pressed = ((uint32_t)PINA << 0) | ((uint32_t)PINC << 8) | ((uint32_t)PINL << 16);
  //last button is on other port and needs to be shifted in as a single bit by 24 places
  g_buttons_pressed |= (uint32_t)(PINJ & B00000010) << 23;

  //state change logic
  if (g_buttons_pressed > 0) {
    switch (g_buttons_pressed)  {
    case (uint32_t)1<<0: // Befüllung Hand
      g_auto_state &= B11111110;
      break;
    case (uint32_t)1<<1: // Befüllung Auto
      g_auto_state |= 1;
      break;
    case (uint32_t)1<<2: // RotFilter Hand
      g_auto_state &= B11111101;
      break;   
    case (uint32_t)1<<3: // RotFilter Auto
      g_auto_state |= 2;
      break; 
    case (uint32_t)1<<4: // Belüftung Hand
      g_auto_state &= B11111011;
      break;
    case (uint32_t)1<<5: // Belüftung Auto
      g_auto_state |= 4;
      break;
    case (uint32_t)1<<6: // Dosierung Hand
      g_auto_state &= B11110111;
      break;
    case (uint32_t)1<<7: // Dosierung Auto
      g_auto_state |= 8;
      break;
    case (uint32_t)1<<8: // Kühlung Hand
      g_auto_state &= B11101111;
      break;
    case (uint32_t)1<<9: // Kühlung Auto
      g_auto_state |= 16;
      break;
    case (uint32_t)1<<10: // Filtration Hand
      g_auto_state &= B11011111;
      break;   
    case (uint32_t)1<<11: // Filtration Auto
      g_auto_state |= 32;
      break; 
    case (uint32_t)1<<12: // Befüllung Aus
      g_manual_button_state &= B11111110;
      break;
    case (uint32_t)1<<13: // Befüllung Ein
      g_manual_button_state |= 1;
      break;
    case (uint32_t)1<<14: // RotFilter Aus
      g_manual_button_state &= B11111001;
      break;
    case (uint32_t)1<<15: // RotFilter Links
      g_manual_button_state |= 2;
      break;
    case (uint32_t)1<<16: // RotFilter Rechts
      g_manual_button_state |= 4;
      break;
    case (uint32_t)1<<17: // Belüftung Aus
      g_manual_button_state &= B11110111;
      break;
    case (uint32_t)1<<18: // Belüftung Ein
      g_manual_button_state |= 8;
      break;   
    case (uint32_t)1<<19: // Dosierung Aus
      g_manual_button_state &= B11101111;
      break; 
    case (uint32_t)1<<20: // Dosierung Ein
      g_manual_button_state |= 16;
      break;
    case (uint32_t)1<<21: // Kühlung Aus
      g_manual_button_state &= B11011111;
      break;
    case (uint32_t)1<<22: // Kühlung Ein
      g_manual_button_state |= 32;
      break;
    case (uint32_t)1<<23: // Filtration Aus
      g_manual_button_state &= B10111111;
      break;
    case (uint32_t)1<<24: // Filtration Ein
      g_manual_button_state |= 64;
      break;
    default:
      break;
    }
  }
}

void setup() {
  // ****** setup serial communication ******
  scom.begin(SERIAL_BAUD_RATE, serial_cmd_exec);

  // ****** setup LC displays ******
  for (i = 0; i < 6; i++) {
    LCD[i].init();
    LCD[i].backlight();
  }

  // ****** setup pinmodes ******
  // all frontpanel buttons -> input
  DDRC = 0x00;  //set entire port as input
  DDRA = 0x00;  //set entire port as input
  DDRL = 0x00;  //set entire port as input
  DDRJ &= B11111101; //pin 14

  // shift registers for LEDs -> output
  DDRJ |= B00000001; //pin 15
  DDRH |= B00000011; //pin 16 & 17

  // digital control -> output
  DDRE |= B00111000; //pin 5, 2 & 3
  DDRG |= B00100000; //pin 4
  DDRH |= B01111000; //pin 6, 7, 8 & 9
  DDRB |= B11110000; //pin 10, 11, 12 & 13

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
}

void loop() {
  // input - read Sensor states
  readAnalogValues();

  // compute control outputs for manual control


  // output - control
  setDigitalControlOut();
  setAnalogControlOut();

  // output - button LEDs
  shiftOut_Fast();

  // update LC displays
  for (i = 0; i < 6; i++) {
    LCD[i].setCursor(0,0);
    LCD[i].print(i); //preliminary test
  }

  // execute Serial Comms events
  scom.read_serial0();
  scom.read_serial1();
}