#include "EFMoST_IO.h"


IO::IO(){

}

// initialize pins and DACs and setup button polling interupt
void IO::begin() {
    //initialize DACs
    DAC[0].begin(38);
    DAC[1].begin(39);
    DAC[2].begin(40);
    DAC[3].begin(41);

    cli(); // stop all interrupts

    //initialize Pinmodes
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
    DDRG |= B00000111; // pin 39, 40 und 41
    DDRD |= B01000000; // pin 38

    // initialize timer interrupt every 50 ms
    TCCR5A = 0;
    TCCR5B = 0;
    TCNT5 = 0;
    OCR5A = 781;
    TCCR5B |= (1 << WGM52);
    TCCR5B |= (1 << CS52) | (1 << CS50);
    TIMSK5 |= (1 << OCIE5A);

    sei(); // enable interrupts
}

// shift out LED states to shift registers (takes about 54 microseconds)
void IO::LED_out(uint32_t led_state) {
    // set compare value
    uint32_t cmp = 16777216;
    // stop all interrupts
    cli(); 
    // Latch pin (17) LOW
    PORTH &= B11111110;
    delayMicroseconds(2);
    for (uint8_t i = 0; i < 25; i++) {
        // data pin (15) HIGH or LOW
        if (led_state & cmp) {
            PORTJ |= B00000001;
        } else {
            PORTJ &= B11111110;
        }
        cmp >>= 1;
        delayMicroseconds(1);
        // pulse clock pin (16)
        PORTH |= B00000010;
        delayMicroseconds(1);
        PORTH &= B11111101;
    }
    PORTH |= B00000001; // Latch pin (17) HIGH
    sei(); // enable all interrupts
}

// set digtial control out using pins 2 - 13
// using port/register manipulation and loops were unrolled to speed execution up
void IO::digital_out(uint16_t values) {
    // set pin 2
    if (values & 1 ) {
        PORTE |= _BV(PE4); 
    } else {
        PORTE &= ~_BV(PE4);
    }
    // set pin 3
    if (values & 2) {
        PORTE |= _BV(PE5);
    } else {
        PORTE &= ~_BV(PE5);
    }
    // set pin 4
    if (values & 4) {
        PORTG |= _BV(PG5);
    } else {
        PORTG &= ~_BV(PG5);
    }
    // set pin 5
    if (values & 8) {
        PORTE |= _BV(PE3);
    } else {
        PORTE &= ~_BV(PE3);
    }
    // set pin 6
    if (values & 16) {
        PORTH |= _BV(PH3);
    } else {
        PORTH &= ~_BV(PH3);
    }
    // set pin 7
    if (values & 32) {
        PORTH |= _BV(PH4);
    } else {
        PORTH &= ~_BV(PH4);
    }
    // set pin 8
    if (values & 64) {
        PORTH |= _BV(PH5);
    } else {
        PORTH &= ~_BV(PH5);
    }
    // set pin 9
    if (values & 128) {
        PORTH |= _BV(PH6);
    } else {
        PORTH &= ~_BV(PH6);
    }
    // set pin 10
    if (values & 256) {
        PORTB |= _BV(PB4);
    } else {
        PORTB &= ~_BV(PB4);
    }
    // set pin 11
    if (values & 512) {
        PORTB |= _BV(PB5);
    } else {
        PORTB &= ~_BV(PB5);
    }
    // set pin 12
    if (values & 1024) {
        PORTB |= _BV(PB6);
    } else {
        PORTB &= ~_BV(PB6);
    }
    // set pin 13
    if (values & 2048) {
        PORTB |= _BV(PB7);
    } else {
        PORTB &= ~_BV(PB7);
    }
}

// set analog control outputs using external DACs connected via SPI
void IO::analog_out(uint16_t * values) {
    DAC[0].analogWrite(values[0]);
    DAC[1].analogWrite(values[1]);
    DAC[2].analogWrite(values[2]);
    DAC[3].analogWrite(values[3]);
}

// read all 6 analog potentiometer values into array
uint16_t * IO::analog_in() {
    static uint16_t AV[6];
    for (uint8_t i = 0; i < 6; i++) {
        AV[i] = analogRead(A0 + i); // dummy read for adc to settle
        AV[i] = 0; 

        // read and accumulate 16 times
        for (uint8_t j = 0; j < 16; j++) {
            AV[i] += analogRead(A0 + i);
        }

        // and right shift by 4 bits (divide by 16)
        AV[i] >>= 4;
    }
    return AV;
}
