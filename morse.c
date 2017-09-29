/*
 * FreeBSD License
 * Copyright (c) 2016, Guenael
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include "config.h"
#include "morse.h"

#include "pll.h"

#include <util/delay.h>


#define MORSE_DOT_DURATION         100               // 60 ms = 20 WPM, 100ms = 12 WPM
#define MORSE_FM_FREQ              700

// Used for FM modulation
#define MORSE_BAUD_RATE            20.0
#define MORSE_TIMING_FUNCTION      44.0f  // n instruction to complete the transmission
#define MORSE_SAMPLING_FREQUENCY   (F_CPU / MORSE_TIMING_FUNCTION)
#define MORSE_SAMPLES_PER_BAUD     (MORSE_SAMPLING_FREQUENCY / MORSE_BAUD_RATE)
#define MORSE_PHASE_INCREMENT      ((MORSE_FM_FREQ / MORSE_SAMPLING_FREQUENCY) * 4294967296)  //(1<<32)


char charMorse(char c) {
    const char Letters[] = { 0x42,0x84,0xA4,0x83,0x01,0x24,0xC3,0x04,0x02,0x74,0xA3,0x44,0xC2,0x82,0xE3,0x64,0xD4,0x43,0x03,0x81,0x23,0x14,0x63,0x94,0xB4,0xC4 };
    const char Numbers[] = { 0x95,0xFD,0x7D,0x3D,0x1D,0x0D,0x05,0x85,0xC5,0xE5,0xF5 };

    if ( (c == 0) || (c == ' ') )
        return(0);     // Not a valid morse character
    else if (c < 'A')  // Get then Morse pattern
        return(Numbers[c - '/']);
    else
        return(Letters[c - 'A']);
}


void morseSendString(char* str) {
    char morseChar;
    uint8_t morseLength;

    while (*str) {
        morseChar = charMorse(*str);
        morseLength = morseChar & 0x07;             // Bit2 to Bit0 of Morse[I] is the length

        for (uint8_t j=0; j<morseLength; j++) {
            pllRfOutput(1);

            if ((morseChar & 0x80) == 0x80)         // If MSB 0 = dot, 1 = dash,
                _delay_ms(MORSE_DOT_DURATION * 3);  // It is a dash, so wait 3 dot durations
            else
                _delay_ms(MORSE_DOT_DURATION);      // It is a dot, so wait 1 dot duration

            pllRfOutput(0);
            _delay_ms(MORSE_DOT_DURATION);
            morseChar = morseChar <<1;              // Point to next bit
        }
        _delay_ms(MORSE_DOT_DURATION * 4);          // Inter morse character pause
        str++;
    }
}


void morse2TonesSendString(char* str) {
    char morseChar;
    uint8_t morseLength;

    while (*str) {
        morseChar = charMorse(*str);
        morseLength = morseChar & 0x07;             // Bit2 to Bit0 of Morse[I] is the length

        for (uint8_t j=0; j<morseLength; j++) {
            pllUpdate(1);

            if ((morseChar & 0x80) == 0x80)         // If MSB 0 = dot, 1 = dash,
                _delay_ms(MORSE_DOT_DURATION * 3);  // It is a dash, so wait 3 dot durations
            else
                _delay_ms(MORSE_DOT_DURATION);      // It is a dot, so wait 1 dot duration

            pllUpdate(0);
            _delay_ms(MORSE_DOT_DURATION);
            morseChar = morseChar <<1;              // Point to next bit
        }
        _delay_ms(MORSE_DOT_DURATION * 4);          // Inter morse character pause
        str++;
    }
}

void morseSendMessage() {
    /* PLL setup */
    pllSetFreq((uint64_t)MORSE_FREQUENCY * 1000000ULL, 0);
    pllUpdate(0);
    _delay_ms(10);

    pllPA(1);
    pllRfOutput(1);

    /* Send the message in CW */
    morseSendString(MORSE_MESSAGE);

    pllRfOutput(0);
    pllPA(0);
}


void morse2TonesSendMessage() {
    /* PLL setup */
    pllSetFreq((uint64_t)(MORSE_FREQUENCY-250.0) * 1000000ULL, 0);  // PI4 compliant : 250 for 144MHz and below, 400 Hz upper
    pllSetFreq((uint64_t)(MORSE_FREQUENCY)       * 1000000ULL, 1);
    pllUpdate(0);

    pllPA(1);
    pllRfOutput(1);

    /* Send the message in CW */
    morse2TonesSendString(MORSE_MESSAGE);

    pllRfOutput(0);
    pllPA(0);

    _delay_ms(500);  // 500ms pause, PI4 procotol description
}
