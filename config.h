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


#pragma once

/* CPU frequency -- DO NOT CHANGE */
#define F_CPU 10000000UL

/* Standard definition used almost everywhere */
#include <stdint.h>

/* Beacon config -- Callsign / Locator / Frequencies */
#define PI4_MESSAGE      "VA2NQ   "        // UPDATE with your Callsign (8 chars, padding with spaces)
#define PI4_FREQUENCY    1296500000.0      // UPDATE with frequency aligned with the frequency bands

#define MORSE_MESSAGE    "VA2NQ  FN35NL "  // UPDATE with your Callsign + Locator
#define MORSE_FREQUENCY  1296500000.0      // UPDATE with frequency aligned with the frequency bands ( Propagation Beacons Exclusive )

#define WSPR_CALLSIGN    "VA2NQ "          // Exactly 6 characters (Padding with space at start. Ex " K1AB " or " K1ABC" or "VE1ABC")
#define WSPR_LOCATOR     "FN35"            // Exactly 4 characters (First part of the locator)
#define WSPR_POWER       33                // Numerical value in dBm (range 0-60, check allowed values)
#define WSPR_FREQUENCY   1296501450.0      // UPDATE with frequency aligned with the wspr frequencies



/* |                                                 |
   |  Example : VA2NQ Beacon -- Frequency band plan  |
   |                                                 |
   |   BAND | CW/PI4 Frequency | WSPR Frequency      |
   |--------|------------------|---------------------|
   |  50MHz |  50295000.0      |  50294450.0         |
   |  70MHz |  NA, Region 2    |  NA, Region 2       |
   | 144MHz |  144491000.0     |  144490450.0        |
   | 222MHz |  222295000.0 +1? |  222294450.0        |
   | 440MHz |  432302000.0     |  432301450.0        |
   | 1.3GHz | 1296500000.0     | 1296501450.0        | */

/* Switch ADI/Si Memo
- pi4.c & wspr.c : change timing 1.0ms 12.0ms
- pll.c : //#define ADI
- config.h : change freq.
*/