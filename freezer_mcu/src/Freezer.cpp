/*
 * Copyright (C) 2014 Remigi Giovanni
 * g.remigi@kineticsystem.org
 * www.kineticsystem.org
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "Freezer.h"  
   
/** Default constructor. */
Freezer::Freezer(byte overridingClearPin, byte clockPin, byte dataPin, byte outputEnabledPin, byte latchPin) {

    this->overridingClearPin = overridingClearPin;
    this->clockPin = clockPin;
    this->dataPin = dataPin;
    this->outputEnabledPin = outputEnabledPin;
    this->latchPin = latchPin;
}
 
/** Desctructor. */
Freezer::~Freezer(){
}

/** Circuit initialization. */
void Freezer::initialize() {

    // Set pins to output so you can control the shift register.
    pinMode(latchPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(dataPin, OUTPUT);
    pinMode(outputEnabledPin, OUTPUT);
    pinMode(overridingClearPin, OUTPUT);
    
    // Set the pins values.
    digitalWrite(latchPin, LOW);
    digitalWrite(clockPin, LOW);
    digitalWrite(dataPin, LOW);
    digitalWrite(outputEnabledPin, LOW);
    digitalWrite(overridingClearPin, HIGH);
}

/**
 * Write the given string, a sequence of 0 and 1, into the shift registers.
 * @param flags The string representing a 
 *     sequence of bits.
 */
void Freezer::write(unsigned int value) {

    unsigned int mask = 1u;
  
    // Clear everything to prepare shift register for bit shifting.
    digitalWrite(dataPin, LOW);
    digitalWrite(clockPin, LOW);
    digitalWrite(latchPin, LOW);

    for (int i = 0; i < 16; i++) {
      
        // Reset the clock pin.
        digitalWrite(clockPin, LOW);

        // Write a bit into the registers.
        if ((value & mask) == mask) {
            digitalWrite(dataPin, HIGH);
        } else {
            digitalWrite(dataPin, LOW);
        }
  
        // Register shifts bits on upstroke of clock pin. 
        digitalWrite(clockPin, HIGH);
        
        // Zero the data pin after shift to prevent bleed through.
        digitalWrite(dataPin, LOW);
        
        mask <<= 1;
    }

    // Stop shifting.
    digitalWrite(clockPin, LOW);
    digitalWrite(latchPin, HIGH);
}
