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
 
#ifndef FREEZER_H
#define FREEZER_H

#include <Arduino.h>

/**
 * This is the main class to control Freezer.
 * A write method with a 16 bits value allows the user to control the
 * 16 outputs of the shift registers.
 */
class Freezer {

public:

    Freezer(byte overridingClearPin, byte clockPin, byte dataPin, byte outputEnabledPin, byte latchPin);
    ~Freezer();
    void initialize();
    void write(unsigned int value);
    
private:

    // Pin D2 connected to MR of of 74HC595. Low active.
    byte overridingClearPin;

    // Pin D3 connected to SH_CP of 74HC595.
    byte clockPin;

    // Pin D4 connected to DS of 74HC595.
    byte dataPin;
    
    // Pin D5 connected to OE of 74HC595. Low active.
    byte outputEnabledPin;
    
    // Pin D6 connected to ST_CP of 74HC595.
    byte latchPin;
};

#endif

