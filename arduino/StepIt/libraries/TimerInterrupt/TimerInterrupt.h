/*
 * Copyright (C) 2022 Remigi Giovanni
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

#ifndef TIMER_INTERRUPT_H
#define TIMER_INTERRUPT_H

#include <avr/io.h>
#include <avr/interrupt.h>

/**
 * This is a class to create a Service Interrupt Routine (SIR) to invoke a
 * callback function at given intervals of time measured in micro seconds.
 */
class TimerInterrupt
{
public:
        /**
         * Start up the SIR. For more information about Arduino interrupt, read
         * the document
         * "Microcontroller tutorial series - AVR and Arduino timer interrupts.pdf".
         * @param usecs The interrupt time in microseconds.
         * @return True if the given time is within the limits of the Arduino
         *         board, false otherwise.
         */
        static bool start(const int usecs);

        /**
         * Stop the SIR.
         */
        static bool stop();

        /**
         * Set the ISR callback function.
         * @param The callback function.
         */
        static void setCallback(void (*foo)());

        /**
         * Execute the callback function.
         */
        static void (*isrCallback)();
};

#endif
