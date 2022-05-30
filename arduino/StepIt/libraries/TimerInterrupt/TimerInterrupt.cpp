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

#include "TimerInterrupt.h"

/** The callback function */
void (*TimerInterrupt::isrCallback)() = 0;

/** This is the function called by the interrupt. */
ISR(TIMER1_COMPA_vect)
{
    TimerInterrupt::isrCallback();
}

bool TimerInterrupt::start(const int usecs)
{
    // In this function, we convert the given time in microseconds into a
    // number of steps and a multiplication factor called prescale.
    // The SIR will use the number of steps and the prescale as a counter to
    // determine when the callback function must be called.
    // The interrupt time in seconds is calculated as steps * clock time * prescaler.

    // Disable global interrupts.
    cli();

    // Configure Timer 1, a 16-bit register stored as TCCR1A and TCCR1B.
    // The interrupt is called when the timer overflows.
    TCCR1A = 0; // Timer Counter Control register 1 A
    TCCR1B = 0; // Timer Counter Control register 1 B

    // The first 3 bits in TCCR1B are used to configure the prescale: they are
    // CS12, CS11, and CS10. By setting these bits in various combinations, we
    // can tell the timer to run at different speeds.

    // CS12   CS11   CS10
    //   0      0      0   no clock source (Timer/conter stopped).
    //   0      0      1   clk/1    - Increment at every clock.
    //   0      1      0   clk/8    - Increment every 8 clocks.
    //   0      1      1   clk/64   - Increment every 64 clocks.
    //   1      0      0   clk/256  - Increment every 256 clocks.
    //   1      0      1   clk/1024 - Increment every 1024 clocks.
    //   1      1      0   external clock on T1 pin. Clock on falling edge.
    //   1      1      1   external clock on T1 pin. Clock on rising edge.

    // Clock time, alias Arduino minimum time resolution in seconds (1/16MHz).
    const float CLOCK_TIME = 6.25e-8;

    // Convert input time to seconds.
    const float secs = ((float)usecs) / 1e6;

    // From the table above we know we can make the register increment every 1,
    // 8, 64, 256, 1024 clocks. Here we convert the given time into steps and
    // prescale.

    int steps = 0;
    int prescaler = 1; // Valid values: 1, 8, 64, 256, 1024
    do
    {
        steps = secs / (CLOCK_TIME * prescaler);
        if (steps < 65535) // Timer 1 is 16-bits counter.
        {
            break;
        }
        prescaler *= 8;
    } while (prescaler <= 1024);
    if (prescaler > 1024) // Time too long.
    {
        return false;
    }
    if (prescaler == 1 && steps < 100) // Time too short (100 * clock time)
    {
        return false;
    }

    // We store the number of steps into the following register. The timer
    // will call the interrupt callback when the timer register will match
    // this value.
    OCR1A = steps;

    // Turn on CTC mode (Clear Timer on Compare Match).
    TCCR1B |= (1 << WGM12);

    switch (prescaler)
    {
    case 1:
        TCCR1B |= (1 << CS10); // 0 0 1
        break;
    case 8:
        TCCR1B |= (1 << CS11); // 0 1 0
        break;
    case 64:
        TCCR1B |= (1 << CS11) & (1 << CS10); // 0 1 1
        break;
    case 256:
        TCCR1B |= (1 << CS12); // 1 0 0
        break;
    case 1024:
        TCCR1B |= (1 << CS12) & (1 << CS10); // 1 0 1
        break;
    }
    // Enable Timer 1 compare interrupt.
    TIMSK1 |= (1 << OCIE1A); // Timer 1 Interrupt Mask register 1.

    // Enable global interrupts.
    sei();
    return true;
}

bool TimerInterrupt::stop()
{
    // disable global interrupts.
    cli();

    // Reset Timer 1.
    TCCR1A = 0; // Timer Counter Control Register 1 A
    TCCR1B = 0; // Timer Counter Control Register 1 B

    // Clear the number of steps.
    OCR1A = 0;

    // Disable Timer 1 compare interrupt.
    TIMSK1 &= ~(1 << OCIE1A);

    // Enable global interrupts.
    sei();
    return true;
}

void TimerInterrupt::setCallback(void (*foo)())
{
    isrCallback = foo;
}
