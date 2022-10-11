/*
 * Copyright (c) 2022, Giovanni Remigi
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
  TCCR1A = 0;  // Timer Counter Control register 1 A
  TCCR1B = 0;  // Timer Counter Control register 1 B

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
  int prescaler = 1;  // Valid values: 1, 8, 64, 256, 1024
  do
  {
    steps = secs / (CLOCK_TIME * prescaler);
    if (steps < 65535)  // Timer 1 is 16-bits counter.
    {
      break;
    }
    prescaler *= 8;
  } while (prescaler <= 1024);
  if (prescaler > 1024)  // Time too long.
  {
    return false;
  }
  if (prescaler == 1 && steps < 100)  // Time too short (100 * clock time)
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
      TCCR1B |= (1 << CS10);  // 0 0 1
      break;
    case 8:
      TCCR1B |= (1 << CS11);  // 0 1 0
      break;
    case 64:
      TCCR1B |= (1 << CS11) & (1 << CS10);  // 0 1 1
      break;
    case 256:
      TCCR1B |= (1 << CS12);  // 1 0 0
      break;
    case 1024:
      TCCR1B |= (1 << CS12) & (1 << CS10);  // 1 0 1
      break;
  }
  // Enable Timer 1 compare interrupt.
  TIMSK1 |= (1 << OCIE1A);  // Timer 1 Interrupt Mask register 1.

  // Enable global interrupts.
  sei();
  return true;
}

bool TimerInterrupt::stop()
{
  // disable global interrupts.
  cli();

  // Reset Timer 1.
  TCCR1A = 0;  // Timer Counter Control Register 1 A
  TCCR1B = 0;  // Timer Counter Control Register 1 B

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
