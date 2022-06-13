## Volatile variables

Following are key scenarios when volatile variables should be used:

1. When Arduino main thread reads a variable but never modifies it, the compiler would convert it into an inline constant as part of the optimization process. The compiler does not know that an ISR may change it. The variable must be marked volatile to avoid this optimization.

2. Additionally, the volatile keyword forces the compiler to generate code that always reads the variable from RAM and does not cache the last read value in a register. Therefore, volatile should always be used on any variable that is modified by an interrupt.

3. If the volatile variable is greater than a byte, the microcontroller cannot read it in one step because it is an 8-bit microcontroller. This means that, while the main code section reads the first 8 bits of the variable, the interrupt might already change the second 8 bits producing random values for the variable.

There is a remedy to the last point: disabling interrupts when the variable is read using the ATOMIC_BLOCK macro.

```
#include <util/atomic.h>

volatile int input_from_interrupt;

ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // code with interrupts blocked
    int result = input_from_interrupt;
}
```
