
 * Hob2Hood receiver - PIC12F675 - XC8
 * v4.4 FINAL + WDT - audited
 *
 * GP0 -> Fan 1 relay
 * GP1 -> Fan 2 relay
 * GP2 -> Fan 3 relay
 * GP3 <- IR receiver OUT
 * GP4 -> Light relay
 * GP5 <- TTL UART RX, 1200 baud, 8N1
 *
 * Relay inputs are assumed ACTIVE LOW.
 *
 * UART commands:
 *   '0' = fan OFF
 *   '1' = fan speed 1
 *   '2' = fan speed 2
 *   '3' = fan speed 3
 *   '4' = fan speed 3 (Hob2Hood intensive -> max available)
 *   'L' = light ON
 *   'l' = light OFF
 *   'X' = fan OFF + light OFF
 *   Send discrete command bytes; this tiny MCU intentionally has no UART FIFO.
 *
 * Design goals:
 *   - no blocking delays
 *   - IR edges have priority
 *   - Timer1 is dedicated to IR timing
 *   - Timer0 interrupts are NOT periodic while idle
 *   - UART uses Timer0 only while a byte is actually being received
 *   - fan break-before-make uses Timer0 only while a transition is pending
 *   - GPIO shadow avoids PIC read-modify-write surprises on relay outputs
 *   - watchdog enabled; deliberately cleared only from healthy main-line code
 *   - WDT has NO prescaler after init because the shared prescaler belongs
 *     to Timer0; nominal WDT timeout is ~18 ms (device-dependent)
 *
 * IMPORTANT HARDWARE REQUIREMENTS:
 *   - external ~10 kOhm pull-up on GP5/UART RX if the UART source may unplug
 *   - relay inputs must be held OFF during PIC reset (e.g. pull-ups for
 *     active-low relay board)
 *   - use a hardware interlock on the mains/motor side; software is secondary
 *   - decouple PIC and IR receiver locally (100 nF each, plus bulk capacitance)
