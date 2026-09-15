# Hob2Hood Receiver

PIC12F675-based Hob2Hood IR receiver with relay control and TTL UART input.

**Firmware:** v4.4 FINAL + WDT
**Compiler:** XC8
**MCU:** PIC12F675

## Pinout

| GPIO  | Direction | Function                     |
| ----- | --------- | ---------------------------- |
| `GP0` | Output    | Fan speed 1 relay            |
| `GP1` | Output    | Fan speed 2 relay            |
| `GP2` | Output    | Fan speed 3 relay            |
| `GP3` | Input     | IR receiver output           |
| `GP4` | Output    | Light relay                  |
| `GP5` | Input     | TTL UART RX — 1200 baud, 8N1 |

> Relay inputs are assumed to be **active LOW**.

## UART Commands

The receiver accepts single-byte UART commands:

| Command | Action                                                                |
| ------- | --------------------------------------------------------------------- |
| `0`     | Fan OFF                                                               |
| `1`     | Fan speed 1                                                           |
| `2`     | Fan speed 2                                                           |
| `3`     | Fan speed 3                                                           |
| `4`     | Fan speed 3 — Hob2Hood intensive mode maps to maximum available speed |
| `L`     | Light ON                                                              |
| `l`     | Light OFF                                                             |
| `X`     | Fan OFF + Light OFF                                                   |

Send commands as discrete bytes.

The PIC12F675 has very limited resources, so the UART implementation intentionally does **not** use a FIFO.

## Design Goals

The firmware is designed around predictable timing and minimal resource usage:

* No blocking delays.
* IR edges have priority over background processing.
* `Timer1` is dedicated exclusively to IR timing.
* `Timer0` does not generate periodic interrupts while the system is idle.
* UART uses `Timer0` only while a byte is actively being received.
* Fan **break-before-make** switching uses `Timer0` only while a relay transition is pending.
* A GPIO shadow register is used to avoid PIC read-modify-write issues on relay outputs.
* The watchdog timer is enabled.
* The watchdog is deliberately cleared only from healthy main-line code.
* After initialization, the WDT has **no prescaler**, because the shared prescaler is assigned to `Timer0`.
* Nominal WDT timeout is approximately **18 ms**, depending on the individual device and operating conditions.

## Hardware Requirements

The following hardware precautions are required for reliable and safe operation:

* Add an external approximately **10 kΩ pull-up** on `GP5 / UART RX` if the UART source can be disconnected or unplugged.
* Relay inputs must default to the **OFF** state while the PIC is in reset.

  * For an active-low relay board, use suitable pull-up resistors.
* Use a **hardware interlock** on the mains / fan motor side.

  * Software interlocking must be treated only as a secondary safety mechanism.
* Provide local decoupling:

  * `100 nF` ceramic capacitor near the PIC.
  * `100 nF` ceramic capacitor near the IR receiver.
  * Additional bulk capacitance on the supply rail.

## Timing / Resource Allocation

| Resource | Purpose                                                           |
| -------- | ----------------------------------------------------------------- |
| `Timer1` | Hob2Hood IR pulse timing                                          |
| `Timer0` | Software UART reception and temporary fan relay transition timing |
| `WDT`    | Recovery from firmware lockup                                     |

`Timer0` is intentionally inactive when neither UART reception nor a relay transition requires it. This avoids unnecessary periodic interrupt activity and keeps IR processing deterministic.

## Safety

This firmware controls mains-powered equipment indirectly through relays.

Do not rely on firmware alone to prevent invalid or dangerous relay combinations. Any mutually exclusive motor-speed outputs should also be protected by an appropriate **hardware interlock**.
