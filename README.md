# Hob2Hood Receiver

PIC12F675-based Hob2Hood IR receiver with relay control and a local push button.

**Firmware:** v4.5 BUTTON + WDT

**Compiler:** XC8

**MCU:** PIC12F675

## Build

The default Makefile settings match XC8 3.10 and MPLAB X 6.35 installed in
their standard macOS locations:

```sh
make
```

The resulting firmware is written to:

```text
build/hob2hood_pic12f675_v4_5_button.hex
```

The firmware source is `main.c` and is compiled directly by XC8.

Optimization is deliberately fixed at `-O2`; the firmware does not fit in the
PIC12F675 program memory with `-O0`. The build also uses `-mno-osccal` because
the firmware writes the measured `OSCCAL_VALUE` itself and therefore must not
depend on the factory calibration instruction at address `0x3FF` during
startup.

Toolchain paths can be overridden without editing the Makefile:

```sh
make XC8_HOME=/path/to/xc8 \
     DFP=/path/to/PIC10-12Fxxx_DFP/version/xc8
```

Run `make help` to see all supported overrides and `make clean` to remove the
generated build directory.

## Pinout

| GPIO  | Direction | Function                       |
| ----- | --------- | ------------------------------ |
| `GP0` | Output    | Fan speed 1 relay              |
| `GP1` | Output    | Fan speed 2 relay              |
| `GP2` | Output    | Fan speed 3 relay              |
| `GP3` | Input     | IR receiver output             |
| `GP4` | Output    | Light relay                    |
| `GP5` | Input     | Push button to GND, active LOW |

Relay inputs are assumed to be **active LOW**.

## Button

Connect a normally-open push button between `GP5` and `GND`. The firmware
enables the PIC's internal GPIO pull-ups, so the released level is HIGH. For
long or electrically noisy wiring, an external approximately `10 kOhm` pull-up
from `GP5` to `VDD` is recommended.

Each debounced short click advances to the next mode and wraps back to mode 1:

| Mode | Light | Fan     |
| ---- | ----- | ------- |
| 1    | ON    | OFF     |
| 2    | ON    | Speed 1 |
| 3    | ON    | Speed 2 |
| 4    | ON    | Speed 3 |

After reset, all outputs are OFF. The first short click selects mode 1. A
press held for approximately one second switches all outputs OFF and resets
the button sequence; the next short click again selects mode 1.
A new button level must remain unchanged for approximately `65 ms` before it
is accepted. Short clicks are acted on at the stable pressed-to-released
transition, so holding the button does not repeat. The long-press threshold is
approximately `1 s` from the physical button press.

The Hob2Hood IR receiver remains active and can still control the fan and light
independently. IR commands do not change the button sequence position.

## Design Goals

* No blocking delays.
* IR edges have priority over background processing.
* `Timer1` is dedicated exclusively to IR timing.
* `Timer0` does not generate periodic interrupts while the system is idle.
* Button debounce and long-press timing use `Timer0` only while needed.
* Fan **break-before-make** switching uses `Timer0` only while a relay
  transition is pending.
* A GPIO shadow register avoids PIC read-modify-write issues on relay outputs.
* The watchdog timer is enabled and is cleared only from healthy main-line code.
* After initialization, the WDT has **no prescaler**, because the shared
  prescaler is assigned to `Timer0`.
* Nominal WDT timeout is approximately **18 ms**, depending on the device and
  operating conditions.

## Hardware Requirements

* Relay inputs must default to the **OFF** state while the PIC is in reset.
  For an active-low relay board, use suitable pull-up resistors.
* Use a **hardware interlock** on the mains / fan motor side. Software
  interlocking must be treated only as a secondary safety mechanism.
* Provide local decoupling: `100 nF` near the PIC, `100 nF` near the IR
  receiver, and additional bulk capacitance on the supply rail.

## Timing / Resource Allocation

| Resource | Purpose                                                |
| -------- | ------------------------------------------------------ |
| `Timer1` | Hob2Hood IR pulse timing                               |
| `Timer0` | Button debounce/hold and fan break-before-make timing  |
| `WDT`    | Recovery from firmware lockup                          |

`Timer0` is inactive when no button debounce/hold or relay transition requires
it. This avoids unnecessary periodic interrupt activity and keeps IR
processing deterministic.

## Safety

This firmware controls mains-powered equipment indirectly through relays.
Do not rely on firmware alone to prevent invalid or dangerous relay
combinations. Mutually exclusive motor-speed outputs should also be protected
by an appropriate hardware interlock.
