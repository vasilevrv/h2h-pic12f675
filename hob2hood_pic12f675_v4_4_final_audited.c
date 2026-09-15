/*
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
 */

#include <xc.h>
#include <stdint.h>

#pragma config FOSC  = INTRCIO
#pragma config WDTE  = ON
#pragma config PWRTE = ON
#pragma config MCLRE = OFF
#pragma config BOREN = ON
#pragma config CP    = OFF
#pragma config CPD   = OFF

#define _XTAL_FREQ 4000000UL

/* Measured/calibrated for this specific PIC12F675. */
#define OSCCAL_VALUE             0x48u

/* GPIO */
#define FAN1_MASK                0x01u  /* GP0 */
#define FAN2_MASK                0x02u  /* GP1 */
#define FAN3_MASK                0x04u  /* GP2 */
#define IR_MASK                  0x08u  /* GP3 */
#define LIGHT_MASK               0x10u  /* GP4 */
#define UART_RX_MASK             0x20u  /* GP5 */

#define INPUT_MASK               (IR_MASK | UART_RX_MASK)
#define FAN_MASK                 (FAN1_MASK | FAN2_MASK | FAN3_MASK)
#define RELAY_MASK               (FAN_MASK | LIGHT_MASK)

/* Hob2Hood receive */
#define RX_MAX                   17u
#define FRAME_GAP_US             6000u

/* Known Hob2Hood IRremote/FNV hashes */
#define H2H_FAN_1                0xE3C01BE2UL
#define H2H_FAN_2                0xD051C301UL
#define H2H_FAN_3                0xC22FFFD7UL
#define H2H_FAN_4                0xB9121B29UL
#define H2H_FAN_OFF              0x055303A3UL
#define H2H_LIGHT_ON             0xE208293CUL
#define H2H_LIGHT_OFF            0x24ACF947UL

#define FNV_BASIS_32             2166136261UL
#define FNV_PRIME_32             16777619UL

/*
 * Timer0 modes.
 *
 * FAN mode:
 *   Fcy ~= 1 MHz, prescaler 1:256
 *   overflow ~= 65.536 ms
 *   3 overflows ~= 196.6 ms dead time
 *
 * UART mode:
 *   Fcy ~= 1 MHz, prescaler 1:8
 *
 *   Start-bit verification:
 *     preload 204 -> about 418 us, near the middle of the start bit.
 *
 *   Data-bit spacing:
 *     preload/add 152 -> about 834 us, close to 833.33 us at 1200 baud.
 *
 *   After the first overflow we add the preload to the current TMR0 value
 *   rather than blindly overwriting it. This compensates most ISR latency
 *   and prevents sampling phase error from accumulating across the byte.
 */
#define T0_MODE_IDLE              0u
#define T0_MODE_FAN               1u
#define T0_MODE_UART              2u

#define OPTION_T0_FAN             0x87u  /* internal, prescaler 1:256 */
#define OPTION_T0_UART            0x82u  /* internal, prescaler 1:8 */

#define UART_START_PRELOAD        204u
#define UART_BIT_PRELOAD          152u
#define UART_STATE_VERIFY_START   0xFFu

#define FAN_DEAD_OVERFLOWS        3u

/* Packed state flags: saves 3 bytes of scarce PIC12F675 SRAM. */
#define FLAG_RX_ACTIVE             0x01u
#define FLAG_RX_BAD                0x02u
#define FLAG_FAN_MAKE              0x04u
#define FLAG_UART_ACTIVE           0x08u
#define FLAG_UART_READY            0x10u
#define FLAG_IR_LOCK               0x20u  /* rx_buf/hash protected; discard IR edges */

/* Global state: PIC12F675 has only 64 bytes of SRAM. */
static volatile uint8_t gpio_shadow;
static volatile uint8_t last_inputs;
static volatile uint8_t state_flags;

/* IR */
static volatile uint8_t rx_buf[RX_MAX];
static volatile uint8_t rx_count;

/* Fan */
static volatile uint8_t fan_current;
static volatile uint8_t fan_pending;
static volatile uint8_t fan_dead_left;

/* Timer0/UART */
static volatile uint8_t t0_mode;
static volatile uint8_t uart_bit;
static volatile uint8_t uart_byte;

/* Hash workspace kept global to reduce XC8 auto/parameter RAM pressure. */
static uint32_t hash_value;
static uint8_t hash_i;

/* --------------------------------------------------------------------- */

static void gpio_commit(void)
{
    GPIO = gpio_shadow;
}

static void fans_off(void)
{
    gpio_shadow |= FAN_MASK;
    gpio_commit();
}

static void fan_on(uint8_t speed)
{
    /* Safety invariant: software never leaves two fan outputs ON. */
    gpio_shadow |= FAN_MASK;

    if (speed == 1u)
        gpio_shadow &= (uint8_t)~FAN1_MASK;
    else if (speed == 2u)
        gpio_shadow &= (uint8_t)~FAN2_MASK;
    else if (speed == 3u)
        gpio_shadow &= (uint8_t)~FAN3_MASK;

    gpio_commit();
}

static void light_set(uint8_t on)
{
    if (on != 0u)
        gpio_shadow &= (uint8_t)~LIGHT_MASK;
    else
        gpio_shadow |= LIGHT_MASK;

    gpio_commit();
}

/* Start/restart Timer0 in fan dead-time mode. Call with T0IE disabled. */
static void timer0_start_fan(void)
{
    OPTION_REG = OPTION_T0_FAN;
    TMR0 = 0u;
    INTCONbits.T0IF = 0u;
    t0_mode = T0_MODE_FAN;
    INTCONbits.T0IE = 1u;
}

/* Start UART timing from a detected falling start edge. */
static void timer0_start_uart(void)
{
    INTCONbits.T0IE = 0u;

    OPTION_REG = OPTION_T0_UART;
    TMR0 = UART_START_PRELOAD;
    INTCONbits.T0IF = 0u;
    t0_mode = T0_MODE_UART;

    INTCONbits.T0IE = 1u;
}

/*
 * Restore fan dead-time timer after UART. If no fan transition is pending,
 * Timer0 interrupt is left disabled, minimizing IR-capture jitter.
 */
static void timer0_after_uart(void)
{
    INTCONbits.T0IE = 0u;

    if (fan_dead_left != 0u) {
        /*
         * Inline the fan Timer0 setup here rather than calling
         * timer0_start_fan(): this function runs from the ISR call graph,
         * while timer0_start_fan() is used from main context. Keeping the
         * call graphs separate avoids XC8 duplicating a non-reentrant helper.
         */
        OPTION_REG = OPTION_T0_FAN;
        TMR0 = 0u;
        INTCONbits.T0IF = 0u;
        t0_mode = T0_MODE_FAN;
        INTCONbits.T0IE = 1u;
    } else {
        t0_mode = T0_MODE_IDLE;
        INTCONbits.T0IF = 0u;
    }
}

static void fan_request(uint8_t speed)
{
    if (speed > 3u)
        speed = 3u;

    /*
     * Called only from main context. Timer0 ownership and FAN_MAKE are shared
     * with the ISR, so inspect/update them atomically.
     */
    INTCONbits.GIE = 0u;

    if ((fan_dead_left == 0u) &&
        ((state_flags & FLAG_FAN_MAKE) == 0u) &&
        (fan_current == speed)) {
        INTCONbits.GIE = 1u;
        return;
    }

    /* Same speed already pending: do not restart dead-time. */
    if (((fan_dead_left != 0u) ||
         ((state_flags & FLAG_FAN_MAKE) != 0u)) &&
        (fan_pending == speed)) {
        INTCONbits.GIE = 1u;
        return;
    }

    INTCONbits.T0IE = 0u;

    /* BREAK immediately. */
    fans_off();

    fan_current = 0u;
    fan_pending = speed;
    fan_dead_left = 0u;
    state_flags &= (uint8_t)~FLAG_FAN_MAKE;

    if (speed == 0u) {
        /*
         * OFF is complete immediately. If UART owns Timer0, leave it alone;
         * otherwise Timer0 stays idle.
         */
        if ((state_flags & FLAG_UART_ACTIVE) == 0u) {
            t0_mode = T0_MODE_IDLE;
            INTCONbits.T0IF = 0u;
        } else {
            INTCONbits.T0IE = 1u;
        }

        INTCONbits.GIE = 1u;
        return;
    }

    fan_dead_left = FAN_DEAD_OVERFLOWS;

    /*
     * UART owns Timer0 while receiving. Fan dead-time is safely paused and
     * starts/restarts after the byte completes.
     */
    if ((state_flags & FLAG_UART_ACTIVE) != 0u) {
        INTCONbits.T0IE = 1u;
        INTCONbits.GIE = 1u;
        return;
    }

    timer0_start_fan();
    INTCONbits.GIE = 1u;
}

static void fan_service(void)
{
    /*
     * Fast-path check first. Then re-check BOTH conditions atomically after
     * masking interrupts: a UART start edge can occur between the first test
     * and GIE=0. Without the second UART_ACTIVE check, an old pending speed
     * could be energized for a few milliseconds while a new UART command is
     * already being received.
     */
    if (((state_flags & FLAG_FAN_MAKE) == 0u) ||
        ((state_flags & FLAG_UART_ACTIVE) != 0u))
        return;

    INTCONbits.GIE = 0u;

    if (((state_flags & FLAG_FAN_MAKE) != 0u) &&
        ((state_flags & FLAG_UART_ACTIVE) == 0u)) {
        state_flags &= (uint8_t)~FLAG_FAN_MAKE;

        if (fan_pending != 0u) {
            fan_on(fan_pending);
            fan_current = fan_pending;
        }
    }

    INTCONbits.GIE = 1u;
}

/* --------------------------------------------------------------------- */

/* Quantize a measured Hob2Hood duration to nominal 725-us units 1..5. */
static uint8_t to_unit(uint16_t t)
{
    if ((t < 400u) || (t > 4300u))
        return 0u;
    if (t < 1100u)
        return 1u;
    if (t < 1825u)
        return 2u;
    if (t < 2550u)
        return 3u;
    if (t < 3275u)
        return 4u;
    return 5u;
}

/* IRremote-style +/-20% relation: 0 shorter, 1 equal, 2 longer. */
static uint8_t relation(uint8_t oldv, uint8_t newv)
{
    if ((uint8_t)(newv * 10u) < (uint8_t)(oldv * 8u))
        return 0u;
    if ((uint8_t)(oldv * 10u) < (uint8_t)(newv * 8u))
        return 2u;
    return 1u;
}

static void calculate_hash(void)
{
    hash_value = FNV_BASIS_32;
    CLRWDT();

    for (hash_i = 0u; (uint8_t)(hash_i + 2u) < rx_count; ++hash_i) {
        hash_value = (hash_value * FNV_PRIME_32) ^
                     (uint32_t)relation(rx_buf[hash_i],
                                        rx_buf[hash_i + 2u]);

        /*
         * 32-bit constant multiplication is relatively expensive on this
         * 8-bit PIC. Feeding WDT here prevents a valid IR frame calculation
         * from being mistaken for a lock-up when WDT is unprescaled.
         */
        CLRWDT();
    }
}

static void dispatch_hash(void)
{
    if (hash_value == H2H_FAN_1)
        fan_request(1u);
    else if (hash_value == H2H_FAN_2)
        fan_request(2u);
    else if (hash_value == H2H_FAN_3)
        fan_request(3u);
    else if (hash_value == H2H_FAN_4)
        fan_request(3u);
    else if (hash_value == H2H_FAN_OFF)
        fan_request(0u);
    else if (hash_value == H2H_LIGHT_ON)
        light_set(1u);
    else if (hash_value == H2H_LIGHT_OFF)
        light_set(0u);
}

static void uart_dispatch(void)
{
    if (uart_byte == '0')
        fan_request(0u);
    else if (uart_byte == '1')
        fan_request(1u);
    else if (uart_byte == '2')
        fan_request(2u);
    else if ((uart_byte == '3') || (uart_byte == '4'))
        fan_request(3u);
    else if (uart_byte == 'L')
        light_set(1u);
    else if (uart_byte == 'l')
        light_set(0u);
    else if (uart_byte == 'X') {
        fan_request(0u);
        light_set(0u);
    }

    INTCONbits.GIE = 0u;
    state_flags &= (uint8_t)~FLAG_UART_READY;
    INTCONbits.GIE = 1u;
}

/* --------------------------------------------------------------------- */

void __interrupt() isr(void)
{
    /*
     * Shared GPIO interrupt-on-change.
     * GP3 = IR, GP5 = UART RX.
     */
    if (INTCONbits.GPIF != 0u) {
        uint8_t pins;
        uint8_t old_inputs;
        uint8_t changed;
        uint8_t unit;
        uint16_t t;

        pins = GPIO; /* clears IOC mismatch */
        old_inputs = last_inputs;
        changed = (uint8_t)((pins ^ old_inputs) & INPUT_MASK);
        last_inputs = (uint8_t)(pins & INPUT_MASK);

        /*
         * IR edge: Timer1 is stopped immediately before read/reset.
         * UART edges never reset Timer1.
         */
        if (((changed & IR_MASK) != 0u) &&
            ((state_flags & FLAG_IR_LOCK) == 0u)) {
            T1CONbits.TMR1ON = 0u;
            t = (uint16_t)(((uint16_t)TMR1H << 8) | (uint16_t)TMR1L);
            TMR1H = 0u;
            TMR1L = 0u;
            T1CONbits.TMR1ON = 1u;

            if ((state_flags & FLAG_RX_ACTIVE) == 0u) {
                if ((pins & IR_MASK) == 0u) {
                    state_flags |= FLAG_RX_ACTIVE;
                    state_flags &= (uint8_t)~FLAG_RX_BAD;
                    rx_count = 0u;
                }
            } else {
                unit = to_unit(t);

                if ((unit == 0u) || (rx_count >= RX_MAX))
                    state_flags |= FLAG_RX_BAD;
                else
                    rx_buf[rx_count++] = unit;
            }
        }

        /*
         * UART start edge. Disable GP5 IOC while receiving the byte so
         * data-bit transitions do not generate useless GPIO interrupts.
         */
        if (((changed & UART_RX_MASK) != 0u) &&
            ((old_inputs & UART_RX_MASK) != 0u) &&
            ((pins & UART_RX_MASK) == 0u) &&
            ((state_flags & FLAG_UART_ACTIVE) == 0u) &&
            ((state_flags & FLAG_UART_READY) == 0u)) {

            state_flags |= FLAG_UART_ACTIVE;
            uart_bit = UART_STATE_VERIFY_START;
            uart_byte = 0u;

            if ((state_flags & FLAG_IR_LOCK) != 0u)
                IOC = 0u;
            else
                IOC = IR_MASK;

            timer0_start_uart();
        }

        INTCONbits.GPIF = 0u;
    }

    if (INTCONbits.T0IF != 0u) {
        INTCONbits.T0IF = 0u;

        if (t0_mode == T0_MODE_UART) {
            /*
             * First overflow verifies that the start bit is still LOW near
             * its center. This rejects short glitches on the UART input.
             */
            if (uart_bit == UART_STATE_VERIFY_START) {
                if ((GPIO & UART_RX_MASK) == 0u) {
                    uart_bit = 0u;

                    /*
                     * Phase-corrected reload: account for Timer0 counts that
                     * accumulated between overflow and ISR service.
                     */
                    TMR0 = (uint8_t)(TMR0 + UART_BIT_PRELOAD);
                } else {
                    /* False start/glitch: abort without producing a byte. */
                    state_flags &= (uint8_t)~FLAG_UART_ACTIVE;

                    /*
                     * GP5 IOC was disabled during byte reception. Refresh only
                     * the GP5 software baseline. Preserve the GP3 baseline:
                     * an IR edge may have happened after the GPIF check at ISR
                     * entry. Do NOT clear GPIF here; if such an IR edge is
                     * pending, the ISR will immediately run again and process it.
                     */
                    last_inputs = (uint8_t)((last_inputs & IR_MASK) |
                                             (GPIO & UART_RX_MASK));

                    if ((state_flags & FLAG_IR_LOCK) != 0u)
                        IOC = UART_RX_MASK;
                    else
                        IOC = INPUT_MASK;

                    timer0_after_uart();
                }
            } else if (uart_bit < 8u) {
                if ((GPIO & UART_RX_MASK) != 0u)
                    uart_byte |= (uint8_t)(1u << uart_bit);

                ++uart_bit;

                /*
                 * Keep the next sample phase referenced to the previous
                 * overflow rather than to ISR completion.
                 */
                TMR0 = (uint8_t)(TMR0 + UART_BIT_PRELOAD);
            } else {
                /* Stop bit must be HIGH. */
                if ((GPIO & UART_RX_MASK) != 0u)
                    state_flags |= FLAG_UART_READY;

                state_flags &= (uint8_t)~FLAG_UART_ACTIVE;

                /*
                 * Re-arm GP5 IOC from the current pin state.
                 */
                /*
                 * Preserve the GP3 software baseline and any pending GPIF.
                 * This prevents a Timer0/UART completion interrupt from
                 * swallowing an IR edge that arrived during this ISR.
                 */
                last_inputs = (uint8_t)((last_inputs & IR_MASK) |
                                         (GPIO & UART_RX_MASK));

                if ((state_flags & FLAG_IR_LOCK) != 0u)
                    IOC = UART_RX_MASK;
                else
                    IOC = INPUT_MASK;

                timer0_after_uart();
            }
        } else if (t0_mode == T0_MODE_FAN) {
            if (fan_dead_left != 0u) {
                --fan_dead_left;

                if (fan_dead_left == 0u) {
                    state_flags |= FLAG_FAN_MAKE;
                    t0_mode = T0_MODE_IDLE;
                    INTCONbits.T0IE = 0u;
                }
            } else {
                t0_mode = T0_MODE_IDLE;
                INTCONbits.T0IE = 0u;
            }
        } else {
            /* Defensive: no Timer0 interrupt is expected in IDLE mode. */
            INTCONbits.T0IE = 0u;
        }
    }
}

/* Stable Timer1 read while Timer1 continues running. */
static uint16_t timer1_now(void)
{
    uint8_t h1;
    uint8_t h2;
    uint8_t l;

    do {
        h1 = TMR1H;
        l = TMR1L;
        h2 = TMR1H;
    } while (h1 != h2);

    return (uint16_t)(((uint16_t)h2 << 8) | (uint16_t)l);
}

static void ir_service(void)
{
    if ((state_flags & FLAG_RX_ACTIVE) == 0u)
        return;

    if ((last_inputs & IR_MASK) == 0u)
        return;

    if (timer1_now() < FRAME_GAP_US)
        return;

    /*
     * Freeze the completed IR frame while it is hashed.
     *
     * FLAG_IR_LOCK is important: UART can start/finish while calculate_hash()
     * runs. UART ISR paths must not accidentally re-enable GP3 and allow a
     * new IR frame to overwrite rx_buf/rx_count while the hash reads them.
     */
    INTCONbits.GIE = 0u;

    if (((state_flags & FLAG_RX_ACTIVE) == 0u) ||
        ((last_inputs & IR_MASK) == 0u) ||
        (timer1_now() < FRAME_GAP_US)) {
        INTCONbits.GIE = 1u;
        return;
    }

    state_flags |= FLAG_IR_LOCK;
    state_flags &= (uint8_t)~FLAG_RX_ACTIVE;

    /*
     * Keep UART start detection available while hashing if UART is idle.
     * GP3 is disabled until FLAG_IR_LOCK is cleared.
     *
     * Do not clear GPIF here. If a UART edge became pending in this tiny
     * critical section, leaving GPIF set lets the ISR process it afterward.
     */
    if ((state_flags & FLAG_UART_ACTIVE) != 0u)
        IOC = 0u;
    else
        IOC = UART_RX_MASK;

    INTCONbits.GIE = 1u;

    if (((state_flags & FLAG_RX_BAD) == 0u) &&
        (rx_count >= 9u) &&
        (rx_count <= RX_MAX)) {
        calculate_hash();
        dispatch_hash();
    }

    /*
     * Re-arm GP3 after hash processing. While GP3 was locked, IR edges were
     * intentionally discarded. Establish a fresh GP3 baseline and preserve
     * GP5's software baseline so a pending UART falling edge is not erased.
     */
    INTCONbits.GIE = 0u;

    TMR1H = 0u;
    TMR1L = 0u;

    last_inputs = (uint8_t)((last_inputs & UART_RX_MASK) |
                             (GPIO & IR_MASK));

    state_flags &= (uint8_t)~FLAG_IR_LOCK;

    if ((state_flags & FLAG_UART_ACTIVE) != 0u)
        IOC = IR_MASK;
    else
        IOC = INPUT_MASK;

    /*
     * Intentionally do not clear GPIF. A UART/IR change that happened during
     * this short critical section will cause an immediate ISR pass. Because
     * GP3's software baseline was refreshed above, any stale IR-only flag is
     * harmless, while a pending GP5 falling edge remains detectable.
     */
    INTCONbits.GIE = 1u;
}

/* --------------------------------------------------------------------- */

static void init_hw(void)
{
    OSCCAL = OSCCAL_VALUE;

    /* All GPIO digital; comparator off. */
    ADCON0 = 0x00u;
    ANSEL  = 0x00u;
    CMCON  = 0x07u;

    /*
     * Set inactive relay latch levels BEFORE making those pins outputs.
     * Active-low relay board: HIGH = OFF.
     */
    gpio_shadow = RELAY_MASK;
    GPIO = gpio_shadow;

    /* GP3 and GP5 inputs; GP0/1/2/4 outputs. */
    TRISIO = 0x28u;

    /* Timer1: internal Fosc/4, prescaler 1:1 ~= 1 us/tick. */
    T1CON = 0x01u;
    TMR1H = 0u;
    TMR1L = 0u;

    /*
     * Timer0 starts idle. On Reset the shared prescaler belongs to WDT.
     * Microchip requires CLRWDT before changing prescaler assignment
     * WDT -> Timer0. After this point WDT runs without a prescaler.
     */
    CLRWDT();
    OPTION_REG = OPTION_T0_FAN;
    TMR0 = 0u;
    t0_mode = T0_MODE_IDLE;

    state_flags = 0u;
    rx_count = 0u;

    fan_current = 0u;
    fan_pending = 0u;
    fan_dead_left = 0u;

    uart_bit = 0u;
    uart_byte = 0u;

    last_inputs = (uint8_t)(GPIO & INPUT_MASK);

    IOC = INPUT_MASK;

    INTCONbits.GPIF = 0u;
    INTCONbits.T0IF = 0u;

    PIE1 = 0x00u;
    PIR1 = 0x00u;

    INTCONbits.T0IE = 0u;
    INTCONbits.GPIE = 1u;
    INTCONbits.PEIE = 0u;
    INTCONbits.GIE = 1u;
}

void main(void)
{
    init_hw();

    for (;;) {
        /*
         * Feed watchdog only from healthy foreground execution.
         * ISR code intentionally never calls CLRWDT().
         */
        CLRWDT();

        ir_service();

        /*
         * Apply a completed UART command before fan_service(). If dead-time
         * expired at the same instant as a new manual command arrived, the
         * newest command wins without briefly energizing the old pending speed.
         */
        if ((state_flags & FLAG_UART_READY) != 0u)
            uart_dispatch();

        fan_service();
    }
}
