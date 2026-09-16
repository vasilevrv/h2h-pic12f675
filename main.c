/*
 * Hob2Hood receiver - PIC12F675 - XC8
 * v4.5 BUTTON + WDT
 *
 * GP0 -> Fan 1 relay
 * GP1 -> Fan 2 relay
 * GP2 -> Fan 3 relay
 * GP3 <- IR receiver OUT
 * GP4 -> Light relay
 * GP5 <- Push button to GND (internal pull-up enabled)
 *
 * Relay inputs are assumed ACTIVE LOW.
 *
 * Each debounced short button click selects the next mode:
 *   1. light ON, fan OFF
 *   2. light ON, fan speed 1
 *   3. light ON, fan speed 2
 *   4. light ON, fan speed 3
 *
 * A button hold of approximately one second switches everything OFF and
 * resets the short-click sequence. The next short click starts at mode 1.
 *
 * Design goals:
 *   - no blocking delays
 *   - IR edges have priority
 *   - Timer1 is dedicated to IR timing
 *   - Timer0 interrupts are NOT periodic while idle
 *   - button debounce is non-blocking and uses Timer0 only while needed
 *   - fan break-before-make uses Timer0 only while a transition is pending
 *   - GPIO shadow avoids PIC read-modify-write surprises on relay outputs
 *   - watchdog enabled; deliberately cleared only from healthy main-line code
 *   - WDT has NO prescaler after init because the shared prescaler belongs
 *     to Timer0; nominal WDT timeout is ~18 ms (device-dependent)
 *
 * IMPORTANT HARDWARE REQUIREMENTS:
 *   - connect a normally-open button between GP5 and GND
 *   - an external ~10 kOhm pull-up on GP5 is recommended in noisy wiring
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
#define BUTTON_MASK              0x20u  /* GP5, active LOW */

#define INPUT_MASK               (IR_MASK | BUTTON_MASK)
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
 * Timer0: Fcy ~= 1 MHz, prescaler 1:256, overflow ~= 65.536 ms.
 * One quiet overflow validates a button state; the long-press state reaches
 * its threshold after ~=0.98 s from the input edge; three overflows provide
 * ~=196.6 ms fan break-before-make time.
 * Timer0 is off when neither is needed.
 * Bit 7 is clear so the GPIO weak pull-ups (including GP5) stay enabled.
 */
#define OPTION_T0                 0x07u  /* internal, prescaler 1:256, pull-ups ON */

#define FAN_DEAD_OVERFLOWS        3u
#define BUTTON_LONG_OVERFLOWS     16u   /* ~= 0.98 s from the input edge */
#define BUTTON_HOLD_HANDLED       0xffu

/* Packed state flags: saves 3 bytes of scarce PIC12F675 SRAM. */
#define FLAG_RX_ACTIVE             0x01u
#define FLAG_RX_BAD                0x02u
#define FLAG_FAN_MAKE              0x04u
#define FLAG_BUTTON_DEBOUNCE       0x08u
#define FLAG_BUTTON_EVENT          0x10u
#define FLAG_IR_LOCK               0x20u  /* rx_buf/hash protected; discard IR edges */
#define FLAG_BUTTON_STABLE_HIGH    0x40u
#define FLAG_BUTTON_CANDIDATE_HIGH 0x80u

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

/* Button mode: 0..3 correspond to light, speed 1, speed 2 and speed 3. */
static uint8_t button_mode;

/*
 * Button hold state. Zero means no stable press is being timed, 1..15 are
 * elapsed Timer0 overflows, 16 means a long-press event is pending, and 0xff
 * means the long press was already handled.
 */
static volatile uint8_t button_hold_ticks;

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

/* Start/restart the shared fan/debounce timer. Call with T0IE disabled. */
static void timer0_start(void)
{
    OPTION_REG = OPTION_T0;
    TMR0 = 0u;
    INTCONbits.T0IF = 0u;
    INTCONbits.T0IE = 1u;
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
        /* OFF is complete immediately; preserve any active button timing. */
        if (((state_flags & FLAG_BUTTON_DEBOUNCE) != 0u) ||
            (((state_flags & FLAG_BUTTON_STABLE_HIGH) == 0u) &&
             (button_hold_ticks != 0u) &&
             (button_hold_ticks < BUTTON_LONG_OVERFLOWS))) {
            INTCONbits.T0IE = 1u;
        } else {
            INTCONbits.T0IF = 0u;
        }

        INTCONbits.GIE = 1u;
        return;
    }

    fan_dead_left = FAN_DEAD_OVERFLOWS;
    timer0_start();
    INTCONbits.GIE = 1u;
}

static void fan_service(void)
{
    if ((state_flags & FLAG_FAN_MAKE) == 0u)
        return;

    INTCONbits.GIE = 0u;

    if ((state_flags & FLAG_FAN_MAKE) != 0u) {
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

static void button_dispatch(void)
{
    INTCONbits.GIE = 0u;
    state_flags &= (uint8_t)~FLAG_BUTTON_EVENT;
    INTCONbits.GIE = 1u;

    if (button_mode == 0u) {
        fan_request(0u);
        light_set(1u);
    } else if (button_mode == 1u) {
        light_set(1u);
        fan_request(1u);
    } else if (button_mode == 2u) {
        light_set(1u);
        fan_request(2u);
    } else if (button_mode == 3u) {
        light_set(1u);
        fan_request(3u);
    }

    ++button_mode;
    if (button_mode >= 4u)
        button_mode = 0u;
}

static void button_long_dispatch(void)
{
    uint8_t hold_state;

    INTCONbits.GIE = 0u;
    hold_state = button_hold_ticks;

    if ((state_flags & FLAG_BUTTON_EVENT) == 0u ||
        (hold_state < BUTTON_LONG_OVERFLOWS)) {
        INTCONbits.GIE = 1u;
        return;
    }

    state_flags &= (uint8_t)~FLAG_BUTTON_EVENT;
    button_hold_ticks = BUTTON_HOLD_HANDLED;
    button_mode = 0u;
    INTCONbits.GIE = 1u;

    fan_request(0u);
    light_set(0u);
}

/* --------------------------------------------------------------------- */

void __interrupt() isr(void)
{
    /*
     * Shared GPIO interrupt-on-change.
     * GP3 = IR, GP5 = push button.
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
         * Button edges never reset Timer1.
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
         * Every button edge restarts a quiet-time interval. Only a level that
         * remains unchanged for a complete Timer0 period is accepted.
         * Restarting Timer0 may only lengthen an active fan dead time, which
         * is safe for the mutually exclusive speed relays.
         */
        if ((changed & BUTTON_MASK) != 0u) {
            if ((pins & BUTTON_MASK) != 0u)
                state_flags |= FLAG_BUTTON_CANDIDATE_HIGH;
            else
                state_flags &= (uint8_t)~FLAG_BUTTON_CANDIDATE_HIGH;

            state_flags |= FLAG_BUTTON_DEBOUNCE;

            INTCONbits.T0IE = 0u;
            OPTION_REG = OPTION_T0;
            TMR0 = 0u;
            INTCONbits.T0IF = 0u;
            INTCONbits.T0IE = 1u;
        }

        INTCONbits.GPIF = 0u;
    }

    if (INTCONbits.T0IF != 0u) {
        INTCONbits.T0IF = 0u;

        if ((state_flags & FLAG_BUTTON_DEBOUNCE) != 0u) {
            if ((((GPIO & BUTTON_MASK) != 0u) &&
                 ((state_flags & FLAG_BUTTON_CANDIDATE_HIGH) != 0u)) ||
                (((GPIO & BUTTON_MASK) == 0u) &&
                 ((state_flags & FLAG_BUTTON_CANDIDATE_HIGH) == 0u))) {
                state_flags &= (uint8_t)~FLAG_BUTTON_DEBOUNCE;

                if ((state_flags & FLAG_BUTTON_CANDIDATE_HIGH) != 0u) {
                    /* A short click is accepted only on button release. */
                    if ((state_flags & FLAG_BUTTON_STABLE_HIGH) == 0u) {
                        if ((button_hold_ticks != 0u) &&
                            (button_hold_ticks < BUTTON_LONG_OVERFLOWS))
                            state_flags |= FLAG_BUTTON_EVENT;

                        if (button_hold_ticks < BUTTON_LONG_OVERFLOWS)
                            button_hold_ticks = 0u;
                        else
                            button_hold_ticks = BUTTON_HOLD_HANDLED;
                    }

                    state_flags |= FLAG_BUTTON_STABLE_HIGH;
                } else if ((state_flags & FLAG_BUTTON_STABLE_HIGH) != 0u) {
                    /* Stable press starts the long-press timer. */
                    state_flags &= (uint8_t)~FLAG_BUTTON_STABLE_HIGH;
                    /* Preserve a pending long event across a rapid re-press. */
                    if (((state_flags & FLAG_BUTTON_EVENT) == 0u) ||
                        (button_hold_ticks < BUTTON_LONG_OVERFLOWS))
                        button_hold_ticks = 1u;
                } else if (button_hold_ticks == 0u) {
                    /* Also handle a button that was already held at reset. */
                    button_hold_ticks = 1u;
                }
            } else {
                if ((GPIO & BUTTON_MASK) != 0u)
                    state_flags |= FLAG_BUTTON_CANDIDATE_HIGH;
                else
                    state_flags &= (uint8_t)~FLAG_BUTTON_CANDIDATE_HIGH;

                TMR0 = 0u;
            }
        }

        if (((state_flags & FLAG_BUTTON_DEBOUNCE) == 0u) &&
            ((state_flags & FLAG_BUTTON_STABLE_HIGH) == 0u) &&
            (button_hold_ticks != 0u) &&
            (button_hold_ticks < BUTTON_LONG_OVERFLOWS)) {
            ++button_hold_ticks;

            if (button_hold_ticks >= BUTTON_LONG_OVERFLOWS)
                state_flags |= FLAG_BUTTON_EVENT;
        }

        if (fan_dead_left != 0u) {
            --fan_dead_left;

            if (fan_dead_left == 0u)
                state_flags |= FLAG_FAN_MAKE;
        }

        if ((fan_dead_left == 0u) &&
            ((state_flags & FLAG_BUTTON_DEBOUNCE) == 0u) &&
            (((state_flags & FLAG_BUTTON_STABLE_HIGH) != 0u) ||
             (button_hold_ticks >= BUTTON_LONG_OVERFLOWS)))
            INTCONbits.T0IE = 0u;
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
     * FLAG_IR_LOCK prevents a new IR frame from overwriting rx_buf/rx_count
     * while calculate_hash() reads them. Button interrupt-on-change remains
     * enabled during this short interval.
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

    /* Keep the button enabled; GP3 stays disabled until hashing is complete. */
    IOC = BUTTON_MASK;

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
     * GP5's button baseline so a pending button edge is not erased.
     */
    INTCONbits.GIE = 0u;

    TMR1H = 0u;
    TMR1L = 0u;

    last_inputs = (uint8_t)((last_inputs & BUTTON_MASK) |
                             (GPIO & IR_MASK));

    state_flags &= (uint8_t)~FLAG_IR_LOCK;

    IOC = INPUT_MASK;

    /*
     * Intentionally do not clear GPIF. A button/IR change that happened during
     * this short critical section will cause an immediate ISR pass. Because
     * GP3's software baseline was refreshed above, any stale IR-only flag is
     * harmless, while a pending GP5 button edge remains detectable.
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

    /* Select the GP5 weak pull-up; OPTION_T0 enables it globally below. */
    WPU = BUTTON_MASK;

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
    OPTION_REG = OPTION_T0;
    TMR0 = 0u;

    state_flags = 0u;
    rx_count = 0u;

    fan_current = 0u;
    fan_pending = 0u;
    fan_dead_left = 0u;

    last_inputs = (uint8_t)(GPIO & INPUT_MASK);
    button_hold_ticks = 0u;

    if ((last_inputs & BUTTON_MASK) != 0u) {
        state_flags |= FLAG_BUTTON_STABLE_HIGH;
        state_flags |= FLAG_BUTTON_CANDIDATE_HIGH;
    } else {
        /* Start timing if the button is already held during reset. */
        button_hold_ticks = 1u;
    }

    /* Power-up state is OFF; the first short click selects mode 1. */
    button_mode = 0u;

    IOC = INPUT_MASK;

    INTCONbits.GPIF = 0u;
    INTCONbits.T0IF = 0u;

    PIE1 = 0x00u;
    PIR1 = 0x00u;

    INTCONbits.T0IE = (button_hold_ticks != 0u) ? 1u : 0u;
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

        /* Long press has priority over a short-click event. */
        if ((state_flags & FLAG_BUTTON_EVENT) != 0u) {
            if (button_hold_ticks >= BUTTON_LONG_OVERFLOWS)
                button_long_dispatch();
            else
                button_dispatch();
        }

        fan_service();
    }
}
