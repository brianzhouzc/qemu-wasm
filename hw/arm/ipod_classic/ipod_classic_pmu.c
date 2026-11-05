/*
 * NXP PCF50635 power management unit, at I2C address 0xe6 (7-bit 0x73).
 *
 * The first chip Apple's retailOS talks to. 0x08325a20 starts the transfer and
 * then blocks on RTXC event 61 with nothing but a retry loop behind it, so an
 * iPod classic with nothing on I2C0 boots as far as its idle task and stays
 * there for ever -- which is exactly where this machine stopped before this
 * file existed.
 *
 * The bus protocol is a register-pointer device: a write starts with the
 * register number and the pointer advances after every byte; a read carries on
 * from wherever the pointer was left. That the pointer survives the end of a
 * transaction is not a guess -- Rockbox's i2c_rd() sends the register number in
 * one transaction and reads in a second one (i2c-s5l8702.c, i2c_wr_internal
 * with len 0 followed by i2c_rd_internal), and Apple's driver does the same.
 *
 * Register numbers are from Rockbox's firmware/export/pcf5063x.h, with the
 * three undocumented ones (INT6 0x85, INT6M 0x86, GPIOSTAT 0x87) from
 * firmware/target/arm/s5l8702/ipod6g/pmu-target.h.
 *
 * WHAT IS MODELLED
 *
 *   - The register file, which is what most of this chip is to a driver:
 *     regulator voltages, enables, interrupt masks and standby control get
 *     written and, when read back, have to read back.
 *   - The A/D converter. ADCC1 bit 0 starts a conversion; ADCS1/ADCS2/ADCS3
 *     return a 10-bit result with ADCRDY set. See the note on the value below.
 *   - The interrupt status registers INT1..INT5 and INT6, which are
 *     read-to-clear on the real chip and therefore must not be a plain
 *     register file: a driver that reads them expects the bits to go away.
 *   - The chip's interrupt output, nIRQ, which is GPIO 0x7b at the SoC: both
 *     firmwares register that pin as a level-low external interrupt (Rockbox
 *     GPIO_EINT_PMU, pmu-6g.c:203; Apple's 2.0.5 at 0x083952a4). The EIC side
 *     is in ipod_classic.c. Both also mask INT1_SECOND, so the tick below does
 *     not pull the line down (measured on 2.0.5, INT1M = 0xb0).
 *   - The hold switch, GPIOSTAT bit 1, and its change interrupt, INT6 bit 1.
 *   - The RTC's time, RTCSC..RTCYR, read from the host's clock in UTC; see
 *     the note at the read.  A write to it is stored and never read back.
 *   - The LED converter that drives the LCD's backlight, LEDOUT..LEDDIM,
 *     with its dimming ramp, and its loss of supply when OOCSHDWN cuts the
 *     power.  The level reaches the page through ipod_wasm_backlight().  See
 *     THE BACKLIGHT below.
 *
 * WHAT IS NOT
 *
 *   - The RTC's alarm: its registers read back whatever was written.  Worth
 *     knowing because on the 5G an absent RTC had a visible side effect:
 *     Rockbox decided an alarm had gone off on every boot.
 *   - Charging, the ON/OFF controller and the regulators. Writing them changes
 *     nothing, because there is no voltage in here for a voltage to mean.
 *
 * ASSUMED, and marked as such because nothing derives them:
 *
 *   - VERSION (0x00) and VARIANT (0x01). No dump of a real PCF50635 exists in
 *     any of our sources, and neither Rockbox nor iPodLinux reads them. They
 *     answer zero. If the firmware turns out to compare them against something
 *     the disassembly will say what, and then they get a real value.
 *   - The battery reading, 844 out of 1023. Through Rockbox's own conversion
 *     (pmu-6g.c: raw * 2000 / 1023 + 2250) that is 3900 mV, an ordinary
 *     charged-but-not-full lithium cell. Nothing in our sources says what a
 *     real one reads; the number is chosen to be unremarkable.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "sysemu/rtc.h"
#include "hw/arm/ipod_classic/ipod_classic_pmu.h"

/*
 * The machine's reaction to the ON/OFF controller's shutdown, defined next to
 * the machine (ipod_classic.c) where the state it sets lives.  Declared here
 * rather than pulling in the machine header, which needs the sysbus types the
 * I2C device does not.
 */
void ipod_classic_power_off(void);
void ipod_classic_wake_vcpu(void);

/*
 * PMUBATT=N overrides the raw battery reading, 0..1023.
 *
 * Why this needs to be reachable: on the way into a song retailOS puts up its
 * "low battery -- connect to power" screen and refuses to play, which is a
 * plausible reason for playback to start and immediately stop. The default 844
 * is 3900 mV only through ROCKBOX's conversion; what Apple's power manager makes
 * of the same number is UNDETERMINED, so the honest thing is to be able to vary
 * it and see, rather than to assume the two firmwares scale it alike.
 */
static uint32_t pmu_battery_raw(void)
{
    /* Read once, like the trace switches (Q18). */
    static int64_t raw = -1;

    if (raw < 0) {
        const char *e = getenv("PMUBATT");

        raw = S5L8702_PMU_ADC_BATTERY_RAW;
        if (e && *e) {
            raw = MIN(strtoul(e, NULL, 0), 1023);
        }
    }
    return (uint32_t)raw;
}

/* PMUCHARGER: 0 unset, 1 USB, 2 adapter.  Read once (Q18). */
static int pmu_charger(void)
{
    static int kind = -1;

    if (kind < 0) {
        const char *e = getenv("PMUCHARGER");

        kind = !e || !*e ? 0 : !strcmp(e, "adapter") ? 2 : 1;
    }
    return kind;
}

static bool pmu_trace_on(void)
{
    static int on = -1;

    if (on < 0) {
        on = getenv("PMUTRACE") != NULL;
    }
    return on;
}

static void pmu_start_adc(IPodClassicPMUState *s)
{
    uint32_t raw = pmu_battery_raw();

    s->reg[PCF5063X_REG_ADCS1] = (raw >> 2) & 0xff;
    s->reg[PCF5063X_REG_ADCS2] = 0;
    s->reg[PCF5063X_REG_ADCS3] = PCF5063X_ADCS3_ADCRDY | (raw & 0x03);
}

/*
 * nIRQ is low while any status bit is pending and not masked. INT6 and its
 * mask sit apart from the other five, at the undocumented 0x85/0x86.
 */
static void pmu_update_irq(IPodClassicPMUState *s)
{
    bool pending = s->reg[PCF50635_REG_INT6] & ~s->reg[PCF50635_REG_INT6M];

    for (int i = 0; i < 5; i++) {
        pending |= s->reg[PCF5063X_REG_INT1 + i] &
                   ~s->reg[PCF5063X_REG_INT1M + i];
    }
    qemu_set_irq(s->nirq, pending);
}

/*
 * THE BACKLIGHT.  The LCD's LEDs hang off this chip's LED boost converter,
 * and both firmwares drive it through the same four registers:
 *
 *   LEDOUT 0x28   output current, 0..63 in bits 5:0
 *   LEDENA 0x29   bit 0 turns the converter on
 *   LEDCTL 0x2a   over-voltage and over-current protection
 *   LEDDIM 0x2b   the dimming step time, 16 * LEDDIM / 32768 s
 *
 * Rockbox (target/arm/s5l8702/ipod6g/backlight-6g.c) writes the brightness
 * setting, 1..63 with 0x20 the default, to LEDOUT, and turns the light on and
 * off with LEDENA.  It sets LEDCTL 0x05 and LEDDIM 0x14, a step of 9.77 ms,
 * and when it must "kill the backlight, instantly" it writes LEDDIM 0 first.
 * That is the evidence for the ramp below: a change of LEDOUT or LEDENA walks
 * the current one step per LEDDIM period, both ways, and LEDDIM 0 jumps.
 * Rockbox has no software fade on this target, so the ramp is the fade.
 *
 * Apple's 2.0.5, MEASURED with PMUPTR: LEDOUT 0x1a and LEDENA 1 at boot; the
 * backlight timer writes LEDENA 0 and any input LEDENA 1; Settings >
 * Brightness writes LEDOUT only, 0x06 at 0 % to 0x2e at 100 %.  It writes
 * neither LEDCTL nor LEDDIM, in a boot, a brightness change or the timer, so
 * on the hardware its fade is whatever its bootloader left in LEDDIM.
 *
 * The reset values are therefore an ASSUMPTION, marked as one: the state a
 * bootloader leaves, which this machine skips (it starts the OS with the boot
 * screen already up, see ipod_classic_lcd_paint_boot_screen()).  Rockbox's
 * bootloader leaves LEDCTL 0x05, LEDDIM 0x14, LEDOUT 0x20, LEDENA 1; Apple's
 * is not in any of our sources, so it gets the same.
 *
 * ALSO ASSUMED: that the light is proportional to LEDOUT.  Nothing we have
 * says how the 64 steps map to current; 64 equal steps is the plain reading,
 * and it is the page's business to turn a level into a picture.
 *
 * The ramp runs on QEMU_CLOCK_REALTIME, like this chip's RTC above: it is the
 * PMU's own 32 kHz oscillator, and it has to look like a fade to someone
 * watching, whatever the virtual clock is doing under -icount.
 */
#define PMU_BL_TICK_NS (NANOSECONDS_PER_SECOND / 60)

static uint8_t pmu_bl_target(IPodClassicPMUState *s)
{
    if (s->standby || !(s->reg[PCF5063X_REG_LEDENA] & PCF5063X_LEDENA_ON)) {
        return 0;
    }
    return s->reg[PCF5063X_REG_LEDOUT] & PCF5063X_LEDOUT_MASK;
}

static uint32_t pmu_bl_step_ns(IPodClassicPMUState *s)
{
    return (uint64_t)s->reg[PCF5063X_REG_LEDDIM] * 16 *
           NANOSECONDS_PER_SECOND / 32768;
}

/* The output current, in LEDOUT steps, at REALTIME `now`. */
static uint8_t pmu_bl_level_at(IPodClassicPMUState *s, int64_t now)
{
    int64_t steps;

    if (s->bl_from == s->bl_to || !s->bl_step_ns) {
        return s->bl_to;
    }
    steps = MAX(now - s->bl_start_ns, 0) / s->bl_step_ns;
    if (s->bl_from < s->bl_to) {
        return MIN(s->bl_from + steps, s->bl_to);
    }
    return MAX((int64_t)s->bl_from - steps, s->bl_to);
}

/*
 * ipod_wasm_backlight()'s word.  Bits 7:0 the level now, 15:8 where the ramp
 * is going, 21:16 LEDOUT, bit 24 LEDENA, bit 25 standby (the PMU has cut the
 * power: the machine is off), bit 31 set once there is a PMU at all.
 */
static void pmu_bl_publish(IPodClassicPMUState *s, uint8_t level)
{
    uint32_t v = level | (uint32_t)s->bl_to << 8 |
                 (uint32_t)(s->reg[PCF5063X_REG_LEDOUT] &
                            PCF5063X_LEDOUT_MASK) << 16 |
                 (uint32_t)(s->reg[PCF5063X_REG_LEDENA] &
                            PCF5063X_LEDENA_ON) << 24 |
                 (uint32_t)s->standby << 25 | 1u << 31;

    qatomic_set(&s->bl_published, v);
}

static void pmu_bl_trace(IPodClassicPMUState *s, uint8_t level)
{
    static int on = -1;

    if (on < 0) {
        on = getenv("BLTRACE") != NULL;
    }
    if (on) {
        fprintf(stderr, "BLTRACE %" PRId64 " level %u -> %u step %u ns%s\n",
                qemu_clock_get_us(QEMU_CLOCK_REALTIME), (unsigned)level,
                (unsigned)s->bl_to, (unsigned)s->bl_step_ns,
                s->standby ? " standby" : "");
    }
}

/*
 * Publish the level now and keep publishing while the ramp walks.  The page
 * samples the word at its own rate, so one publish per step, or per 60 Hz
 * frame when the steps are shorter, is all it can use.
 */
static void pmu_bl_tick(void *opaque)
{
    IPodClassicPMUState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    uint8_t level = pmu_bl_level_at(s, now);

    pmu_bl_publish(s, level);
    if (level != s->bl_to) {
        timer_mod_ns(s->bl_ramp, now + MAX(s->bl_step_ns, PMU_BL_TICK_NS));
    }
}

/*
 * A write moved the target or the step time: start a new ramp from wherever
 * the current is now.  A write that changes neither leaves the ramp alone --
 * Apple's firmware writes LEDENA 1 twice for every key, and restarting the
 * ramp on each one would hold a fade back by up to a step per key -- but
 * still publishes, for a LEDOUT written while the light is off.
 */
static void pmu_bl_update(IPodClassicPMUState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    uint8_t target = pmu_bl_target(s);
    /* A cut supply does not ramp: the converter just stops. */
    uint32_t step = s->standby ? 0 : pmu_bl_step_ns(s);
    uint8_t level;

    if (target == s->bl_to && step == s->bl_step_ns) {
        pmu_bl_publish(s, pmu_bl_level_at(s, now));
        return;
    }
    level = s->standby ? 0 : pmu_bl_level_at(s, now);
    s->bl_from = level;
    s->bl_to = target;
    s->bl_step_ns = step;
    s->bl_start_ns = now;
    pmu_bl_trace(s, level);
    if (level == target || !step) {
        s->bl_from = target;
        timer_del(s->bl_ramp);
        pmu_bl_publish(s, target);
        return;
    }
    pmu_bl_tick(s);
}

#ifdef EMSCRIPTEN
#include <emscripten.h>

static IPodClassicPMUState *wasm_pmu_state;

/*
 * The backlight for the page, packed as pmu_bl_publish() says; 0 before the
 * PMU exists.  Read from the page's thread, so it is one word written whole.
 */
EMSCRIPTEN_KEEPALIVE uint32_t ipod_wasm_backlight(void)
{
    return wasm_pmu_state ? qatomic_read(&wasm_pmu_state->bl_published) : 0;
}
#endif

void ipod_classic_pmu_set_hold(IPodClassicPMUState *s, bool on)
{
    if (s->hold == on) {
        return;
    }
    s->hold = on;
    s->reg[PCF50635_REG_INT6] |= PCF50635_INT6_HOLD;
    pmu_update_irq(s);
}

static uint8_t pmu_reg_read(IPodClassicPMUState *s, uint8_t addr)
{
    uint8_t val = s->reg[addr];

    switch (addr) {
    case PCF5063X_REG_INT1:
    case PCF5063X_REG_INT2:
    case PCF5063X_REG_INT3:
    case PCF5063X_REG_INT4:
    case PCF5063X_REG_INT5:
    case PCF50635_REG_INT6:
        /* Read to clear, which is what makes an interrupt line drop. */
        s->reg[addr] = 0;
        pmu_update_irq(s);
        break;
    case PCF50635_REG_GPIOSTAT:
        /*
         * The hold switch. See the long note at the reset function, and why
         * this MUST NOT depend on whatever a write has left in s->reg[0x87]:
         * Apple's own GPIO code writes this register (measured: a byte 0x87
         * goes to reg 0x87), so the stored value means nothing and the read
         * comes from the switch no matter what was written.
         */
        val = s->hold ? 0 : PCF50635_GPIOSTAT_HOLD_OFF;
        break;
    case PCF5063X_REG_OOCSTAT:
        /*
         * On/off-controller status, forced to a healthy powered value. Apple's
         * power manager polls this; with BATOK/SYSOK clear it concludes the
         * battery is empty and software-resets ("battery low", tag "--------").
         * A plain register file leaves it stale/zero. See the header.
         */
        val = PCF5063X_OOCSTAT_HEALTHY;
        break;
    case PCF5063X_REG_MBCS1:
        /*
         * Main-battery-charger status. The stale value reads as a spurious
         * TBAT_BELOW temperature fault; force the honest "temp OK, no charger"
         * state instead. See the header.
         */
        val = PCF5063X_MBCS1_HEALTHY;
        /*
         * PMUCHARGER=1: report external power present and OK.
         *
         * MEASURED: with the default "on battery, no charger" report the
         * retailOS eventually puts up "Low Battery -- Connect to Power" and
         * stops playing, and raising the battery ADC to full scale does NOT
         * prevent it (see the note above pmu_battery_raw). That is coherent --
         * it is asking to be plugged in, and nothing here ever says it is.
         *
         * An emulated iPod has no cell to discharge, so "sitting in a dock"
         * is at least as honest a description of it as "running on a battery
         * that never moves", and it is the one the firmware can act on. Kept
         * behind a variable because it IS a model choice, not a measurement:
         * which of USB or adapter a real device would report depends on what
         * it is plugged into, and we are plugged into nothing.
         */
        {
            int kind = pmu_charger();
            if (kind) {
                /*
                 * WHICH KIND of external power matters, and getting it wrong
                 * was MEASURED: reporting the wall ADAPTER puts the device
                 * into its full-screen "Charging" mode and it stops playing --
                 * which is what a real iPod does on a charger, so the model was
                 * right and the choice was wrong. USB power is the one that
                 * leaves it usable. Default is USB; PMUCHARGER=adapter keeps
                 * the other one reachable for testing that screen on purpose.
                 */
                if (kind == 2) {
                    val |= PCF5063X_MBCS1_ADAPTPRES | PCF5063X_MBCS1_ADAPTOK;
                } else {
                    val |= PCF5063X_MBCS1_USBPRES | PCF5063X_MBCS1_USBOK;
                }
            }
        }
        break;
    case PCF5063X_REG_RTCSC:
    case PCF5063X_REG_RTCSC + 1:
    case PCF5063X_REG_RTCSC + 2:
    case PCF5063X_REG_RTCSC + 3:
    case PCF5063X_REG_RTCSC + 4:
    case PCF5063X_REG_RTCSC + 5:
    case PCF5063X_REG_RTCSC + 6:
        /*
         * THE CLOCK RUNS. Measured (PMUTRACE=1): the firmware re-reads this
         * whole block, 0x59..0x5f, over and over, and a plain register file
         * answers zero every time -- a real part's seconds field cannot stay
         * still. That is visible on screen too: the Clock and Calendar screens
         * show a fixed nonsense date.
         *
         * PCF506xx keeps these fields in BCD, and the block is
         * second, minute, hour, weekday, day, month, year (year is 00..99).
         * It is the host's time, not qemu_get_timedate(), so -rtc on the
         * command line does not reach it.
         */
        {
            struct tm tm;
            unsigned v;

            {
                /*
                 * The RTC must follow the host, not QEMU_CLOCK_VIRTUAL:
                 * under -icount the virtual clock is warped to the next timer
                 * while the vCPU is halted, which made the guest clock race.
                 */
                time_t host_now = time(NULL);
                gmtime_r(&host_now, &tm);
            }
            switch (addr - PCF5063X_REG_RTCSC) {
            case 0: v = tm.tm_sec;          break;
            case 1: v = tm.tm_min;          break;
            case 2: v = tm.tm_hour;         break;
            case 3: v = tm.tm_wday;         break;
            case 4: v = tm.tm_mday;         break;
            case 5: v = tm.tm_mon + 1;      break;
            default: v = tm.tm_year % 100;  break;
            }
            val = (uint8_t)(((v / 10) << 4) | (v % 10));
        }
        break;
    default:
        break;
    }

    /*
     * PMUTRACE=1: which registers does Apple's power manager actually consult?
     *
     * Needed because raising the battery ADC to full scale (PMUBATT=1023) did
     * NOT stop retailOS putting up "low battery -- connect to power", so the
     * verdict is not coming from the channel this model answers with. Only the
     * PMU and the CS42L55 sit on I2C here, so it has to be some other register
     * of this part. Counting each address, and printing the first few reads of
     * each, says which ones without burying the firmware's own semihosting
     * output on the same stream.
     */
    if (pmu_trace_on()) {
        static uint32_t seen[256];
        if (seen[addr]++ < 3) {
            fprintf(stderr, "PMUTRACE: read reg 0x%02x -> 0x%02x\n", addr, val);
        }
    }

    return val;
}

static void pmu_reg_write(IPodClassicPMUState *s, uint8_t addr, uint8_t val)
{
    switch (addr) {
    case PCF5063X_REG_INT1:
    case PCF5063X_REG_INT2:
    case PCF5063X_REG_INT3:
    case PCF5063X_REG_INT4:
    case PCF5063X_REG_INT5:
    case PCF50635_REG_INT6:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: INT%d is read-only\n", __func__,
                      addr == PCF50635_REG_INT6 ? 6 : addr - 1);
        return;
    case PCF5063X_REG_INT1M:
    case PCF5063X_REG_INT2M:
    case PCF5063X_REG_INT3M:
    case PCF5063X_REG_INT4M:
    case PCF5063X_REG_INT5M:
    case PCF50635_REG_INT6M:
        s->reg[addr] = val;
        pmu_update_irq(s);
        return;
    case PCF5063X_REG_OOCSHDWN:
        /*
         * The ON/OFF controller's shutdown.  Rockbox writes 0x01 here at the
         * end of power_off(), after painting "Shutting down...", and then
         * waits for the supply to drop -- which never happens in a model with
         * no supply.  Tell the machine instead, so a later key press can power
         * it back on with a reset (see ipod_classic_power_off()).
         *
         * The one supply this model can cut is the backlight's: the screen of
         * an iPod that is off is dark, whatever the panel last showed, and
         * that is what tells "off" from "backlight timed out" on the page.
         */
        s->reg[addr] = val;
        if (val) {
            s->standby = true;
            pmu_bl_update(s);
            ipod_classic_power_off();
        }
        return;
    case PCF5063X_REG_LEDOUT:
    case PCF5063X_REG_LEDENA:
    case PCF5063X_REG_LEDDIM:
        s->reg[addr] = val;
        pmu_bl_update(s);
        return;
    case PCF5063X_REG_ADCC1:
        s->reg[addr] = val;
        if (val & PCF5063X_ADCC1_ADCSTART) {
            pmu_start_adc(s);
        }
        return;
    default:
        s->reg[addr] = val;
        return;
    }
}

static int ipod_classic_pmu_event(I2CSlave *i2c, enum i2c_event event)
{
    IPodClassicPMUState *s = IPOD_CLASSIC_PMU(i2c);

    switch (event) {
    case I2C_START_SEND:
        /* The next byte is the register number, not data. */
        s->addr_pending = true;
        /*
         * Remember where the pointer stood so a register write can put it back
         * at STOP; see the note above ipod_classic_pmu_send.
         */
        s->addr_saved = s->addr;
        s->wrote_data = false;
        break;
    case I2C_START_RECV:
        s->wrote_data = false;
        break;
    case I2C_FINISH:
        /*
         * A register WRITE (an address byte followed by one or more data bytes)
         * must not leave the shared register pointer moved: another task may
         * have a read in flight that already set the pointer and is about to do
         * its own START_RECV. Restore it. An address-only "set the pointer"
         * write (no data byte) leaves the pointer where it was aimed, which is
         * exactly what the set-pointer-then-read pattern needs.
         */
        if (s->wrote_data) {
            s->addr = s->addr_saved;
        }
        s->wrote_data = false;
        break;
    case I2C_NACK:
        break;
    default:
        break;
    }
    return 0;
}

/*
 * THE REGISTER-POINTER RACE, and why the fix lives at STOP.
 *
 * MEASURED (temporary I2C-pointer trace): Apple's keycode dispatcher reads the
 * hold switch (GPIOSTAT, 0x87), and its power manager reads OOCSTAT (0x12) and
 * MBCS1 (0x4b), each as two I2C transactions -- one that sets the register
 * pointer, a STOP, then a one-byte read -- and the pointer survives the STOP by
 * design. Apple's GPIO housekeeping meanwhile WRITES register 0x87. When that
 * write lands between another read's set-pointer and its data phase, it moves
 * the shared pointer, so the read comes back with the wrong register: the hold
 * read saw bit 1 clear and SELECT was dropped (about five boots in eight), and
 * the OOCSTAT read saw BATOK clear and the firmware software-reset as "battery
 * empty" (about one navtest run in six).
 *
 * On real hardware the firmware serialises PMU access so the write never
 * interleaves; our timing does not reproduce that serialisation, so the model
 * compensates: a register write restores the pointer at STOP (above), which
 * un-clobbers whatever pointer a pending read had set. Combined with GPIOSTAT
 * coming from the switch and the forced healthy-status reads, the three status
 * reads are now stable regardless of interleaving. The pointer still
 * auto-advances within a
 * transaction, so multi-byte reads and writes are unchanged.
 */
/*
 * PMUPTR=1: log every pointer set, read and write with the host wall time (so
 * it lines up with when a key was sent) and the virtual time, to see whether a
 * hold-switch read (pointer set to 0x87) comes back with some other register
 * because a second set-pointer landed in between.
 */
static void pmu_ptr_trace(const char *what, uint8_t addr, int val)
{
    static int on = -1;

    if (on < 0) {
        on = getenv("PMUPTR") != NULL;
    }
    if (on) {
        fprintf(stderr, "PMUPTR %" PRId64 " %" PRId64 " %s %02x",
                g_get_real_time(), qemu_clock_get_us(QEMU_CLOCK_VIRTUAL),
                what, addr);
        if (val >= 0) {
            fprintf(stderr, "=%02x", val);
        }
        fputc('\n', stderr);
    }
}

static uint8_t ipod_classic_pmu_recv(I2CSlave *i2c)
{
    IPodClassicPMUState *s = IPOD_CLASSIC_PMU(i2c);
    uint8_t val = pmu_reg_read(s, s->addr);

    pmu_ptr_trace("R", s->addr, val);
    s->addr++;
    return val;
}

static int ipod_classic_pmu_send(I2CSlave *i2c, uint8_t data)
{
    IPodClassicPMUState *s = IPOD_CLASSIC_PMU(i2c);

    if (s->addr_pending) {
        s->addr = data;
        s->addr_pending = false;
        pmu_ptr_trace("P", data, -1);
    } else {
        pmu_ptr_trace("W", s->addr, data);
        pmu_reg_write(s, s->addr, data);
        s->addr++;
        s->wrote_data = true;
    }
    return 0;
}

static void pmu_rtc_second_tick(void *opaque)
{
    IPodClassicPMUState *s = opaque;

    s->reg[PCF5063X_REG_INT1] |= 0x80; /* INT1_SECOND */
    pmu_update_irq(s);
    ipod_classic_wake_vcpu();
    timer_mod_ns(s->rtc_second,
                 qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 1000000000LL);
}

static void ipod_classic_pmu_reset(DeviceState *dev)
{
    IPodClassicPMUState *s = IPOD_CLASSIC_PMU(dev);

    if (!s->rtc_second) {
        s->rtc_second = timer_new_ns(QEMU_CLOCK_REALTIME,
                                     pmu_rtc_second_tick, s);
    }
    timer_mod_ns(s->rtc_second,
                 qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + 1000000000LL);

    memset(s->reg, 0, sizeof(s->reg));
    s->addr = 0;
    s->addr_pending = false;
    s->addr_saved = 0;
    s->wrote_data = false;
    pmu_start_adc(s);

    /*
     * THE HOLD SWITCH, and it is not decoration: with this bit at zero every
     * button press on the click wheel is silently thrown away.
     *
     * Two independent sources say what GPIOSTAT bit 1 means, and they agree.
     * Rockbox, firmware/target/arm/s5l8702/ipod6g/pmu-6g.c:244 --
     *
     *     pmu_input_holdswitch = !(pmu_read(PCF50635_REG_GPIOSTAT)
     *                                 & PCF50635_GPIOSTAT_GPIO2);
     *
     * -- so the bit SET means the hold switch is OFF and input is allowed.
     * Apple's retailOS, disassembled: 0x08294f80 reads one byte of register
     * 0x87 over I2C and stores (byte >> 1) & 1 ("mov r0,#0x87" / "bl I2C read"
     * / "and r0,r0,#2" / "lsr r0,r0,#1"); 0x080634a4 turns that into "1 - it"
     * ("rsbs r0,r0,#1" / "movcc r0,#0"); and the keycode dispatcher
     * 0x0803f390 calls that chain at 0x0803f3e0 -- through the tail branch
     * 0x08030d48, which is a bare "b 0x080634a4" -- and returns immediately
     * when the answer is non-zero ("cmp r0,#0" / "popne {r4,r5,r6,pc}" at
     * 0x0803f3e4). Same polarity, arrived at from the other end.
     *
     * (0x0803f3e0 is the 21st instruction of that function, not the second:
     * the entry first remaps the keycode through four compares. Nothing
     * depends on where it sits, but an earlier version of this comment said
     * "one instruction after the entry" and that was simply wrong.)
     *
     * MEASURED, before and after. With the register at 0 the wheel worked and
     * the buttons did not: holding ENTER down for three seconds and reading
     * the firmware's own state block showed 0x089116d8+0x1C = 1 (the packet
     * arrived) and +0x14 = 1 (the edge detector ran and called the dispatcher)
     * -- and the screen never changed, because the dispatcher dropped the key
     * one instruction later. The turn of the wheel does not pass through
     * 0x0803f390 at all, which is exactly why it was not affected and why the
     * failure looked like a wheel bug rather than a hold switch.
     *
     * NOT RESET HERE: s->hold is where the switch physically is, and a reset
     * does not move a switch. It starts off, because the object starts zeroed.
     * The other two GPIOSTAT bits stay zero -- Rockbox marks both "TBC" and
     * nothing we have reads them.
     *
     * Reading the pin is half of it. Neither firmware polls it for the wheel:
     * both wait for INT6, then cut the wheel's pins themselves. See
     * S5L8702_GPIO_WHEEL_GROUP in ipod_classic_gpio.h.
     */

    /*
     * Power-status registers seeded to their healthy powered values so a
     * read-back before the first poll is coherent; pmu_reg_read forces them on
     * every read anyway (see the OOCSTAT/MBCS1 cases and the header note). This
     * is what stops Apple's power manager from deciding the battery is empty and
     * software-resetting to the "--------" image a few seconds into the boot.
     */
    s->reg[PCF5063X_REG_OOCSTAT] = PCF5063X_OOCSTAT_HEALTHY;
    s->reg[PCF5063X_REG_MBCS1] = PCF5063X_MBCS1_HEALTHY;

    /*
     * The backlight as a bootloader leaves it, lit and at rest: ASSUMED, see
     * THE BACKLIGHT.  A reset is also how the machine powers on, which is when
     * the supply comes back.
     */
    if (!s->bl_ramp) {
        s->bl_ramp = timer_new_ns(QEMU_CLOCK_REALTIME, pmu_bl_tick, s);
    }
    timer_del(s->bl_ramp);
    s->reg[PCF5063X_REG_LEDCTL] = 0x05;
    s->reg[PCF5063X_REG_LEDDIM] = 0x14;
    s->reg[PCF5063X_REG_LEDOUT] = 0x20;
    s->reg[PCF5063X_REG_LEDENA] = PCF5063X_LEDENA_ON;
    s->standby = false;
    s->bl_to = s->bl_from = pmu_bl_target(s);
    s->bl_step_ns = pmu_bl_step_ns(s);
    s->bl_start_ns = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    pmu_bl_publish(s, s->bl_to);

    pmu_update_irq(s);
}

static void ipod_classic_pmu_init(Object *obj)
{
    IPodClassicPMUState *s = IPOD_CLASSIC_PMU(obj);

    qdev_init_gpio_out(DEVICE(obj), &s->nirq, 1);
#ifdef EMSCRIPTEN
    wasm_pmu_state = s;
#endif
}

/*
 * The ramp is saved as where it stood, and walks on from there after the
 * restore: its start time is REALTIME, which means nothing in another session.
 * Where it is going comes from the registers.  `standby` is not saved, like
 * the machine's own powered_off it pairs with.
 */
static int ipod_classic_pmu_pre_save(void *opaque)
{
    IPodClassicPMUState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);

    /* The same ramp, restarted from here, so the running machine is unmoved. */
    s->bl_from = pmu_bl_level_at(s, now);
    s->bl_start_ns = now;
    return 0;
}

static int ipod_classic_pmu_post_load(void *opaque, int version_id)
{
    IPodClassicPMUState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);

    s->standby = false;
    s->bl_to = pmu_bl_target(s);
    s->bl_step_ns = pmu_bl_step_ns(s);
    s->bl_start_ns = now;
    if (s->bl_from == s->bl_to || !s->bl_step_ns) {
        s->bl_from = s->bl_to;
        timer_del(s->bl_ramp);
        pmu_bl_publish(s, s->bl_to);
    } else {
        pmu_bl_tick(s);
    }
    return 0;
}

/*
 * The switch is saved with the machine: the guest has already acted on it, so
 * a restore that came back with it elsewhere would leave the wheel's pins and
 * GPIOSTAT disagreeing with no interrupt to reconcile them. nIRQ itself is not
 * state here; what it last drove is kept at the EIC end.
 */
static const VMStateDescription vmstate_ipod_classic_pmu = {
    .name = TYPE_IPOD_CLASSIC_PMU,
    .version_id = 5,
    .minimum_version_id = 5,
    .pre_save = ipod_classic_pmu_pre_save,
    .post_load = ipod_classic_pmu_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, IPodClassicPMUState),
        VMSTATE_UINT8_ARRAY(reg, IPodClassicPMUState, S5L8702_PMU_NUM_REGS),
        VMSTATE_UINT8(addr, IPodClassicPMUState),
        VMSTATE_BOOL(addr_pending, IPodClassicPMUState),
        VMSTATE_UINT8(addr_saved, IPodClassicPMUState),
        VMSTATE_BOOL(wrote_data, IPodClassicPMUState),
        VMSTATE_TIMER_PTR(rtc_second, IPodClassicPMUState),
        VMSTATE_BOOL(hold, IPodClassicPMUState),
        VMSTATE_UINT8(bl_from, IPodClassicPMUState),
        VMSTATE_END_OF_LIST()
    },
};

static void ipod_classic_pmu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->desc = "NXP PCF50635 power management unit";
    dc->reset = ipod_classic_pmu_reset;
    dc->vmsd = &vmstate_ipod_classic_pmu;
    k->event = ipod_classic_pmu_event;
    k->recv = ipod_classic_pmu_recv;
    k->send = ipod_classic_pmu_send;
}

static const TypeInfo ipod_classic_pmu_info = {
    .name          = TYPE_IPOD_CLASSIC_PMU,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(IPodClassicPMUState),
    .instance_init = ipod_classic_pmu_init,
    .class_init    = ipod_classic_pmu_class_init,
};

static void ipod_classic_pmu_register_types(void)
{
    type_register_static(&ipod_classic_pmu_info);
}

type_init(ipod_classic_pmu_register_types)
