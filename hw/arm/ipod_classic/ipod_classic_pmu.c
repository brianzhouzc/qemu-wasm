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
 *
 * WHAT IS NOT
 *
 *   - The RTC. It reads back whatever was written, which is wrong, and is
 *     flagged here because on the 5G an absent RTC had a visible side effect:
 *     Rockbox decided an alarm had gone off on every boot.
 *   - Charging, the ON/OFF controller and the regulators. Writing them changes
 *     nothing, because there is no voltage in here for a voltage to mean.
 *   - The chip's interrupt output. Nothing is wired to it yet.
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
    const char *e = getenv("PMUBATT");
    if (e && *e) {
        unsigned long v = strtoul(e, NULL, 0);
        if (v > 1023) {
            v = 1023;
        }
        return (uint32_t)v;
    }
    return S5L8702_PMU_ADC_BATTERY_RAW;
}

static void pmu_start_adc(IPodClassicPMUState *s)
{
    uint32_t raw = pmu_battery_raw();

    s->reg[PCF5063X_REG_ADCS1] = (raw >> 2) & 0xff;
    s->reg[PCF5063X_REG_ADCS2] = 0;
    s->reg[PCF5063X_REG_ADCS3] = PCF5063X_ADCS3_ADCRDY | (raw & 0x03);
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
        break;
    case PCF50635_REG_GPIOSTAT:
        /*
         * The hold switch, reported OPEN unconditionally. See the long note at
         * the reset function for why "open" is the only usable value on a
         * machine with no switch to flip -- and why it MUST NOT depend on
         * whatever a write has left in s->reg[0x87]. Apple's own GPIO code
         * writes this register (measured: a byte 0x87 goes to reg 0x87), so the
         * stored value is not reliably HOLD_OFF; forcing the read makes the
         * keycode dispatcher's gate see "open" no matter what was written.
         */
        val = PCF50635_GPIOSTAT_HOLD_OFF;
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
            const char *e = getenv("PMUCHARGER");
            if (e && *e) {
                /*
                 * WHICH KIND of external power matters, and getting it wrong
                 * was MEASURED: reporting the wall ADAPTER puts the device
                 * into its full-screen "Charging" mode and it stops playing --
                 * which is what a real iPod does on a charger, so the model was
                 * right and the choice was wrong. USB power is the one that
                 * leaves it usable. Default is USB; PMUCHARGER=adapter keeps
                 * the other one reachable for testing that screen on purpose.
                 */
                if (!strcmp(e, "adapter")) {
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
         * qemu_get_timedate is the same source hw/rtc/pl031.c uses, so -rtc on
         * the command line works here as it does everywhere else in QEMU.
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
    if (getenv("PMUTRACE")) {
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
    case PCF5063X_REG_OOCSHDWN:
        /*
         * The ON/OFF controller's shutdown.  Rockbox writes 0x01 here at the
         * end of power_off(), after painting "Shutting down...", and then
         * waits for the supply to drop -- which never happens in a model with
         * no supply.  Tell the machine instead, so a later key press can power
         * it back on with a reset (see ipod_classic_power_off()).
         */
        s->reg[addr] = val;
        if (val) {
            ipod_classic_power_off();
        }
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
 * un-clobbers whatever pointer a pending read had set. Combined with the forced
 * HOLD_OFF / healthy-status reads, the three status reads are now stable
 * regardless of interleaving. The pointer still auto-advances within a
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
     * CONVENIENCE VALUE, and marked as one: a machine with no hold switch to
     * flip has to boot with it off or it cannot be used. The other two bits
     * stay zero -- Rockbox marks both "TBC" and nothing we have reads them.
     */
    s->reg[PCF50635_REG_GPIOSTAT] = PCF50635_GPIOSTAT_HOLD_OFF;

    /*
     * Power-status registers seeded to their healthy powered values so a
     * read-back before the first poll is coherent; pmu_reg_read forces them on
     * every read anyway (see the OOCSTAT/MBCS1 cases and the header note). This
     * is what stops Apple's power manager from deciding the battery is empty and
     * software-resetting to the "--------" image a few seconds into the boot.
     */
    s->reg[PCF5063X_REG_OOCSTAT] = PCF5063X_OOCSTAT_HEALTHY;
    s->reg[PCF5063X_REG_MBCS1] = PCF5063X_MBCS1_HEALTHY;
}

static const VMStateDescription vmstate_ipod_classic_pmu = {
    .name = TYPE_IPOD_CLASSIC_PMU,
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, IPodClassicPMUState),
        VMSTATE_UINT8_ARRAY(reg, IPodClassicPMUState, S5L8702_PMU_NUM_REGS),
        VMSTATE_UINT8(addr, IPodClassicPMUState),
        VMSTATE_BOOL(addr_pending, IPodClassicPMUState),
        VMSTATE_UINT8(addr_saved, IPodClassicPMUState),
        VMSTATE_BOOL(wrote_data, IPodClassicPMUState),
        VMSTATE_TIMER_PTR(rtc_second, IPodClassicPMUState),
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
    .class_init    = ipod_classic_pmu_class_init,
};

static void ipod_classic_pmu_register_types(void)
{
    type_register_static(&ipod_classic_pmu_info);
}

type_init(ipod_classic_pmu_register_types)
