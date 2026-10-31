/*
 * S5L8702 timer block.
 *
 * See include/hw/arm/ipod_classic/ipod_classic_timer.h for where the register
 * layout comes from. Only interval mode is modelled, which is the only mode
 * Rockbox uses; anything else is logged as unimplemented rather than silently
 * doing the wrong thing.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/arm/ipod_classic/ipod_classic_timer.h"

/* Cached: this is checked on every channel reload (Q18). */
static bool timer_trace_on(void)
{
    static int on = -1;
    if (on < 0) {
        on = getenv("TIMERTRACE") != NULL;
    }
    return on;
}

static bool chan_is_32bit(const IPodClassicTimerChannel *c)
{
    return c->index >= S5L8702_NUM_TIMERS_16;
}

/* Interrupt status bits (INT0, INT1, INTOVF) currently latched for a channel. */
static uint32_t chan_status(const IPodClassicTimerChannel *c)
{
    if (chan_is_32bit(c)) {
        uint32_t shift = S5L8702_TSTAT_SHIFT(c->index - S5L8702_NUM_TIMERS_16);
        return (c->parent->tstat >> shift) & S5L8702_TIMER_CON_INT_MASK;
    }
    return (c->con >> S5L8702_TIMER_CON_INT_STAT_SHIFT) &
           S5L8702_TIMER_CON_INT_MASK;
}

static uint32_t chan_enabled_ints(const IPodClassicTimerChannel *c)
{
    return (c->con >> S5L8702_TIMER_CON_INT_EN_SHIFT) &
           S5L8702_TIMER_CON_INT_MASK;
}

static void ipod_classic_timer_update_irq(IPodClassicTimerState *s)
{
    bool level16 = false, level32 = false;

    for (unsigned i = 0; i < S5L8702_NUM_TIMERS; i++) {
        IPodClassicTimerChannel *c = &s->chan[i];
        if (!(chan_status(c) & chan_enabled_ints(c))) {
            continue;
        }
        if (chan_is_32bit(c)) {
            level32 = true;
        } else {
            level16 = true;
        }
    }

    qemu_set_irq(s->irq16, level16);
    qemu_set_irq(s->irq32, level32);
}

/* Raise INT0 for a channel, i.e. "the counter reached DATA0". */
static void chan_set_int0(IPodClassicTimerChannel *c)
{
    if (chan_is_32bit(c)) {
        uint32_t shift = S5L8702_TSTAT_SHIFT(c->index - S5L8702_NUM_TIMERS_16);
        c->parent->tstat |= 1u << shift;
    } else {
        c->con |= 1u << S5L8702_TIMER_CON_INT_STAT_SHIFT;
    }
}

void ipod_classic_wake_vcpu(void);

static void ipod_classic_timer_tick(void *opaque)
{
    IPodClassicTimerChannel *c = opaque;

    chan_set_int0(c);
    ipod_classic_timer_update_irq(c->parent);
    /*
     * A halted vCPU may be waiting for the guest timer interrupt itself
     * (timer B's kernel tick or timer F's user timeout).  Kick it on every
     * timer interrupt, not just B, so Rockbox's UI timeout and Running Time
     * redraw run while the screen is idle.
     */
    ipod_classic_wake_vcpu();
}

static void ipod_classic_timer_host_tick(void *opaque)
{
    IPodClassicTimerChannel *c = opaque;

    if (!c->host_tick_enabled) {
        return;
    }
    c->host_tick_count += c->host_tick_step;
    ipod_classic_timer_tick(c);
    /*
     * Schedule the next tick from the previous deadline, not from "now".
     * The browser main loop can be late by a millisecond or two; scheduling
     * from now made 100 ticks take 1.09 s and Running Time lost about 7%.
     */
    c->host_tick_next_ns += c->host_tick_period_ns;
    if (c->host_tick_next_ns <= qemu_clock_get_ns(QEMU_CLOCK_REALTIME)) {
        c->host_tick_next_ns =
            qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + c->host_tick_period_ns;
    }
    timer_mod_ns(c->host_tick, c->host_tick_next_ns);
}

/*
 * Source clock for a channel, from CON[10:8] (Tx_CS) and CON[6] (ECLK select).
 *
 *   Tx_CS   CON[6]=0        CON[6]=1
 *   000     PCLK / 2        ECLK / 2
 *   001     PCLK / 4        ECLK / 4
 *   010     PCLK / 16       ECLK / 16
 *   011     PCLK / 64       ECLK / 64
 *   10x     PCLK            ECLK            (timers E..H)
 *   10x     Ext. Clock 0    Ext. Clock 0    (timers A..D)
 *   11x     Ext. Clock 1    Ext. Clock 1
 *
 * On the iPod classic Ext. Clock 0 is not connected and Ext. Clock 1 is the
 * 32768 Hz oscillator. PCLK is not modelled yet: the clock block is a plain
 * register file, so we do not know the PLL settings the guest asked for. Both
 * Rockbox timers select ECLK, so treating PCLK as ECLK is enough for now and
 * is flagged when it happens.
 */
static uint32_t chan_freq(IPodClassicTimerChannel *c)
{
    uint32_t cs = (c->con >> S5L8702_TIMER_CON_CS_SHIFT) &
                  S5L8702_TIMER_CON_CS_MASK;
    uint32_t base = S5L8702_TIMER_ECLK_HZ;
    uint32_t div;

    /*
     * Only complain about a configuration the guest is actually going to run
     * with: drivers routinely clear the timer before writing CON, so a
     * half-configured channel is normal and not worth reporting.
     */
    bool running = c->cmd & S5L8702_TIMER_CMD_EN;

    if (running && !(c->con & S5L8702_TIMER_CON_ECLK)) {
        qemu_log_mask(LOG_UNIMP,
                      "%s: timer %c selects PCLK, which is not modelled; "
                      "using ECLK (%u Hz)\n", __func__, 'A' + c->index, base);
    }

    switch (cs) {
    case 0: div = 2;  break;
    case 1: div = 4;  break;
    case 2: div = 16; break;
    case 3: div = 64; break;
    case 4:
    case 5:
        if (chan_is_32bit(c)) {
            div = 1;
        } else {
            qemu_log_mask(LOG_UNIMP,
                          "%s: timer %c selects Ext. Clock 0, which is not "
                          "connected on this board\n", __func__,
                          'A' + c->index);
            div = 1;
        }
        break;
    default:                    /* 11x: Ext. Clock 1, the 32768 Hz OSC1 */
        base = 32768;
        div = 1;
        break;
    }

    /*
     * Compute in 64 bits and never hand ptimer_set_freq() a zero: a guest
     * write of PRE = 0xffffffff (or any PRE >= base / div) used to divide
     * by zero, or set a 0 Hz period, and crash the process on the next
     * reload (Q14).
     */
    uint64_t freq = (uint64_t)base / div / ((uint64_t)c->pre + 1);

    if (freq == 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "s5l8702 timer: prescaler 0x%x gives 0 Hz, clamping\n",
                      c->pre);
        return 1;
    }
    return (uint32_t)MIN(freq, (uint64_t)UINT32_MAX);
}

/*
 * Reprogram a channel and PUT THE COUNT BACK TO THE TOP. The second half is a
 * known deviation, kept deliberately and written down here rather than fixed
 * blind:
 *
 *   ptimer_set_limit(..., 1) reloads the counter there and then. On the
 *   hardware only TB_CLR in CMD initialises it, so a write of DATA0 or of PRE
 *   that lands mid-period ought to take effect at the NEXT reload and leave
 *   the current count running. MEASURED as wrong with
 *   tools/ipod_classic/tests/timer_audit.c: rewriting TBDATA0 with the value
 *   it already held took TBCNT from 3872 to 403, and rewriting TBPRE took it
 *   from 5411 to 119.
 *
 * It is left alone because no driver we run does it in a hot path -- Rockbox
 * and the retailOS both write DATA0 and PRE while the timer is stopped, right
 * before CMD_CLR -- so fixing it would change nothing we can measure while
 * risking the one case that matters, a driver that sets DATA0 and then EN
 * without CLR and needs the new limit to be live. The ack path through CON is
 * the one that was measurably wrong, and that one IS fixed; see the CON case
 * in ipod_classic_timer_write().
 */
#define S5L8702_TIMER_E 4

static uint32_t timer_e_count(IPodClassicTimerChannel *c)
{
    int64_t elapsed;

    if (!c->host_anchor) {
        return c->host_count;
    }
    elapsed = qemu_clock_get_ns(QEMU_CLOCK_REALTIME) - c->host_start_ns;
    if (elapsed <= 0) {
        return c->host_count;
    }
    return c->host_count +
           (uint32_t)((uint64_t)elapsed * chan_freq(c) / 1000000000ULL);
}

static void chan_reload(IPodClassicTimerChannel *c)
{
    uint64_t limit = c->data0;
    uint32_t mode = (c->con >> S5L8702_TIMER_CON_MODE_SHIFT) & 0x3;

    if (mode != 0) {
        qemu_log_mask(LOG_UNIMP, "%s: timer %c mode %u is not modelled, "
                      "treating it as interval mode\n", __func__,
                      'A' + c->index, mode);
    }

    /*
     * TIMERTRACE=1: what rate does each channel actually end up at?
     *
     * Needed because the audio task is dispatched with 17-21 SECOND gaps where
     * it should run ~40 times a second (wiki/19 s11.50), and the natural
     * suspect is a kernel tick that runs far too slowly. PCLK is not modelled
     * -- chan_freq substitutes ECLK for it -- so a channel the guest programmed
     * against PCLK comes out at the wrong rate, and this says by how much.
     */
    if (timer_trace_on()) {
        uint32_t f = chan_freq(c);
        fprintf(stderr, "TIMERTRACE: chan %c  freq=%u Hz  data0=%u  pre=%u  "
                        "con=%08x cmd=%08x  -> period %.3f ms\n",
                'A' + c->index, f, (unsigned)c->data0, (unsigned)c->pre,
                (unsigned)c->con, (unsigned)c->cmd,
                f ? (double)limit * 1000.0 / (double)f : 0.0);
    }

    /* A zero period would make the ptimer fire continuously. */
    if (limit == 0) {
        limit = chan_is_32bit(c) ? 0x100000000ULL : 0x10000;
    }

    bool host_b = c->index == 1 && chan_freq(c) == 10000 &&
                  limit == 100;

    ptimer_transaction_begin(c->ptimer);
    ptimer_set_freq(c->ptimer, chan_freq(c));
    ptimer_set_limit(c->ptimer, limit, 1);
    if (host_b) {
        /* Rockbox timer B: the host-tick QEMUTimer below drives it. */
        ptimer_stop(c->ptimer);
    } else if (c->cmd & S5L8702_TIMER_CMD_EN) {
        ptimer_run(c->ptimer, 0);       /* 0: reload and keep going */
    } else {
        ptimer_stop(c->ptimer);
    }
    ptimer_transaction_commit(c->ptimer);

    if (c->index == 1) {
        uint32_t freq = chan_freq(c);
        if (!host_b) {
            c->host_tick_enabled = false;
            timer_del(c->host_tick);
        } else {
            c->host_tick_step = (uint32_t)limit;
            c->host_tick_period_ns =
                freq ? (uint64_t)limit * 1000000000ULL / freq : 1000000ULL;
            if (c->host_tick_period_ns == 0) {
                c->host_tick_period_ns = 1000000ULL;
            }
            if (c->cmd & S5L8702_TIMER_CMD_EN) {
                c->host_tick_count = 0;
                c->host_tick_enabled = true;
                c->host_tick_next_ns =
                    qemu_clock_get_ns(QEMU_CLOCK_REALTIME) +
                    c->host_tick_period_ns;
                timer_mod_ns(c->host_tick, c->host_tick_next_ns);
            } else {
                c->host_tick_enabled = false;
                timer_del(c->host_tick);
            }
        }
    }

    if (c->index == S5L8702_TIMER_E) {
        if (c->cmd & S5L8702_TIMER_CMD_EN) {
            c->host_start_ns = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
            c->host_count = 0;
            c->host_anchor = true;
        } else {
            c->host_count = timer_e_count(c);
            c->host_anchor = false;
        }
    }
}

/* Map an MMIO offset to a channel and a register, or return false. */
static bool decode_offset(hwaddr offset, unsigned *chan, hwaddr *reg)
{
    if (offset < S5L8702_TIMER_AD_BASE +
                 S5L8702_NUM_TIMERS_16 * S5L8702_TIMER_STRIDE) {
        *chan = (offset - S5L8702_TIMER_AD_BASE) / S5L8702_TIMER_STRIDE;
        *reg = offset % S5L8702_TIMER_STRIDE;
        return true;
    }
    if (offset >= S5L8702_TIMER_EH_BASE &&
        offset < S5L8702_TIMER_EH_BASE +
                 S5L8702_NUM_TIMERS_16 * S5L8702_TIMER_STRIDE) {
        *chan = S5L8702_NUM_TIMERS_16 +
                (offset - S5L8702_TIMER_EH_BASE) / S5L8702_TIMER_STRIDE;
        *reg = (offset - S5L8702_TIMER_EH_BASE) % S5L8702_TIMER_STRIDE;
        return true;
    }
    return false;
}

static uint64_t ipod_classic_timer_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    IPodClassicTimerState *s = opaque;
    unsigned ch;
    hwaddr reg;

    if (offset == S5L8702_TIMER_TSTAT) {
        return s->tstat;
    }

    if (!decode_offset(offset, &ch, &reg)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        return 0;
    }

    IPodClassicTimerChannel *c = &s->chan[ch];

    switch (reg) {
    case S5L8702_TIMER_CON:
        return c->con;
    case S5L8702_TIMER_CMD:
        return c->cmd;
    case S5L8702_TIMER_DATA0:
        return c->data0;
    case S5L8702_TIMER_DATA1:
        return c->data1;
    case S5L8702_TIMER_PRE:
        return c->pre;
    case S5L8702_TIMER_CNT:
        if (c->index == 1) {
            return c->host_tick_count;
        }
        if (c->index == S5L8702_TIMER_E) {
            return timer_e_count(c);
        }
        return c->data0 - ptimer_get_count(c->ptimer);
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad register 0x%" HWADDR_PRIx
                      " of timer %c\n", __func__, reg, 'A' + ch);
        return 0;
    }
}

static void ipod_classic_timer_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    IPodClassicTimerState *s = opaque;
    unsigned ch;
    hwaddr reg;

    if (offset == S5L8702_TIMER_TSTAT) {
        /* Write-one-to-clear, which is how Rockbox acknowledges timer F. */
        s->tstat &= ~(uint32_t)value;
        ipod_classic_timer_update_irq(s);
        return;
    }

    if (!decode_offset(offset, &ch, &reg)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad offset 0x%" HWADDR_PRIx
                      " (value 0x%" PRIx64 ")\n", __func__, offset, value);
        return;
    }

    IPodClassicTimerChannel *c = &s->chan[ch];

    switch (reg) {
    case S5L8702_TIMER_CON: {
        /*
         * On the 16-bit timers the status bits live in this register and are
         * cleared by writing them back, which is what "TBCON = TBCON" in
         * Rockbox's tick handler does. Keep any status bit the guest did not
         * write a one to.
         */
        uint32_t status_mask = S5L8702_TIMER_CON_INT_MASK <<
                               S5L8702_TIMER_CON_INT_STAT_SHIFT;
        uint32_t kept = c->con & status_mask & ~(uint32_t)value;
        uint32_t old_cfg = c->con & ~status_mask;
        uint32_t new_cfg = (uint32_t)value & ~status_mask;

        c->con = new_cfg | (chan_is_32bit(c) ? 0 : kept);
        /*
         * AN ACK IS NOT A RESTART, and getting this wrong cost 14% of the
         * clock. chan_reload() ends in ptimer_set_limit(..., reload = 1),
         * which puts the count back to the top immediately; on the hardware
         * only TB_CLR in CMD initialises the counter. So an interrupt handler
         * that acks with "TBCON = TBCON" -- which is exactly what Rockbox's
         * INT_TIMERB does (kernel-s5l8702.c:31), and the idiom the status bits
         * living in CON force on everyone -- used to restart the period from
         * wherever the handler happened to run, and every period came out one
         * interrupt latency too long.
         *
         * MEASURED with tools/ipod_classic/tests/timer_audit.c, timer B at
         * 1 kHz (ECLK/2, DATA0 = 6000), counted against USEC_TIMER over two
         * seconds:
         *
         *     acking through CON     1713 interrupts   expected 2000
         *     acking through TSTAT   1997 interrupts   expected 2000
         *
         * The second line is the control: timer F, same period, same VIC, same
         * handler, differing only in that its status lives in TSTAT so its ack
         * never touches CON. It kept time; timer B did not. That pins it on
         * this write and nothing else.
         *
         * So reload only when the configuration -- clock source, mode,
         * interrupt enables -- actually changed. A write that only clears
         * status bits now leaves the count alone, which is what the hardware
         * does. Writes of DATA0 and of PRE still reload; see the note in
         * chan_reload().
         *
         * The retailOS does not notice either way: the only timer it uses is
         * E, free-running at 1 MHz, set up at 0x08324e48 and read at
         * 0x08324e38, and E is 32-bit so its status never lived in CON. The
         * three navtest.sh runs across this change give the same pixel counts.
         * Who collects is Rockbox and the bare-metal tests. See wiki/11 s.19.2.
         */
        if (new_cfg != old_cfg) {
            chan_reload(c);
        }
        ipod_classic_timer_update_irq(s);
        break;
    }
    case S5L8702_TIMER_CMD:
        c->cmd = value;
        if (value & S5L8702_TIMER_CMD_CLR) {
            /* Initialise: reset the counter and latch DATA0/DATA1. */
            chan_reload(c);
        }
        bool host_b = c->index == 1 && chan_freq(c) == 10000 &&
                      (c->data0 ? c->data0 : 0x10000) == 100;
        if (host_b) {
            /*
             * Rockbox's timer B is driven only by the host tick.  Running the
             * virtual ptimer here as well was the 3.5x Running Time bug: the
             * guest's TBCMD enable write started the virtual ptimer in
             * addition to the host QEMUTimer, and the icount warp made it
             * fire far more often than 100 Hz.
             */
            if (value & S5L8702_TIMER_CMD_EN) {
                uint32_t freq = chan_freq(c);
                uint32_t limit = c->data0 ? c->data0 : 0x10000;
                c->host_tick_step = limit;
                c->host_tick_period_ns =
                    freq ? (uint64_t)limit * 1000000000ULL / freq
                         : 1000000ULL;
                if (c->host_tick_period_ns == 0) {
                    c->host_tick_period_ns = 1000000ULL;
                }
                c->host_tick_count = 0;
                c->host_tick_enabled = true;
                c->host_tick_next_ns =
                    qemu_clock_get_ns(QEMU_CLOCK_REALTIME) +
                    c->host_tick_period_ns;
                timer_mod_ns(c->host_tick, c->host_tick_next_ns);
            } else {
                c->host_tick_enabled = false;
                timer_del(c->host_tick);
            }
        }
        ptimer_transaction_begin(c->ptimer);
        if (!host_b && (value & S5L8702_TIMER_CMD_EN)) {
            ptimer_run(c->ptimer, 0);
        } else {
            ptimer_stop(c->ptimer);
        }
        ptimer_transaction_commit(c->ptimer);
        break;
    case S5L8702_TIMER_DATA0:
        c->data0 = value;
        chan_reload(c);
        break;
    case S5L8702_TIMER_DATA1:
        c->data1 = value;
        break;
    case S5L8702_TIMER_PRE:
        c->pre = value;
        chan_reload(c);
        break;
    case S5L8702_TIMER_CNT:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: CNT of timer %c is read-only\n",
                      __func__, 'A' + ch);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad register 0x%" HWADDR_PRIx
                      " of timer %c\n", __func__, reg, 'A' + ch);
        break;
    }
}

static const MemoryRegionOps ipod_classic_timer_ops = {
    .read = ipod_classic_timer_read,
    .write = ipod_classic_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void ipod_classic_timer_reset(DeviceState *dev)
{
    IPodClassicTimerState *s = IPOD_CLASSIC_TIMER(dev);

    s->tstat = 0;
    for (unsigned i = 0; i < S5L8702_NUM_TIMERS; i++) {
        IPodClassicTimerChannel *c = &s->chan[i];

        c->con = c->cmd = c->data0 = c->data1 = c->pre = 0;
        c->host_start_ns = 0;
        c->host_count = 0;
        c->host_anchor = false;
        c->host_tick_count = 0;
        c->host_tick_next_ns = 0;
        c->host_tick_enabled = false;
        if (c->host_tick) {
            timer_del(c->host_tick);
        }
        ptimer_transaction_begin(c->ptimer);
        ptimer_stop(c->ptimer);
        ptimer_transaction_commit(c->ptimer);
    }
    ipod_classic_timer_update_irq(s);
}

static void ipod_classic_timer_init(Object *obj)
{
    IPodClassicTimerState *s = IPOD_CLASSIC_TIMER(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ipod_classic_timer_ops, s,
                          TYPE_IPOD_CLASSIC_TIMER, S5L8702_TIMER_MEM_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    sysbus_init_irq(sbd, &s->irq16);
    sysbus_init_irq(sbd, &s->irq32);

    for (unsigned i = 0; i < S5L8702_NUM_TIMERS; i++) {
        s->chan[i].parent = s;
        s->chan[i].index = i;
        s->chan[i].ptimer =
            ptimer_init(ipod_classic_timer_tick, &s->chan[i],
                        PTIMER_POLICY_LEGACY);
        if (i == 1) {
            s->chan[i].host_tick =
                timer_new_ns(QEMU_CLOCK_REALTIME,
                             ipod_classic_timer_host_tick, &s->chan[i]);
        }
    }
}

static const VMStateDescription vmstate_ipod_classic_timer_channel = {
    .name = "ipod_classic_timer_channel",
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (VMStateField[]) {
        VMSTATE_PTIMER(ptimer, IPodClassicTimerChannel),
        VMSTATE_UINT32(con, IPodClassicTimerChannel),
        VMSTATE_UINT32(cmd, IPodClassicTimerChannel),
        VMSTATE_UINT32(data0, IPodClassicTimerChannel),
        VMSTATE_UINT32(data1, IPodClassicTimerChannel),
        VMSTATE_UINT32(pre, IPodClassicTimerChannel),
        VMSTATE_INT64(host_start_ns, IPodClassicTimerChannel),
        VMSTATE_UINT32(host_count, IPodClassicTimerChannel),
        VMSTATE_BOOL(host_anchor, IPodClassicTimerChannel),
        VMSTATE_UINT64(host_tick_period_ns, IPodClassicTimerChannel),
        VMSTATE_UINT32(host_tick_count, IPodClassicTimerChannel),
        VMSTATE_UINT32(host_tick_step, IPodClassicTimerChannel),
        VMSTATE_BOOL(host_tick_enabled, IPodClassicTimerChannel),
        VMSTATE_INT64(host_tick_next_ns, IPodClassicTimerChannel),
        VMSTATE_TIMER_PTR(host_tick, IPodClassicTimerChannel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ipod_classic_timer = {
    .name = TYPE_IPOD_CLASSIC_TIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(chan, IPodClassicTimerState, S5L8702_NUM_TIMERS,
                             1, vmstate_ipod_classic_timer_channel,
                             IPodClassicTimerChannel),
        VMSTATE_UINT32(tstat, IPodClassicTimerState),
        VMSTATE_END_OF_LIST()
    }
};

static void ipod_classic_timer_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = ipod_classic_timer_reset;
    dc->vmsd = &vmstate_ipod_classic_timer;
    dc->desc = "S5L8702 timers";
}

static const TypeInfo ipod_classic_timer_info = {
    .name          = TYPE_IPOD_CLASSIC_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodClassicTimerState),
    .instance_init = ipod_classic_timer_init,
    .class_init    = ipod_classic_timer_class_init,
};

static void ipod_classic_timer_register_types(void)
{
    type_register_static(&ipod_classic_timer_info);
}

type_init(ipod_classic_timer_register_types)
