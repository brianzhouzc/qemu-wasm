/*
 * Arm PrimeCell PL080/PL081 DMA controller
 *
 * Copyright (c) 2006 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/dma/pl080.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"

#define PL080_CONF_E    0x1
#define PL080_CONF_M1   0x2
#define PL080_CONF_M2   0x4

#define PL080_CCONF_H   0x40000
#define PL080_CCONF_A   0x20000
#define PL080_CCONF_L   0x10000
#define PL080_CCONF_ITC 0x08000
#define PL080_CCONF_IE  0x04000
#define PL080_CCONF_E   0x00001

#define PL080_CCTRL_I   0x80000000
#define PL080_CCTRL_DI  0x08000000
#define PL080_CCTRL_SI  0x04000000
#define PL080_CCTRL_D   0x02000000
#define PL080_CCTRL_S   0x01000000

static const VMStateDescription vmstate_pl080_channel = {
    .name = "pl080_channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(src, pl080_channel),
        VMSTATE_UINT32(dest, pl080_channel),
        VMSTATE_UINT32(lli, pl080_channel),
        VMSTATE_UINT32(ctrl, pl080_channel),
        VMSTATE_UINT32(conf, pl080_channel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_pl080 = {
    .name = "pl080",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(tc_int, PL080State),
        VMSTATE_UINT8(tc_mask, PL080State),
        VMSTATE_UINT8(err_int, PL080State),
        VMSTATE_UINT8(err_mask, PL080State),
        VMSTATE_UINT32(conf, PL080State),
        VMSTATE_UINT32(sync, PL080State),
        VMSTATE_UINT32(req_single, PL080State),
        VMSTATE_UINT32(req_burst, PL080State),
        /*
         * These duplicates are upstream's wire format for every PL080 board;
         * removing them would change it.  Keep all three (S4).
         */
        VMSTATE_UINT8(tc_int, PL080State),
        VMSTATE_UINT8(tc_int, PL080State),
        VMSTATE_UINT8(tc_int, PL080State),
        VMSTATE_STRUCT_ARRAY(chan, PL080State, PL080_MAX_CHANNELS,
                             1, vmstate_pl080_channel, pl080_channel),
        VMSTATE_INT32(running, PL080State),
        /* A paced channel saved with a completion pending (Q20, version 2). */
        VMSTATE_UINT32_V(paced_pending, PL080State, 2),
        VMSTATE_END_OF_LIST()
    }
};

static const unsigned char pl080_id[] =
{ 0x80, 0x10, 0x04, 0x0a, 0x0d, 0xf0, 0x05, 0xb1 };

static const unsigned char pl081_id[] =
{ 0x81, 0x10, 0x04, 0x0a, 0x0d, 0xf0, 0x05, 0xb1 };

/*
 * DMATRACE=1: report what the guest programs into each DMA channel, and how
 * many terminal-count interrupts each channel raises.
 *
 * This exists to settle one question for the iPod classic 6G audio work. The
 * DMA interrupt handler was measured running 7 185 times, which was taken as
 * evidence that the DMA was healthy -- but a global handler count does not say
 * WHICH channel those interrupts came from. If they are all display or ATA
 * traffic and nothing ever points a channel at the I2S TX FIFO (request line
 * 0xA), then the audio DMA never runs at all and the conclusion was wrong.
 * Observational only, and silent unless the variable is set.
 */
static int dmatrace = -1;

static bool pl080_trace_on(void)
{
    if (dmatrace < 0) {
        const char *e = getenv("DMATRACE");
        dmatrace = (e && *e && *e != '0') ? 1 : 0;
    }
    return dmatrace == 1;
}

/*
 * Both DMACs share this model, so the trace has to say which one it is. The
 * machine creates DMAC0 before DMAC1, so labelling by first-seen instance is
 * enough here and costs no plumbing through the device tree.
 */
static const void *pl080_trace_first;

static const char *pl080_trace_tag(PL080State *s)
{
    if (!pl080_trace_first) {
        pl080_trace_first = s;
    }
    return (pl080_trace_first == s) ? "dmac0" : "dmac1";
}

static void pl080_update(PL080State *s)
{
    bool tclevel = (s->tc_int & s->tc_mask);
    bool errlevel = (s->err_int & s->err_mask);

    qemu_set_irq(s->interr, errlevel);
    qemu_set_irq(s->inttc, tclevel);
    qemu_set_irq(s->irq, errlevel || tclevel);
}

/*
 * Load a channel's working registers from the linked-list descriptor at `lli`
 * (src @ +0, dest @ +4, next-link @ +8, control @ +12). This is exactly the
 * fetch the controller does at an LLI boundary; factored out so the terminal
 * relink path (pl080_write) and the mid-run boundary (pl080_run) stay identical.
 */
static void pl080_ch_load_lli(PL080State *s, pl080_channel *ch, uint32_t lli)
{
    ch->src  = address_space_ldl_le(&s->downstream_as, lli,
                                    MEMTXATTRS_UNSPECIFIED, NULL);
    ch->dest = address_space_ldl_le(&s->downstream_as, lli + 4,
                                    MEMTXATTRS_UNSPECIFIED, NULL);
    ch->ctrl = address_space_ldl_le(&s->downstream_as, lli + 12,
                                    MEMTXATTRS_UNSPECIFIED, NULL);
    ch->lli  = address_space_ldl_le(&s->downstream_as, lli + 8,
                                    MEMTXATTRS_UNSPECIFIED, NULL);
}

static void pl080_run(PL080State *s)
{
    int c;
    int flow;
    pl080_channel *ch;
    int swidth;
    int dwidth;
    int xsize;
    int n;
    int src_id;
    int dest_id;
    int size;
    int moved;
    int paced_id;
    bool paced;
    uint8_t buff[4];
    uint32_t req;

    s->tc_mask = 0;
    for (c = 0; c < s->nchannels; c++) {
        if (s->chan[c].conf & PL080_CCONF_ITC)
            s->tc_mask |= 1 << c;
        if (s->chan[c].conf & PL080_CCONF_IE)
            s->err_mask |= 1 << c;
    }

    if ((s->conf & PL080_CONF_E) == 0)
        return;

    /* If we are already in the middle of a DMA operation then indicate that
       there may be new DMA requests and return immediately.  */
    if (s->running) {
        s->running++;
        return;
    }
    s->running = 1;
    while (s->running) {
        for (c = 0; c < s->nchannels; c++) {
            ch = &s->chan[c];
again:
            /* Test if thiws channel has any pending DMA requests.  */
            if ((ch->conf & (PL080_CCONF_H | PL080_CCONF_E))
                    != PL080_CCONF_E)
                continue;
            /*
             * Resume a completed-pending paced channel that the guest has since
             * relinked: it wrote the next task's descriptor pointer to the LINK
             * register while we were holding the enable bit high (see the
             * terminal handling below). Load that descriptor now -- this is the
             * point at which real hardware consumes the LINK register -- and drop
             * the pending flag so the channel runs the new task normally.
             */
            if ((s->paced_pending & (1u << c)) && ch->lli != 0) {
                pl080_ch_load_lli(s, ch, ch->lli);
                s->paced_pending &= ~(1u << c);
            }
            flow = (ch->conf >> 11) & 7;
            if (flow >= 4) {
                static bool warned;
                /*
                 * Guest data reached an unimplemented mode.  Do not abort
                 * (Q22), and do not return either: that leaves s->running
                 * set and every later pl080_run() just increments it (S3).
                 * Skip this channel and say so once.
                 */
                if (!warned) {
                    warned = true;
                    qemu_log_mask(LOG_GUEST_ERROR,
                                  "pl080_run: peripheral flow control %u not "
                                  "implemented, skipping the channel\n", flow);
                }
                continue;
            }
            src_id = (ch->conf >> 1) & 0x1f;
            dest_id = (ch->conf >> 6) & 0x1f;
            size = ch->ctrl & 0xfff;
            req = s->req_single | s->req_burst;
            /*
             * The stock model transferred mem -> peripheral unconditionally
             * (the request check was commented out), which drains the whole
             * buffer at once. A line a peripheral has marked paced moves only
             * what pl080_pace_request() allows; every other line keeps the old
             * full-speed behaviour, so this cannot change any DMA that worked
             * before. See paced_lines in the header.
             */
            paced_id = flow == 1 ? dest_id : flow == 2 ? src_id : -1;
            paced = paced_id >= 0 && (s->paced_lines & (1u << paced_id));
            if (paced && (paced_id != s->paced_line
                          || s->paced_budget < (1u << ((ch->ctrl >> 18) & 7)))) {
                s->paced_stalled |= size != 0;
                continue;
            }
            /*
             * One interrupt per terminal count.  On the hardware the next one
             * is a whole task away, and Rockbox counts a task for each
             * interrupt.  Here a tick pays for 15 ms at once and its first
             * tasks are 2.9 ms: MEASURED, two fell inside one tick, their
             * interrupts merged, and the driver took the callback of one for
             * the other and stopped queueing.  So the channel waits for the
             * guest to take each one, and the clear kicks the peripheral.
             */
            if (paced && (s->tc_int & (1u << c))) {
                s->paced_held |= 1u << c;
                continue;
            }
            switch (flow) {
            case 0:
            case 1:
            case 2:
                break;
            case 3:
                if ((req & (1u << src_id)) == 0
                        || (req & (1u << dest_id)) == 0)
                    size = 0;
                break;
            }
            if (!size) {
                continue;
            }

            /* Transfer one element.  */
            /* ??? Should transfer multiple elements for a burst request.  */
            /* ??? Unclear what the proper behavior is when source and
               destination widths are different.  */
            swidth = 1 << ((ch->ctrl >> 18) & 7);
            dwidth = 1 << ((ch->ctrl >> 21) & 7);
            moved = 1;
            if (paced && s->paced_sink && swidth == dwidth
                    && (ch->ctrl & PL080_CCTRL_SI)
                    && !(ch->ctrl & PL080_CCTRL_DI)) {
                /*
                 * 88,200 samples a second, each a read and a device write with
                 * the BQL held against the vCPU: hand them over in one go.
                 */
                uint8_t block[4096];

                moved = MIN(size, s->paced_budget / swidth);
                moved = MIN(moved, sizeof(block) / swidth);
                address_space_read(&s->downstream_as, ch->src,
                                   MEMTXATTRS_UNSPECIFIED, block,
                                   moved * swidth);
                ch->src += moved * swidth;
                s->paced_sink(s->paced_opaque, block, moved * swidth);
            } else {
                for (n = 0; n < dwidth; n+= swidth) {
                    address_space_read(&s->downstream_as, ch->src,
                                       MEMTXATTRS_UNSPECIFIED, buff + n, swidth);
                    if (ch->ctrl & PL080_CCTRL_SI)
                        ch->src += swidth;
                }
                xsize = (dwidth < swidth) ? swidth : dwidth;
                /* ??? This may pad the value incorrectly for dwidth < 32.  */
                for (n = 0; n < xsize; n += dwidth) {
                    address_space_write(&s->downstream_as, ch->dest + n,
                                        MEMTXATTRS_UNSPECIFIED, buff + n,
                                        dwidth);
                    if (ch->ctrl & PL080_CCTRL_DI)
                        ch->dest += swidth;
                }
            }
            if (paced) {
                s->paced_budget -= moved * swidth;
            }

            size -= moved;
            ch->ctrl = (ch->ctrl & 0xfffff000) | size;
            if (size == 0) {
                /*
                 * The interrupt is the finished descriptor's to raise.  The
                 * test below reads the next one's, which is how this model has
                 * always behaved, and it stays so for every line that worked
                 * that way; on a paced line the interrupt is how the guest
                 * learns what has really been played.
                 */
                uint32_t done_ctrl = ch->ctrl;

                /* Transfer complete.  */
                if (ch->lli) {
                    pl080_ch_load_lli(s, ch, ch->lli);
                } else if (paced) {
                    /*
                     * Terminal LLI (no link) on a PACED channel. Do NOT clear
                     * the enable bit here. Rockbox's audio driver keeps a single
                     * continuous DMA alive by extending the LLI chain from its
                     * terminal-count IRQ handler (dmac_ch_queue_2d, QUEUE_LINK):
                     * when it sees DMACCxLINK==0 it writes the next task's link
                     * and then re-reads DMACCxCONFIG, and it panics
                     * "DMA ch%d: link error" if the enable bit is already clear.
                     * Our paced drain runs from a host QEMUTimer that the guest's
                     * IRQ masking cannot hold off, so auto-disabling the channel
                     * the instant the terminal element drains races that check
                     * and makes playback panic under some timings (graphical
                     * window, loaded host). Instead we keep the channel enabled
                     * and idle (completed-pending); it resumes when the guest
                     * writes a new LINK (pl080_write case 2) and is disabled only
                     * when the guest explicitly aborts (writes CONFIG.E=0). This
                     * removes the race for ANY relative timing: a late relink now
                     * costs at most a brief underrun, never a panic. Non-paced
                     * channels keep the exact stock auto-disable below.
                     */
                    s->paced_pending |= (1u << c);
                } else {
                    ch->conf &= ~PL080_CCONF_E;
                }
                if ((paced ? done_ctrl : ch->ctrl) & PL080_CCTRL_I) {
                    s->tc_int |= 1 << c;
                    if (pl080_trace_on()) {
                        s->tc_count[c]++;
                        if (s->tc_count[c] <= 3
                                || s->tc_count[c] % 500 == 0) {
                            fprintf(stderr,
                                    "DMATRACE %s ch%d TC #%u srcid=%d dstid=%d "
                                    "srcend=0x%08x dst=0x%08x lli=0x%08x\n",
                                    pl080_trace_tag(s), c, s->tc_count[c],
                                    src_id, dest_id, ch->src, ch->dest,
                                    ch->lli);
                        }
                    }
                }
            }
            goto again;
        }
        if (--s->running)
            s->running = 1;
    }
    pl080_update(s);
    /*
     * The peripheral stops asking when it has nothing to do, so a channel that
     * has just been given work has to say so.
     */
    if (s->paced_stalled && !s->pacing) {
        s->paced_stalled = false;
        if (s->paced_kick) {
            s->paced_kick(s->paced_opaque);
        }
    }
}

void pl080_set_paced_line(PL080State *s, int line, PL080PacedSink *sink,
                          PL080PacedKick *kick, void *opaque)
{
    s->paced_lines |= (1u << line);
    s->paced_sink = sink;
    s->paced_kick = kick;
    s->paced_opaque = opaque;
}

uint32_t pl080_pace_request(PL080State *s, int line, uint32_t bytes)
{
    uint32_t left;

    s->paced_line = line;
    s->paced_budget = bytes;
    s->pacing = true;
    pl080_run(s);
    s->pacing = false;
    left = s->paced_budget;
    /*
     * Nothing outlives the call.  A budget left over would drain a whole task
     * the instant the guest enables the channel (measured: Rockbox's dmac_ch_run
     * for the first audio task, followed at once by a terminal-count that clears
     * the channel E-bit, so the guest's very next dmac_ch_queue_2d link check
     * sees E=0 and panics "DMA ch link error").
     */
    s->paced_budget = 0;
    s->paced_stalled = false;
    return bytes - left;
}

static uint64_t pl080_read_inner(void *opaque, hwaddr offset,
                           unsigned size)
{
    PL080State *s = (PL080State *)opaque;
    uint32_t i;
    uint32_t mask;

    if (offset >= 0xfe0 && offset < 0x1000) {
        if (s->nchannels == 8) {
            return pl080_id[(offset - 0xfe0) >> 2];
        } else {
            return pl081_id[(offset - 0xfe0) >> 2];
        }
    }
    if (offset >= 0x100 && offset < 0x200) {
        i = (offset & 0xe0) >> 5;
        if (i >= s->nchannels)
            goto bad_offset;
        switch ((offset >> 2) & 7) {
        case 0: /* SrcAddr */
            return s->chan[i].src;
        case 1: /* DestAddr */
            return s->chan[i].dest;
        case 2: /* LLI */
            return s->chan[i].lli;
        case 3: /* Control */
            return s->chan[i].ctrl;
        case 4: /* Configuration */
            return s->chan[i].conf;
        default:
            goto bad_offset;
        }
    }
    switch (offset >> 2) {
    case 0: /* IntStatus */
        return (s->tc_int & s->tc_mask) | (s->err_int & s->err_mask);
    case 1: /* IntTCStatus */
        return (s->tc_int & s->tc_mask);
    case 3: /* IntErrorStatus */
        return (s->err_int & s->err_mask);
    case 5: /* RawIntTCStatus */
        return s->tc_int;
    case 6: /* RawIntErrorStatus */
        return s->err_int;
    case 7: /* EnbldChns */
        mask = 0;
        for (i = 0; i < s->nchannels; i++) {
            if (s->chan[i].conf & PL080_CCONF_E)
                mask |= 1 << i;
        }
        return mask;
    case 8: /* SoftBReq */
    case 9: /* SoftSReq */
    case 10: /* SoftLBReq */
    case 11: /* SoftLSReq */
        /* ??? Implement these. */
        return 0;
    case 12: /* Configuration */
        return s->conf;
    case 13: /* Sync */
        return s->sync;
    default:
    bad_offset:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl080_read: Bad offset %x\n", (int)offset);
        return 0;
    }
}

/*
 * Trace wrapper. The channel-register cases above return directly, so the tally
 * cannot live inside them without restructuring the switch. This asks the one
 * question that matters for the audio work: which DMAC register does the
 * firmware POLL to decide how much room the output has, and what value does the
 * model hand back? The codec is being called with a sample count of zero, and
 * that count is derived from the guest's own book-keeping of DMA progress.
 */
static uint64_t pl080_read(void *opaque, hwaddr offset, unsigned size)
{
    uint64_t val = pl080_read_inner(opaque, offset, size);

    if (pl080_trace_on() && offset < 0x200) {
        static uint64_t n[128];
        unsigned slot = (unsigned)(offset >> 2) & 127;
        uint64_t c = ++n[slot];

        if ((c & (c - 1)) == 0 || c % 100000 == 0) {
            fprintf(stderr, "DMATRACE %s RD +0x%03" HWADDR_PRIx
                    " -> 0x%08x  n=%llu\n",
                    pl080_trace_tag((PL080State *)opaque),
                    offset, (uint32_t)val, (unsigned long long)c);
        }
    }
    return val;
}

static void pl080_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    PL080State *s = (PL080State *)opaque;
    int i;


    if (offset >= 0x100 && offset < 0x200) {
        i = (offset & 0xe0) >> 5;
        if (i >= s->nchannels)
            goto bad_offset;
        switch ((offset >> 2) & 7) {
        case 0: /* SrcAddr */
            s->chan[i].src = value;
            break;
        case 1: /* DestAddr */
            s->chan[i].dest = value;
            break;
        case 2: /* LLI */
            s->chan[i].lli = value;
            /*
             * If this channel is a completed-pending paced channel (see the
             * terminal handling in pl080_run), the guest is extending the chain:
             * we only RECORD the new link here. The channel is reloaded from it
             * in pl080_run on the next drain, mirroring the hardware, which
             * consumes the LINK register only when it reaches the end of the
             * terminal LLI. Reloading synchronously here would change what the
             * guest's own read-back of DMACCxLINK returns a few instructions
             * later and corrupt its book-keeping. Because the enable bit was
             * never dropped, the guest's follow-up CONFIG.E read passes instead
             * of panicking "DMA ch link error".
             *
             * This is also how a channel that has run dry gets going again:
             * the write runs nothing here, so the peripheral has to be told.
             * MEASURED: Rockbox's first two tasks are 12 ms between them, its
             * handler ran after they had gone, and without this the drain
             * stayed off for good.
             */
            if ((s->paced_pending & (1u << i)) && value && s->paced_kick) {
                s->paced_kick(s->paced_opaque);
            }
            break;
        case 3: /* Control */
            s->chan[i].ctrl = value;
            /* A fresh descriptor with real work clears any completed-pending
               state (e.g. dmac_ch_run starting a new task). */
            if (value & 0xfff) {
                s->paced_pending &= ~(1u << i);
            }
            break;
        case 4: /* Configuration */
            if (!(value & PL080_CCONF_E)) {
                /* Explicit disable/abort resolves the completed-pending state. */
                s->paced_pending &= ~(1u << i);
            }
            s->chan[i].conf = value;
            if (pl080_trace_on() && (value & PL080_CCONF_E)) {
                fprintf(stderr,
                        "DMATRACE %s ch%d ENABLE conf=0x%08x flow=%d "
                        "srcid=%d dstid=%d src=0x%08x dst=0x%08x lli=0x%08x "
                        "xsize=%u\n",
                        pl080_trace_tag(s), i, (uint32_t)value,
                        (int)((value >> 11) & 7), (int)((value >> 1) & 0x1f),
                        (int)((value >> 6) & 0x1f), s->chan[i].src,
                        s->chan[i].dest, s->chan[i].lli,
                        (unsigned)(s->chan[i].ctrl & 0xfff));
            }
            pl080_run(s);
            break;
        }
        return;
    }
    switch (offset >> 2) {
    case 2: /* IntTCClear */
        s->tc_int &= ~value;
        if ((s->paced_held & value) && s->paced_kick) {
            s->paced_held &= ~value;
            s->paced_kick(s->paced_opaque);
        }
        break;
    case 4: /* IntErrorClear */
        s->err_int &= ~value;
        break;
    case 8: /* SoftBReq */
    case 9: /* SoftSReq */
    case 10: /* SoftLBReq */
    case 11: /* SoftLSReq */
        /* ??? Implement these.  */
        qemu_log_mask(LOG_UNIMP, "pl080_write: Soft DMA not implemented\n");
        break;
    case 12: /* Configuration */
        s->conf = value;
        if (s->conf & (PL080_CONF_M1 | PL080_CONF_M2)) {
            qemu_log_mask(LOG_UNIMP,
                          "pl080_write: Big-endian DMA not implemented\n");
        }
        pl080_run(s);
        break;
    case 13: /* Sync */
        s->sync = value;
        break;
    default:
    bad_offset:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl080_write: Bad offset %x\n", (int)offset);
    }
    pl080_update(s);
}

static const MemoryRegionOps pl080_ops = {
    .read = pl080_read,
    .write = pl080_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void pl080_reset(DeviceState *dev)
{
    PL080State *s = PL080(dev);
    int i;

    s->tc_int = 0;
    s->tc_mask = 0;
    s->err_int = 0;
    s->err_mask = 0;
    s->conf = 0;
    s->sync = 0;
    s->req_single = 0;
    s->req_burst = 0;
    s->running = 0;
    s->paced_pending = 0;
    s->paced_held = 0;

    for (i = 0; i < s->nchannels; i++) {
        s->chan[i].src = 0;
        s->chan[i].dest = 0;
        s->chan[i].lli = 0;
        s->chan[i].ctrl = 0;
        s->chan[i].conf = 0;
    }
}

static void pl080_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    PL080State *s = PL080(obj);

    memory_region_init_io(&s->iomem1, OBJECT(s), &pl080_ops, s, "pl080", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem1);
    /*
     * Second window: kept for the boards that use it.  The iPod Classic
     * machine maps only iomem1, and the model serves both from the same
     * state, so nothing here is iPod-specific (Q24).
     */
    memory_region_init_io(&s->iomem2, OBJECT(s), &pl080_ops, s, "pl080", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem2);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->interr);
    sysbus_init_irq(sbd, &s->inttc);
    s->nchannels = 8;
}

static void pl080_realize(DeviceState *dev, Error **errp)
{
    PL080State *s = PL080(dev);

    if (!s->downstream) {
        error_setg(errp, "PL080 'downstream' link not set");
        return;
    }

    address_space_init(&s->downstream_as, s->downstream, "pl080-downstream");
}

static void pl081_init(Object *obj)
{
    PL080State *s = PL080(obj);

    s->nchannels = 2;
}

static Property pl080_properties[] = {
    DEFINE_PROP_LINK("downstream", PL080State, downstream,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_END_OF_LIST(),
};

static void pl080_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->vmsd = &vmstate_pl080;
    dc->realize = pl080_realize;
    device_class_set_props(dc, pl080_properties);
    dc->reset = pl080_reset;
}

static const TypeInfo pl080_info = {
    .name          = TYPE_PL080,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PL080State),
    .instance_init = pl080_init,
    .class_init    = pl080_class_init,
};

static const TypeInfo pl081_info = {
    .name          = TYPE_PL081,
    .parent        = TYPE_PL080,
    .instance_init = pl081_init,
};

/* The PL080 and PL081 are the same except for the number of channels
   they implement (8 and 2 respectively).  */
static void pl080_register_types(void)
{
    type_register_static(&pl080_info);
    type_register_static(&pl081_info);
}

type_init(pl080_register_types)
