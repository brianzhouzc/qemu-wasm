/*
 * S5L8702 display pipe, 0x38900000.
 *
 * See include/hw/arm/ipod_classic/ipod_classic_disp.h for the register map and
 * where every line of it comes from. In short: this is the block that holds the
 * panel geometry, the three layer descriptors, the source-address slots and the
 * gamma tables, and it is what the LCD frame engine (LCD +0x70) presents.
 *
 * The model is a register file with honest readback, a loud trace, and a
 * compositor that blits whichever layer slots currently carry a source address
 * into the panel's image memory. A boot that never mounts a volume never writes
 * a source address into any slot, so on that path it composes nothing and the
 * panel stays black -- that is the guest's behaviour, not the model's.
 *
 * Readback alone is not cosmetic. Six functions in the osos do
 * read-modify-write on +0x008 and one on +0x200; while this block was
 * create_unimplemented_device() every one of them read zero and dropped every
 * field written before it. That is the same failure mode the DMAC0 note in
 * wiki/11 describes, and it was silently true here too.
 *
 * DISPTRACE=1 prints every access below +0x400 with its value and the guest PC,
 * and names the register when we know its name; the 768 gamma words are counted
 * rather than printed, and the totals go out at exit. Independently of the
 * trace, the FIRST source-address write of a run is announced on stderr: that
 * is the event the whole display investigation waited for.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "hw/core/cpu.h"
#include "exec/address-spaces.h"
#include "sysemu/cpu-timers.h"
#include "hw/arm/ipod_classic/ipod_classic_disp.h"

/*
 * INSTRUMENTATION ONLY -- nothing below changes what the model does. Same
 * pattern as ATATRACE=1 and LCDTRACE=1 elsewhere in this machine.
 */
static int disptrace = -1;
static void disp_trace_totals(void);

static bool disp_trace_on(void)
{
    if (disptrace < 0) {
        const char *e = getenv("DISPTRACE");
        disptrace = (e && *e && *e != '0') ? 1 : 0;
        if (disptrace) {
            atexit(disp_trace_totals);
        }
    }
    return disptrace == 1;
}

/*
 * The last PC the CPU synchronised. Under TCG that is the head of the running
 * translation block, not the address of the store -- io_prepare() does not call
 * cpu_restore_state() -- so it is the same granularity as -d in_asm and useful
 * for the same thing: naming the function. Printed as "pc~" to say so.
 */
static uint32_t disp_trace_pc(void)
{
    CPUState *cs = current_cpu;
    return cs ? (uint32_t)CPU_GET_CLASS(cs)->get_pc(cs) : 0;
}

static uint64_t disp_reg_writes;
static uint64_t disp_gamma_writes;
static uint64_t disp_src_writes;
static uint64_t disp_ctrl_reads;

static void disp_trace_totals(void)
{
    fprintf(stderr, "DISPTRACE totals: %" PRIu64 " register writes below "
            "+0x400, %" PRIu64 " gamma-table writes, %" PRIu64 " reads of the "
            "control register +0x008, and %" PRIu64 " writes of a SOURCE "
            "ADDRESS (+0x038..+0x044, +0x060/78/90/a8/c0)\n",
            disp_reg_writes, disp_gamma_writes, disp_ctrl_reads,
            disp_src_writes);
}

/* Is this offset one of the nine source-address slots? See the header. */
static bool disp_is_source(hwaddr off)
{
    if (off >= S5L8702_DISP_SRC_PLANE0 && off <= S5L8702_DISP_SRC_PLANE3) {
        return true;
    }
    for (int i = 0; i < S5L8702_DISP_NSLOT; i++) {
        if (off == S5L8702_DISP_SRC_SLOT0 + i * S5L8702_DISP_SRC_STRIDE) {
            return true;
        }
    }
    return false;
}

static const char *disp_reg_name(hwaddr off)
{
    switch (off) {
    case S5L8702_DISP_CONTROL:    return "CONTROL";
    case S5L8702_DISP_DST_ORIGIN: return "DST_ORIGIN";
    case S5L8702_DISP_DST_LIMIT:  return "DST_LIMIT";
    case 0x038:                   return "SRC_PLANE0";
    case 0x03c:                   return "SRC_PLANE1";
    case 0x040:                   return "SRC_PLANE2";
    case 0x044:                   return "SRC_PLANE3";
    case S5L8702_DISP_DST_SIZE:   return "DST_SIZE";
    case S5L8702_DISP_DST_OFF:    return "DST_OFF";
    case S5L8702_DISP_DST_CLIP:   return "DST_CLIP";
    case 0x060:                   return "SRC_SLOT0";
    case 0x078:                   return "SRC_SLOT1";
    case 0x090:                   return "SRC_SLOT2";
    case 0x0a8:                   return "SRC_SLOT3";
    case 0x0c0:                   return "SRC_SLOT4";
    case 0x0d8:                   return "LAYER0_CFG";
    case 0x0dc:                   return "LAYER0_W1";
    case 0x0e0:                   return "LAYER1_CFG";
    case 0x0e4:                   return "LAYER1_W1";
    case 0x0e8:                   return "LAYER2_CFG";
    case 0x0ec:                   return "LAYER2_W1";
    case S5L8702_DISP_STATUS200:  return "STATUS200";
    case S5L8702_DISP_GEOMETRY:   return "GEOMETRY";
    case S5L8702_DISP_FORMAT:     return "FORMAT";
    default:                      return NULL;
    }
}

static uint64_t ipod_classic_disp_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    IPodClassicDispState *s = opaque;
    uint32_t val = s->reg[offset >> 2];

    if (offset == S5L8702_DISP_CONTROL) {
        disp_ctrl_reads++;
    }

    if (disp_trace_on()) {
        const char *n = disp_reg_name(offset);
        fprintf(stderr, "DISPTRACE rd +0x%03" HWADDR_PRIx " -> 0x%08x  %-10s "
                "pc~0x%08x\n", offset, val, n ? n : "", disp_trace_pc());
    }
    return val;
}

static void ipod_classic_disp_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
    IPodClassicDispState *s = opaque;
    bool gamma = offset >= S5L8702_DISP_GAMMA0;

    s->reg[offset >> 2] = value;

    if (gamma) {
        disp_gamma_writes++;
    } else {
        disp_reg_writes++;
    }

    /*
     * The one event worth shouting about: a source address, i.e. the block
     * being told where to read pixels from. A diskless boot writes zero of
     * these and draws nothing; a boot with a disk writes one per layer per
     * frame update, so this cannot be printed every time -- stderr is the
     * channel the firmware itself talks on (lesson 1 of wiki/09).
     *
     * So: announce the FIRST one unconditionally, because that is the event the
     * whole display investigation waited for, and after that say nothing unless
     * DISPTRACE=1 is set. The running total is printed at exit and it counts
     * every write, not every change -- an earlier version compared against the
     * last value seen and so counted transitions, which undercounts any layer
     * that is re-armed with the same address (wiki/11 §8, "a log that only
     * announces the first occurrence cannot count").
     */
    if (disp_is_source(offset) && value != 0) {
        static bool announced;

        disp_src_writes++;
        if (!announced) {
            announced = true;
            /*
             * The timestamps are for the browser port's measurements: the
             * first display write is the earliest moment a script can call
             * the screen up, and the wall clock, guest clock and instruction
             * count together say how long the boot took in a way that can be
             * compared across hosts (docs/proposals/browser-emulator.md).
             */
            fprintf(stderr, "s5l8702.disp: SOURCE ADDRESS +0x%03" HWADDR_PRIx
                    " = 0x%08x  (pc~0x%08x) -- first time this block has been "
                    "given something to read from (rt_ns=%" PRId64
                    " vclock_ns=%" PRId64 " icount=%" PRId64 ")\n",
                    offset, (uint32_t)value, disp_trace_pc(),
                    qemu_clock_get_ns(QEMU_CLOCK_REALTIME),
                    qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                    icount_enabled() ? icount_get_raw() : -1);
        }
    }

    if (disp_trace_on()) {
        const char *n = disp_reg_name(offset);
        if (gamma) {
            /* 768 of these per boot; the total is printed at exit. */
            return;
        }
        fprintf(stderr, "DISPTRACE wr +0x%03" HWADDR_PRIx " = 0x%08x  %-10s "
                "pc~0x%08x\n", offset, (uint32_t)value, n ? n : "",
                disp_trace_pc());
    }
}

/* --------------------------------------------------------------- compose */

/*
 * Bytes per pixel by format code. From the switch at 0x08126f90, which turns
 * the format field into a multiplier r1 and then computes the source stride as
 * r1 * srcW / 2 -- so r1 is twice the bytes per pixel. VERIFIED against both
 * layers a real boot programs: fmt 7 with stride 1280 for a 320-wide surface
 * (4 bytes) and fmt 3 with stride 640 (2 bytes).
 */
static unsigned disp_bpp(unsigned fmt)
{
    switch (fmt) {
    case 0:  return 0;   /* 4 bpp; never seen, and a half-byte pixel needs its
                          * own path. Skipped and logged rather than guessed. */
    case 1:  return 1;   /* 8 bpp, presumably paletted. Never seen. */
    case 2: case 3: case 4: case 5: return 2;
    case 6: case 7:                 return 4;
    default: return 0;
    }
}

static bool disp_alpha_on(void)
{
    static int on = -1;
    if (on < 0) {
        const char *e = getenv("DISP_ALPHA");
        on = (e && *e && *e != '0') ? 1 : 0;
        if (on) {
            fprintf(stderr, "DISP_ALPHA: capas compuestas de delante a atras con alfa\n");
        }
    }
    return on == 1;
}

static inline uint16_t disp_rgb565(unsigned r, unsigned g, unsigned b)
{
    return ((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3);
}

/*
 * Draw one layer. Returns true if any pixel landed on the panel.
 *
 * PIXEL ORDER, and this is the one INFERRED thing in the whole path. A 32-bit
 * source word is read as little-endian ARGB8888, i.e. the bytes in memory are
 * B, G, R, A; a 16-bit one as RGB565. The argument, in the order it carries
 * weight:
 *   - the panel is RGB565 with red in the high bits. That is not our claim:
 *     Rockbox's config for this exact device says LCD_PIXELFORMAT RGB565
 *     (firmware/export/config/ipod6g.h:85). So a 16-bit layer handed to it
 *     unchanged is RGB565, and read that way the "Welcome" pane really is the
 *     cool blue-grey gradient the iPod has.
 *   - the 32-bit layer holds the same artwork, already composited -- the right
 *     third of it IS that gradient. Only the ARGB8888 reading makes the two
 *     agree; the other one turns both brown.
 *   - and then the selection bar comes out 0xff0484da, i.e. #0484da, Apple's
 *     iPod selection blue. Read the other way round it is #da8404, an orange
 *     that appears nowhere in this user interface.
 * It is still an inference: no datasheet, and the pipe does carry colour-matrix
 * tables it could in principle swizzle with. If better evidence turns up it is
 * two lines below and nothing else changes.
 */
static bool disp_draw_layer(IPodClassicDispState *s, int k, uint16_t *fb,
                            unsigned width, unsigned height)
{
    hwaddr base = S5L8702_DISP_SLOT_BASE + k * S5L8702_DISP_SLOT_STEP;
    uint32_t stride = s->reg[(base + S5L8702_DISP_SLOT_STRIDE) >> 2];
    uint32_t fmtw   = s->reg[(base + S5L8702_DISP_SLOT_FORMAT) >> 2];
    uint32_t src    = s->reg[(base + S5L8702_DISP_SLOT_SRC) >> 2];
    uint32_t sizew  = s->reg[(base + S5L8702_DISP_SLOT_SIZE) >> 2];
    uint32_t posw   = s->reg[(base + S5L8702_DISP_SLOT_POS) >> 2];
    unsigned fmt = (fmtw >> 8) & 0xff;
    unsigned bpp = disp_bpp(fmt);
    /* Signed: a pane sliding off the right edge is written with width -55. */
    int w = (int16_t)(sizew >> 16), h = (int16_t)(sizew & 0xffff);
    int x = (int16_t)(posw >> 16),  y = (int16_t)(posw & 0xffff);

    if (src == 0 || w <= 0 || h <= 0) {
        return false;
    }
    /*
     * THE GATE. A layer is only on the panel while its enable bit is set in
     * CONTROL; the guest clears the bit at the end of a transition and leaves
     * the descriptor -- source address, size, position -- exactly as it was.
     * Drawing on the source address alone therefore keeps a dead surface on
     * screen for ever, and that is what hid the whole main menu behind the
     * leftover right-hand pane of the screen before it.
     *
     * See S5L8702_DISP_LAYER_EN_BIT in the header for the disassembly of the
     * setter and for the traced write order that pins each bit to its layer.
     *
     * Layers 3 and 4 are not gated because we do not know which bit, if any,
     * gates them: no boot has ever written their descriptors, so they are
     * skipped above on src == 0 and this line never decides anything for them.
     */
    if (k < S5L8702_DISP_NLAYER_GATED &&
        !(s->reg[S5L8702_DISP_CONTROL >> 2] & S5L8702_DISP_LAYER_EN_BIT(k))) {
        return false;
    }
    if (bpp != 2 && bpp != 4) {
        /*
         * 4- and 8-bit formats need a palette we have never seen written, so
         * there is nothing to decode them with. No boot has ever programmed
         * one; if this fires, that has changed and it is worth knowing.
         */
        qemu_log_mask(LOG_UNIMP, "s5l8702.disp: layer %d has pixel format %u "
                      "(%u bytes per pixel), which this model does not decode; "
                      "skipped\n", k, fmt, bpp);
        return false;
    }
    if (x >= (int)width || y >= (int)height || x + w <= 0 || y + h <= 0) {
        return false;
    }

    /* Clip to the panel. Left/top clipping also has to advance the source. */
    if (x < 0) {
        src += (unsigned)(-x) * bpp;
        w += x;
        x = 0;
    }
    if (y < 0) {
        src += (unsigned)(-y) * stride;
        h += y;
        y = 0;
    }
    if (x + w > (int)width) {
        w = width - x;
    }
    if (y + h > (int)height) {
        h = height - y;
    }
    /* The row scratch is one panel-width of 32-bit pixels; never exceed it. */
    if ((unsigned)w * bpp > sizeof(s->rowbuf)) {
        w = sizeof(s->rowbuf) / bpp;
    }

    for (int row = 0; row < h; row++) {
        uint16_t *dst = fb + (y + row) * width + x;

        address_space_read(&address_space_memory, src + (hwaddr)row * stride,
                           MEMTXATTRS_UNSPECIFIED, s->rowbuf, w * bpp);
        if (bpp == 4) {
            for (int i = 0; i < w; i++) {
                const uint8_t *p = s->rowbuf + i * 4;
                /*
                 * ALPHA (DISP_ALPHA=1). The 32-bit layer really does carry one:
                 * dumped while Now Playing is up, its alpha byte takes 243
                 * distinct values across the surface, which no "ignore it"
                 * reading survives. Straight source-over onto what is already
                 * in the framebuffer.
                 */
                if (disp_alpha_on()) {
                    unsigned a = p[3];
                    if (a == 0) {
                        continue;
                    }
                    if (a != 0xff) {
                        uint16_t d = dst[i];
                        unsigned dr = (d >> 8) & 0xf8, dg = (d >> 3) & 0xfc;
                        unsigned db = (d << 3) & 0xf8;
                        dst[i] = disp_rgb565((p[2] * a + dr * (255 - a)) / 255,
                                             (p[1] * a + dg * (255 - a)) / 255,
                                             (p[0] * a + db * (255 - a)) / 255);
                        continue;
                    }
                }
                dst[i] = disp_rgb565(p[2], p[1], p[0]);
            }
        } else {
            for (int i = 0; i < w; i++) {
                dst[i] = s->rowbuf[i * 2] | (s->rowbuf[i * 2 + 1] << 8);
            }
        }
    }
    return true;
}

bool ipod_classic_disp_compose(IPodClassicDispState *s, uint16_t *fb,
                               unsigned width, unsigned height)
{
    bool drew = false;

    /*
     * Slot order is Z order, lowest index at the back. INFERRED, but a
     * measurement does back it, and an earlier version of this comment claimed
     * the opposite ("no two layers have ever overlapped"), which is false:
     * slots 0 and 1 overlap the whole time the language screen appears.
     *
     * MEASURED with DISPTRACE=1 over one boot with a disk. Slot 0 is 320x240 at
     * (0,0) for the whole run. Slot 1 is a 16-bit surface whose right edge is
     * pinned to the panel edge while its left edge slides right -- +0x084/+0x07c
     * go (x=80, w=240), (82, 238), (95, 225), (122, 198), (168, 152), (232, 88),
     * (310, 10), (400, -80), so x + w is 320 at every step until the surface
     * goes negative and stops being drawn. That is a wipe, and a wipe only works
     * if the sliding surface is IN FRONT: slot 1 holds the outgoing artwork (the
     * bare gradient, no text -- dumped and looked at) and slot 0 the incoming
     * one. With the Z order the other way round slot 0 would cover slot 1 from
     * the first frame and there would be no transition at all.
     *
     * A layer is drawn when it has a source address, a positive size AND its
     * enable bit set in CONTROL (+0x008): bit 6 for layer 0, bit 5 for layer 1,
     * bit 4 for layer 2. An earlier version of this comment said "nothing else
     * gates it" and named bits 16, 20 and 24 as the only candidates, dismissing
     * them because they are set once during init and never touched again. That
     * was right about 16/20/24 and wrong about the conclusion: the gate is
     * bits 4..6, they move all the time, and ignoring them left every finished
     * transition's surface on the panel for ever. See the header.
     */
    /*
     * Z ORDER, and DISP_ALPHA=1 turns it over.
     *
     * The order below (slot 0 at the back) was inferred from the language
     * screen's wipe, on the assumption that every layer is opaque. MEASURED,
     * that assumption is false: the 32-bit layer carries a real alpha channel,
     * and while Now Playing is on screen slot 0 holds the ENTIRE player UI --
     * title bar, artist, album, "4 of 4", progress bar -- while slot 1 holds
     * the album art, opaque and full-screen. Drawn back-to-front the art wipes
     * the player out completely, which is exactly the blank right-hand side
     * that made the screen look half-drawn.
     *
     * Front-to-back with source-over alpha explains BOTH: the player UI sits on
     * top of the art, and the language wipe still works because the incoming
     * pane is transparent where the outgoing one must show through.
     */
    if (disp_alpha_on()) {
        for (int k = S5L8702_DISP_NSLOTS - 1; k >= 0; k--) {
            drew |= disp_draw_layer(s, k, fb, width, height);
        }
        return drew;
    }
    for (int k = 0; k < S5L8702_DISP_NSLOTS; k++) {
        drew |= disp_draw_layer(s, k, fb, width, height);
    }
    return drew;
}

static const MemoryRegionOps ipod_classic_disp_ops = {
    .read = ipod_classic_disp_read,
    .write = ipod_classic_disp_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void ipod_classic_disp_reset(DeviceState *dev)
{
    IPodClassicDispState *s = IPOD_CLASSIC_DISP(dev);

    /*
     * All zero. NOT a measurement: we have no reset values for this block, and
     * zero is what the guest saw while it was unimplemented, so nothing that
     * worked before can break on it.
     */
    memset(s->reg, 0, sizeof(s->reg));
}

static void ipod_classic_disp_realize(DeviceState *dev, Error **errp)
{
    IPodClassicDispState *s = IPOD_CLASSIC_DISP(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &ipod_classic_disp_ops, s,
                          TYPE_IPOD_CLASSIC_DISP, S5L8702_DISP_MEM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
}

static const VMStateDescription vmstate_ipod_classic_disp = {
    .name = TYPE_IPOD_CLASSIC_DISP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(reg, IPodClassicDispState, S5L8702_DISP_NREG),
        VMSTATE_END_OF_LIST()
    },
};

static void ipod_classic_disp_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "S5L8702 display pipe";
    dc->realize = ipod_classic_disp_realize;
    dc->reset = ipod_classic_disp_reset;
    dc->vmsd = &vmstate_ipod_classic_disp;
    dc->user_creatable = false;
}

static const TypeInfo ipod_classic_disp_info = {
    .name          = TYPE_IPOD_CLASSIC_DISP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodClassicDispState),
    .class_init    = ipod_classic_disp_class_init,
};

static void ipod_classic_disp_register_types(void)
{
    type_register_static(&ipod_classic_disp_info);
}

type_init(ipod_classic_disp_register_types)
