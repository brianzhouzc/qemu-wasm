/*
 * S5L8702 display pipe, 0x38900000.
 *
 * See include/hw/arm/ipod_classic/ipod_classic_disp.h for the register map and
 * where every line of it comes from. In short: this is the block that holds the
 * panel geometry, the three layer descriptors, the source-address slots and the
 * gamma tables, and it is what the LCD frame engine (LCD +0x70) presents.
 *
 * The model is a register file with honest readback, a loud trace, and a
 * compositor that mixes the enabled layers into the panel's image memory the
 * way the guest programs the block's mixer. A boot that never mounts a volume
 * never writes a source address into any slot, so on that path it composes
 * nothing and the panel stays black -- that is the guest's behaviour, not the
 * model's.
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
#include "qemu/bswap.h"
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
    case S5L8702_DISP_BACKGROUND: return "BACKGROUND";
    case S5L8702_DISP_PLANAR_FMT: return "PL_FORMAT";
    case S5L8702_DISP_PLANAR_STRIDE: return "PL_STRIDE";
    case S5L8702_DISP_PLANAR_ORIGIN: return "PL_ORIGIN";
    case S5L8702_DISP_PLANAR_LIMIT:  return "PL_LIMIT";
    case 0x038:                   return "SRC_PLANE0";
    case 0x03c:                   return "SRC_PLANE1";
    case 0x040:                   return "SRC_PLANE2";
    case 0x044:                   return "SRC_PLANE3";
    case S5L8702_DISP_PLANAR_STEP: return "PL_STEP";
    case S5L8702_DISP_PLANAR_POS:  return "PL_POS";
    case S5L8702_DISP_PLANAR_SIZE: return "PL_SIZE";
    case 0x060:                   return "SRC_SLOT0";
    case 0x078:                   return "SRC_SLOT1";
    case 0x090:                   return "SRC_SLOT2";
    case 0x0a8:                   return "SRC_SLOT3";
    case 0x0c0:                   return "SRC_SLOT4";
    case S5L8702_DISP_ORDER:      return "ORDER";
    case 0x0d8:                   return "STAGE0";
    case 0x0dc:                   return "STAGE0_W1";
    case 0x0e0:                   return "STAGE1";
    case 0x0e4:                   return "STAGE1_W1";
    case 0x0e8:                   return "STAGE2";
    case 0x0ec:                   return "STAGE2_W1";
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
 * A word on the addresses in the newer notes here and in the header. The 2.0.5
 * image copies itself down at boot and runs 0xAF98 below where its file offset
 * would put it: code at offset F runs at 0x08000000 + F - 0xAF98, which is what
 * DISPTRACE prints and what those notes quote. Layer buffers read at the wrong
 * base come out 35 rows and 38 pixels off, which is how this was found. The
 * older notes are from an earlier retailOS.
 */

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

static inline uint16_t disp_rgb565(uint32_t rgb)
{
    return ((rgb >> 8) & 0xf800) | ((rgb >> 5) & 0x07e0) | ((rgb >> 3) & 0x001f);
}

/* One RGB565 pixel as an opaque 0xAARRGGBB; packs back to itself. */
static inline uint32_t disp_expand565(uint16_t px)
{
    unsigned r = px >> 11, g = (px >> 5) & 0x3f, b = px & 0x1f;

    return 0xff000000u | ((r << 3 | r >> 2) << 16) | ((g << 2 | g >> 4) << 8) |
           (b << 3 | b >> 2);
}

static inline unsigned disp_clamp8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* BT.601 limited range; see the header for how that was measured. */
static inline uint32_t disp_ycbcr(int y, int cb, int cr)
{
    int c = 298 * (y - 16), d = cb - 128, e = cr - 128;

    return 0xff000000u | (disp_clamp8((c + 409 * e + 128) >> 8) << 16) |
           (disp_clamp8((c - 100 * d - 208 * e + 128) >> 8) << 8) |
           disp_clamp8((c + 516 * d + 128) >> 8);
}

/*
 * One stage's blend, as a pair of tables indexed by the source pixel's alpha.
 * Worked out once per input per frame so the pixel loop decides nothing.
 */
typedef struct DispBlend {
    uint8_t src[256];
    uint8_t dst[256];
} DispBlend;

/* The factors the firmware's blend setter can write; see the header. */
static unsigned disp_factor(unsigned sel, unsigned alpha, unsigned global)
{
    switch (sel) {
    case 0:  return 0;
    case 1:  return 255;
    case 4:  return alpha;
    case 5:  return 255 - alpha;
    case 8:  return global;
    case 9:  return 255 - global;
    case 10: return alpha * global / 255;
    case 11: return 255 - alpha * global / 255;
    default:
        qemu_log_mask(LOG_UNIMP, "s5l8702.disp: blend factor %u is not known; "
                      "treating the layer as opaque\n", sel);
        return sel & 1 ? 0 : 255;
    }
}

static void disp_blend_init(DispBlend *b, uint32_t stage)
{
    unsigned df = stage >> 28, sf = (stage >> 12) & 0xf, global = stage & 0xff;

    for (unsigned a = 0; a < 256; a++) {
        b->src[a] = disp_factor(sf, a, global);
        b->dst[a] = disp_factor(df, a, global);
    }
}

static inline unsigned disp_mix_channel(unsigned s, unsigned d, unsigned sf,
                                        unsigned df)
{
    unsigned v = (s * sf + d * df + 127) / 255;

    return v > 255 ? 255 : v;
}

/* Mix a row of 0xAARRGGBB source pixels into the frame. */
static void disp_mix_row(uint32_t *dst, const uint32_t *src, int n,
                         const DispBlend *b)
{
    for (int i = 0; i < n; i++) {
        uint32_t c = src[i], d = dst[i];
        unsigned a = c >> 24, sf = b->src[a], df = b->dst[a];

        if (sf == 255 && df == 0) {
            dst[i] = c & 0xffffff;
        } else if (sf != 0 || df != 255) {
            dst[i] = disp_mix_channel((c >> 16) & 0xff, (d >> 16) & 0xff, sf, df)
                         << 16 |
                     disp_mix_channel((c >> 8) & 0xff, (d >> 8) & 0xff, sf, df)
                         << 8 |
                     disp_mix_channel(c & 0xff, d & 0xff, sf, df);
        }
    }
}

/*
 * Mix one of the five RGB layers into the frame. Returns true if any pixel
 * landed on the panel.
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
 * a line below and nothing else changes.
 *
 * Which formats carry alpha is the firmware's own answer: it asks for per-pixel
 * blending (mode 2) after programming a layer of format 4, 5 or 7 (0x0814f010 by
 * file address) and leaves 3 and 6 alone. Format 6 is therefore opaque here.
 * Formats 4 and 5 are 16-bit with alpha in some layout nothing has shown us; no
 * screen uses them, and they are logged and drawn as RGB565.
 */
static bool disp_draw_layer(IPodClassicDispState *s, int k, const DispBlend *blend,
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
    uint32_t opaque = fmt == 7 ? 0 : 0xff000000u;
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
     */
    if (!(s->reg[S5L8702_DISP_CONTROL >> 2] & S5L8702_DISP_LAYER_EN_BIT(k))) {
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
    if (fmt == 4 || fmt == 5) {
        qemu_log_mask(LOG_UNIMP, "s5l8702.disp: layer %d has pixel format %u, "
                      "whose alpha this model does not decode; drawn opaque\n",
                      k, fmt);
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
    if ((unsigned)w > ARRAY_SIZE(s->rowbuf)) {
        w = ARRAY_SIZE(s->rowbuf);
    }

    for (int row = 0; row < h; row++) {
        address_space_read(&address_space_memory, src + (hwaddr)row * stride,
                           MEMTXATTRS_UNSPECIFIED, s->rowbuf, w * bpp);
        if (bpp == 4) {
            for (int i = 0; i < w; i++) {
                s->rowbuf[i] = ldl_le_p(&s->rowbuf[i]) | opaque;
            }
        } else {
            /* Backwards, so a 16-bit row can grow where it landed. */
            for (int i = w - 1; i >= 0; i--) {
                const uint8_t *p = (const uint8_t *)s->rowbuf + i * 2;
                s->rowbuf[i] = disp_expand565(lduw_le_p(p));
            }
        }
        disp_mix_row(s->frame + (y + row) * width + x, s->rowbuf, w, blend);
    }
    return true;
}

/*
 * Mix layer 5 into the frame. See the header for the registers. The hardware
 * filters when it scales, with coefficients the guest loads; this picks the
 * nearest sample, and chroma the sample it covers, which is what the 1:1 frames
 * measured against. No screen we have seen scales this layer.
 */
static bool disp_draw_planar(IPodClassicDispState *s, const DispBlend *blend,
                             unsigned width, unsigned height)
{
    uint32_t origin = s->reg[S5L8702_DISP_PLANAR_ORIGIN >> 2];
    uint32_t limit  = s->reg[S5L8702_DISP_PLANAR_LIMIT >> 2];
    uint32_t pos    = s->reg[S5L8702_DISP_PLANAR_POS >> 2];
    uint32_t size   = s->reg[S5L8702_DISP_PLANAR_SIZE >> 2];
    uint32_t step   = s->reg[S5L8702_DISP_PLANAR_STEP >> 2];
    uint32_t stride = s->reg[S5L8702_DISP_PLANAR_STRIDE >> 2];
    uint32_t luma   = s->reg[S5L8702_DISP_SRC_PLANE0 >> 2];
    uint32_t cb     = s->reg[S5L8702_DISP_SRC_PLANE1 >> 2];
    uint32_t cr     = s->reg[S5L8702_DISP_SRC_PLANE3 >> 2];
    unsigned ox = origin >> 16, oy = origin & 0xffff;
    unsigned sw = (limit >> 16) - ox, sh = (limit & 0xffff) - oy;
    unsigned x = pos >> 16, y = pos & 0xffff;
    unsigned w = (size >> 16) & 0x3ff, h = size & 0x3ff;
    unsigned xstep = step >> 16, ystep = step & 0xffff;
    unsigned lstride = stride & 0xffff, cstride = stride >> 16;

    if (!(s->reg[S5L8702_DISP_CONTROL >> 2] & S5L8702_DISP_PLANAR_EN_BIT) ||
        !luma || !cb || !cr) {
        return false;
    }
    if (s->reg[S5L8702_DISP_PLANAR_FMT >> 2] != 0x100) {
        qemu_log_mask(LOG_UNIMP, "s5l8702.disp: layer 5 has format word 0x%x, "
                      "which this model does not decode; skipped\n",
                      s->reg[S5L8702_DISP_PLANAR_FMT >> 2]);
        return false;
    }
    if ((limit >> 16) <= ox || (limit & 0xffff) <= oy || !w || !h ||
        x >= width || y >= height) {
        return false;
    }
    w = MIN(w, width - x);
    h = MIN(h, height - y);
    /* The samples this row needs: never more than the scratch or the window. */
    sw = MIN(sw, MIN(((w - 1) * xstep >> 12) + 1, sizeof(s->lumarow)));

    for (unsigned row = 0; row < h; row++) {
        unsigned sy = (row * ystep) >> 12;

        if (sy >= sh) {
            break;
        }
        sy += oy;
        address_space_read(&address_space_memory,
                           luma + (hwaddr)sy * lstride + ox,
                           MEMTXATTRS_UNSPECIFIED, s->lumarow, sw);
        address_space_read(&address_space_memory,
                           cb + (hwaddr)(sy / 2) * cstride + ox / 2,
                           MEMTXATTRS_UNSPECIFIED, s->cbrow, sw / 2 + 1);
        address_space_read(&address_space_memory,
                           cr + (hwaddr)(sy / 2) * cstride + ox / 2,
                           MEMTXATTRS_UNSPECIFIED, s->crrow, sw / 2 + 1);
        for (unsigned i = 0; i < w; i++) {
            unsigned sx = MIN((i * xstep) >> 12, sw - 1);
            unsigned cx = ((ox + sx) >> 1) - (ox >> 1);

            s->rowbuf[i] = disp_ycbcr(s->lumarow[sx], s->cbrow[cx], s->crrow[cx]);
        }
        disp_mix_row(s->frame + (y + row) * width + x, s->rowbuf, w, blend);
    }
    return true;
}

/*
 * The stage each mixer input uses, by the order in +0x0d4. The firmware's own
 * table, read out of guest memory at 0x083f35ec. Only orders 1 and 0 have been
 * seen in use; the rest are here because the table is.
 */
enum { DISP_INPUT_LAYER0, DISP_INPUT_SHARED, DISP_INPUT_PLANAR, DISP_NINPUT };

static const uint8_t disp_stage_of[6][DISP_NINPUT] = {
    { 1, 2, 0 }, { 2, 1, 0 }, { 0, 2, 1 }, { 2, 0, 1 }, { 0, 1, 2 }, { 1, 0, 2 },
};

/*
 * Back to front within the shared input; see the header for what backs this.
 * Layer 1 last is the guess: it has never overlapped layer 3.
 */
static const uint8_t disp_shared_layers[] = { 2, 3, 1 };

static bool disp_draw_input(IPodClassicDispState *s, int input,
                            const DispBlend *blend, unsigned width,
                            unsigned height)
{
    bool drew = false;

    switch (input) {
    case DISP_INPUT_LAYER0:
        drew = disp_draw_layer(s, 0, blend, width, height);
        break;
    case DISP_INPUT_SHARED:
        for (unsigned i = 0; i < ARRAY_SIZE(disp_shared_layers); i++) {
            drew |= disp_draw_layer(s, disp_shared_layers[i], blend, width,
                                    height);
        }
        break;
    case DISP_INPUT_PLANAR:
        drew = disp_draw_planar(s, blend, width, height);
        break;
    }
    return drew;
}

bool ipod_classic_disp_compose(IPodClassicDispState *s, uint16_t *fb,
                               unsigned width, unsigned height)
{
    uint32_t control = s->reg[S5L8702_DISP_CONTROL >> 2];
    uint32_t background = s->reg[S5L8702_DISP_BACKGROUND >> 2] & 0xffffff;
    unsigned order = s->reg[S5L8702_DISP_ORDER >> 2] & 0xff;
    unsigned pixels = width * height;
    DispBlend blend;
    bool drew = false;

    /*
     * Nothing on, nothing to do, and the panel's image memory stays as it is:
     * this is every frame of Rockbox, which pushes its pixels through the LCD
     * port, and of tools/ipod_classic/lcdtest.S.
     */
    if (!(control & (S5L8702_DISP_PLANAR_EN_BIT |
                     S5L8702_DISP_LAYER_EN_BIT(0) |
                     S5L8702_DISP_LAYER_EN_BIT(1) |
                     S5L8702_DISP_LAYER_EN_BIT(2) |
                     S5L8702_DISP_LAYER_EN_BIT(3) |
                     S5L8702_DISP_LAYER_EN_BIT(4)))) {
        return false;
    }
    if (control & S5L8702_DISP_LAYER_EN_BIT(4)) {
        qemu_log_mask(LOG_UNIMP, "s5l8702.disp: layer 4 is on, and the firmware "
                      "gives it no mixer input; skipped\n");
    }
    if (order >= ARRAY_SIZE(disp_stage_of)) {
        qemu_log_mask(LOG_UNIMP, "s5l8702.disp: stage order %u is not in the "
                      "firmware's table; using 1\n", order);
        order = 1;
    }

    /*
     * Every frame starts from the background, not from the last frame. The
     * model used to draw over whatever was in fb, which with opaque layers
     * cost nothing and with alpha turned the old frame into a layer of its own:
     * that is the "both pane icons at once" that got qemu-wasm#9 reverted.
     */
    for (unsigned i = 0; i < pixels; i++) {
        s->frame[i] = background;
    }

    /*
     * Back to front, stage by stage. An earlier version drew the slots in
     * index order with no alpha, so layer 0 went at the back with nothing
     * blended, and layer 5 not at all. Layer 0 holds the user interface and
     * belongs in front, so that one mistake was three symptoms: the main
     * menu's translucent pane on black, with its background in layer 5 and its
     * alpha ignored; the whole interface hidden under layer 2 while it was on;
     * and Cover Flow's and Now Playing's text hidden under layer 1. See
     * docs/DISPLAY_ALPHA.md in iPodWasm for the measurements.
     */
    for (int stage = 0; stage < S5L8702_DISP_NSTAGE; stage++) {
        uint32_t reg = s->reg[(S5L8702_DISP_STAGE0 +
                               stage * S5L8702_DISP_STAGE_STEP) >> 2];

        disp_blend_init(&blend, reg);
        for (int input = 0; input < DISP_NINPUT; input++) {
            if (disp_stage_of[order][input] == stage) {
                drew |= disp_draw_input(s, input, &blend, width, height);
            }
        }
    }
    if (!drew) {
        return false;
    }
    for (unsigned i = 0; i < pixels; i++) {
        fb[i] = disp_rgb565(s->frame[i]);
    }
    return true;
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
