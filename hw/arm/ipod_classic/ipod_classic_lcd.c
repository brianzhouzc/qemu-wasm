/*
 * S5L8702 LCD interface and the panel behind it.
 *
 * Two halves.
 *
 * The BUS side is the MPU/8080 command port: the handshake bits every driver
 * spins on, the command and data ports, and a read cycle that completes. That
 * part is specified twice over -- see the header: Rockbox and Apple's retailOS
 * spin on the same three status bits -- so it can be written down with
 * confidence.
 *
 * The PANEL side is new, and it is the reason anything can appear on screen at
 * all. It is a 320x240 RGB565 image memory with a write window, driven by the
 * three commands the retailOS actually emits. Every number in it comes from a
 * measurement, not from a datasheet we do not have:
 *
 *   - The command set is the 8-bit one. VERIFIED: the guest writes
 *     LCD_CON = 0x80000c21 at 0x080b6fe0, and 0x080b6f8c only produces that
 *     value on the "lcd_get_type() in {0,1}" arm, which is Rockbox's
 *     LCD_CMDSET_8BIT (lcd-6g.c:201-214).
 *   - The commands are 0x2A column-address-set, 0x2B row-address-set,
 *     0x2C memory-write. VERIFIED: 0x080c1164 is Rockbox's displaylcd_setup()
 *     compiled from other sources -- same three commands, same four-byte
 *     big-endian parameters -- and the trace shows exactly
 *     0x2A {00 00 01 3f} 0x2B {00 00 00 ef} 0x2C.
 *   - 320x240 and RGB565: see the header.
 *
 * WHERE THE PIXELS COME FROM, which was the open question here for a long time
 * and is now answered. Not through this port: the retailOS pushes zero pixel
 * words through it in a whole boot, and no DMA channel is ever pointed at
 * LCD_WDATA either (Rockbox does that, Apple does not). The frame is composed
 * by the display pipe at 0x38900000 from layers in DRAM, and the frame engine
 * at +0x70 is what presents it. So gfx_update asks that block for the frame --
 * see ipod_classic_disp.h for the whole register map, and the note in
 * ipod_classic_lcd_gfx_update() for why the ask happens there and not on the
 * write to +0x70.
 *
 * +0x74..+0x8C are still only stored: +0x74 takes the geometry, +0x8C reports
 * "not busy" so the two spin loops at 0x080a4090 and 0x080b0128 terminate, and
 * what +0x78, +0x7C, +0x80 and +0x88 mean is still UNDETERMINED.
 *
 * Set LCDTRACE=1 to get every access with its value and the guest PC.
 *
 * See include/hw/arm/ipod_classic/ipod_classic_lcd.h for the register map and
 * where every bit comes from.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"
#include "hw/core/cpu.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "hw/arm/ipod_classic/ipod_classic_lcd.h"
#include "hw/arm/ipod_classic/ipod_classic_disp.h"
#include "hw/arm/ipod_classic/ipod_classic_bootscreen.h"
#include "hw/arm/ipod_classic/ipod_classic_np_overlay.h"

/*
 * INSTRUMENTATION ONLY -- LCDTRACE=1 in the environment prints every access to
 * this window, with the guest PC that made it, to stderr. Nothing here changes
 * what the model does. Same pattern as ATATRACE=1 in ipod_classic_ata.c.
 * See wiki/11-classic6g-retailos.md sec.3.
 */
static int lcdtrace = -1;
static void lcd_trace_totals(void);

static bool lcd_trace_on(void)
{
    if (lcdtrace < 0) {
        const char *e = getenv("LCDTRACE");
        lcdtrace = (e && *e && *e != '0') ? 1 : 0;
        if (lcdtrace) {
            atexit(lcd_trace_totals);
        }
    }
    return lcdtrace == 1;
}

/*
 * The last PC the CPU synchronised, which under TCG is the head of the
 * translation block that is running -- accel/tcg/cputlb.c:io_prepare() does not
 * call cpu_restore_state(), so this is NOT the address of the store. It is the
 * same granularity as -d in_asm and useful for the same thing: naming the
 * function. Printed as "pc~" to say so.
 */
static uint32_t lcd_trace_pc(void)
{
    CPUState *cs = current_cpu;
    return cs ? (uint32_t)CPU_GET_CLASS(cs)->get_pc(cs) : 0;
}

/*
 * Running totals, reported by LCDTRACE. Never read by the model itself.
 * The per-write lines are throttled, so these are the only exact counts.
 */
static uint64_t lcd_wdata_writes;
static uint64_t lcd_wcmd_writes;
static uint64_t lcd_pixels_total;
static uint64_t lcd_frame_starts;
static uint64_t lcd_composed_frames;

static void lcd_trace_totals(void)
{
    fprintf(stderr, "LCDTRACE  totals: %" PRIu64 " writes to the command port "
            "+0x04, %" PRIu64 " to the data port +0x40..0x5C, of which %"
            PRIu64 " were pixels; %" PRIu64 " frame-engine starts (+0x70); %"
            PRIu64 " frames composed from the display pipe\n",
            lcd_wcmd_writes, lcd_wdata_writes, lcd_pixels_total,
            lcd_frame_starts, lcd_composed_frames);
}

/* ------------------------------------------------------------------ panel */

/* ILI934x-family 8-bit command set. Only the three the guest emits. */
#define PANEL_CMD_DISPOFF   0x28
#define PANEL_CMD_DISPON    0x29
#define PANEL_CMD_CASET     0x2A
#define PANEL_CMD_PASET     0x2B
#define PANEL_CMD_RAMWR     0x2C

static void panel_command(IPodClassicLCDState *s, uint8_t cmd)
{
    s->cmd = cmd;
    s->param_n = 0;

    switch (cmd) {
    case PANEL_CMD_CASET:
    case PANEL_CMD_PASET:
        s->phase = S5L8702_PANEL_PARAMS;
        s->param_want = 4;
        break;

    case PANEL_CMD_RAMWR:
        s->phase = S5L8702_PANEL_PIXELS;
        s->param_want = 0;
        s->cur_x = s->win_x0;
        s->cur_y = s->win_y0;
        s->pixels_this_frame = 0;
        break;

    case PANEL_CMD_DISPON:
        s->display_on = true;
        s->phase = S5L8702_PANEL_IDLE;
        s->param_want = 0;
        break;

    case PANEL_CMD_DISPOFF:
        s->display_on = false;
        s->phase = S5L8702_PANEL_IDLE;
        s->param_want = 0;
        break;

    default:
        /*
         * Any other command: we do not know how many parameters it takes, so
         * the safe thing is to swallow the data writes that follow rather than
         * mistake them for pixels. PARAMS with param_want 0 does exactly that.
         */
        s->phase = S5L8702_PANEL_PARAMS;
        s->param_want = 0;
        qemu_log_mask(LOG_UNIMP, "s5l8702.lcd: panel command 0x%02x is not "
                      "modelled; its parameters will be dropped\n", cmd);
        break;
    }
}

static void panel_params_done(IPodClassicLCDState *s)
{
    uint16_t a = (s->param[0] << 8) | s->param[1];
    uint16_t b = (s->param[2] << 8) | s->param[3];

    if (a > b) {
        uint16_t t = a; a = b; b = t;
    }

    if (s->cmd == PANEL_CMD_CASET) {
        s->win_x0 = MIN(a, S5L8702_PANEL_WIDTH - 1);
        s->win_x1 = MIN(b, S5L8702_PANEL_WIDTH - 1);
    } else {
        s->win_y0 = MIN(a, S5L8702_PANEL_HEIGHT - 1);
        s->win_y1 = MIN(b, S5L8702_PANEL_HEIGHT - 1);
    }
    s->phase = S5L8702_PANEL_IDLE;
}

static void panel_pixel(IPodClassicLCDState *s, uint16_t px)
{
    if (s->cur_y < S5L8702_PANEL_HEIGHT && s->cur_x < S5L8702_PANEL_WIDTH) {
        s->fb[s->cur_y * S5L8702_PANEL_WIDTH + s->cur_x] = px;
        s->dirty = true;
        /*
         * A whole panel's worth of pixels means the guest is driving the panel,
         * not that the boot screen is gone on the strength of one stray write.
         */
        if (s->boot_screen_pixels < S5L8702_PANEL_WIDTH * S5L8702_PANEL_HEIGHT) {
            s->boot_screen_pixels++;
            if (s->boot_screen_pixels ==
                S5L8702_PANEL_WIDTH * S5L8702_PANEL_HEIGHT) {
                qatomic_set(&s->boot_screen_active, 0);
            }
        }
    }
    s->pixels_this_frame++;
    lcd_pixels_total++;

    if (s->cur_x >= s->win_x1) {
        s->cur_x = s->win_x0;
        if (s->cur_y >= s->win_y1) {
            /*
             * Wrap back to the top of the window. INFERRED, not measured: it
             * is what ILI934x-family parts do, and no guest we have ever
             * overruns the window, so nothing in this tree can tell it apart
             * from clamping.
             */
            s->cur_y = s->win_y0;
        } else {
            s->cur_y++;
        }
    } else {
        s->cur_x++;
    }
}

static void panel_data(IPodClassicLCDState *s, uint32_t value)
{
    switch (s->phase) {
    case S5L8702_PANEL_PARAMS:
        if (s->param_n < s->param_want) {
            s->param[s->param_n++] = value & 0xff;
            if (s->param_n == s->param_want) {
                panel_params_done(s);
            }
        }
        break;

    case S5L8702_PANEL_PIXELS:
        /*
         * In frame mode (LCD_CON = LCD_MODE_P16, which the guest writes at
         * 0x080b7010 straight after the 0x2C) the port carries one 16-bit
         * pixel per access. A 32-bit store therefore delivers one pixel in its
         * low half; if the high half is ever non-zero this assumption is wrong
         * and the log below says so.
         */
        if (value >> 16) {
            static bool warned;
            if (!warned) {
                warned = true;
                qemu_log_mask(LOG_UNIMP, "s5l8702.lcd: data word 0x%08x has a "
                              "non-zero high half; the model treats the data "
                              "port as one 16-bit pixel per access\n", value);
            }
        }
        panel_pixel(s, value & 0xffff);
        break;

    default:
        break;
    }
}

/*
 * The frame engine: the guest writes the geometry to +0x74 and a 1 to +0x70,
 * then spins on +0x8C bits [1:0] until they clear. We report "done"
 * immediately, which is what the previous read-as-zero behaviour did.
 *
 * Where the frame comes from is no longer a mystery -- it is composed by the
 * display pipe at 0x38900000 out of layers in DRAM (ipod_classic_disp.h). What
 * is still UNDETERMINED is the TIMING: whether +0x70 latches one frame or just
 * enables a continuous scan-out. Measured, a boot with a disk writes +0x70 once
 * in 45 s and then keeps re-pointing the layer descriptors as the user
 * interface animates, so latching here would freeze the picture on the first
 * frame. The model therefore composes in gfx_update instead and this write only
 * gets counted -- see ipod_classic_lcd_gfx_update() for that argument in full.
 */
/* Drop the frame-done line; see lcd_frame_irq() for the shape and why. */
static void lcd_irq_off(void *opaque)
{
    IPodClassicLCDState *s = opaque;

    if (s->irq_pending) {
        s->irq_pending = false;
        qemu_irq_lower(s->irq);
    }
}

static bool lcd_frame_irq_on(void)
{
    static int on = -1;
    if (on < 0) {
        const char *e = getenv("LCD_FRAME_IRQ");
        on = (e && *e && *e != '0') ? 1 : 0;
        if (on) {
            fprintf(stderr, "LCD_FRAME_IRQ: el motor de fotograma avisa al "
                    "terminar\n");
        }
    }
    return on == 1;
}

/*
 * Report the frame as done.
 *
 * Our frame engine finishes inside the store that starts it -- the picture is
 * composed from the display pipe on every refresh -- so "done" is now. What is
 * NOT known is how the guest acknowledges: no ack register has been identified
 * in this block. So the line is a short pulse rather than a level held until
 * someone clears it, the same shape and for the same reason as the SM1 engine
 * interrupt: a level nobody knows how to drop wedges the handler, and that is
 * a measured failure mode in this tree, not a hypothetical one.
 *
 * Gated on LCD_FRAME_IRQ so the default build stays exactly as measured.
 */
static void lcd_frame_irq(IPodClassicLCDState *s)
{
    if (!lcd_frame_irq_on() || !s->irq_off || s->irq_pending) {
        return;
    }
    s->irq_pending = true;
    qemu_irq_raise(s->irq);
    timer_mod(s->irq_off, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 200000);
}

static void panel_frame_go(IPodClassicLCDState *s, uint32_t value)
{
    if (value == 0) {
        return;
    }
    lcd_frame_irq(s);
    /*
     * Counted here, after the zero-write filter, so the number printed by
     * lcd_trace_totals() is starts and not writes to +0x70. Every +0x70 write
     * measured so far is a 1, so the two agree today; they would not if the
     * driver ever cleared the bit.
     */
    lcd_frame_starts++;
    /*
     * Instrumentation only, and gated on LCDTRACE on purpose: stderr is the
     * channel the guest firmware itself talks on (it prints
     * "CPU_exception_handler" there through semihosting), and lesson 1 of
     * wiki/09-historia-5g.md is to keep that channel readable.
     */
    if (lcd_trace_on()) {
        fprintf(stderr, "LCDTRACE  frame engine started (+0x70 = 0x%x), "
                "geometry +0x74 = 0x%08x (%ux%u), window %u,%u..%u,%u, "
                "%" PRIu64 " pixels have arrived through the data port since "
                "the last 0x2C\n", value, s->fr_geom, s->fr_geom >> 16,
                s->fr_geom & 0xffff, s->win_x0, s->win_y0, s->win_x1,
                s->win_y1, s->pixels_this_frame);
    }
    qemu_log_mask(LOG_UNIMP, "s5l8702.lcd: frame engine start (+0x70 = 0x%x) "
                  "does not latch a frame here; the picture is composed from "
                  "the display pipe at 0x38900000 on every refresh instead. "
                  "Whether real hardware latches on this write is "
                  "UNDETERMINED\n", value);
}

/* ------------------------------------------------------------------- mmio */

static uint64_t ipod_classic_lcd_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    IPodClassicLCDState *s = opaque;

    if (lcd_trace_on() && offset != S5L8702_LCD_STATUS) {
        fprintf(stderr, "LCDTRACE  rd +0x%02" HWADDR_PRIx "            pc~0x%08x\n",
                offset, lcd_trace_pc());
    }

    switch (offset) {
    case S5L8702_LCD_STATUS:
        /*
         * Always idle and never busy. Every transfer we accept completes
         * inside the store that started it, so there is no window in which
         * the FIFO could be full -- and reporting busy here, or failing to
         * report idle, is an unbreakable spin in both drivers.
         */
        return S5L8702_LCD_STATUS_RESET |
               (s->dbuff_valid ? S5L8702_LCD_STATUS_RDATA : 0);

    case S5L8702_LCD_DBUFF:
        /*
         * Rockbox reads this as "*buf++ = LCD_DBUFF >> 1", so the panel's byte
         * sits above bit 0. We have no panel read path, so this is zero; the
         * shift is the driver's business either way.
         */
        s->dbuff_valid = false;
        return s->dbuff;

    case S5L8702_LCD_CON:
        return s->con_reg;
    case S5L8702_LCD_INTCON:
        return s->intcon;
    case S5L8702_LCD_PHTIME:
        return s->phtime;
    case S5L8702_LCD_RST_TIME:
        return s->rst_time;
    case S5L8702_LCD_DRV_RST:
        return s->drv_rst;
    case S5L8702_LCD_RCMD:
        return s->rcmd;

    case S5L8702_LCD_FR_GEOM:
        return s->fr_geom;
    case S5L8702_LCD_FR_UNK78:
        return s->fr_unk78;
    case S5L8702_LCD_FR_UNK7C:
        return s->fr_unk7c;
    case S5L8702_LCD_FR_HOLD:
        return s->fr_hold;
    case S5L8702_LCD_FR_UNK88:
        return s->fr_unk88;

    case S5L8702_LCD_FR_GO:
        /*
         * CONVENIENCE VALUE, not a measurement: no guest we have traced ever
         * reads +0x70, so nothing constrains this. Zero means "not running",
         * which is the only answer consistent with the frame engine finishing
         * inside the store that started it.
         */
        return 0;

    case S5L8702_LCD_FR_STATUS:
        /*
         * "while (LCD[0x8c] & 3);" at 0x080a4090 and 0x080b0128, both with no
         * timeout. Bits [1:0] are transfer-in-progress and must read zero.
         */
        return 0;

    default:
        qemu_log_mask(LOG_UNIMP, "%s: read of +0x%" HWADDR_PRIx "\n", __func__,
                      offset);
        return 0;
    }
}

static void ipod_classic_lcd_write(void *opaque, hwaddr offset, uint64_t value,
                                   unsigned size)
{
    IPodClassicLCDState *s = opaque;

    if (lcd_trace_on()) {
        if (offset >= S5L8702_LCD_WDATA && offset <= S5L8702_LCD_WDATA_LAST) {
            /*
             * Only the first 64 are printed in full: a driver that gets going
             * pushes them by the hundred thousand. The running total is
             * printed on every power of two so the log always says how many.
             */
            uint64_t n = lcd_wdata_writes + 1;
            if (n <= 64 || (n & (n - 1)) == 0) {
                fprintf(stderr, "LCDTRACE  wr +0x%02" HWADDR_PRIx " = 0x%08x  "
                        "DATA[%" PRIu64 "]  pc~0x%08x\n", offset,
                        (uint32_t)value, n, lcd_trace_pc());
            }
        } else {
            fprintf(stderr, "LCDTRACE  wr +0x%02" HWADDR_PRIx " = 0x%08x  %s"
                    "pc~0x%08x\n", offset, (uint32_t)value,
                    offset == S5L8702_LCD_WCMD ? "CMD           " : "              ",
                    lcd_trace_pc());
        }
    }

    if (offset >= S5L8702_LCD_WDATA && offset <= S5L8702_LCD_WDATA_LAST) {
        lcd_wdata_writes++;
        panel_data(s, value);
        return;
    }

    switch (offset) {
    case S5L8702_LCD_CON:
        s->con_reg = value;
        break;

    case S5L8702_LCD_WCMD:
        lcd_wcmd_writes++;
        panel_command(s, value & 0xff);
        break;

    case S5L8702_LCD_RCMD:
        s->rcmd = value;
        break;

    case S5L8702_LCD_RDATA:
        /*
         * Writing here starts a read cycle; the result appears in LCD_DBUFF
         * and bit 0 of LCD_STATUS goes up. With no panel read path the result
         * is zero, but the handshake still has to complete or the driver waits
         * for ever at lcd-s5l8702.c:241.
         */
        s->dbuff = 0;
        s->dbuff_valid = true;
        break;

    case S5L8702_LCD_INTCON:
        s->intcon = value;
        break;
    case S5L8702_LCD_PHTIME:
        s->phtime = value;
        break;
    case S5L8702_LCD_RST_TIME:
        s->rst_time = value;
        break;
    case S5L8702_LCD_DRV_RST:
        s->drv_rst = value;
        break;

    case S5L8702_LCD_FR_GEOM:
        s->fr_geom = value;
        break;
    case S5L8702_LCD_FR_UNK78:
        s->fr_unk78 = value;
        break;
    case S5L8702_LCD_FR_UNK7C:
        s->fr_unk7c = value;
        break;
    case S5L8702_LCD_FR_HOLD:
        s->fr_hold = value;
        break;
    case S5L8702_LCD_FR_UNK88:
        s->fr_unk88 = value;
        break;

    case S5L8702_LCD_FR_GO:
        panel_frame_go(s, value);
        break;

    case S5L8702_LCD_STATUS:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: LCD_STATUS is read-only\n",
                      __func__);
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "%s: write of 0x%" PRIx64 " to +0x%"
                      HWADDR_PRIx "\n", __func__, value, offset);
        break;
    }
}

static const MemoryRegionOps ipod_classic_lcd_ops = {
    .read = ipod_classic_lcd_read,
    .write = ipod_classic_lcd_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    /*
     * The retailOS touches this block only in 32-bit words, but Rockbox does
     * not: it pushes the frame as one pixel per 16-bit access to the data port,
     * driven by DMAC0 (lcd-s5l8702.c displaylcd_dma(), dwidth = WIDTH_16 to
     * S5L8702_DADDR_PERI_LCD_WR = LCD_BASE+0x40). With a 4-byte floor every one
     * of those DMA writes was rejected as "invalid size" and 0 pixels reached
     * the panel -- MEASURED: the screen stayed black while the driver ran a full
     * update. Accept halfword accesses too; panel_data() already takes one pixel
     * from the low 16 bits, so a 16-bit or a 32-bit write both deliver one pixel.
     * impl is pinned to [2,4] so the core hands the handler the guest's real
     * access size and never folds a 16-bit pixel store into a read-modify-write.
     */
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
    .impl.min_access_size = 2,
    .impl.max_access_size = 4,
};

/* ---------------------------------------------------------------- console */

/*
 * Paint the boot screen into the panel's image memory.  Called from the panel's
 * own reset, so the panel comes up on the boot screen exactly when it would
 * come up on whatever the bootrom/bootloader left there; the machine turns the
 * flag on for the paths that start a kernel directly and so skip that chain.
 *
 * The picture is the firmware's own: the 320x240 RGB565 boot graphic from the
 * 2.0.5 image, copied straight into image memory.  Our panel's framebuffer has
 * the same 5-6-5 layout and byte order, so this is byte-for-byte what the
 * hardware's boot graphics carry -- no reconstruction and no dither of ours.
 */
static void ipod_classic_lcd_paint_boot_screen(IPodClassicLCDState *s)
{
    memcpy(s->fb, ipod_boot_screen, sizeof(s->fb));
    s->dirty = true;
    s->boot_screen_pixels = 0;
    qatomic_set(&s->boot_screen_active, 1);
}

void ipod_classic_lcd_set_boot_screen(IPodClassicLCDState *s, bool on)
{
    s->boot_screen = on;
}

/*
 * Compose the panel from the display pipe and the HLE overlay, and report
 * whether a new frame is available.  Split out for the browser build, which
 * composes on its own timer and has no console frontend to update.
 */
static bool ipod_classic_lcd_compose(IPodClassicLCDState *s)
{

    /*
     * Ask the display pipe for a frame before showing anything.
     *
     * WHY HERE and not only on the "go" write to +0x70. The retailOS writes
     * +0x70 = 1 once per boot with a disk attached (MEASURED: LCDTRACE reports
     * one frame-engine start in 45 seconds) and then keeps re-pointing the
     * layer descriptors as the user interface animates, without touching +0x70
     * again. So +0x70 is not a per-frame trigger on this path, and a model that
     * only composed there would show one stale frame. Composing at refresh time
     * is the behaviour of an engine that scans its layers out continuously,
     * which is what the register set looks like -- but it IS a modelling
     * choice, not a measurement, and it is the one to revisit if the display
     * ever looks a frame behind.
     *
     * Nothing composes when no layer has a source address, so the panel's own
     * image memory -- and with it tools/ipod_classic/lcdtest.S, the control
     * case that proves this panel works -- behaves exactly as before. Once a
     * layer does have one, the composed frame OVERWRITES anything that arrived
     * through the data port. That ordering is a model choice too, and it is
     * untested because no guest we have traced ever uses both paths: the
     * retailOS pushes zero pixels through the port, and lcdtest.S programs no
     * layers.
     */
    if (s->disp && ipod_classic_disp_compose(s->disp, s->fb,
                                             S5L8702_PANEL_WIDTH,
                                             S5L8702_PANEL_HEIGHT)) {
        s->composed_frames++;
        lcd_composed_frames++;
        s->dirty = true;
        /* Guest content is on the panel: the boot screen is gone. */
        qatomic_set(&s->boot_screen_active, 0);
    }

    /*
     * HLE Now Playing overlay (wiki/19 s11.23): paint the player UI over the
     * composed guest frame. Kept dirty every refresh while visible so the clock
     * and progress bar advance. No-op unless HLE_NP_OVERLAY is set.
     */
    if (ipod_classic_np_overlay_active()) {
        ipod_classic_np_overlay_draw(s->fb, S5L8702_PANEL_WIDTH,
                                     S5L8702_PANEL_HEIGHT);
        s->dirty = true;
    }

    if (!s->dirty) {
        return false;
    }
    s->dirty = false;
    return true;
}

static void ipod_classic_lcd_gfx_update(void *opaque)
{
    IPodClassicLCDState *s = opaque;

    if (ipod_classic_lcd_compose(s)) {
        dpy_gfx_update_full(s->con);
    }
}

static void ipod_classic_lcd_invalidate(void *opaque)
{
    IPodClassicLCDState *s = opaque;

    s->dirty = true;
}

static const GraphicHwOps ipod_classic_lcd_gfx_ops = {
    .invalidate = ipod_classic_lcd_invalidate,
    .gfx_update = ipod_classic_lcd_gfx_update,
};

#ifdef EMSCRIPTEN
#include <emscripten.h>

/*
 * The browser build runs with -display none, so no UI frontend drives the
 * console's refresh and the composed frame would never be made (the panel's
 * framebuffer is filled by ipod_classic_lcd_gfx_update, which the refresh
 * loop calls).  Compose at 30 Hz from a real-time timer instead -- it runs
 * with the BQL held, like the vCPU's device accesses -- and publish the
 * 320x240 RGB565 frame to the page, which polls the sequence number.
 *
 * The compose target (s->fb) is not what the page reads: two published
 * buffers are compared with the new frame and one is only replaced when the
 * picture really changed (Q17).  That keeps the sequence number still on a
 * still screen -- the page then skips the conversion entirely -- and stops the
 * page from reading a frame while it is being drawn.
 *
 * This is display-only: the model's behaviour and the guest's view of the
 * panel are unchanged.
 */
static IPodClassicLCDState *wasm_lcd_state;
static uint16_t *wasm_lcd_pub[2];
static int wasm_lcd_pub_idx;

/*
 * What the page sees while the guest has the panel off.  Rockbox sends Display
 * Off 20 s after the last input (15 s of backlight, 5 s to LCD sleep) and then
 * pushes no pixels until it wakes, so without this the page keeps the last frame
 * and a sleeping iPod looks like a hung one: the hardware's screen is dark.
 *
 * Only the published frame changes.  s->fb is the panel's image memory, which
 * survives a display-off and which the guest does not repaint unless it has
 * something new to show, so it has to be there when the panel comes back.
 */
static uint16_t *wasm_lcd_black;

/*
 * Whether the frame in wasm_lcd_pub[i] is still the boot screen, published with
 * that frame so a reader never gets a state newer than the pixels it is judging
 * (ipod_wasm_lcd_boot_screen_active() indexes it by the published one).
 */
static uint32_t wasm_lcd_pub_boot_screen[2];

static void wasm_lcd_frame_tick(void *opaque)
{
    IPodClassicLCDState *s = opaque;
    const size_t bytes = S5L8702_PANEL_WIDTH * S5L8702_PANEL_HEIGHT * 2;
    int cur = qatomic_read(&wasm_lcd_pub_idx);
    const uint16_t *frame;

    /*
     * 30 Hz is enough for a 320x240 panel and halves the time the compose
     * spends holding the BQL against the vCPU; skipping dpy_gfx_update_full
     * keeps the timer off the console, which has no frontend here anyway.
     */
    ipod_classic_lcd_compose(s);
    frame = s->display_on ? s->fb : wasm_lcd_black;

    if (memcmp(frame, wasm_lcd_pub[cur], bytes) != 0) {
        int next = cur ^ 1;
        memcpy(wasm_lcd_pub[next], frame, bytes);
        wasm_lcd_pub_boot_screen[next] =
            qatomic_read(&s->boot_screen_active);
        qatomic_set(&wasm_lcd_pub_idx, next);
        qatomic_store_release(&s->wasm_seq,
                              qatomic_read(&s->wasm_seq) + 1);
    }
    timer_mod(s->wasm_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) +
                             NANOSECONDS_PER_SECOND / 30);
}

EMSCRIPTEN_KEEPALIVE uint32_t ipod_wasm_lcd_width(void)
{
    return S5L8702_PANEL_WIDTH;
}

EMSCRIPTEN_KEEPALIVE uint32_t ipod_wasm_lcd_height(void)
{
    return S5L8702_PANEL_HEIGHT;
}

EMSCRIPTEN_KEEPALIVE uint32_t ipod_wasm_lcd_stride(void)
{
    return S5L8702_PANEL_WIDTH * 2;
}

EMSCRIPTEN_KEEPALIVE uint32_t ipod_wasm_lcd_seq(void)
{
    return wasm_lcd_state ? qatomic_read(&wasm_lcd_state->wasm_seq) : 0;
}

/*
 * True while the panel is still showing the boot screen painted at reset --
 * i.e. until the guest first composes or has pushed a full panel's worth of
 * pixels.  The page uses this to tell "the boot screen is up" from "the
 * firmware has drawn something", which matters for Rockbox: its bootloader runs
 * down the same -kernel path and its readiness check must not mistake the static
 * boot screen for its own output.
 */
EMSCRIPTEN_KEEPALIVE uint32_t ipod_wasm_lcd_boot_screen_active(void)
{
    int idx;

    if (!wasm_lcd_state) {
        return 0;
    }
    idx = qatomic_read(&wasm_lcd_pub_idx);
    return wasm_lcd_pub_boot_screen[idx];
}

EMSCRIPTEN_KEEPALIVE uint32_t ipod_wasm_lcd_ptr(void)
{
    int idx;
    if (!wasm_lcd_state) {
        return 0;
    }
    idx = qatomic_read(&wasm_lcd_pub_idx);
    return (uint32_t)(uintptr_t)wasm_lcd_pub[idx];
}
#endif

static void ipod_classic_lcd_reset(DeviceState *dev)
{
    IPodClassicLCDState *s = IPOD_CLASSIC_LCD(dev);

    s->con_reg = 0;
    s->intcon = 0;
    /* s5l87xx.h records the reset values of these two as 0x0060 and 0x07FF. */
    s->phtime = 0x0060;
    s->rst_time = 0x07FF;
    s->drv_rst = 0;
    s->dbuff = 0;
    s->dbuff_valid = false;
    s->rcmd = 0;

    s->fr_geom = 0;
    s->fr_unk78 = 0;
    s->fr_unk7c = 0;
    s->fr_hold = 0;
    s->fr_unk88 = 0;
    s->composed_frames = 0;

    memset(s->fb, 0, sizeof(s->fb));
    if (s->boot_screen) {
        /* The panel starts on the boot screen, see ipod_classic_lcd_paint_boot_screen(). */
        ipod_classic_lcd_paint_boot_screen(s);
    }
    s->win_x0 = 0;
    s->win_y0 = 0;
    s->win_x1 = S5L8702_PANEL_WIDTH - 1;
    s->win_y1 = S5L8702_PANEL_HEIGHT - 1;
    s->cur_x = 0;
    s->cur_y = 0;
    s->phase = S5L8702_PANEL_IDLE;
    s->cmd = 0;
    s->param_n = 0;
    s->param_want = 0;
    s->pixels_this_frame = 0;
    /*
     * The guest never sends 0x29 (display on) in any boot measured so far, and
     * a panel that stays dark until it does would hide real pixels behind an
     * unmodelled command. We show whatever is in image memory. NOTE: this is a
     * convenience value, not a measurement.
     */
    s->display_on = true;
    s->dirty = true;
}

static void ipod_classic_lcd_realize(DeviceState *dev, Error **errp)
{
    IPodClassicLCDState *s = IPOD_CLASSIC_LCD(dev);
    DisplaySurface *surface;

    memory_region_init_io(&s->iomem, OBJECT(s), &ipod_classic_lcd_ops, s,
                          TYPE_IPOD_CLASSIC_LCD, S5L8702_LCD_MEM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    /* Only armed when LCD_FRAME_IRQ is set; see lcd_frame_irq(). */
    s->irq_off = timer_new_ns(QEMU_CLOCK_VIRTUAL, lcd_irq_off, s);

    s->con = graphic_console_init(dev, 0, &ipod_classic_lcd_gfx_ops, s);
    qemu_console_resize(s->con, S5L8702_PANEL_WIDTH, S5L8702_PANEL_HEIGHT);
    surface = qemu_create_displaysurface_from(S5L8702_PANEL_WIDTH,
                                              S5L8702_PANEL_HEIGHT,
                                              PIXMAN_r5g6b5,
                                              S5L8702_PANEL_WIDTH * 2,
                                              (uint8_t *)s->fb);
    dpy_gfx_replace_surface(s->con, surface);

#ifdef EMSCRIPTEN
    wasm_lcd_state = s;
    wasm_lcd_pub[0] = g_malloc0(S5L8702_PANEL_WIDTH *
                                S5L8702_PANEL_HEIGHT * 2);
    wasm_lcd_pub[1] = g_malloc0(S5L8702_PANEL_WIDTH *
                                S5L8702_PANEL_HEIGHT * 2);
    wasm_lcd_black = g_malloc0(S5L8702_PANEL_WIDTH *
                               S5L8702_PANEL_HEIGHT * 2);
    wasm_lcd_pub_idx = 0;
    s->wasm_timer = timer_new_ns(QEMU_CLOCK_REALTIME, wasm_lcd_frame_tick, s);
    timer_mod(s->wasm_timer, qemu_clock_get_ns(QEMU_CLOCK_REALTIME) +
                             NANOSECONDS_PER_SECOND / 30);
#endif
}

static const VMStateDescription vmstate_ipod_classic_lcd = {
    .name = TYPE_IPOD_CLASSIC_LCD,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(con_reg, IPodClassicLCDState),
        VMSTATE_UINT32(intcon, IPodClassicLCDState),
        VMSTATE_UINT32(phtime, IPodClassicLCDState),
        VMSTATE_UINT32(rst_time, IPodClassicLCDState),
        VMSTATE_UINT32(drv_rst, IPodClassicLCDState),
        VMSTATE_UINT32(dbuff, IPodClassicLCDState),
        VMSTATE_BOOL(dbuff_valid, IPodClassicLCDState),
        VMSTATE_UINT32(rcmd, IPodClassicLCDState),
        VMSTATE_UINT32(fr_geom, IPodClassicLCDState),
        VMSTATE_UINT32(fr_unk78, IPodClassicLCDState),
        VMSTATE_UINT32(fr_unk7c, IPodClassicLCDState),
        VMSTATE_UINT32(fr_hold, IPodClassicLCDState),
        VMSTATE_UINT32(fr_unk88, IPodClassicLCDState),
        VMSTATE_UINT16_ARRAY(fb, IPodClassicLCDState,
                             S5L8702_PANEL_WIDTH * S5L8702_PANEL_HEIGHT),
        VMSTATE_UINT16(win_x0, IPodClassicLCDState),
        VMSTATE_UINT16(win_x1, IPodClassicLCDState),
        VMSTATE_UINT16(win_y0, IPodClassicLCDState),
        VMSTATE_UINT16(win_y1, IPodClassicLCDState),
        VMSTATE_UINT16(cur_x, IPodClassicLCDState),
        VMSTATE_UINT16(cur_y, IPodClassicLCDState),
        VMSTATE_UINT8(phase, IPodClassicLCDState),
        VMSTATE_UINT8(cmd, IPodClassicLCDState),
        VMSTATE_UINT8_ARRAY(param, IPodClassicLCDState, 8),
        VMSTATE_UINT8(param_n, IPodClassicLCDState),
        VMSTATE_UINT8(param_want, IPodClassicLCDState),
        VMSTATE_BOOL(display_on, IPodClassicLCDState),
        VMSTATE_END_OF_LIST()
    },
};

static void ipod_classic_lcd_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "S5L8702 LCD interface";
    dc->realize = ipod_classic_lcd_realize;
    dc->reset = ipod_classic_lcd_reset;
    dc->vmsd = &vmstate_ipod_classic_lcd;
    dc->user_creatable = false;
}

static const TypeInfo ipod_classic_lcd_info = {
    .name          = TYPE_IPOD_CLASSIC_LCD,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodClassicLCDState),
    .class_init    = ipod_classic_lcd_class_init,
};

static void ipod_classic_lcd_register_types(void)
{
    type_register_static(&ipod_classic_lcd_info);
}

type_init(ipod_classic_lcd_register_types)
