/*
 * S5L8702 LCD interface, 0x38300000 (LCD_BASE).
 *
 * This is the MPU/8080-style command port that talks to the panel controller.
 * It is not a scanout engine: there is no framebuffer address here and no
 * video timing. Software hands it commands and pixel words and the panel keeps
 * its own image memory. The separate block at 0x39200000 (LCDBASE in Rockbox,
 * "CLCD CONTROLLER") is the one with frame buffer start/end addresses, and the
 * retailOS does not appear to use it on this machine.
 *
 * Register names and offsets from Rockbox, firmware/export/s5l87xx.h, the
 * "CONFIG_CPU==S5L8702 || CONFIG_CPU==S5L8720" arm of the #if at line 1473.
 * Note that the same header defines LCD_BASE as 0x3C100000 for the S5L8700 --
 * a different chip, and not to be confused with the unnamed block this machine
 * maps at that address.
 *
 * The status bits are the part that matters, because every one of them is a
 * spin with no timeout on both sides. Rockbox and Apple's retailOS agree on
 * all three, which is what makes them safe to build on:
 *
 *   bit 0  read data in LCD_DBUFF is valid
 *          rockbox lcd-s5l8702.c:241  "while (!(LCD_STATUS & 1));"
 *   bit 1  interface idle, a control write may proceed
 *          rockbox lcd-s5l8702.c:207  "while (!(LCD_STATUS & 0x2));"
 *          retailOS 0x080b6f98, 0x080b6ffc  same loop, then LCD_CON = x
 *   bit 4  write FIFO busy, do not push another word
 *          rockbox lcd-s5l8702.c:215  "while (LCD_STATUS & 0x10);"
 *          retailOS 0x080a90b4         same loop
 *
 * s5l87xx.h records the reset value as 0x0106, i.e. bits 1, 2 and 8 set and
 * bit 4 clear -- idle and not busy, which is consistent with the above. What
 * bits 2 and 8 mean is UNDETERMINED; no driver we have tests them.
 */

#ifndef HW_ARM_IPOD_CLASSIC_LCD_H
#define HW_ARM_IPOD_CLASSIC_LCD_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "ui/console.h"

#define TYPE_IPOD_CLASSIC_LCD "s5l8702.lcd"
OBJECT_DECLARE_SIMPLE_TYPE(IPodClassicLCDState, IPOD_CLASSIC_LCD)

/*
 * Rockbox names registers up to LCD_WDATA at 0x40, "0x40...0x5C FIXME", but
 * the block is wider than Rockbox knows: the retailOS writes 0x3830007C and
 * 0x38300088 during its very first LCD setup, before it touches anything
 * Rockbox names. Those two are UNDETERMINED -- see ipod_classic_lcd.c -- but
 * the window has to cover them or they fall through to the machine's io floor.
 */
#define S5L8702_LCD_MEM_SIZE    0x100

#define S5L8702_LCD_CON         0x00
#define S5L8702_LCD_WCMD        0x04
#define S5L8702_LCD_RCMD        0x0C
#define S5L8702_LCD_RDATA       0x10
#define S5L8702_LCD_DBUFF       0x14
#define S5L8702_LCD_INTCON      0x18
#define S5L8702_LCD_STATUS      0x1C
#define S5L8702_LCD_PHTIME      0x20
#define S5L8702_LCD_RST_TIME    0x24
#define S5L8702_LCD_DRV_RST     0x28
#define S5L8702_LCD_WDATA       0x40   /* 0x40..0x5C all write pixel data */
#define S5L8702_LCD_WDATA_LAST  0x5C

/*
 * The upper half of the block, which no source outside Apple's own code names.
 * Every one of these is read off the retailOS disassembly; see the long note in
 * ipod_classic_lcd.c for the sites.
 *
 *   +0x70  written 1 at 0x08127f94, right after the geometry below -> "go"
 *   +0x74  written (width << 16) | height at 0x08127f8c
 *   +0x78  written 0x0005000a at 0x08127f3c                    UNDETERMINED
 *   +0x7C  written 0x00000804 at 0x080afb1c                    UNDETERMINED
 *   +0x80  written 1 then 0 around a full-screen window set
 *          (0x0808b000/0x0808b030, 0x080a0ef8/0x080a0f28)      INFERRED: hold
 *   +0x88  written 0x01000000 at 0x080afb0c                    UNDETERMINED
 *   +0x8C  "while (LCD[0x8c] & 3);" at 0x080a4090 and 0x080b0128 -- a spin with
 *          no timeout, so bits [1:0] are "transfer in progress" and they MUST
 *          read back zero or the guest never leaves those two loops.
 */
#define S5L8702_LCD_FR_GO       0x70
#define S5L8702_LCD_FR_GEOM     0x74
#define S5L8702_LCD_FR_UNK78    0x78
#define S5L8702_LCD_FR_UNK7C    0x7C
#define S5L8702_LCD_FR_HOLD     0x80
#define S5L8702_LCD_FR_UNK88    0x88
#define S5L8702_LCD_FR_STATUS   0x8C

#define S5L8702_LCD_STATUS_RDATA   (1u << 0)
#define S5L8702_LCD_STATUS_IDLE    (1u << 1)
#define S5L8702_LCD_STATUS_BUSY    (1u << 4)
/* Bits 2 and 8 of the 0x0106 reset value; meaning UNDETERMINED. */
#define S5L8702_LCD_STATUS_RESET   0x0106

/*
 * The panel. 320x240 is not a guess: the three panel driver records the
 * retailOS picks between (0x08911f90, 0x08911fb8, 0x08911fe0) all start with
 * 0x00f00140 = (240 << 16) | 320, the guest sets the window to 0..0x13f by
 * 0..0xef, and it writes (320 << 16) | 240 to +0x74. Rockbox agrees
 * (firmware/export/config/ipod6g.h, LCD_WIDTH/LCD_HEIGHT).
 */
#define S5L8702_PANEL_WIDTH     320
#define S5L8702_PANEL_HEIGHT    240

/* What the data port is carrying right now. */
typedef enum {
    S5L8702_PANEL_IDLE = 0,     /* nothing asked for; writes are dropped */
    S5L8702_PANEL_PARAMS,       /* parameter bytes of the last command */
    S5L8702_PANEL_PIXELS,       /* after 0x2C: pixels into the window */
} S5L8702PanelPhase;

struct IPodClassicLCDState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QemuConsole *con;

#ifdef EMSCRIPTEN
    /* Browser build: periodic compose (no UI frontend) and frame counter. */
    QEMUTimer *wasm_timer;
    uint32_t wasm_seq;
#endif

    uint32_t con_reg;
    uint32_t intcon;
    /*
     * FRAME-DONE INTERRUPT (LCD_FRAME_IRQ=1).
     *
     * The block has an interrupt line, wired to S5L8702_LCD_IRQ in the machine,
     * and an INTCON at +0x18 that the model stores and never acts on -- so the
     * line has never been raised once. MEASURED symptom: entering Now Playing
     * starts the album-art animation and it freezes on its first frame, with
     * the CPU asleep in the idle WFI rather than spinning. An animation that
     * stops with the machine idle is waiting for something that never arrives.
     */
    QEMUTimer *irq_off;
    bool irq_pending;
    uint32_t phtime;
    uint32_t rst_time;
    uint32_t drv_rst;

    /* Result of the last read cycle, handed back through LCD_DBUFF. */
    uint32_t dbuff;
    bool dbuff_valid;
    /* Command byte of the read cycle in progress, from LCD_RCMD. */
    uint32_t rcmd;

    /* The frame-engine registers above, kept so they read back. */
    uint32_t fr_geom;
    uint32_t fr_unk78;
    uint32_t fr_unk7c;
    uint32_t fr_hold;
    uint32_t fr_unk88;

    /* Panel state. */
    uint16_t fb[S5L8702_PANEL_WIDTH * S5L8702_PANEL_HEIGHT];
    uint16_t win_x0, win_x1, win_y0, win_y1;
    uint16_t cur_x, cur_y;
    uint8_t phase;              /* S5L8702PanelPhase */
    uint8_t cmd;                /* command the parameters belong to */
    uint8_t param[8];
    uint8_t param_n;
    uint8_t param_want;
    bool display_on;
    bool dirty;

    /*
     * The display pipe at 0x38900000, or NULL. On this SoC the pixels do not
     * come through this port at all -- the retailOS pushes zero of them -- they
     * are composed by that block and pushed into the panel's image memory by
     * the frame engine at +0x70. So the panel asks it for a frame. Set by the
     * machine; see ipod_classic_disp.h for what the block is and how it was
     * found.
     */
    struct IPodClassicDispState *disp;

    /* Counters, for the trace only. */
    uint64_t pixels_this_frame;
    uint64_t composed_frames;

    /*
     * The panel comes up on the firmware's boot screen instead of black.  Set
     * by the machine for the paths that start a kernel directly (-kernel) and
     * so skip the bootrom/bootloader that would have painted it on hardware;
     * the panel's reset then paints it.  With a real bootrom the chain owns the
     * panel, so the flag stays off.
     */
    bool boot_screen;

    /*
     * Non-zero while the panel is still showing the boot screen painted at
     * reset.  Cleared when the display pipe composes, or when the guest has
     * pushed a whole panel's worth of pixels through the data port -- a single
     * stray write does not count as the guest taking the panel over.  Written
     * by the QEMU thread and read from the page thread, so it goes through
     * qatomic (the wasm LCD exports publish it with the frame; see
     * ipod_wasm_lcd_boot_screen_active()).
     */
    uint32_t boot_screen_active;

    /* Pixels pushed through the data port since the boot screen was painted. */
    uint32_t boot_screen_pixels;
};

/* Paint the boot screen on the panel's next reset (see boot_screen above). */
void ipod_classic_lcd_set_boot_screen(IPodClassicLCDState *s, bool on);

#endif /* HW_ARM_IPOD_CLASSIC_LCD_H */
