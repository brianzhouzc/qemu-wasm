/*
 * S5L8702 display pipe, 0x38900000.
 *
 * NO SOURCE WE HAVE NAMES THIS BLOCK. Rockbox's s5l87xx.h does not list it,
 * freemyipod's model has nothing there, and Rockbox never touches it -- Rockbox
 * pushes its frame through DMAC0 into LCD_WDATA instead (lcd-s5l8702.c:172).
 * Everything below comes from disassembling Apple's own osos, and from a
 * measured boot; where the two disagree the measurement wins and it is said so.
 *
 * WHY IT MATTERS. 0x08127f14, the retailOS's "draw the whole screen", ends
 * with two stores to the LCD frame engine (LCD +0x74 geometry, LCD +0x70 = 1
 * "go") and no pixel ever goes through the LCD data port. Nothing in the LCD's
 * own 256-byte window says where that frame comes from. This block does: it has
 * the panel geometry, three layer descriptors, five-plus source-address slots,
 * three 256-entry gamma tables and four colour-matrix coefficient tables. It is
 * the composition engine the LCD frame engine presents.
 *
 * ---------------------------------------------------------------- registers
 *
 * Every entry below is either (V) verified from a store or load in the osos
 * whose base register was loaded with the literal 0x38900000 -- the complete
 * list of those twenty-seven sites is in wiki/11 -- or (M) additionally
 * confirmed by the value seen in a real boot, or (U) undetermined.
 *
 *  +0x000  (M) written 1 by the init path. Meaning UNDETERMINED.
 *  +0x004  (M) written 1 by 0x0812729c. Meaning UNDETERMINED.
 *  +0x008  (V,M) THE CONTROL REGISTER. Six different functions do
 *          read-modify-write on it, one bit field each, which is why leaving
 *          this block unimplemented was actively harmful: reads returned 0, so
 *          every RMW threw away every bit set before it.
 *            bit 0      0x080a508c arg r3 == 1
 *            bit 1      0x080a511c arg != 0
 *            bits 2..7  0x081271c8, six selectors (6,5,4,3,2,7 for sel 0..5)
 *            bit 8      0x080822e8 arg == 0
 *            bit 15     0x0808dfe4 arg == 1
 *            bits 17:16 0x080a508c byte arg at sp+10 (1 or 2)
 *            bit 20     0x080a508c byte arg at sp+9 == 1
 *            bits 25:24 0x080a508c byte arg at sp+8 (1 or 2)
 *            bit 28     0x080a508c arg r1 == 1
 *            bit 29     0x080a508c arg r0 != 0
 *            bit 30     0x080be518 arg != 0
 *          MEASURED, diskless boot: the values that reach it are 0x01110001,
 *          0x00008000, 0x00000100 and 0x40000000 -- each one a single field
 *          written onto a zero readback. 0x01110001 is exactly what
 *          0x080a508c(0, 0, {1,1,1}, 1) computes, which is the call the init
 *          path makes, so the decode above is confirmed end to end.
 *  +0x00c  (M) 0x000f0f0f from a literal at 0x08127460. UNDETERMINED.
 *  +0x020  (M) written 1 next to +0x004. UNDETERMINED.
 *  +0x024  (M) the panel's BACKGROUND, as an ARGB colour: what shows where no
 *          layer lands. INFERRED: 0x00ffffff at init and 0xffffffff when a
 *          page slide starts, both from the same setter, and nothing else
 *          covers the panel when Cover Flow slides away (only a part-width
 *          layer 0 is on), whose white ground this matches.
 *  +0x028..+0x054 (V,M) LAYER 5, the planar layer; see below.
 *  +0x060  (V) SOURCE ADDRESS, slot 0  \
 *  +0x078  (V) SOURCE ADDRESS, slot 1   |  0x081276dc cases 0..4. Stride 0x18.
 *  +0x090  (V) SOURCE ADDRESS, slot 2   |  Which slot is used is chosen by the
 *  +0x0a8  (V) SOURCE ADDRESS, slot 3   |  caller, not by this block.
 *  +0x0c0  (V) SOURCE ADDRESS, slot 4  /
 *  +0x0d4  (V,M) THE STAGE ORDER, (order | b << 8); see the mixer note below.
 *          Only 0x201 and, for a back slide, 0x200 are ever written.
 *  +0x0d8  (V,M) MIXER STAGE 0, +0x0dc its second word
 *  +0x0e0  (V,M) MIXER STAGE 1, +0x0e4 its second word
 *  +0x0e8  (V,M) MIXER STAGE 2, +0x0ec its second word
 *          These are NOT one per layer, which is what an earlier version of
 *          this comment called them; see the mixer note below. The word is
 *          built as (dst factor << 28) | (src factor << 12) | global alpha.
 *          The second words stay 0 in every run; what they do is UNDETERMINED.
 *  +0x0f0..+0x17c (V) nine 4-word entries, layer 5's scaler coefficients,
 *          copied by 0x081276a0. Stored; the model scales without them.
 *  +0x180..+0x1c4 (V) nine 2-word entries, the same, copied by 0x08127664
 *  +0x1ec..+0x1fc (V,M) five words copied straight out of a struct by
 *          0x080bf110. MEASURED 0, 0, 0x10, 0, 0. UNDETERMINED.
 *  +0x200  (V,M) bit 0 set/cleared by 0x08096eb8, and OR'd with 0x00010080 by
 *          0x081273f8. Both are read-modify-write and it is genuinely read
 *          back, so a zero readback here really did lose state.
 *  +0x204  (V,M) 2, at 0x0812740c.   +0x208 (V,M) 0, at 0x08127428.
 *  +0x20c  (V,M) 2, at 0x08127410.
 *  +0x210  (V,M) 0x00010110, a literal at 0x08127464.
 *  +0x214  (V,M) 0x013f00ef -- (319 << 16) | 239, THE PANEL GEOMETRY, to the
 *          pixel, a literal at 0x08127468. Nothing else in this block is as
 *          unambiguous.
 *          These six live in the init function 0x08127268, which reaches them
 *          through r6 rather than a fresh literal, which is why scanning for
 *          pc-relative loads of 0x38900000 did not find them. The trace found
 *          them: DISPTRACE prints the guest PC with every access.
 *  +0x31c..+0x360 (V) nine 2-word entries, copied by 0x081275ec
 *  +0x364..+0x3a8 (V) nine 2-word entries, copied by 0x08127628
 *  +0x3ac  (V,M) 0x080bcef8 builds it as
 *          (w << 20) | (h << 8) | (c << 2) | (d << 1) | e, and 0x08127f54 calls
 *          it with (64, 64, 0, 0, 0) -> 0x04004000, which is exactly what the
 *          boot writes. Bit 0 is read back by the blit at five places and
 *          swaps width for height everywhere it is tested, so bit 0 is
 *          "rotated 90 degrees". INFERRED from that use, not from a datasheet.
 *  +0x400..+0x7fc (M) 256-entry gamma/palette table, channel 0
 *  +0x800..+0xbfc (M) channel 1
 *  +0xc00..+0xffc (M) channel 2
 *          0x0806fe9c is the writer and 0x08127268 builds the argument as
 *          tbl[i] = i * 4 for i in 0..255, so what the boot loads is the
 *          identity ramp, left-shifted by two. MEASURED: all 768 words, value
 *          == index * 4, in all three tables.
 *
 * Nothing above +0xfff was touched in a whole diskless boot, so the window is
 * 4 KB and the rest of the 1 MB block still falls to the machine's io floor,
 * where -d unimp will report it if that ever stops being true.
 *
 * WHAT THIS MODEL DOES: store every register honestly, and compose the layers
 * into the panel when the panel is about to be shown. See the layer-descriptor
 * and mixer notes below, and ipod_classic_disp.c for the composition itself.
 *
 * WHAT IT DOES NOT DO: the gamma tables, the scaler's coefficient tables and
 * the rotate bit in +0x3ac are stored and read back but do not affect the
 * picture. The first two are the identity in every boot measured so far -- the
 * gamma tables are literally tbl[i] = i * 4 -- so honouring them would change
 * nothing today and could only be tested against a guest that stops using the
 * identity. Layer 5 is scaled nearest-neighbour, and no screen the stock
 * firmware has shown us scales it at all (+0x04c is 0x10001000 every time).
 *
 * A boot with no disk never writes a source address into any slot at all
 * (MEASURED: zero writes to +0x038..+0x0c0 in a 40-second diskless boot,
 * because the user interface dies for want of fonts long before it composes
 * anything), so it draws nothing and the panel stays black. That is the guest's
 * behaviour, not the model's. Set DISPTRACE=1 to watch the registers.
 */

#ifndef HW_ARM_IPOD_CLASSIC_DISP_H
#define HW_ARM_IPOD_CLASSIC_DISP_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_IPOD_CLASSIC_DISP "s5l8702.disp"
OBJECT_DECLARE_SIMPLE_TYPE(IPodClassicDispState, IPOD_CLASSIC_DISP)

/* The window we answer for. Everything above this falls to the io floor. */
#define S5L8702_DISP_MEM_SIZE   0x1000
#define S5L8702_DISP_NREG       (S5L8702_DISP_MEM_SIZE / 4)

/*
 * THE FIVE LAYER DESCRIPTORS, and this is what puts pixels on the panel.
 *
 * 0x0812705c..0x0812716c is one five-way switch on the layer index (jump table
 * at 0x08127068, arms at 0x08127080, 0x081270b0, 0x081270e0, 0x08127110 and
 * 0x08127140); every arm writes the same FIVE registers at base 0x058 + k*0x18:
 *
 *   +0x00  source stride in bytes            (r2 = bpp * srcW / 2 ... see below)
 *   +0x04  (align << 24) | (fmt << 8) | ...  bit 16 set when NOT 32 bpp
 *   +0x0c  (W << 16) | H          destination size
 *   +0x10  W * bytes-per-pixel / 4          (a cross-check, not used here)
 *   +0x14  (X << 16) | Y          destination position
 *
 * The sixth word of the descriptor, and the one that matters most --
 *
 *   +0x08  SOURCE ADDRESS, already advanced to the first visible pixel
 *
 * -- is NOT written by this switch. It comes from a separate five-way switch in
 * 0x081276dc, which is why a boot can program a descriptor's geometry without
 * ever giving it something to read from.
 *
 * VERIFIED against a real boot with a disk. Layer 0:
 *   +0x058 = 0x00000500 (1280 = 320 * 4), +0x05c = 0x00000700 (fmt 7, and bit
 *   16 clear so 32 bpp), +0x060 = 0x09767a24, +0x064 = 0x014000f0 (320 x 240),
 *   +0x06c = 0. Layer 1:
 *   +0x070 = 0x00000280 (640 = 320 * 2), +0x074 = 0x02010300 (fmt 3, bit 16
 *   set so 16 bpp), +0x078 = 0x0be2326c, +0x07c = 0x006c00f0 (108 x 240),
 *   +0x084 = 0x00d40000 (x = 212, and 212 + 108 = 320 exactly).
 *
 * The bytes-per-pixel table comes from the switch at 0x08126f90, which maps the
 * format field to a multiplier r1 in {1, 2, 4, 8} and then computes the stride
 * as r1 * srcW / 2 -- so r1 is twice the bytes per pixel:
 *   fmt 0 -> 4 bpp, fmt 1 -> 8 bpp, fmt 2..5 -> 16 bpp, fmt 6..7 -> 32 bpp.
 * Both measured layers agree with it to the byte.
 *
 * W and H are SIGNED. MEASURED: while the right-hand pane slides off screen the
 * firmware writes +0x07c = 0xffc900f0, i.e. width -55. Anything with a
 * non-positive width or height, or an origin off the panel, is not drawn.
 *
 * LAYER 5, THE PLANAR LAYER. A sixth layer, with registers of its own and an
 * enable bit of its own (bit 7 of CONTROL), and the one the user interface
 * keeps its background on while a page or a pane is in motion. MEASURED: format
 * 8 every time, 320x240 at 1:1, on every screen a run through the menus reached.
 * Decoded from the 2.0.5 image; the addresses are where it runs, 0xAF98 below
 * where its file offset would put it (see the note in ipod_classic_disp.c):
 *
 *   +0x028  0x100 for format 8, YCbCr 4:2:0; 0 for 9 and 10         (0x08143d88)
 *   +0x02c  luma stride | chroma stride << 16                       (0x08143de0)
 *   +0x030  source origin, (x << 16) | y                            (0x081448a0)
 *   +0x034  source limit, ((x + w) << 16) | (y + h)                 (0x081448b8)
 *   +0x038  luma plane                                              (0x081446ac)
 *   +0x03c  Cb plane                                                (0x081446b4)
 *   +0x040  0 for format 8                                          (0x081446bc)
 *   +0x044  Cr plane                                                (0x08144704)
 *   +0x04c  source step per panel pixel, (x << 16) | y, in 4.12     (0x08144c80)
 *   +0x050  panel position, (x << 16) | y                           (0x081448d8)
 *   +0x054  panel size, (w << 16) | h, ten bits each                (0x081448d0)
 *
 * Formats 9 and 10 are the two-plane and four-plane paths at the same site. No
 * screen has used them; they are logged and skipped.
 *
 * WHICH CHROMA PLANE IS WHICH, and the colour matrix, are MEASURED. Three frames
 * of the main menu were composed by hand and compared with what the firmware
 * itself flattens into layer 0 when the same transition ends, which is the only
 * reference we have that is not this model: mean error per channel over the
 * whole panel 0.36/255 with Cb in +0x03c and BT.601 limited range, 1.8 with full
 * range, 8.4 with the chroma planes swapped.
 *
 * THE MIXER. There are three blend stages, stacked, and the layers do not own
 * them. Each INPUT goes to a stage, and there are three inputs, not six:
 *
 *     layer 0 / layers 1, 2 and 3, sharing one / layer 5
 *
 * Layer 4 has no input: the firmware's blend setter (0x0814472c) refuses it.
 * Which stage an input gets is the low byte of +0x0d4 through a table in the
 * firmware (0x083f35ec, see disp_stage_of[]), so the same layer's blend register
 * moves when the order does. That is why +0x0e8 is rewritten on every flip of
 * layer 0: at the usual order, 1, layer 0 is on stage 2.
 *
 * Stage 2 is the top. Order 1 is layer 5 at the back and layer 0 in front, and
 * the firmware switches to order 0 (layers 1..3 in front of layer 0) for a back
 * slide, where the outgoing page in layer 3 starts sliding later than the
 * incoming one in layer 0 does -- which only reads as a slide if layer 3 covers
 * layer 0 while it waits.
 *
 * WITHIN the shared input, layer 3 covers layer 2 (the firmware slides layer 3
 * off an opaque layer 2 when a page is pushed) and layer 1 covers layer 2 (a
 * finished Cover Flow frame in layer 1 is what comes out from under layer 0, not
 * the bare gradient in layer 2). Layers 1 and 3 have never overlapped, so their
 * order against each other is a guess.
 *
 * A stage register holds two factors and a global alpha, and blends
 *
 *     out = src * F(bits 15:12) + below * F(bits 31:28)
 *
 * The firmware only ever sets one of five pairs, by mode:
 *
 *     mode  src  dst     meaning                         seen
 *     0     0    1       the layer contributes nothing   no
 *     1     1    0       opaque                          at reset
 *     2     4    5       per-pixel alpha, straight       yes, formats 4, 5, 7
 *     3     8    9       global alpha                    yes, 0xff
 *     4     10   11      per-pixel times global          no
 *
 * The pairs are each a factor and its complement. Factors 4 and 5 are MEASURED:
 * the same three frames rule out premultiplied alpha, at 6..10/255 against 0.36.
 * Factors 8 and 9 are MEASURED only at 0xff; that mode 2 ignores the global
 * alpha is MEASURED too, because stage 2 sits at 0x50004000, global alpha 0, with
 * layer 0 plainly visible. Factors 10 and 11 are INFERRED from the pattern.
 */
#define S5L8702_DISP_SLOT_BASE   0x058
#define S5L8702_DISP_SLOT_STEP   0x018
#define S5L8702_DISP_NSLOTS      5
#define S5L8702_DISP_SLOT_STRIDE 0x00
#define S5L8702_DISP_SLOT_FORMAT 0x04
#define S5L8702_DISP_SLOT_SRC    0x08
#define S5L8702_DISP_SLOT_SIZE   0x0c
#define S5L8702_DISP_SLOT_POS    0x14

/* Named registers, all justified in the comment above. */
#define S5L8702_DISP_UNK00      0x000
#define S5L8702_DISP_UNK04      0x004
#define S5L8702_DISP_CONTROL    0x008
#define S5L8702_DISP_UNK0C      0x00c
#define S5L8702_DISP_BACKGROUND 0x024
#define S5L8702_DISP_PLANAR_FMT 0x028
#define S5L8702_DISP_PLANAR_STRIDE 0x02c
#define S5L8702_DISP_PLANAR_ORIGIN 0x030
#define S5L8702_DISP_PLANAR_LIMIT  0x034
#define S5L8702_DISP_SRC_PLANE0 0x038
#define S5L8702_DISP_SRC_PLANE1 0x03c
#define S5L8702_DISP_SRC_PLANE2 0x040
#define S5L8702_DISP_SRC_PLANE3 0x044
#define S5L8702_DISP_PLANAR_STEP 0x04c
#define S5L8702_DISP_PLANAR_POS  0x050
#define S5L8702_DISP_PLANAR_SIZE 0x054
#define S5L8702_DISP_SRC_SLOT0  0x060
#define S5L8702_DISP_SRC_STRIDE 0x018
#define S5L8702_DISP_NSLOT      5
#define S5L8702_DISP_ORDER      0x0d4
#define S5L8702_DISP_STAGE0     0x0d8
#define S5L8702_DISP_STAGE_STEP 0x008
#define S5L8702_DISP_NSTAGE     3
#define S5L8702_DISP_STATUS200  0x200
#define S5L8702_DISP_GEOMETRY   0x214
#define S5L8702_DISP_FORMAT     0x3ac
#define S5L8702_DISP_GAMMA0     0x400

/* Control-register bit that the blit path reads back. See +0x3ac above. */
#define S5L8702_DISP_FORMAT_ROT 0x1

/*
 * THE PER-LAYER ENABLE BITS, in CONTROL (+0x008). Bit 6 turns layer 0 on,
 * bit 5 layer 1, bit 4 layer 2, bit 3 layer 3 and bit 2 layer 4 -- so layer k
 * is bit (6 - k) -- and bit 7 layer 5, the planar one.
 *
 * This is the gate that was missing, and without it the model kept drawing
 * layers the guest had switched off but left fully programmed, which covered
 * the whole user interface with a stale surface. Two independent lines of
 * evidence, both from a boot with a disk:
 *
 * 1. THE SETTER, disassembled. 0x081271c8 takes an index in r1 and a boolean
 *    in r2, jumps through the table at 0x081271d8, and does
 *    "bic r0, r0, #BIT / orrne r0, r0, #BIT" on the word it read from +0x008
 *    before storing it back at 0x08127250. The arms are, in index order:
 *      0 -> 0x40   1 -> 0x20   2 -> 0x10   3 -> 0x08   4 -> 0x04   5 -> 0x80
 *    All six are layers: the same index picks the layer in the descriptor and
 *    source-address switches. An earlier version of this comment took 3..5 for
 *    something else, which is how bit 7 stayed unmodelled and left the main
 *    menu's translucent pane on black.
 *
 * 2. THE ORDER OF WRITES, traced with DISPTRACE=1. Every single time, the
 *    guest programs a descriptor, writes its SOURCE ADDRESS, and only then
 *    sets the matching bit:
 *      +0x060 = 0x089d42d4 (layer 0) ... +0x008: 0x41118101 -> 0x41118141
 *      +0x078 = 0x0be2326c (layer 1) ... +0x008: 0x41118141 -> 0x41118161
 *      +0x090 = 0x0bfb4ff8 (layer 2) ... +0x008: 0x41118161 -> 0x41118171
 *    and when a transition ends it clears them again, 0x41118151 then
 *    0x41118141, WITHOUT clearing the source address or the size. A settled
 *    screen sits at 0x41118141: layer 0 alone.
 *
 * Not to be confused with bits 16, 20 and 24, which also look one-per-layer.
 * An earlier version of the compositor weighed those, found them set once
 * during init and never touched again, and rejected them as a gate -- which
 * was correct about 16/20/24, and became the wrong conclusion only because it
 * stopped there and declared that nothing gated a layer at all.
 *
 */
#define S5L8702_DISP_LAYER_EN_BIT(k) (1u << (6 - (k)))
#define S5L8702_DISP_PLANAR_EN_BIT   (1u << 7)

struct IPodClassicDispState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    /*
     * A flat register file. Readback is the point: six functions in the osos
     * read-modify-write +0x008 and one reads +0x200, and with the block
     * unimplemented every one of those read zero and destroyed whatever the
     * previous caller had set.
     */
    uint32_t reg[S5L8702_DISP_NREG];

    /*
     * Scratch for one source row, as 32-bit words so a row of format 7 can be
     * mixed where it lands. Biggest case: 320 pixels at 32 bpp.
     */
    uint32_t rowbuf[320];
    /* Scratch for one row of the planar layer. +0x054's fields are ten bits. */
    uint8_t lumarow[1024];
    uint8_t cbrow[1024 / 2 + 1];
    uint8_t crrow[1024 / 2 + 1];
    /*
     * The frame being mixed, 0x00RRGGBB. Mixed at eight bits and packed once at
     * the end, so a frame with three layers rounds once and not three times.
     */
    uint32_t frame[320 * 240];
};

/*
 * Composite whatever the layer descriptors currently point at into a 320x240
 * RGB565 image, which is what the panel behind the LCD port holds. Returns
 * true if anything was drawn. Called by the LCD device, because on this SoC the
 * panel's image memory is what the user finally sees -- see ipod_classic_lcd.c.
 */
bool ipod_classic_disp_compose(IPodClassicDispState *s, uint16_t *fb,
                               unsigned width, unsigned height);

#endif
