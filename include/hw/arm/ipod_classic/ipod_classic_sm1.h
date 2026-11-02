/*
 * S5L8702 "SM1" audio decoder engine, 0x38500000.
 *
 * Stock firmware does not decode MP3 or AAC on the ARM.  It hands the
 * compressed frames to this block and collects PCM from a ring in IRAM1.
 * Rockbox decodes on the ARM and never touches it.
 *
 * The block is a DSP with memories of its own, and the OS loads it: the image
 * carries a program for each format (MP3 0x089460f4, AAC 0x0894ea90, on 2.0.5),
 * 32-bit program words and data sections holding 24-bit constants such as
 * 0x5a827a.  What core that is has not been established and nothing here runs
 * those programs.  This models the interface instead and decodes natively:
 * the loads land in IRAM1 as they should, and are not looked at.
 *
 * All of this is from 2.0.5.  The call chain is in docs/architecture.md.
 */

#ifndef HW_ARM_IPOD_CLASSIC_SM1_H
#define HW_ARM_IPOD_CLASSIC_SM1_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_IPOD_CLASSIC_SM1 "s5l8702.sm1"
OBJECT_DECLARE_SIMPLE_TYPE(IPodClassicSM1State, IPOD_CLASSIC_SM1)

/*
 * The window is 1 MB and the firmware's registers are in the low 4 KB, apart
 * from a few writes at +0x1004..+0x100c that nothing reads back.
 */
#define S5L8702_SM1_MEM_SIZE   0x100000
#define S5L8702_SM1_REG_BYTES  0x1000
#define S5L8702_SM1_REG_WORDS  (S5L8702_SM1_REG_BYTES / 4)

/* A mismatch here is an eternal `b .` at 0x0809ac64. */
#define S5L8702_SM1_ID          0xc80
#define S5L8702_SM1_ID_MAGIC    0x004e7309

/*
 * Starts the bootstrap, the first half of a program.  The firmware then spins
 * on S5L8702_SM1_EVENT with no timeout (0x0809adac) and acknowledges it.
 */
#define S5L8702_SM1_BOOT        0x984
/* Starts the rest, once, with the mailbox filled in. */
#define S5L8702_SM1_ENTRY       0x988

/*
 * Interrupt enable set and clear, status, and acknowledge, with
 * S5L8702_SM1_EVENT the one bit in use: "the engine has something for you",
 * on VIC0 line 18.  The handler is 0x0802eb20.
 */
#define S5L8702_SM1_INT_SET     0xa8c
#define S5L8702_SM1_INT_CLEAR   0xa90
#define S5L8702_SM1_STAT        0xa98
#define S5L8702_SM1_ACK         0xc48
#define S5L8702_SM1_EVENT       (1u << 2)

/* Read back by the two pollers of the run bit. */
#define S5L8702_SM1_RUN         0x400

/*
 * The DMA channels: +0x00 descriptor, +0x10 command, +0x14 state.  Up (4) and
 * run (0x10) read back 7, and stop (0x60) reads back 0.  The firmware will not
 * queue a frame unless channel 4 reads 0 (0x0809b388).
 */
#define S5L8702_SM1_SUB_CMD_UP      0x04
#define S5L8702_SM1_SUB_CMD_RUN     0x10
#define S5L8702_SM1_SUB_STATE_READY 7
/* Channel 4 carries the compressed frames. */
#define S5L8702_SM1_FRAME_CHAIN     0x100
#define S5L8702_SM1_CHAIN_GO        1

/* A descriptor, all 32-bit words.  The top half of ctrl is the byte count. */
#define S5L8702_SM1_DESC_NEXT   0
#define S5L8702_SM1_DESC_CTRL   1
#define S5L8702_SM1_DESC_ADDR   2
#define S5L8702_SM1_DESC_WORDS  4

/*
 * What the engine's memory holds for the ARM.  These are the firmware's own
 * constants, the table at 0x08a172b4.
 *
 *   MAILBOX, halfwords: +0 what follows is valid, 1 for MP3 and 7 for AAC;
 *            +6 the AAC sample rate index.  The firmware ORs bits into +0, so
 *            it counts on the bootstrap having cleared it, and it does not
 *            start out clear: the OS's boot image name is parked there.
 *   STATUS,  halfwords: +0 output is valid, +2 an error, 0 for none.
 *   SLOTS,   halfwords: +2 how many slots, +4 samples a slot; then from +0xa a
 *            pair for each slot, ready flag and samples in it.
 *   PCM      the slots, one after the other.
 *
 * 0x080bdda8 is the reader.  It takes slots in order from 0, clears the flag
 * and wraps at the count.
 *
 * A mono stream is mono here, one sample a frame in the slot, because the OS
 * doubles it later.  0x22006430 divides the slot's count by the channels it
 * knows, [state+0x14], and copies four bytes a frame whatever the stream is.
 * Measured, for a 60 s MP3: a stereo slot with a stereo count overran the OS's
 * buffers of 1152 frames, half pitch and gone in 6.5 s, and a stereo slot with
 * a mono count played the first half of each frame at half pitch and the right
 * speed.  The mailbox does not give the channels, so they come from the stream.
 *
 * The geometry is not in any register, and two slots of a whole frame each is
 * read off the firmware.  It saves the engine's data around this ring, and that
 * save resumes at 0x2203a500 for MP3 and 0x2203a100 for AAC: from PCM that is
 * 0x2400 and 0x2000 bytes, two frames of 1152 and of 1024 stereo samples.
 */
#define S5L8702_SM1_MAILBOX         0x2203ff00
#define S5L8702_SM1_MAILBOX_BYTES   0x40
#define S5L8702_SM1_MAILBOX_RATE    6
#define S5L8702_SM1_MAILBOX_MP3     1
#define S5L8702_SM1_MAILBOX_AAC     7
#define S5L8702_SM1_STATUS          0x2203ff28
#define S5L8702_SM1_STATUS_VALID    1
#define S5L8702_SM1_SLOTS           0x22038000
#define S5L8702_SM1_SLOT_COUNT      2
#define S5L8702_SM1_SLOT_FLAG(n)    (S5L8702_SM1_SLOTS + 0xa + (n) * 4)
#define S5L8702_SM1_SLOT_SAMPLES(n) (S5L8702_SM1_SLOTS + 0xc + (n) * 4)
#define S5L8702_SM1_PCM             0x22038100

/* Bigger than any frame of either format at the rates the iPod plays. */
#define S5L8702_SM1_FRAME_MAX       0x2000

struct IPodClassicSM1Codec;

struct IPodClassicSM1State {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t reg[S5L8702_SM1_REG_WORDS];

    bool event;             /* +0xa98, until acknowledged */
    uint32_t irq_enable;
    qemu_irq irq;

    uint32_t codec_type;    /* IPodClassicSM1CodecType, from the mailbox */
    uint32_t rate_index;
    uint32_t slot_samples;
    uint32_t slot;          /* the slot the next frame goes in */
    uint32_t chain;         /* the descriptor being worked on, or 0 */
    QEMUTimer *frame_timer;

    /*
     * Not in the vmstate.  A restored machine builds a new one from
     * codec_type, which loses a frame of overlap and no more.
     */
    struct IPodClassicSM1Codec *codec;
};

#endif
