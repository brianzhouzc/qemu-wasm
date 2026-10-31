/*
 * S5L8702 timer block, 0x3C700000 (TIMER_BASE).
 *
 * Eight channels. A..D are 16-bit and share IRQ 8; E..H are 32-bit and share
 * IRQ 7. Register layout and semantics from Rockbox: the block comment above
 * TSTAT in firmware/export/s5l87xx.h, plus the two drivers that actually
 * drive it, timer-s5l8702.c (timer F) and kernel-s5l8702.c (timer B).
 */

#ifndef HW_ARM_IPOD_CLASSIC_TIMER_H
#define HW_ARM_IPOD_CLASSIC_TIMER_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/ptimer.h"
#include "qom/object.h"

#define TYPE_IPOD_CLASSIC_TIMER "s5l8702.timer"
OBJECT_DECLARE_SIMPLE_TYPE(IPodClassicTimerState, IPOD_CLASSIC_TIMER)

#define S5L8702_TIMER_MEM_SIZE  0x11C

#define S5L8702_NUM_TIMERS      8    /* A..H */
#define S5L8702_NUM_TIMERS_16   4    /* A..D, the 16-bit ones */

/* Per-channel registers, relative to the channel base. */
#define S5L8702_TIMER_CON       0x00
#define S5L8702_TIMER_CMD       0x04
#define S5L8702_TIMER_DATA0     0x08
#define S5L8702_TIMER_DATA1     0x0C
#define S5L8702_TIMER_PRE       0x10
#define S5L8702_TIMER_CNT       0x14
#define S5L8702_TIMER_STRIDE    0x20

/* Channel bases: A..D start at 0, E..H at 0xA0. */
#define S5L8702_TIMER_AD_BASE   0x00
#define S5L8702_TIMER_EH_BASE   0xA0

/*
 * THE HOLE BETWEEN THEM IS NOT EMPTY, and nothing here decodes it.
 *
 * A..D end at 0x7f and E..H begin at 0xa0. In the gap Rockbox puts a 64-bit
 * free-running counter: TICNTH at +0x80 and TICNTL at +0x84 (s5l87xx.h:782),
 * which s5l87xx.h:793 turns into FIVE_USEC_TIMER and :794 into its USEC_TIMER.
 * decode_offset() covers neither, so both answer zero for ever and log a
 * guest error. MEASURED with tools/ipod_classic/tests/timer_audit.c:
 *
 *     TICNTH/L 0x00000000/0x00000000 -> 0x00000000/0x00000000  FROZEN
 *
 * It costs the retailOS nothing -- the image does not contain the literal
 * 0x3c700080 or 0x3c700084 anywhere in its 9.9 MB, and the time it measures it
 * measures with timer E at 1 MHz (set up at 0x08324e48, read at 0x08324e38) --
 * but a guest that span on FIVE_USEC_TIMER would hang for ever.
 *
 * NOT MODELLED ON PURPOSE. Adding the register is easy; no source we have says
 * what it reads out of reset, whether it can be stopped, or its exact rate, and
 * a counter invented at the wrong rate is worse than one that is missing --
 * the missing one is noticed. See wiki/11 s.19.3.
 */

/* Shared status register for the 32-bit timers. */
#define S5L8702_TIMER_TSTAT     0x118

/*
 * CON bits. The three interrupt enables sit at 12..14 (INT0, INT1, INTOVF).
 * For A..D the matching status bits are at 16..18 in the same register, which
 * is why Rockbox tests "TACON & (TACON >> 4) & 0x7000". For E..H the status
 * lives in TSTAT instead.
 */
#define S5L8702_TIMER_CON_INT_EN_SHIFT    12
#define S5L8702_TIMER_CON_INT_STAT_SHIFT  16
#define S5L8702_TIMER_CON_INT_MASK        0x7
#define S5L8702_TIMER_CON_CS_SHIFT        8     /* clock select, 3 bits */
#define S5L8702_TIMER_CON_CS_MASK         0x7
#define S5L8702_TIMER_CON_ECLK            (1 << 6)
#define S5L8702_TIMER_CON_MODE_SHIFT      4

/* CMD bits. */
#define S5L8702_TIMER_CMD_EN     (1 << 0)
#define S5L8702_TIMER_CMD_CLR    (1 << 1)

/* TSTAT: three bits per 32-bit channel, E at 24, F at 16, G at 8, H at 0. */
#define S5L8702_TSTAT_SHIFT(ch32)  (24 - 8 * (ch32))

/*
 * ECLK, the external clock the iPod classic drives the timers from.
 * TIMER_FREQ in Rockbox's s5l87xx.h.
 */
#define S5L8702_TIMER_ECLK_HZ    12000000

typedef struct IPodClassicTimerState IPodClassicTimerState;

typedef struct {
    IPodClassicTimerState *parent;
    unsigned index;             /* 0..7, i.e. A..H */
    ptimer_state *ptimer;
    uint32_t con;
    uint32_t cmd;
    uint32_t data0;
    uint32_t data1;
    uint32_t pre;
    int64_t host_start_ns;
    uint32_t host_count;
    bool host_anchor;

    /* Channel B's dedicated host-time tick (Rockbox kernel tick). */
    QEMUTimer *host_tick;
    uint64_t host_tick_period_ns;
    uint32_t host_tick_count;
    uint32_t host_tick_step;
    bool host_tick_enabled;
} IPodClassicTimerChannel;

struct IPodClassicTimerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    IPodClassicTimerChannel chan[S5L8702_NUM_TIMERS];
    uint32_t tstat;

    qemu_irq irq16;             /* IRQ 8, shared by A..D */
    qemu_irq irq32;             /* IRQ 7, shared by E..H */
};

#endif
