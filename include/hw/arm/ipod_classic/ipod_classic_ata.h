/*
 * S5L8702 ATA controller, 0x38700000 (ATA_BASE).
 *
 * A bridge, not a task file. The eight ATA device registers appear at
 * 0x54..0x74, but they are not readable directly: reading one starts a bus
 * cycle whose result turns up later in ATA_PIO_RDATA. Rockbox writes the
 * protocol out in full (storage_ata-6g.c):
 *
 *     static uint16_t ata_read_cbr(uint32_t volatile* reg)
 *     {
 *         while (!(ATA_PIO_READY & 2));
 *         volatile uint32_t dummy = *reg;
 *         (void)dummy;
 *         while (!(ATA_PIO_READY & 1));
 *         return ATA_PIO_RDATA;
 *     }
 *
 *     static void ata_write_cbr(uint32_t volatile* reg, uint16_t data)
 *     {
 *         while (!(ATA_PIO_READY & 2));
 *         *reg = data;
 *     }
 *
 * Apple's driver is the same function compiled from different source:
 * 0x08096824 waits on bit 1, does the dummy read, waits on bit 0 and takes the
 * answer from +0x7c. So ATA_PIO_READY bit 1 is "a cycle may start" and bit 0 is
 * "a read result is waiting", and a model that lets the eight registers answer
 * on their own would work by accident for writes and never for reads.
 *
 * Register names and offsets from Rockbox firmware/export/s5l87xx.h.
 */

#ifndef HW_ARM_IPOD_CLASSIC_ATA_H
#define HW_ARM_IPOD_CLASSIC_ATA_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/ide/internal.h"
#include "qom/object.h"

#define TYPE_IPOD_CLASSIC_ATA "s5l8702.ata"
OBJECT_DECLARE_SIMPLE_TYPE(IPodClassicATAState, IPOD_CLASSIC_ATA)

#define S5L8702_ATA_MEM_SIZE     0x100

#define S5L8702_ATA_CONTROL      0x00   /* enable, and clock-down status */
#define S5L8702_ATA_STATUS       0x04
#define S5L8702_ATA_COMMAND      0x08
#define S5L8702_ATA_SWRST        0x0c
#define S5L8702_ATA_IRQSTAT      0x10   /* sources, write-1-to-clear */
#define S5L8702_ATA_IRQMASK      0x14
#define S5L8702_ATA_CFG          0x18
#define S5L8702_ATA_MDMA_TIME    0x28
#define S5L8702_ATA_PIO_TIME     0x2c
#define S5L8702_ATA_UDMA_TIME    0x30
#define S5L8702_ATA_XFR_NUM      0x34
#define S5L8702_ATA_XFR_CNT      0x38
#define S5L8702_ATA_TBUF_START   0x3c
#define S5L8702_ATA_TBUF_SIZE    0x40
#define S5L8702_ATA_SBUF_START   0x44
#define S5L8702_ATA_SBUF_SIZE    0x48
#define S5L8702_ATA_CADR_TBUF    0x4c
#define S5L8702_ATA_CADR_SBUF    0x50

/* The device registers, one 32-bit word each. */
#define S5L8702_ATA_PIO_DTR      0x54   /* data */
#define S5L8702_ATA_PIO_FED      0x58   /* feature (w) / error (r) */
#define S5L8702_ATA_PIO_SCR      0x5c   /* sector count */
#define S5L8702_ATA_PIO_LLR      0x60   /* LBA low */
#define S5L8702_ATA_PIO_LMR      0x64   /* LBA mid */
#define S5L8702_ATA_PIO_LHR      0x68   /* LBA high */
#define S5L8702_ATA_PIO_DVR      0x6c   /* device */
#define S5L8702_ATA_PIO_CSD      0x70   /* command (w) / status (r) */
#define S5L8702_ATA_PIO_DAD      0x74   /* device control (w) / alt status (r) */
#define S5L8702_ATA_PIO_READY    0x78
#define S5L8702_ATA_PIO_RDATA    0x7c

#define S5L8702_ATA_READY_RDATA  (1u << 0)   /* a read result is waiting */
#define S5L8702_ATA_READY_CYCLE  (1u << 1)   /* a cycle may be started */

#define S5L8702_ATA_CONTROL_ENABLE     (1u << 0)
#define S5L8702_ATA_CONTROL_CLOCKDOWN  (1u << 1)

/*
 * The DMA engine. Rockbox writes the whole sequence out in
 * storage_ata-6g.c:851-880, and Apple's driver was traced doing the same
 * thing register for register, which is what makes the bit meanings safe:
 *
 *     <ATA command 0xc8 to ATA_PIO_CSD>
 *     ATA_TBUF_START = buffer;            (ATA_SBUF_START when writing)
 *     ATA_TBUF_SIZE  = bytes;
 *     ATA_XFR_NUM    = bytes - 1;
 *     ATA_CFG |= dma_flags; ATA_CFG &= ~(BIT(7) | BIT(8));
 *     ATA_IRQ = BITRANGE(0, 4);
 *     ATA_IRQ_MASK = BIT(0);
 *     ATA_COMMAND = BIT(0);               -- and only now does it move
 *     <wait for the interrupt>
 *     ATA_COMMAND = BIT(1);               -- stop
 *
 * The order is what shapes the model. The buffer address is programmed AFTER
 * the ATA command, so start_dma cannot know where the data goes; all it can do
 * is latch the callback the IDE core hands it, and the write of the go bit is
 * what fires it. (The PP502x on the 5G needs the same split for the opposite
 * reason -- there the address comes first and the go bit still comes last.)
 *
 * CFG bit 4 chooses the direction: set means the source buffer, i.e. a write
 * to the disk. VERIFIED from Rockbox, which sets it on the write path and
 * clears it on the read path, and consistent with the trace, where Apple's
 * read leaves it clear (ATA_CFG = 0x64c).
 */
#define S5L8702_ATA_COMMAND_START      (1u << 0)
#define S5L8702_ATA_COMMAND_STOP       (1u << 1)
#define S5L8702_ATA_CFG_WRITE          (1u << 4)

/*
 * INFERRED: bit 0 of ATA_IRQ is the device's INTRQ, gated by ATA_IRQMASK.
 * Nothing states the bit layout. Rockbox never reads ATA_IRQ, and Apple only
 * ever writes it back to itself ("ATA_IRQ = ATA_IRQ" at 0x08325f14), which
 * establishes that it is write-one-to-clear and nothing else. Both drivers
 * poll ATA_PIO_CSD rather than waiting for an interrupt -- 0x08325dec reads
 * the alternate status and then the status and tests ERR, DRQ and DF by hand
 * -- so an interrupt that never arrives costs nothing, and one that arrives
 * with the wrong bit set would be acknowledged and ignored.
 */
#define S5L8702_ATA_INTRQ_BIT    (1u << 0)

struct IPodClassicATAState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    IDEBus bus;
    IDEDMA dma;
    qemu_irq irq;

    uint32_t reg[S5L8702_ATA_MEM_SIZE / 4];  /* the plain ones, 0x00..0x50 */
    uint32_t rdata;
    bool rdata_valid;
    bool intrq;

    /* The transfer in flight: where it is going and how much is left. */
    BlockCompletionFunc *dma_cb;
    uint32_t dma_cur;
    uint32_t dma_left;
};

void ipod_classic_ata_init_drive(DeviceState *dev, DriveInfo *hd);

/*
 * True between the IDE command that sets a DMA up and the go bit that runs
 * it: the core's callback is latched, and a snapshot cannot carry it.
 */
bool ipod_classic_ata_dma_armed(IPodClassicATAState *s);

#endif
