/*
 * S5L8702 SPI controller.  See include/hw/arm/ipod_classic/ipod_classic_spi.h
 * for the register map and where it comes from.
 *
 * Why it exists: before it, 0x3C300000 fell to the machine's I/O floor, which
 * reads 0, and Rockbox's spi_wait_ready() waits for a non-zero RX level with
 * no timeout.  System > Debug > View SysCfg and Dump bootflash to file both
 * start with a NOR read, so both left the UI thread spinning on STATUS
 * (MEASURED on qemu 332b67d: pc 0x0808bc6c, 24,000 reads of 0x3C300008 in a
 * few seconds, the JIT and TCI alike).  The machine's header had SPI0 at
 * 0x3CD00000, the S5L8700's SPI_BASE, so nothing was ever modelled where this
 * SoC has it.
 *
 * SPITRACE=1 prints every access with the guest PC, like GPIOTRACE.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/cpu.h"
#include "migration/vmstate.h"
#include "hw/arm/ipod_classic/ipod_classic_spi.h"

static bool spi_trace_on(void)
{
    static int on = -1;

    if (on < 0) {
        const char *e = getenv("SPITRACE");
        on = (e && *e && *e != '0') ? 1 : 0;
    }
    return on == 1;
}

/* Head of the running translation block, not the address of the access. */
static uint32_t spi_trace_pc(void)
{
    CPUState *cs = current_cpu;
    return cs ? (uint32_t)CPU_GET_CLASS(cs)->get_pc(cs) : 0;
}

/* Clock in what the receive still owes, as far as the RX FIFO has room. */
static void ipod_classic_spi_fill(IPodClassicSPIState *s)
{
    while (s->rx_pending && !fifo8_is_full(&s->rx)) {
        fifo8_push(&s->rx, ssi_transfer(s->bus, S5L8702_SPI_FILLER) & 0xff);
        s->rx_pending--;
    }
}

static uint32_t ipod_classic_spi_status(IPodClassicSPIState *s)
{
    /* The TX FIFO is always empty: a byte goes out as it is written. */
    return s->flags |
           (fifo8_num_used(&s->rx) << S5L8702_SPI_STATUS_RX_POS);
}

/*
 * Words, and the bytes and halfwords in them, like the clock block: a narrow
 * read returns its lane, a narrow write changes only its lane.  No driver we
 * have makes one, but a bus error on one would be the model's invention.
 */
static uint64_t ipod_classic_spi_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    IPodClassicSPIState *s = opaque;
    uint32_t val;

    switch (offset & ~3) {
    case S5L8702_SPI_CTRL:
        val = s->ctrl;
        break;
    case S5L8702_SPI_SETUP:
        val = s->setup;
        break;
    case S5L8702_SPI_STATUS:
        val = ipod_classic_spi_status(s);
        break;
    case S5L8702_SPI_PIN:
        val = s->pin;
        break;
    case S5L8702_SPI_RXDATA:
        if (fifo8_is_empty(&s->rx)) {
            qemu_log_mask(LOG_GUEST_ERROR, "s5l8702.spi: RXDATA read with "
                          "nothing received\n");
            val = 0;
        } else {
            val = fifo8_pop(&s->rx);
            ipod_classic_spi_fill(s);
        }
        break;
    case S5L8702_SPI_CLKDIV:
        val = s->clkdiv;
        break;
    case S5L8702_SPI_RXLIMIT:
        val = s->rxlimit;
        break;
    case S5L8702_SPI_DD:
        val = s->dd;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "s5l8702.spi: read of +0x%" HWADDR_PRIx "\n",
                      offset);
        val = 0;
        break;
    }
    if (spi_trace_on()) {
        fprintf(stderr, "SPITRACE rd +0x%02" HWADDR_PRIx " -> 0x%08x  "
                "pc~0x%08x\n", offset, val, spi_trace_pc());
    }
    return extract32(val, (offset & 3) * 8, size * 8);
}

static void ipod_classic_spi_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    IPodClassicSPIState *s = opaque;
    unsigned shift = (offset & 3) * 8;
    uint32_t mask = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t lane = (uint32_t)(value << shift) & mask;
    uint32_t old;

    if (spi_trace_on()) {
        fprintf(stderr, "SPITRACE wr +0x%02" HWADDR_PRIx " = 0x%08x  "
                "pc~0x%08x\n", offset, (uint32_t)value, spi_trace_pc());
    }

    switch (offset & ~3) {
    case S5L8702_SPI_CTRL:
        if (lane & S5L8702_SPI_CTRL_FLUSH) {
            fifo8_reset(&s->rx);
            s->rx_pending = 0;
        }
        s->ctrl = ((s->ctrl & ~mask) | lane) & ~S5L8702_SPI_CTRL_FLUSH;
        break;
    case S5L8702_SPI_SETUP:
        old = s->setup;
        s->setup = (s->setup & ~mask) | lane;
        if (!(s->setup & S5L8702_SPI_SETUP_RX)) {
            s->rx_pending = 0;
        } else if (!(old & S5L8702_SPI_SETUP_RX)) {
            s->rx_pending = s->rxlimit;
            ipod_classic_spi_fill(s);
        }
        break;
    case S5L8702_SPI_STATUS:
        s->flags &= ~(lane & S5L8702_SPI_STATUS_FLAGS);
        break;
    case S5L8702_SPI_PIN:
        s->pin = (s->pin & ~mask) | lane;
        break;
    case S5L8702_SPI_TXDATA:
        /* One 8-bit frame per write, from the low byte. */
        if (shift) {
            qemu_log_mask(LOG_GUEST_ERROR, "s5l8702.spi: TXDATA write at "
                          "+0x%" HWADDR_PRIx " ignored\n", offset);
            break;
        }
        {
            uint8_t in = ssi_transfer(s->bus, value & 0xff) & 0xff;

            if (fifo8_is_full(&s->rx)) {
                qemu_log_mask(LOG_GUEST_ERROR, "s5l8702.spi: RX FIFO "
                              "overflow, byte dropped\n");
            } else {
                fifo8_push(&s->rx, in);
            }
        }
        break;
    case S5L8702_SPI_RXDATA:
        qemu_log_mask(LOG_GUEST_ERROR, "s5l8702.spi: RXDATA is read-only\n");
        break;
    case S5L8702_SPI_CLKDIV:
        s->clkdiv = (s->clkdiv & ~mask) | lane;
        break;
    case S5L8702_SPI_RXLIMIT:
        s->rxlimit = (s->rxlimit & ~mask) | lane;
        break;
    case S5L8702_SPI_DD:
        s->dd = (s->dd & ~mask) | lane;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "s5l8702.spi: write of 0x%" PRIx64
                      " to +0x%" HWADDR_PRIx "\n", value, offset);
        break;
    }
}

static const MemoryRegionOps ipod_classic_spi_ops = {
    .read = ipod_classic_spi_read,
    .write = ipod_classic_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void ipod_classic_spi_reset(DeviceState *dev)
{
    IPodClassicSPIState *s = IPOD_CLASSIC_SPI(dev);

    s->ctrl = 0;
    s->setup = 0;
    s->flags = 0;
    s->pin = 0;
    s->clkdiv = 0;
    s->rxlimit = 0;
    s->dd = 0;
    s->rx_pending = 0;
    fifo8_reset(&s->rx);
}

static void ipod_classic_spi_realize(DeviceState *dev, Error **errp)
{
    IPodClassicSPIState *s = IPOD_CLASSIC_SPI(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &ipod_classic_spi_ops, s,
                          TYPE_IPOD_CLASSIC_SPI, S5L8702_SPI_MEM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    s->bus = ssi_create_bus(dev, "ssi");
    fifo8_create(&s->rx, S5L8702_SPI_FIFO_DEPTH);
}

static const VMStateDescription vmstate_ipod_classic_spi = {
    .name = TYPE_IPOD_CLASSIC_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(ctrl, IPodClassicSPIState),
        VMSTATE_UINT32(setup, IPodClassicSPIState),
        VMSTATE_UINT32(flags, IPodClassicSPIState),
        VMSTATE_UINT32(pin, IPodClassicSPIState),
        VMSTATE_UINT32(clkdiv, IPodClassicSPIState),
        VMSTATE_UINT32(rxlimit, IPodClassicSPIState),
        VMSTATE_UINT32(dd, IPodClassicSPIState),
        VMSTATE_UINT32(rx_pending, IPodClassicSPIState),
        VMSTATE_FIFO8(rx, IPodClassicSPIState),
        VMSTATE_END_OF_LIST()
    },
};

static void ipod_classic_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "S5L8702 SPI controller";
    dc->realize = ipod_classic_spi_realize;
    dc->reset = ipod_classic_spi_reset;
    dc->vmsd = &vmstate_ipod_classic_spi;
    dc->user_creatable = false;
}

static const TypeInfo ipod_classic_spi_info = {
    .name          = TYPE_IPOD_CLASSIC_SPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodClassicSPIState),
    .class_init    = ipod_classic_spi_class_init,
};

static void ipod_classic_spi_register_types(void)
{
    type_register_static(&ipod_classic_spi_info);
}

type_init(ipod_classic_spi_register_types)
