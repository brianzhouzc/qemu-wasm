/*
 * S5L8702 SPI controller.
 *
 * The SoC has three, at the SPIBASE(i) addresses of Rockbox's s5l87xx.h for
 * this chip: 0x3C300000, 0x3CE00000 and 0x3D200000.  The machine models the
 * first, because that is the one with something on it: the 1 MB SPI NOR that
 * holds the SysCfg block and the bootloader (norboot-target.h, SPI_PORT 0).
 *
 * The register map is Rockbox's (the SPI* macros under CONFIG_CPU ==
 * S5L8702), and what the bits do is INFERRED from the one driver there is,
 * Rockbox's spi-s5l8702.c, which is emCORE's driver for this SoC:
 *
 *   +0x00 CTRL     bit 0 enables the port.  spi_prepare() ORs in 0xc twice
 *                  while it sets the port up, read here as "flush the
 *                  FIFOs": the bits do not stay set.
 *   +0x04 SETUP    0x10618 on this SoC.  Bit 0 starts a receive: with it set
 *                  the port clocks filler out until RXLIMIT bytes have come
 *                  in, as fast as the RX FIFO drains (spi_read()).
 *   +0x08 STATUS   bits 4-8 the TX FIFO level (spi_write() waits while it
 *                  is 0x100, a full FIFO of 16), bits 9-13 the RX FIFO level
 *                  (spi_wait_ready() waits for any of them).  Bits 0-3 are
 *                  written with 0xf before a transfer; nothing here sets
 *                  them, so what they flag is UNDETERMINED.
 *   +0x0c PIN      written with 6.
 *   +0x10 TXDATA   one byte out; the byte that came back goes to the RX FIFO.
 *   +0x20 RXDATA   the oldest received byte.
 *   +0x30 CLKDIV, +0x34 RXLIMIT, +0x38 DD ("TBC" in Rockbox).
 *
 * The chip select is not in this block.  Rockbox drives the NOR's from GPIO
 * 0.0 with GPIOCMD (0xe low, 0xf high), so the machine wires that pin from
 * the GPIO model to the flash.
 *
 * Transfers complete at once: the TX FIFO never fills, and the interrupt and
 * DMA requests (IRQ 9, DMA request 5) are not raised, because no driver we
 * have uses them.
 */

#ifndef HW_ARM_IPOD_CLASSIC_SPI_H
#define HW_ARM_IPOD_CLASSIC_SPI_H

#include "qemu/osdep.h"
#include "qemu/fifo8.h"
#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"

#define TYPE_IPOD_CLASSIC_SPI "s5l8702.spi"
OBJECT_DECLARE_SIMPLE_TYPE(IPodClassicSPIState, IPOD_CLASSIC_SPI)

#define S5L8702_SPI_MEM_SIZE      0x100

#define S5L8702_SPI_CTRL          0x00
#define S5L8702_SPI_SETUP         0x04
#define S5L8702_SPI_STATUS        0x08
#define S5L8702_SPI_PIN           0x0C
#define S5L8702_SPI_TXDATA        0x10
#define S5L8702_SPI_RXDATA        0x20
#define S5L8702_SPI_CLKDIV        0x30
#define S5L8702_SPI_RXLIMIT       0x34
#define S5L8702_SPI_DD            0x38

#define S5L8702_SPI_CTRL_FLUSH    0xc
#define S5L8702_SPI_SETUP_RX      (1u << 0)
#define S5L8702_SPI_STATUS_FLAGS  0xf
#define S5L8702_SPI_STATUS_TX_POS 4
#define S5L8702_SPI_STATUS_RX_POS 9

#define S5L8702_SPI_FIFO_DEPTH    16

/* What the port clocks out while it receives (SETUP bit 0). */
#define S5L8702_SPI_FILLER        0xff

struct IPodClassicSPIState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SSIBus *bus;

    uint32_t ctrl;
    uint32_t setup;
    uint32_t flags;
    uint32_t pin;
    uint32_t clkdiv;
    uint32_t rxlimit;
    uint32_t dd;
    /* Bytes the receive started by SETUP bit 0 still has to clock in. */
    uint32_t rx_pending;
    Fifo8 rx;
};

#endif /* HW_ARM_IPOD_CLASSIC_SPI_H */
