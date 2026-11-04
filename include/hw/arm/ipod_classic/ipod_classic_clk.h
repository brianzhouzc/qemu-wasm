/*
 * S5L8702 clock and PLL block, 0x3C500000 (CLK_BASE).
 *
 * Register names and offsets from Rockbox, firmware/export/s5l87xx.h, the
 * CONFIG_CPU == S5L8702 arm of the #if. That header is the only written
 * specification of this block we have; the divider encodings in the comments
 * below are quoted from it.
 *
 * This replaces the iPod Touch (S5L8900) clock model the machine used to
 * borrow. The two layouts agree up to 0x4C by accident of history, but the
 * S5L8702 has three registers the S5L8900 does not -- SWRCON, RSTSR and
 * PLLMOD2 -- and Apple's retailOS writes all three. The borrowed model called
 * hw_error() on them, which aborts the whole QEMU process.
 */

#ifndef HW_ARM_IPOD_CLASSIC_CLK_H
#define HW_ARM_IPOD_CLASSIC_CLK_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_IPOD_CLASSIC_CLK "s5l8702.clk"
OBJECT_DECLARE_SIMPLE_TYPE(IPodClassicClkState, IPOD_CLASSIC_CLK)

/*
 * The block decodes far less than a megabyte; 0x80 covers every register
 * Rockbox names. The machine puts an unimplemented-device floor under the
 * whole peripheral window, so anything above this is logged rather than
 * aborting.
 */
#define S5L8702_CLK_MEM_SIZE   0x80

#define S5L8702_CLK_CLKCON0    0x00
#define S5L8702_CLK_CLKCON1    0x04   /* CPU/AHB/APB dividers */
#define S5L8702_CLK_CLKCON2    0x08
#define S5L8702_CLK_CLKCON3    0x0C
#define S5L8702_CLK_CLKCON4    0x10
#define S5L8702_CLK_CLKCON5    0x14
#define S5L8702_CLK_PLLPMS0    0x20   /* PLLPMS(i) = 0x20 + i * 4 */
#define S5L8702_CLK_PLLPMS1    0x24
#define S5L8702_CLK_PLLPMS2    0x28
#define S5L8702_CLK_PLLCNT0    0x30   /* PLLCNT(i) = 0x30 + i * 4 */
#define S5L8702_CLK_PLLCNT1    0x34
#define S5L8702_CLK_PLLCNT2    0x38
#define S5L8702_CLK_PLLLOCK    0x40
#define S5L8702_CLK_PLLMODE    0x44
#define S5L8702_CLK_PWRCON0    0x48   /* AHB clock gates */
#define S5L8702_CLK_PWRCON1    0x4C   /* APB clock gates */
#define S5L8702_CLK_SWRCON     0x50   /* software reset */
#define S5L8702_CLK_RSTSR      0x54   /* reset status */
#define S5L8702_CLK_PWRCON2    0x58
#define S5L8702_CLK_PLLMOD2    0x60
#define S5L8702_CLK_PWRCON3    0x68
#define S5L8702_CLK_PWRCON4    0x6C

/*
 * PLLMODE bit n enables PLL n. PLLLOCK reports "started" in bits 0..2 and
 * "divisor mode locked" in bits 4..6.
 */
#define S5L8702_PLLLOCK_LCK(n)    (1u << (n))
#define S5L8702_PLLLOCK_DMLCK(n)  (1u << (4 + (n)))

/* RSTSR: which event caused the last reset. */
#define S5L8702_RSTSR_HWR  (1u << 0)   /* hardware / power on */
#define S5L8702_RSTSR_SWR  (1u << 1)   /* SWRCON */
#define S5L8702_RSTSR_WDR  (1u << 2)   /* watchdog */

/*
 * SWRCON: any non-zero write resets the machine.  The older retailOS's reboot
 * did "WDTCON = 0; SWRCON = 0xaa5" and spun; Rockbox never writes it.
 */

struct IPodClassicClkState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t reg[S5L8702_CLK_MEM_SIZE / 4];
};

#endif
