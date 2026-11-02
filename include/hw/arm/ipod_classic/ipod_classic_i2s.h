/*
 * S5L8702 I2S transmitter/receiver, 0x3CA00000 (I2S_BASE).
 *
 * This is the audio data path. The ARM (retailOS) or the audio DSP fills a PCM
 * buffer in DRAM, DMAC0 (a PL080) streams it 16 bits at a time into this
 * block's TX FIFO at +0x10, and the serialiser clocks it out to the CS42L55
 * codec at the sample rate set by I2SCLKDIV.
 *
 * The register map is Rockbox's (firmware/export/s5l87xx.h:1186-1196), and every
 * value the retailOS writes was confirmed live (see wiki/14-classic6g-audio.md):
 *
 *   +0x00 I2SCLKCON  clock enable          retailOS writes 1
 *   +0x04 I2STXCON   TX configuration      retailOS writes 0x0b100019
 *   +0x08 I2STXCOM   TX command            start=0x6 (retailOS), 0xe/0xa (Rockbox)
 *   +0x10 I2STXDB0   TX data FIFO          <- DMAC0 writes PCM samples here
 *   +0x30 I2SRXCON   RX configuration      retailOS writes 0x1000
 *   +0x34 I2SRXCOM   RX command
 *   +0x38 I2SRXDB    RX data FIFO
 *   +0x3C I2SSTATUS  status
 *   +0x40 I2SCLKDIV  MCLK/samplerate       retailOS writes 0x110=272 (44100 Hz)
 *
 * Format (VERIFIED, both reports): 16-bit, stereo (L/R interleaved), 44100 Hz by
 * default. MCLK = 12 MHz, so I2SCLKDIV = 12e6 / samplerate.
 */

#ifndef HW_ARM_IPOD_CLASSIC_I2S_H
#define HW_ARM_IPOD_CLASSIC_I2S_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "qemu/timer.h"
#include "hw/dma/pl080.h"

#define TYPE_IPOD_CLASSIC_I2S "s5l8702.i2s"
OBJECT_DECLARE_SIMPLE_TYPE(IPodClassicI2SState, IPOD_CLASSIC_I2S)

/* The window we answer for; the rest of the 1 MB block falls to the io floor. */
#define S5L8702_I2S_MEM_SIZE   0x1000

/* Register offsets from I2S_BASE. */
#define S5L8702_I2S_CLKCON     0x00
#define S5L8702_I2S_TXCON      0x04
#define S5L8702_I2S_TXCOM      0x08
#define S5L8702_I2S_TXDB0      0x10
#define S5L8702_I2S_RXCON      0x30
#define S5L8702_I2S_RXCOM      0x34
#define S5L8702_I2S_RXDB       0x38
#define S5L8702_I2S_STATUS     0x3c
#define S5L8702_I2S_CLKDIV     0x40

/*
 * TX command values. Rockbox uses 0xe=start / 0xa=stop (pcm-s5l8702.c:147,155);
 * the retailOS uses 0x6=start (VERIFIED live, I2STRACE at pc~0x22008774). The
 * bit that tells start from stop is bit 2 (0x4): start 0x6 and 0xe set it, stop
 * 0xa clears it. Bit 1 (0x2) is set even in Rockbox's stop word 0xa, so it is
 * NOT the run bit -- an earlier version used 0x2 here and would have read a
 * Rockbox stop as "still running".
 */
#define S5L8702_I2S_TXCOM_RUN  0x4

/* MCLK feeding the I2S divider, 12 MHz (Rockbox pcm-s5l8702.c:160 MCLK_FREQ). */
#define S5L8702_I2S_MCLK_HZ    12000000

/*
 * The DMAC0 request line the TX FIFO drives. IIS0_TX = 0xA
 * (dma-s5l8702.h:50). The PL080 selects it in the channel CONFIG register's
 * DestPeripheral field, so a mem->peri channel targeting the FIFO has
 * dest_id == 0xA and is paced by request bit (1 << 0xA).
 */
#define S5L8702_I2S_DMA_IIS0_TX  0xA

struct IPodClassicI2SState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    /* Register file, read back honestly. */
    uint32_t clkcon;
    uint32_t txcon;
    uint32_t txcom;
    uint32_t rxcon;
    uint32_t rxcom;
    uint32_t rxdb;
    uint32_t clkdiv;

    /*
     * Pacing. The FIFO is drained at the sample rate by this timer, which lets
     * DMAC0 move what the clock has used so the PL080 hands over the audio as
     * it plays instead of emptying the whole buffer at once. Only armed while
     * the transmitter is running and has something to send.
     */
    QEMUTimer *drain_timer;
    PL080State *dmac0;      /* set by the machine; may be NULL */
    bool tx_running;
    int64_t pace_last_ns;   /* the sample clock has been paid for up to here */
    uint64_t pace_frac;     /* what did not make a whole byte, in ns*bytes/s */
    uint64_t pace_due;      /* bytes owed: clocked out and not yet moved */

    /* Running count of 16-bit samples clocked out, for diagnostics. */
    uint64_t tx_samples;
};

/* Give the I2S the DMAC0 it paces. Called from the machine after both exist. */
void ipod_classic_i2s_set_dmac0(IPodClassicI2SState *s, PL080State *dmac0);

#endif
