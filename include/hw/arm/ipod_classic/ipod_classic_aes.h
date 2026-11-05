/*
 * S5L8702 AES engine.
 *
 * The register map is Rockbox's (firmware/export/s5l87xx.h, "Hardware AES
 * crypto unit - S5L8701+", and firmware/target/arm/s5l8702/crypto-s5l8702.c),
 * which agrees with openiBoot's S5L8900 driver (plat-s5l8900/aes.c) on every
 * register except the in/out address pair, which openiBoot swaps.  Both
 * drivers only ever run in place, so neither could tell; Apple's retailOS
 * 2.0.5 decrypts out of place, and MEASURED on it: the ciphertext is at
 * +0x28 and the buffer +0x20 points at holds stale heap.  Rockbox's names
 * are the right ones.
 */
#ifndef HW_ARM_IPOD_CLASSIC_AES_H
#define HW_ARM_IPOD_CLASSIC_AES_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_IPOD_CLASSIC_AES "s5l8702.aes"
OBJECT_DECLARE_SIMPLE_TYPE(IPodClassicAESState, IPOD_CLASSIC_AES)

/* The whole 1 MB block; the registers are the first 0x90 bytes of it. */
#define S5L8702_AES_MEM_SIZE    0x100000
#define S5L8702_AES_REGS_SIZE   0x100

#define S5L8702_AES_CONTROL     0x00
#define S5L8702_AES_GO          0x04   /* write 1: run the job */
#define S5L8702_AES_UNKREG0     0x08   /* pulsed 1 then 0 before a job */
#define S5L8702_AES_STATUS      0x0C   /* bit 0: done; write 1 to clear */
#define S5L8702_AES_UNKREG1     0x10
#define S5L8702_AES_KEYLEN      0x14   /* see the bits below */
#define S5L8702_AES_OUTSIZE     0x18
#define S5L8702_AES_OUTADDR     0x20
#define S5L8702_AES_INSIZE      0x24
#define S5L8702_AES_INADDR      0x28
#define S5L8702_AES_AUXSIZE     0x2C
#define S5L8702_AES_AUXADDR     0x30
#define S5L8702_AES_SIZE3       0x34
#define S5L8702_AES_KEY         0x4C   /* 8 words, a shorter key at the end */
#define S5L8702_AES_TYPE        0x6C   /* which key: see below */
#define S5L8702_AES_IV          0x74   /* 4 words */
#define S5L8702_AES_TYPE2       0x88   /* ~TYPE, in Rockbox's driver */
#define S5L8702_AES_UNKREG2     0x8C

#define S5L8702_AES_KEY_WORDS   8
#define S5L8702_AES_IV_WORDS    4

/* The most one job runs: the size of DRAM.  See ipod_classic_aes_run(). */
#define S5L8702_AES_MAX_JOB     0x4000000u

/*
 * KEYLEN.  Bit 0 is the direction (1 encrypt, 0 decrypt) in every driver.
 * Bits 16-17 are the key length in openiBoot (GET_KEYLEN: 0 for AES-128, 1
 * for 192, 2 for 256), and a key shorter than 256 bits sits at the end of
 * the key registers.  openiBoot's job sets bits 1-3 (6, then bit 3), which is
 * 0xe for a decrypt: exactly what retailOS 2.0.5 writes for every job.  What
 * bits 1-5 select is not known; every driver we have uses CBC with the IV
 * registers, and this model does the same whatever they hold.
 */
#define S5L8702_AES_KEYLEN_ENCRYPT  (1u << 0)
#define S5L8702_AES_KEYLEN_SIZE_SHIFT 16

/* TYPE: 0 the key registers, 1 and 2 the chip's own keys (GID and UID). */
#define S5L8702_AES_TYPE_CUSTOM 0
#define S5L8702_AES_TYPE_GID    1
#define S5L8702_AES_TYPE_UID    2

/* STATUS bit 0, what retailOS 2.0.5 polls for (Rockbox and openiBoot: 0xf). */
#define S5L8702_AES_STATUS_DONE (1u << 0)

struct IPodClassicAESState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t reg[S5L8702_AES_REGS_SIZE / 4];
    /* Jobs run since reset; for AESTRACE and the vmstate. */
    uint32_t jobs;
};

#endif
