/*
 * Cirrus Logic CS42L55 audio codec of the iPod classic 6G, on I2C0 at 7-bit
 * address 0x4A (8-bit 0x94).
 *
 * Both drivers agree on the address and protocol. Rockbox: cscodec-6g.c:39
 * does i2c_write(0, 0x94, reg, 1, &data) -- bus 0, 8-bit slave 0x94, a
 * sub-address (register number) then one data byte. Apple's retailOS: the
 * cscodec_write veneer at osos VA 0x0808e1ac loads r0=0x4a and calls the same
 * I2C transfer routine the PMU uses (0x08325a20). See wiki/14.
 *
 * To a driver this chip is a 128-register file. Every access is either a plain
 * write or a read-modify-write (Rockbox cscodec_setbits, cs42l55.c:49; Apple's
 * init is the same shape), so a register file with honest readback is all the
 * firmware needs. Nothing in either driver reads CHIPVERSION (0x01), but it is
 * seeded with a plausible id in case a future path checks it.
 *
 * What the listener hears does depend on it: the registers below set the gain
 * of the path from the I2S to the headphones, and the model publishes that to
 * the machine's audio output as they change. The iPod has no speaker, and the
 * dock's line out, the beeper and the tone controls are not modelled.
 */

#ifndef HW_ARM_IPOD_CLASSIC_CS42L55_H
#define HW_ARM_IPOD_CLASSIC_CS42L55_H

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_IPOD_CLASSIC_CS42L55 "cs42l55"
OBJECT_DECLARE_SIMPLE_TYPE(IPodClassicCS42L55State, IPOD_CLASSIC_CS42L55)

#define S5L8702_CS42L55_I2C_ADDR   0x4a   /* 0x94 >> 1 */
#define S5L8702_CS42L55_NUM_REGS   0x80

#define CS42L55_REG_CHIPVERSION    0x01
#define CS42L55_REG_PWRCTL1        0x02
#define CS42L55_PWRCTL1_PDN_CODEC  0x01
#define CS42L55_REG_PWRCTL2        0x03
#define CS42L55_PWRCTL2_PDN_HPA    0x30   /* 3 << 4 is "always off" */
#define CS42L55_PWRCTL2_PDN_HPB    0xc0
#define CS42L55_REG_PLAYCTL        0x0f
#define CS42L55_PLAYCTL_MSTAMUTE   0x01
#define CS42L55_PLAYCTL_MSTBMUTE   0x02
#define CS42L55_PLAYCTL_PDN_DSP    0x80
#define CS42L55_REG_PMIXACTL       0x12   /* PCM mixer, 0.5 dB steps */
#define CS42L55_REG_PMIXBCTL       0x13
#define CS42L55_REG_MSTAVOL        0x18   /* master, 0.5 dB steps */
#define CS42L55_REG_MSTBVOL        0x19
#define CS42L55_REG_HPACTL         0x1a   /* headphone amp, 1 dB steps */
#define CS42L55_REG_HPBCTL         0x1b
#define CS42L55_VOL_MASK           0x7f
#define CS42L55_VOL_MUTE           0x80
/*
 * CONVENIENCE. The CS42L55 reports its device id in the high bits of register
 * 0x01. No dump of a real part is in our sources and no driver we have reads
 * it; 0x5A is the value the freemyipod research notes for this codec. If a
 * firmware path is found that compares it, this is where it gets a real value.
 */
#define S5L8702_CS42L55_CHIP_ID    0x5a

struct IPodClassicCS42L55State {
    I2CSlave parent_obj;

    uint8_t reg[S5L8702_CS42L55_NUM_REGS];
    uint8_t addr;         /* register pointer, survives a STOP like the PMU */
    bool addr_pending;    /* next byte written is the register number */
};

#endif
