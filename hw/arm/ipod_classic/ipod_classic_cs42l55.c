/*
 * Cirrus Logic CS42L55 audio codec, I2C0 slave 0x4A.
 *
 * See include/hw/arm/ipod_classic/ipod_classic_cs42l55.h for the address and
 * protocol, and wiki/14-classic6g-audio.md for the audio path.
 *
 * The chip is a register-pointer I2C device, exactly like the PMU next to it on
 * the same bus: a write starts with the register number and the pointer
 * advances after every byte; a read carries on from wherever the pointer was
 * left. That is what makes the firmware's read-modify-write of a codec register
 * (read reg, change some bits, write it back) come out right. While this slave
 * did not exist the retailOS's writes to 0x4A were NACKed and every cscodec_read
 * returned bus garbage.
 *
 * Gain is worked out here and applied where the sound is played: the path from
 * here to the speakers already holds a tenth of a second of audio, so a change
 * cannot take effect any sooner than that either way, and the guest's DMA should
 * not spend time multiplying samples for a page that may be muted.
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "hw/arm/ipod_classic/ipod_classic_cs42l55.h"
#include "hw/arm/ipod_classic/ipod_classic_audio_out.h"

/* The volume registers are two's complement, of 7 bits or of 8. */
static double cs42l55_db(uint8_t raw, int bits, double step)
{
    return sextract32(raw, 0, bits) * step;
}

static uint32_t cs42l55_channel_gain(IPodClassicCS42L55State *s, int channel)
{
    uint8_t power = s->reg[CS42L55_REG_PWRCTL1];
    uint8_t amp = s->reg[CS42L55_REG_PWRCTL2];
    uint8_t play = s->reg[CS42L55_REG_PLAYCTL];
    uint8_t hp = s->reg[CS42L55_REG_HPACTL + channel];
    uint8_t pmix = s->reg[CS42L55_REG_PMIXACTL + channel];
    uint8_t amp_off = channel ? CS42L55_PWRCTL2_PDN_HPB
                              : CS42L55_PWRCTL2_PDN_HPA;
    uint8_t mute = channel ? CS42L55_PLAYCTL_MSTBMUTE
                           : CS42L55_PLAYCTL_MSTAMUTE;
    double db;

    if ((power & CS42L55_PWRCTL1_PDN_CODEC) || (amp & amp_off) == amp_off ||
        (play & mute) || (hp & CS42L55_VOL_MUTE) ||
        (pmix & CS42L55_VOL_MUTE)) {
        return 0;
    }
    db = cs42l55_db(hp, 7, 1.0) + cs42l55_db(pmix, 7, 0.5);
    /* The master volume is in the signal processing engine. */
    if (!(play & CS42L55_PLAYCTL_PDN_DSP)) {
        db += cs42l55_db(s->reg[CS42L55_REG_MSTAVOL + channel], 8, 0.5);
    }
    return lrint(pow(10.0, db / 20.0) * IPOD_AUDIO_OUT_UNITY);
}

static void cs42l55_publish_gain(IPodClassicCS42L55State *s)
{
    ipod_classic_audio_out_set_gain(cs42l55_channel_gain(s, 0),
                                    cs42l55_channel_gain(s, 1));
}

static int ipod_classic_cs42l55_event(I2CSlave *i2c, enum i2c_event event)
{
    IPodClassicCS42L55State *s = IPOD_CLASSIC_CS42L55(i2c);

    switch (event) {
    case I2C_START_SEND:
        /* The next byte is the register number, not data. */
        s->addr_pending = true;
        break;
    case I2C_START_RECV:
    case I2C_FINISH:
    case I2C_NACK:
        break;
    default:
        break;
    }
    return 0;
}

static uint8_t ipod_classic_cs42l55_recv(I2CSlave *i2c)
{
    IPodClassicCS42L55State *s = IPOD_CLASSIC_CS42L55(i2c);
    uint8_t val = s->reg[s->addr & (S5L8702_CS42L55_NUM_REGS - 1)];

    s->addr++;
    return val;
}

static int ipod_classic_cs42l55_send(I2CSlave *i2c, uint8_t data)
{
    IPodClassicCS42L55State *s = IPOD_CLASSIC_CS42L55(i2c);

    if (s->addr_pending) {
        s->addr = data;
        s->addr_pending = false;
    } else {
        s->reg[s->addr & (S5L8702_CS42L55_NUM_REGS - 1)] = data;
        s->addr++;
        cs42l55_publish_gain(s);
    }
    return 0;
}

static void ipod_classic_cs42l55_reset(DeviceState *dev)
{
    IPodClassicCS42L55State *s = IPOD_CLASSIC_CS42L55(dev);

    memset(s->reg, 0, sizeof(s->reg));
    s->addr = 0;
    s->addr_pending = false;
    s->reg[CS42L55_REG_CHIPVERSION] = S5L8702_CS42L55_CHIP_ID;
    cs42l55_publish_gain(s);
}

static int ipod_classic_cs42l55_post_load(void *opaque, int version_id)
{
    cs42l55_publish_gain(opaque);
    return 0;
}

static const VMStateDescription vmstate_ipod_classic_cs42l55 = {
    .name = TYPE_IPOD_CLASSIC_CS42L55,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ipod_classic_cs42l55_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, IPodClassicCS42L55State),
        VMSTATE_UINT8_ARRAY(reg, IPodClassicCS42L55State,
                            S5L8702_CS42L55_NUM_REGS),
        VMSTATE_UINT8(addr, IPodClassicCS42L55State),
        VMSTATE_BOOL(addr_pending, IPodClassicCS42L55State),
        VMSTATE_END_OF_LIST()
    },
};

static void ipod_classic_cs42l55_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->desc = "Cirrus Logic CS42L55 audio codec";
    dc->reset = ipod_classic_cs42l55_reset;
    dc->vmsd = &vmstate_ipod_classic_cs42l55;
    k->event = ipod_classic_cs42l55_event;
    k->recv = ipod_classic_cs42l55_recv;
    k->send = ipod_classic_cs42l55_send;
}

static const TypeInfo ipod_classic_cs42l55_info = {
    .name          = TYPE_IPOD_CLASSIC_CS42L55,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(IPodClassicCS42L55State),
    .class_init    = ipod_classic_cs42l55_class_init,
};

static void ipod_classic_cs42l55_register_types(void)
{
    type_register_static(&ipod_classic_cs42l55_info);
}

type_init(ipod_classic_cs42l55_register_types)
