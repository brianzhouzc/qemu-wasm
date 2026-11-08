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
#include "hw/core/cpu.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "hw/arm/ipod_classic/ipod_classic_cs42l55.h"
#include "hw/arm/ipod_classic/ipod_classic_audio_out.h"

/* CS42TRACE=1 prints every register write, the same pattern as I2STRACE. */
static int cs42trace = -1;

static bool cs42_trace_on(void)
{
    if (cs42trace < 0) {
        const char *e = getenv("CS42TRACE");
        cs42trace = (e && *e && *e != '0') ? 1 : 0;
    }
    return cs42trace == 1;
}

/*
 * The three volumes, as the register tables in the datasheet (DS773F1, 6.15,
 * 6.20 and 6.21) give them.  None is plain two's complement: each counts down
 * from its top code, 0x18, through 0 and round to its bottom, so a code above
 * the top is not a gain but the far end of the attenuation, or a clamp.
 */

/*
 * Headphone amp: 1 dB steps, +12 dB from 0x0c to 0x3f, -60 dB from 0x40 to
 * 0x44.  The datasheet says the bottom is about -58 dB in fact and the steps
 * may stray from 1 dB (its figure 26); this keeps to the table.
 */
static double cs42l55_hp_db(uint8_t raw)
{
    int v = sextract32(raw, 0, 7);

    return MAX(-60, MIN(12, v));
}

/* PCM mixer: 0.5 dB steps, +12 dB at 0x18, -0.5 dB at 0x7f, -51.5 dB at 0x19. */
static double cs42l55_pmix_db(uint8_t raw)
{
    int v = raw & CS42L55_VOL_MASK;

    return (v <= 0x18 ? v : v - 0x80) * 0.5;
}

/* Master: 0.5 dB steps, +12 dB at 0x18, -0.5 at 0xff, -102 from 0x34 to 0x19. */
static double cs42l55_master_db(uint8_t raw)
{
    return MAX(-102.0, (raw <= 0x18 ? raw : raw - 0x100) * 0.5);
}

/*
 * The gain from the serial input to the headphone amp, for one channel.  The
 * PCM mixer is in the DSP engine and powering that down (PDN_DSP) takes it out
 * of the path, its mute with it; the master volume and its mute come after the
 * engine, ahead of the DAC, and apply either way (6.13.1, and 4.6: the master
 * volume sets the beep's level too).  PLYBCKB=A gangs channel B's volumes to
 * channel A's registers but leaves B's mutes its own (6.13.3).
 */
static double cs42l55_channel_db(const uint8_t *reg, int channel)
{
    uint8_t play = reg[CS42L55_REG_PLAYCTL];
    uint8_t amp_off = channel ? CS42L55_PWRCTL2_PDN_HPB
                              : CS42L55_PWRCTL2_PDN_HPA;
    uint8_t mute = channel ? CS42L55_PLAYCTL_MSTBMUTE
                           : CS42L55_PLAYCTL_MSTAMUTE;
    int vol = (play & CS42L55_PLAYCTL_PLYBCKB_A) ? 0 : channel;
    uint8_t hp = reg[CS42L55_REG_HPACTL + channel];
    uint8_t pmix = reg[CS42L55_REG_PMIXACTL + channel];
    bool dsp = !(play & CS42L55_PLAYCTL_PDN_DSP);
    double db;

    if ((reg[CS42L55_REG_PWRCTL1] & CS42L55_PWRCTL1_PDN_CODEC) ||
        (reg[CS42L55_REG_PWRCTL2] & amp_off) == amp_off || (play & mute) ||
        (hp & CS42L55_VOL_MUTE) || (dsp && (pmix & CS42L55_VOL_MUTE))) {
        return -INFINITY;
    }
    db = cs42l55_hp_db(reg[CS42L55_REG_HPACTL + vol]) +
         cs42l55_master_db(reg[CS42L55_REG_MSTAVOL + vol]);
    if (dsp) {
        db += cs42l55_pmix_db(reg[CS42L55_REG_PMIXACTL + vol]);
    }
    return db;
}

static uint32_t cs42l55_channel_gain(IPodClassicCS42L55State *s, int channel)
{
    double db = cs42l55_channel_db(s->reg, channel);

    return isinf(db) ? 0 : lrint(pow(10.0, db / 20.0) * IPOD_AUDIO_OUT_UNITY);
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
        if (cs42_trace_on()) {
            CPUState *cs = current_cpu;

            fprintf(stderr, "CS42TRACE wr 0x%02x = 0x%02x  pc~0x%08x\n", s->addr,
                    data, cs ? (uint32_t)CPU_GET_CLASS(cs)->get_pc(cs) : 0);
        }
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
