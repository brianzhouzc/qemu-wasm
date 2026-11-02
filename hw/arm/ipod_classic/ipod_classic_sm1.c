/*
 * S5L8702 "SM1" audio decoder engine, 0x38500000.
 *
 * See include/hw/arm/ipod_classic/ipod_classic_sm1.h for what the block is.
 * The firmware's side of it, in order:
 *
 *   1. Loads the first half of a program and writes +0x984; the bootstrap
 *      clears the mailbox and raises the event.  The firmware acknowledges.
 *   2. Loads the rest, fills in the mailbox and writes +0x988.
 *   3. For each compressed frame, writes +0x98c (nothing here needs that),
 *      copies the frame to a buffer, points channel 4 at a descriptor for it,
 *      then waits up to 500 ms on a semaphore line 18 signals, twice before
 *      calling it the end of the stream (0x220061a0).
 *   4. Reads the PCM out of a ring in IRAM1.
 *
 * So one frame goes in and one comes out, and nothing runs ahead: the firmware
 * does not offer another until it has taken the last.
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"
#include "hw/core/cpu.h"
#include "exec/cpu-common.h"
#include "hw/irq.h"
#include "hw/arm/ipod_classic/ipod_classic_sm1.h"
#include "hw/arm/ipod_classic/ipod_classic_sm1_codec.h"

/*
 * How long a frame takes.  The firmware only needs it to come after it has
 * gone to wait, and a fraction of a frame's playing time is what a DSP would
 * take.  Virtual time, so it warps with the guest idle.
 */
#define SM1_FRAME_NS (200 * SCALE_US)

/* SM1TRACE=1 prints every access with its value and the guest PC. */
static int sm1trace = -1;

static bool sm1_trace_on(void)
{
    if (sm1trace < 0) {
        const char *e = getenv("SM1TRACE");
        sm1trace = (e && *e && *e != '0') ? 1 : 0;
    }
    return sm1trace == 1;
}

static uint32_t sm1_trace_pc(void)
{
    CPUState *cs = current_cpu;
    return cs ? (uint32_t)CPU_GET_CLASS(cs)->get_pc(cs) : 0;
}

/* The state registers of the eight channels, each 4 above its command. */
static const hwaddr sm1_sub_state_off[] = {
    0x0014, 0x0034, 0x0054, 0x0074,
    0x0114, 0x0194, 0x0214, 0x0244,
};

static bool sm1_is_sub_state(hwaddr offset)
{
    for (size_t i = 0; i < ARRAY_SIZE(sm1_sub_state_off); i++) {
        if (offset == sm1_sub_state_off[i]) {
            return true;
        }
    }
    return false;
}

static uint16_t sm1_mem_read16(hwaddr addr)
{
    uint16_t v;

    cpu_physical_memory_read(addr, &v, sizeof(v));
    return le16_to_cpu(v);
}

static void sm1_mem_write16(hwaddr addr, uint16_t value)
{
    uint16_t v = cpu_to_le16(value);

    cpu_physical_memory_write(addr, &v, sizeof(v));
}

static void sm1_update_irq(IPodClassicSM1State *s)
{
    qemu_set_irq(s->irq, s->event &&
                 (s->irq_enable & S5L8702_SM1_EVENT));
}

static void sm1_set_event(IPodClassicSM1State *s)
{
    s->event = true;
    sm1_update_irq(s);
}

static void sm1_drop_codec(IPodClassicSM1State *s)
{
    ipod_classic_sm1_codec_free(s->codec);
    s->codec = NULL;
}

/* What the first half of a program does that the second depends on. */
static void sm1_boot(IPodClassicSM1State *s)
{
    uint8_t zero[S5L8702_SM1_MAILBOX_BYTES] = { 0 };

    cpu_physical_memory_write(S5L8702_SM1_MAILBOX, zero, sizeof(zero));
    timer_del(s->frame_timer);
    s->chain = 0;
    sm1_set_event(s);
}

/*
 * A stream starts here, and a format this does not know leaves codec_type at
 * none: no output, and the firmware gives up on the track and moves on, which
 * is what it did for everything before there were decoders.
 */
static void sm1_start(IPodClassicSM1State *s)
{
    uint16_t format = sm1_mem_read16(S5L8702_SM1_MAILBOX);

    sm1_drop_codec(s);
    timer_del(s->frame_timer);
    s->chain = 0;
    s->slot = 0;
    s->rate_index = sm1_mem_read16(S5L8702_SM1_MAILBOX +
                                   S5L8702_SM1_MAILBOX_RATE);
    switch (format) {
    case S5L8702_SM1_MAILBOX_MP3:
        s->codec_type = IPOD_SM1_CODEC_MP3;
        s->slot_samples = 1152 * 2;
        break;
    case S5L8702_SM1_MAILBOX_AAC:
        s->codec_type = IPOD_SM1_CODEC_AAC;
        s->slot_samples = 1024 * 2;
        break;
    default:
        s->codec_type = IPOD_SM1_CODEC_NONE;
        return;
    }
    /* The second half's own data load has been over this by now. */
    sm1_mem_write16(S5L8702_SM1_SLOTS + 2, S5L8702_SM1_SLOT_COUNT);
    sm1_mem_write16(S5L8702_SM1_SLOTS + 4, s->slot_samples);
    for (int i = 0; i < S5L8702_SM1_SLOT_COUNT; i++) {
        sm1_mem_write16(S5L8702_SM1_SLOT_FLAG(i), 0);
        sm1_mem_write16(S5L8702_SM1_SLOT_SAMPLES(i), 0);
    }
    sm1_mem_write16(S5L8702_SM1_STATUS, S5L8702_SM1_STATUS_VALID);
    sm1_mem_write16(S5L8702_SM1_STATUS + 2, 0);
}

static void sm1_frame(void *opaque)
{
    IPodClassicSM1State *s = opaque;
    uint32_t desc[S5L8702_SM1_DESC_WORDS];
    static uint8_t frame[S5L8702_SM1_FRAME_MAX];
    static int16_t pcm[IPOD_SM1_CODEC_MAX_FRAMES * 2];
    uint32_t bytes;
    unsigned frames;

    if (!s->chain || s->codec_type == IPOD_SM1_CODEC_NONE) {
        return;
    }
    if (!s->codec) {
        s->codec = ipod_classic_sm1_codec_new(s->codec_type, s->rate_index);
        if (!s->codec) {
            return;
        }
    }
    cpu_physical_memory_read(s->chain, desc, sizeof(desc));
    s->chain = 0;
    bytes = le32_to_cpu(desc[S5L8702_SM1_DESC_CTRL]) >> 16;
    if (bytes > sizeof(frame) || desc[S5L8702_SM1_DESC_NEXT]) {
        qemu_log_mask(LOG_GUEST_ERROR, "s5l8702.sm1: a chain of %u bytes, "
                      "next 0x%x\n", bytes,
                      le32_to_cpu(desc[S5L8702_SM1_DESC_NEXT]));
        bytes = MIN(bytes, sizeof(frame));
    }
    cpu_physical_memory_read(le32_to_cpu(desc[S5L8702_SM1_DESC_ADDR]), frame,
                             bytes);

    frames = ipod_classic_sm1_codec_decode(s->codec, frame, bytes, pcm);
    frames = MIN(frames, s->slot_samples / 2);
    for (unsigned i = 0; i < frames * 2; i++) {
        pcm[i] = cpu_to_le16(pcm[i]);
    }
    cpu_physical_memory_write(S5L8702_SM1_PCM +
                              s->slot * s->slot_samples * sizeof(pcm[0]),
                              pcm, frames * 2 * sizeof(pcm[0]));
    /* The flag last: it is what says the rest is there. */
    sm1_mem_write16(S5L8702_SM1_SLOT_SAMPLES(s->slot), frames * 2);
    sm1_mem_write16(S5L8702_SM1_SLOT_FLAG(s->slot), 1);
    s->slot = (s->slot + 1) % S5L8702_SM1_SLOT_COUNT;
    sm1_set_event(s);
}

static uint64_t ipod_classic_sm1_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    IPodClassicSM1State *s = opaque;
    uint32_t val;

    if (offset >= S5L8702_SM1_REG_BYTES) {
        return 0;
    }

    switch (offset) {
    case S5L8702_SM1_ID:
        val = S5L8702_SM1_ID_MAGIC;
        break;
    case S5L8702_SM1_STAT:
        val = s->event ? S5L8702_SM1_EVENT : 0;
        break;
    default:
        if (sm1_is_sub_state(offset)) {
            uint32_t cmd = s->reg[(offset - 4) / 4];

            val = (cmd == S5L8702_SM1_SUB_CMD_UP ||
                   cmd == S5L8702_SM1_SUB_CMD_RUN)
                  ? S5L8702_SM1_SUB_STATE_READY : 0;
        } else {
            /*
             * Everything else reads back what was written.  That includes
             * +0xa44 and +0xa48, four pairs 0x14 apart, which 0x080ab6d0 adds
             * to the engine's memory base (0x22020000) to place what it loads.
             * Nothing writes them, and zero is what puts the program in IRAM1.
             */
            val = s->reg[offset / 4];
        }
        break;
    }

    if (sm1_trace_on()) {
        fprintf(stderr, "SM1TRACE rd +0x%03" HWADDR_PRIx " -> 0x%08x  "
                "pc~0x%08x\n", offset, val, sm1_trace_pc());
    }
    return val;
}

static void ipod_classic_sm1_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    IPodClassicSM1State *s = opaque;

    if (offset >= S5L8702_SM1_REG_BYTES) {
        return;
    }

    if (sm1_trace_on()) {
        fprintf(stderr, "SM1TRACE wr +0x%03" HWADDR_PRIx " = 0x%08x  "
                "pc~0x%08x\n", offset, (uint32_t)value, sm1_trace_pc());
    }

    s->reg[offset / 4] = (uint32_t)value;
    switch (offset) {
    case S5L8702_SM1_BOOT:
        sm1_boot(s);
        break;
    case S5L8702_SM1_ENTRY:
        sm1_start(s);
        break;
    case S5L8702_SM1_INT_SET:
        s->irq_enable |= value;
        sm1_update_irq(s);
        break;
    case S5L8702_SM1_INT_CLEAR:
        s->irq_enable &= ~value;
        sm1_update_irq(s);
        break;
    case S5L8702_SM1_ACK:
        if (value & S5L8702_SM1_EVENT) {
            s->event = false;
            sm1_update_irq(s);
        }
        break;
    case S5L8702_SM1_FRAME_CHAIN:
        if (value & S5L8702_SM1_CHAIN_GO) {
            s->chain = value & ~3u;
            timer_mod(s->frame_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SM1_FRAME_NS);
        }
        break;
    default:
        break;
    }
}

static const MemoryRegionOps ipod_classic_sm1_ops = {
    .read = ipod_classic_sm1_read,
    .write = ipod_classic_sm1_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void ipod_classic_sm1_reset(DeviceState *dev)
{
    IPodClassicSM1State *s = IPOD_CLASSIC_SM1(dev);

    memset(s->reg, 0, sizeof(s->reg));
    s->event = false;
    s->irq_enable = 0;
    s->codec_type = IPOD_SM1_CODEC_NONE;
    s->rate_index = 0;
    s->slot_samples = 0;
    s->slot = 0;
    s->chain = 0;
    timer_del(s->frame_timer);
    sm1_drop_codec(s);
    qemu_irq_lower(s->irq);
}

static void ipod_classic_sm1_init(Object *obj)
{
    IPodClassicSM1State *s = IPOD_CLASSIC_SM1(obj);

    memory_region_init_io(&s->iomem, obj, &ipod_classic_sm1_ops, s,
                          TYPE_IPOD_CLASSIC_SM1, S5L8702_SM1_MEM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sm1_frame, s);
}

static const VMStateDescription vmstate_ipod_classic_sm1 = {
    .name = TYPE_IPOD_CLASSIC_SM1,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(reg, IPodClassicSM1State, S5L8702_SM1_REG_WORDS),
        VMSTATE_BOOL(event, IPodClassicSM1State),
        VMSTATE_UINT32(irq_enable, IPodClassicSM1State),
        VMSTATE_UINT32(codec_type, IPodClassicSM1State),
        VMSTATE_UINT32(rate_index, IPodClassicSM1State),
        VMSTATE_UINT32(slot_samples, IPodClassicSM1State),
        VMSTATE_UINT32(slot, IPodClassicSM1State),
        VMSTATE_UINT32(chain, IPodClassicSM1State),
        VMSTATE_TIMER_PTR(frame_timer, IPodClassicSM1State),
        VMSTATE_END_OF_LIST()
    },
};

static void ipod_classic_sm1_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "S5L8702 SM1 audio decoder engine";
    dc->reset = ipod_classic_sm1_reset;
    dc->vmsd = &vmstate_ipod_classic_sm1;
    dc->user_creatable = false;
}

static const TypeInfo ipod_classic_sm1_info = {
    .name          = TYPE_IPOD_CLASSIC_SM1,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodClassicSM1State),
    .instance_init = ipod_classic_sm1_init,
    .class_init    = ipod_classic_sm1_class_init,
};

static void ipod_classic_sm1_register_types(void)
{
    type_register_static(&ipod_classic_sm1_info);
}

type_init(ipod_classic_sm1_register_types)
