/*
 * S5L8702 GPIO. See include/hw/arm/ipod_classic/ipod_classic_gpio.h for the
 * register map, for how Apple's gpio_get_pin() indexes it, and for the one
 * measurement this device exists to satisfy: pin 86 gates whether the storage
 * task tells the volume manager "mount" or "no media", and with it stuck at 0
 * the whole user interface waits for ever.
 *
 * Deliberately minimal. Reads of PDAT return modelled INPUT levels, which are
 * all zero except pin 86; that is bit-for-bit what create_unimplemented_device()
 * used to return, so nothing else in the machine can have changed behaviour.
 * The other thing it does is tell the wheel when the guest cuts its link, which
 * is how the hold switch reaches the wheel; see the header.
 *
 * GPIOTRACE=1 prints every access with the guest PC, like ATATRACE and
 * LCDTRACE.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/core/cpu.h"
#include "hw/arm/ipod_classic/ipod_classic_gpio.h"

static int gpiotrace = -1;

static bool gpio_trace_on(void)
{
    if (gpiotrace < 0) {
        const char *e = getenv("GPIOTRACE");
        gpiotrace = (e && *e && *e != '0') ? 1 : 0;
    }
    return gpiotrace == 1;
}

/* Head of the running translation block, not the address of the access. */
static uint32_t gpio_trace_pc(void)
{
    CPUState *cs = current_cpu;
    return cs ? (uint32_t)CPU_GET_CLASS(cs)->get_pc(cs) : 0;
}

static uint64_t ipod_classic_gpio_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    IPodClassicGPIOState *s = opaque;
    unsigned group = offset / S5L8702_GPIO_STRIDE;
    unsigned reg = offset % S5L8702_GPIO_STRIDE;
    uint64_t val = 0;

    if (group < S5L8702_GPIO_GROUPS) {
        switch (reg) {
        case S5L8702_GPIO_PCON:
            val = s->pcon[group];
            break;
        case S5L8702_GPIO_PDAT:
            val = s->pdat[group];
            break;
        case S5L8702_GPIO_PUNA:
            val = s->puna[group];
            break;
        case S5L8702_GPIO_PUNB:
            val = s->punb[group];
            break;
        case S5L8702_GPIO_PUNC:
            val = s->punc[group];
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "s5l8702.gpio: read of group %u +0x%x\n",
                          group, reg);
            break;
        }
    } else {
        qemu_log_mask(LOG_UNIMP, "s5l8702.gpio: read of +0x%" HWADDR_PRIx
                      ", past the sixteen groups\n", offset);
    }

    if (gpio_trace_on()) {
        fprintf(stderr, "GPIOTRACE rd +0x%03" HWADDR_PRIx " -> 0x%08x  "
                "pc~0x%08x\n", offset, (uint32_t)val, gpio_trace_pc());
    }
    return val;
}

/*
 * A pin driven as an output: PCON nibble 1 as Rockbox's gpio_group_get() reads
 * it, or 0xe/0xf as both firmwares write it.
 */
static bool gpio_pin_output(IPodClassicGPIOState *s, unsigned group,
                            unsigned pin)
{
    unsigned func = (s->pcon[group] >> (pin * 4)) & 0xf;

    return func == 0x1 || func == 0xe || func == 0xf;
}

static void gpio_update_wheel_link(IPodClassicGPIOState *s)
{
    qemu_set_irq(s->wheel_cut,
                 gpio_pin_output(s, S5L8702_GPIO_WHEEL_GROUP,
                                 S5L8702_GPIO_WHEEL_PIN_A) &&
                 gpio_pin_output(s, S5L8702_GPIO_WHEEL_GROUP,
                                 S5L8702_GPIO_WHEEL_PIN_B));
}

static void ipod_classic_gpio_cmd(IPodClassicGPIOState *s, uint32_t value)
{
    unsigned group = (value >> 16) & 0xff;
    unsigned pin = (value >> 8) & 0x7;

    if (group >= S5L8702_GPIO_GROUPS) {
        qemu_log_mask(LOG_UNIMP, "s5l8702.gpio: GPIOCMD 0x%x, past the sixteen "
                      "groups\n", value);
        return;
    }
    s->pcon[group] = deposit32(s->pcon[group], pin * 4, 4, value & 0xf);
    gpio_update_wheel_link(s);
}

static void ipod_classic_gpio_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
    IPodClassicGPIOState *s = opaque;
    unsigned group = offset / S5L8702_GPIO_STRIDE;
    unsigned reg = offset % S5L8702_GPIO_STRIDE;

    if (gpio_trace_on()) {
        fprintf(stderr, "GPIOTRACE wr +0x%03" HWADDR_PRIx " = 0x%08x  "
                "pc~0x%08x\n", offset, (uint32_t)value, gpio_trace_pc());
    }

    if (offset == S5L8702_GPIO_CMD) {
        ipod_classic_gpio_cmd(s, value);
        return;
    }

    if (group >= S5L8702_GPIO_GROUPS) {
        qemu_log_mask(LOG_UNIMP, "s5l8702.gpio: write of 0x%" PRIx64 " to +0x%"
                      HWADDR_PRIx ", past the sixteen groups\n", value, offset);
        return;
    }

    switch (reg) {
    case S5L8702_GPIO_PCON:
        s->pcon[group] = value;
        gpio_update_wheel_link(s);
        break;
    case S5L8702_GPIO_PUNA:
        s->puna[group] = value;
        break;
    case S5L8702_GPIO_PUNB:
        s->punb[group] = value;
        break;
    case S5L8702_GPIO_PUNC:
        s->punc[group] = value;
        break;

    case S5L8702_GPIO_PDAT:
        /*
         * Driving an output. Dropped on purpose: PDAT reads report modelled
         * INPUT levels, and letting a write change what a later read returns
         * would silently invent a loopback the hardware may not have. Nothing
         * in the boot depends on reading back a pin it wrote -- that was
         * already true when this window was an unimplemented device that read
         * as zero.
         */
        qemu_log_mask(LOG_UNIMP, "s5l8702.gpio: PDAT(%u) = 0x%" PRIx64
                      " ignored; outputs are not modelled\n", group, value);
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "s5l8702.gpio: write of 0x%" PRIx64
                      " to group %u +0x%x\n", value, group, reg);
        break;
    }
}

static const MemoryRegionOps ipod_classic_gpio_ops = {
    .read = ipod_classic_gpio_read,
    .write = ipod_classic_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void ipod_classic_gpio_reset(DeviceState *dev)
{
    IPodClassicGPIOState *s = IPOD_CLASSIC_GPIO(dev);

    memset(s->pcon, 0, sizeof(s->pcon));
    memset(s->puna, 0, sizeof(s->puna));
    memset(s->punb, 0, sizeof(s->punb));
    memset(s->punc, 0, sizeof(s->punc));
    memset(s->pdat, 0, sizeof(s->pdat));

    /*
     * The one modelled input. CONVENIENCE VALUE, and it is the point of this
     * device: 0x080aeca4 reads pin 86 to decide "mount" against "no media", so
     * with a drive attached it has to be 1 or the user interface never starts.
     * What the pin is actually wired to on the real board is UNDETERMINED.
     */
    if (s->media_present) {
        s->pdat[S5L8702_GPIO_PIN_MEDIA / 8] |=
            1u << (S5L8702_GPIO_PIN_MEDIA % 8);
    }
    gpio_update_wheel_link(s);
}

static Property ipod_classic_gpio_props[] = {
    DEFINE_PROP_BOOL("media-present", IPodClassicGPIOState, media_present,
                     false),
    DEFINE_PROP_END_OF_LIST(),
};

static void ipod_classic_gpio_realize(DeviceState *dev, Error **errp)
{
    IPodClassicGPIOState *s = IPOD_CLASSIC_GPIO(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &ipod_classic_gpio_ops, s,
                          TYPE_IPOD_CLASSIC_GPIO, S5L8702_GPIO_MEM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    qdev_init_gpio_out_named(dev, &s->wheel_cut, "wheel-cut", 1);
}

static const VMStateDescription vmstate_ipod_classic_gpio = {
    .name = TYPE_IPOD_CLASSIC_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(pcon, IPodClassicGPIOState, S5L8702_GPIO_GROUPS),
        VMSTATE_UINT32_ARRAY(pdat, IPodClassicGPIOState, S5L8702_GPIO_GROUPS),
        VMSTATE_UINT32_ARRAY(puna, IPodClassicGPIOState, S5L8702_GPIO_GROUPS),
        VMSTATE_UINT32_ARRAY(punb, IPodClassicGPIOState, S5L8702_GPIO_GROUPS),
        VMSTATE_UINT32_ARRAY(punc, IPodClassicGPIOState, S5L8702_GPIO_GROUPS),
        VMSTATE_END_OF_LIST()
    },
};

static void ipod_classic_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "S5L8702 GPIO";
    dc->realize = ipod_classic_gpio_realize;
    dc->reset = ipod_classic_gpio_reset;
    dc->vmsd = &vmstate_ipod_classic_gpio;
    dc->user_creatable = false;
    device_class_set_props(dc, ipod_classic_gpio_props);
}

static const TypeInfo ipod_classic_gpio_info = {
    .name          = TYPE_IPOD_CLASSIC_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodClassicGPIOState),
    .class_init    = ipod_classic_gpio_class_init,
};

static void ipod_classic_gpio_register_types(void)
{
    type_register_static(&ipod_classic_gpio_info);
}

type_init(ipod_classic_gpio_register_types)
