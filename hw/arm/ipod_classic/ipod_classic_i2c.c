/*
 * S5L8702 I2C master.
 *
 * See include/hw/arm/ipod_classic/ipod_classic_i2c.h for the register map and
 * for why the "go" bit and the interrupt flag live in different registers.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/arm/ipod_classic/ipod_classic_i2c.h"

static void ipod_classic_i2c_update_irq(IPodClassicI2CState *s)
{
    uint32_t active = s->iicstat2 & s->iiccon & S5L8702_IICCON_INT_MASK;

    qemu_set_irq(s->irq, active != 0);
}

/* Arm the "one byte has gone down the wire" event. */
static void ipod_classic_i2c_start_byte(IPodClassicI2CState *s)
{
    timer_mod(s->byte_timer, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) +
                             S5L8702_I2C_BYTE_US);
}

static void ipod_classic_i2c_byte_done(void *opaque)
{
    IPodClassicI2CState *s = opaque;

    if (getenv("I2CTRACE")) {
        fprintf(stderr, "I2C %" PRId64 " byte-done\n", g_get_real_time());
    }
    s->byte_done = true;
    s->iicstat2 |= S5L8702_IICSTAT2_BYTE;
    if (s->in_address) {
        s->iicstat2 |= S5L8702_IICSTAT2_START;
        s->in_address = false;
    }
    ipod_classic_i2c_update_irq(s);
}

static void ipod_classic_i2c_set_lrb(IPodClassicI2CState *s, int nack)
{
    if (nack) {
        s->iicstat |= S5L8702_IICSTAT_LRB;
    } else {
        s->iicstat &= ~S5L8702_IICSTAT_LRB;
    }
}

/* IICCON bit 4 written as 1 with a byte outstanding: send or receive the next. */
static void ipod_classic_i2c_go(IPodClassicI2CState *s)
{
    uint32_t mode = s->iicstat & S5L8702_IICSTAT_MODE_MASK;

    s->byte_done = false;
    s->iicds_new = false;

    switch (mode) {
    case S5L8702_IICSTAT_START_TX:
        ipod_classic_i2c_set_lrb(s, i2c_send(s->bus, s->iicds & 0xff));
        ipod_classic_i2c_start_byte(s);
        break;
    case S5L8702_IICSTAT_START_RX:
        s->iicds = i2c_recv(s->bus);
        /*
         * ACK_GEN clear means "do not acknowledge this one", which is how both
         * drivers mark the last byte of a read: Apple's handler drops the bit
         * on the way into state 22 (0x083257a8) and Rockbox writes
         * "IICCON &= ~(len ? 0 : 0x80)".
         */
        if (!(s->iiccon & S5L8702_IICCON_ACK_GEN)) {
            i2c_nack(s->bus);
        }
        ipod_classic_i2c_start_byte(s);
        break;
    default:
        /*
         * Go with no transfer running. Rockbox does exactly this in i2c_stop()
         * ("IICCON = 0x10" after clearing the busy bit), so it is not an error;
         * there is simply nothing left to send.
         */
        break;
    }
}

/* I2CTRACE=1: every register access, with host wall time, for the PMU race. */
static void i2c_trace(const char *what, hwaddr offset, uint64_t value)
{
    static int on = -1;

    if (on < 0) {
        on = getenv("I2CTRACE") != NULL;
    }
    if (on && offset != S5L8702_IICUNK10) {
        fprintf(stderr, "I2C %" PRId64 " %s %02" HWADDR_PRIx "=%" PRIx64 "\n",
                g_get_real_time(), what, offset, value);
    }
}

static uint64_t ipod_classic_i2c_read_reg(IPodClassicI2CState *s,
                                          hwaddr offset);

static uint64_t ipod_classic_i2c_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    uint64_t value = ipod_classic_i2c_read_reg(opaque, offset);

    i2c_trace("r", offset, value);
    return value;
}

static uint64_t ipod_classic_i2c_read_reg(IPodClassicI2CState *s,
                                          hwaddr offset)
{
    switch (offset) {
    case S5L8702_IICCON:
        return s->iiccon | (s->byte_done ? S5L8702_IICCON_GO : 0);
    case S5L8702_IICSTAT:
        return s->iicstat;
    case S5L8702_IICADD:
        return s->iicadd;
    case S5L8702_IICDS:
        return s->iicds;
    case S5L8702_IICUNK10:
        /*
         * Every register access in both drivers is preceded by
         * "while (IICUNK10) ;". Whatever it counts -- a clock-domain crossing,
         * most likely -- we are never busy, so it always reads zero. A model
         * that ever returned non-zero here would deadlock the guest, since
         * nothing else can clear it.
         */
        return 0;
    case S5L8702_IICUNK14:
        return s->iicunk14;
    case S5L8702_IICUNK18:
        return s->iicunk18;
    case S5L8702_IICSTAT2:
        return s->iicstat2;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: read of 0x%" HWADDR_PRIx "\n", __func__,
                      offset);
        return 0;
    }
}

static void ipod_classic_i2c_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    IPodClassicI2CState *s = opaque;

    i2c_trace("w", offset, value);
    switch (offset) {
    case S5L8702_IICCON:
        s->iiccon = value & ~S5L8702_IICCON_GO;
        if ((value & S5L8702_IICCON_GO) && s->byte_done) {
            uint32_t mode = s->iicstat & S5L8702_IICSTAT_MODE_MASK;

            /*
             * In a transmit the driver writes IICDS and then IICCON with the
             * go bit. It also writes IICCON back to itself in the interrupt
             * epilogue ("IICCON = IICCON", the same resume idiom Rockbox
             * uses), with the go bit still set. If the byte timer happens to
             * expire between the two writes -- possible when the main loop
             * and the vCPU share the host, more so in the browser -- that
             * epilogue write looked like a command and re-sent the previous
             * byte, corrupting the PMU transaction. A transmit go only means
             * something when IICDS was written since the last one; in a
             * receive the go is a clock and needs no new data.
             */
            if (mode != S5L8702_IICSTAT_START_TX || s->iicds_new) {
                ipod_classic_i2c_go(s);
            }
        }
        ipod_classic_i2c_update_irq(s);
        break;

    case S5L8702_IICSTAT: {
        uint32_t mode = value & S5L8702_IICSTAT_MODE_MASK;
        /*
         * Before the write below: the busy bit is inside the mode field, so
         * the stop's own value clears it, and a test after that never ended a
         * transfer at all.  The bus then kept the first slave it had found and
         * never scanned again, so everything addressed to the codec went to
         * the PMU, and what Rockbox and Apple read back from it was the PMU's.
         * MEASURED: every codec write on the wire, none reaching the codec.
         */
        bool busy = s->iicstat & S5L8702_IICSTAT_BUSY;

        s->iicstat = (s->iicstat & ~(S5L8702_IICSTAT_MODE_MASK |
                                     S5L8702_IICSTAT_SOE)) |
                     (value & (S5L8702_IICSTAT_MODE_MASK |
                               S5L8702_IICSTAT_SOE));

        if (!(value & S5L8702_IICSTAT_SOE)) {
            /* Serial output off: whatever was going on is over. */
            if (busy) {
                i2c_end_transfer(s->bus);
                s->iicstat &= ~S5L8702_IICSTAT_BUSY;
            }
            break;
        }

        switch (mode) {
        case S5L8702_IICSTAT_START_TX:
            ipod_classic_i2c_set_lrb(s,
                i2c_start_send(s->bus, (s->iicds & 0xff) >> 1));
            s->iicstat |= S5L8702_IICSTAT_BUSY;
            s->in_address = true;
            ipod_classic_i2c_start_byte(s);
            break;
        case S5L8702_IICSTAT_START_RX:
            ipod_classic_i2c_set_lrb(s,
                i2c_start_recv(s->bus, (s->iicds & 0xff) >> 1));
            s->iicstat |= S5L8702_IICSTAT_BUSY;
            s->in_address = true;
            ipod_classic_i2c_start_byte(s);
            break;
        case S5L8702_IICSTAT_STOP_TX:
        case S5L8702_IICSTAT_STOP_RX:
            if (busy) {
                i2c_end_transfer(s->bus);
                s->iicstat &= ~S5L8702_IICSTAT_BUSY;
            }
            s->byte_done = false;
            timer_del(s->byte_timer);
            s->iicstat2 |= S5L8702_IICSTAT2_STOP;
            break;
        default:
            break;
        }
        ipod_classic_i2c_update_irq(s);
        break;
    }

    case S5L8702_IICADD:
        s->iicadd = value;
        break;
    case S5L8702_IICDS:
        s->iicds = value;
        s->iicds_new = true;
        break;
    case S5L8702_IICUNK10:
        /* Written as 0 by Apple's block init; nothing reads it back. */
        break;
    case S5L8702_IICUNK14:
        s->iicunk14 = value;
        break;
    case S5L8702_IICUNK18:
        s->iicunk18 = value;
        break;
    case S5L8702_IICSTAT2:
        s->iicstat2 &= ~(uint32_t)value;
        ipod_classic_i2c_update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: write of 0x%" PRIx64 " to 0x%"
                      HWADDR_PRIx "\n", __func__, value, offset);
        break;
    }
}

static const MemoryRegionOps ipod_classic_i2c_ops = {
    .read = ipod_classic_i2c_read,
    .write = ipod_classic_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void ipod_classic_i2c_reset(DeviceState *dev)
{
    IPodClassicI2CState *s = IPOD_CLASSIC_I2C(dev);

    timer_del(s->byte_timer);
    s->iiccon = s->reset_con;
    s->iicstat = 0;
    s->iicadd = 0;
    s->iicds = 0;
    s->iicunk14 = 0;
    s->iicunk18 = 0;
    s->iicstat2 = 0;
    s->byte_done = false;
    s->iicds_new = false;
    s->in_address = false;
    ipod_classic_i2c_update_irq(s);
}

static void ipod_classic_i2c_init(Object *obj)
{
    IPodClassicI2CState *s = IPOD_CLASSIC_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ipod_classic_i2c_ops, s,
                          TYPE_IPOD_CLASSIC_I2C, S5L8702_I2C_MEM_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
    s->byte_timer = timer_new_us(QEMU_CLOCK_VIRTUAL,
                                 ipod_classic_i2c_byte_done, s);
}

static const VMStateDescription vmstate_ipod_classic_i2c = {
    .name = TYPE_IPOD_CLASSIC_I2C,
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(iiccon, IPodClassicI2CState),
        VMSTATE_UINT32(iicstat, IPodClassicI2CState),
        VMSTATE_UINT32(iicadd, IPodClassicI2CState),
        VMSTATE_UINT32(iicds, IPodClassicI2CState),
        VMSTATE_UINT32(iicunk14, IPodClassicI2CState),
        VMSTATE_UINT32(iicunk18, IPodClassicI2CState),
        VMSTATE_UINT32(iicstat2, IPodClassicI2CState),
        /* A save between the IICDS write and the go bit (Q20). */
        VMSTATE_BOOL_V(iicds_new, IPodClassicI2CState, 2),
        VMSTATE_BOOL(byte_done, IPodClassicI2CState),
        VMSTATE_BOOL(in_address, IPodClassicI2CState),
        VMSTATE_TIMER_PTR(byte_timer, IPodClassicI2CState),
        VMSTATE_END_OF_LIST()
    },
};

static Property ipod_classic_i2c_properties[] = {
    DEFINE_PROP_UINT32("reset-con", IPodClassicI2CState, reset_con, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void ipod_classic_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, ipod_classic_i2c_properties);
    dc->desc = "S5L8702 I2C master";
    dc->reset = ipod_classic_i2c_reset;
    dc->vmsd = &vmstate_ipod_classic_i2c;
}

static const TypeInfo ipod_classic_i2c_info = {
    .name          = TYPE_IPOD_CLASSIC_I2C,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodClassicI2CState),
    .instance_init = ipod_classic_i2c_init,
    .class_init    = ipod_classic_i2c_class_init,
};

static void ipod_classic_i2c_register_types(void)
{
    type_register_static(&ipod_classic_i2c_info);
}

type_init(ipod_classic_i2c_register_types)
