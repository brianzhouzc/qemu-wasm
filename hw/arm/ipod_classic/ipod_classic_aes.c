/*
 * S5L8702 AES engine.
 *
 * Apple's retailOS 2.0.5 keeps the games under Extras > Games (iPod Quiz,
 * Klondike, Vortex) encrypted, and decrypts each one through this block in
 * 4 KB jobs as it loads it.  The block used to be an unimplemented window
 * that read 0, and the driver (0x0802e87c in the running image) polls STATUS
 * bit 0 with no timeout, so starting any game spun there for ever: the
 * firmware's "Loading..." spinner kept turning, the keys were still read,
 * and no button got back to the menus (MEASURED on qemu 7837aed, the JIT
 * and TCI alike).
 *
 * One job, as 2.0.5 programs it: TYPE = 0 and a 128-bit key in KEY[4..7],
 * UNKREG0 pulsed, CONTROL = 1, KEYLEN = 0xe (decrypt), the three sizes 0x1000,
 * INADDR and AUXADDR the ciphertext, OUTADDR a second buffer, the IV, STATUS
 * cleared with 7, GO = 1, then the poll.  Decrypting the measured ciphertext
 * with that key and IV as AES-128-CBC, every word big-endian, gives an "eapp"
 * header followed by ARM code; no other mode or byte order gives anything but
 * noise.
 *
 * The job runs inside the GO write: 4 KB is a few hundred AES blocks, so the
 * guest's first poll already sees it done.  The data goes through the system
 * address space, so the uncached alias at 0x88000000 the retailOS uses
 * works, and code written this way is invalidated like any other DMA.
 *
 * Not modelled: the interrupt (VIC line 39, which no driver we have enables),
 * whatever bits 1-5 of KEYLEN select, and the chip's own keys.  TYPE 1 and 2
 * are the GID and UID keys burnt into the SoC, which nobody outside Apple
 * has.  A job that asks for one runs with a stand-in key instead of hanging:
 * data this machine encrypts with it decrypts again, data encrypted on a real
 * iPod does not.  2.0.5 never asks for one in a boot or a game.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "crypto/aes.h"
#include "exec/address-spaces.h"
#include "exec/memory.h"
#include "migration/vmstate.h"
#include "hw/arm/ipod_classic/ipod_classic_aes.h"

/* AESTRACE=1: one line per job. */
static bool aes_trace_on(void)
{
    static int on = -1;

    if (on < 0) {
        on = getenv("AESTRACE") != NULL;
    }
    return on;
}

/* The words of a register run, most significant byte first. */
static void ipod_classic_aes_words(const uint32_t *w, int n, uint8_t *out)
{
    for (int i = 0; i < n; i++) {
        stl_be_p(out + i * 4, w[i]);
    }
}

/*
 * Stand-ins for the GID and UID keys.  Plain text on purpose: they are not
 * Apple's, and nothing should mistake them for a secret.
 */
static const char ipod_classic_aes_gid_standin[] = "iPodWasm GID key";
static const char ipod_classic_aes_uid_standin[] = "iPodWasm UID key";

/* The key the job asked for, and its length in bits; 0 if there is none. */
static int ipod_classic_aes_key(IPodClassicAESState *s, uint8_t key[32])
{
    uint32_t type = s->reg[S5L8702_AES_TYPE / 4];
    static bool warned[3];

    switch (type) {
    case S5L8702_AES_TYPE_CUSTOM: {
        uint32_t size = extract32(s->reg[S5L8702_AES_KEYLEN / 4],
                                  S5L8702_AES_KEYLEN_SIZE_SHIFT, 2);
        int words;

        if (size > 2) {
            qemu_log_mask(LOG_GUEST_ERROR, "s5l8702.aes: key length %u in "
                          "KEYLEN 0x%08x\n", size,
                          s->reg[S5L8702_AES_KEYLEN / 4]);
            return 0;
        }
        /* 128, 192 or 256 bits, at the end of the eight key words. */
        words = 4 + 2 * size;
        ipod_classic_aes_words(&s->reg[S5L8702_AES_KEY / 4 +
                                       S5L8702_AES_KEY_WORDS - words],
                               words, key);
        return words * 32;
    }
    case S5L8702_AES_TYPE_GID:
    case S5L8702_AES_TYPE_UID:
        if (!warned[type]) {
            warned[type] = true;
            qemu_log_mask(LOG_UNIMP, "s5l8702.aes: a job with the %s key, "
                          "which is not known; a stand-in key is used\n",
                          type == S5L8702_AES_TYPE_GID ? "GID" : "UID");
        }
        memcpy(key, type == S5L8702_AES_TYPE_GID ?
               ipod_classic_aes_gid_standin : ipod_classic_aes_uid_standin,
               16);
        return 128;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "s5l8702.aes: key type %u\n", type);
        return 0;
    }
}

/* One job: INSIZE bytes from INADDR through AES-CBC to OUTADDR. */
static void ipod_classic_aes_run(IPodClassicAESState *s)
{
    bool encrypt = s->reg[S5L8702_AES_KEYLEN / 4] & S5L8702_AES_KEYLEN_ENCRYPT;
    hwaddr in = s->reg[S5L8702_AES_INADDR / 4];
    hwaddr out = s->reg[S5L8702_AES_OUTADDR / 4];
    uint32_t size = s->reg[S5L8702_AES_INSIZE / 4];
    uint8_t key[32];
    uint8_t iv[16];
    uint8_t buf[4096];
    AES_KEY k;
    int bits;

    s->jobs++;
    if (size != s->reg[S5L8702_AES_OUTSIZE / 4] || (size & 15)) {
        qemu_log_mask(LOG_GUEST_ERROR, "s5l8702.aes: INSIZE 0x%x, OUTSIZE "
                      "0x%x; running whole blocks of the smaller\n", size,
                      s->reg[S5L8702_AES_OUTSIZE / 4]);
        size = MIN(size, s->reg[S5L8702_AES_OUTSIZE / 4]) & ~15u;
    }
    if (aes_trace_on()) {
        fprintf(stderr, "s5l8702.aes: job %u %s type %u keylen 0x%x "
                "in 0x%08" HWADDR_PRIx " out 0x%08" HWADDR_PRIx " size 0x%x\n",
                s->jobs, encrypt ? "encrypt" : "decrypt",
                s->reg[S5L8702_AES_TYPE / 4], s->reg[S5L8702_AES_KEYLEN / 4],
                in, out, size);
    }

    bits = ipod_classic_aes_key(s, key);
    if (bits) {
        if (encrypt) {
            AES_set_encrypt_key(key, bits, &k);
        } else {
            AES_set_decrypt_key(key, bits, &k);
        }
        ipod_classic_aes_words(&s->reg[S5L8702_AES_IV / 4],
                               S5L8702_AES_IV_WORDS, iv);

        while (size) {
            uint32_t n = MIN(size, sizeof(buf));

            address_space_read(&address_space_memory, in,
                               MEMTXATTRS_UNSPECIFIED, buf, n);
            for (uint32_t at = 0; at < n; at += 16) {
                uint8_t *b = buf + at;
                uint8_t c[16];

                if (encrypt) {
                    for (int i = 0; i < 16; i++) {
                        b[i] ^= iv[i];
                    }
                    AES_encrypt(b, b, &k);
                    memcpy(iv, b, 16);
                } else {
                    memcpy(c, b, 16);
                    AES_decrypt(b, b, &k);
                    for (int i = 0; i < 16; i++) {
                        b[i] ^= iv[i];
                    }
                    memcpy(iv, c, 16);
                }
            }
            address_space_write(&address_space_memory, out,
                                MEMTXATTRS_UNSPECIFIED, buf, n);
            in += n;
            out += n;
            size -= n;
        }
    }
    /* Done even when the job was refused: a poll that never ends is worse. */
    s->reg[S5L8702_AES_STATUS / 4] |= S5L8702_AES_STATUS_DONE;
}

static uint64_t ipod_classic_aes_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    IPodClassicAESState *s = opaque;

    if (offset + size > S5L8702_AES_REGS_SIZE) {
        qemu_log_mask(LOG_UNIMP, "s5l8702.aes: read at 0x%" HWADDR_PRIx
                      " (size %u)\n", offset, size);
        return 0;
    }
    return extract32(s->reg[offset / 4], (offset & 3) * 8, size * 8);
}

/*
 * Words, and the halfwords and bytes in them, like the clock block: a narrow
 * access a word-only block refused would be a bus error, then a data abort.
 */
static void ipod_classic_aes_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    IPodClassicAESState *s = opaque;
    unsigned shift = (offset & 3) * 8;
    uint32_t mask = MAKE_64BIT_MASK(shift, size * 8);
    uint32_t lane = (uint32_t)(value << shift) & mask;

    if (offset + size > S5L8702_AES_REGS_SIZE) {
        qemu_log_mask(LOG_UNIMP, "s5l8702.aes: write at 0x%" HWADDR_PRIx
                      " (size %u, value 0x%" PRIx64 ")\n", offset, size,
                      value);
        return;
    }

    offset &= ~3;
    switch (offset) {
    case S5L8702_AES_GO:
        s->reg[offset / 4] = (s->reg[offset / 4] & ~mask) | lane;
        if (lane & 1) {
            ipod_classic_aes_run(s);
        }
        break;
    case S5L8702_AES_STATUS:
        s->reg[offset / 4] &= ~lane;
        break;
    default:
        s->reg[offset / 4] = (s->reg[offset / 4] & ~mask) | lane;
        break;
    }
}

static const MemoryRegionOps ipod_classic_aes_ops = {
    .read = ipod_classic_aes_read,
    .write = ipod_classic_aes_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void ipod_classic_aes_reset(DeviceState *dev)
{
    IPodClassicAESState *s = IPOD_CLASSIC_AES(dev);

    memset(s->reg, 0, sizeof(s->reg));
    s->jobs = 0;
}

static void ipod_classic_aes_init(Object *obj)
{
    IPodClassicAESState *s = IPOD_CLASSIC_AES(obj);

    memory_region_init_io(&s->iomem, obj, &ipod_classic_aes_ops, s,
                          TYPE_IPOD_CLASSIC_AES, S5L8702_AES_MEM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_ipod_classic_aes = {
    .name = TYPE_IPOD_CLASSIC_AES,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(reg, IPodClassicAESState,
                             S5L8702_AES_REGS_SIZE / 4),
        VMSTATE_UINT32(jobs, IPodClassicAESState),
        VMSTATE_END_OF_LIST()
    },
};

static void ipod_classic_aes_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "S5L8702 AES engine";
    dc->reset = ipod_classic_aes_reset;
    dc->vmsd = &vmstate_ipod_classic_aes;
}

static const TypeInfo ipod_classic_aes_info = {
    .name          = TYPE_IPOD_CLASSIC_AES,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodClassicAESState),
    .instance_init = ipod_classic_aes_init,
    .class_init    = ipod_classic_aes_class_init,
};

static void ipod_classic_aes_register_types(void)
{
    type_register_static(&ipod_classic_aes_info);
}

type_init(ipod_classic_aes_register_types)
