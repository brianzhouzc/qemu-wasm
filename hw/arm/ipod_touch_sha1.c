#include "hw/arm/ipod_touch_sha1.h"
#include "qemu/log.h"
#include "migration/vmstate.h"

/* getenv() on the digest read path is measurable under TCI (Q18). */
static bool sha1_trace_on(void)
{
    static int on = -1;
    if (on < 0) {
        on = getenv("SHA1TRACE") != NULL;
    }
    return on;
}

/*
 * SHA-1 (FIPS 180-4), so the device model does not depend on OpenSSL, which
 * the WebAssembly build cannot carry. The digests are identical to the
 * SHA1_* calls this replaced; SHA1TRACE=1 prints them the same way.
 */
static uint32_t sha1_rotl(uint32_t v, int n)
{
    return (v << n) | (v >> (32 - n));
}

static void sha1_transform(uint32_t h[5], const uint8_t block[64])
{
    uint32_t w[80];
    uint32_t a, b, c, d, e;
    int i;

    for (i = 0; i < 16; i++) {
        w[i] = (uint32_t)block[i * 4] << 24 |
               (uint32_t)block[i * 4 + 1] << 16 |
               (uint32_t)block[i * 4 + 2] << 8 |
               (uint32_t)block[i * 4 + 3];
    }
    for (i = 16; i < 80; i++) {
        w[i] = sha1_rotl(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    }

    a = h[0];
    b = h[1];
    c = h[2];
    d = h[3];
    e = h[4];
    for (i = 0; i < 80; i++) {
        uint32_t f, k, t;

        if (i < 20) {
            f = (b & c) | (~b & d);
            k = 0x5a827999;
        } else if (i < 40) {
            f = b ^ c ^ d;
            k = 0x6ed9eba1;
        } else if (i < 60) {
            f = (b & c) | (b & d) | (c & d);
            k = 0x8f1bbcdc;
        } else {
            f = b ^ c ^ d;
            k = 0xca62c1d6;
        }
        t = sha1_rotl(a, 5) + f + e + k + w[i];
        e = d;
        d = c;
        c = sha1_rotl(b, 30);
        b = a;
        a = t;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
}

static void ipod_sha1_init(IPodTouchSHA1Ctx *ctx)
{
    ctx->h[0] = 0x67452301;
    ctx->h[1] = 0xefcdab89;
    ctx->h[2] = 0x98badcfe;
    ctx->h[3] = 0x10325476;
    ctx->h[4] = 0xc3d2e1f0;
    ctx->bytes = 0;
}

static void ipod_sha1_update(IPodTouchSHA1Ctx *ctx, const void *data, size_t len)
{
    const uint8_t *p = data;
    size_t used = ctx->bytes % 64;

    ctx->bytes += len;
    while (len > 0) {
        size_t take = 64 - used;

        if (take > len) {
            take = len;
        }
        memcpy(ctx->block + used, p, take);
        used += take;
        p += take;
        len -= take;
        if (used == 64) {
            sha1_transform(ctx->h, ctx->block);
            used = 0;
        }
    }
}

/*
 * The block is a streaming SHA-1 core, not a one-shot hash.  The firmware
 * pads its own messages: the seed for hash58 arrives as one already-padded
 * 64-byte block, and the HMAC inner/outer messages arrive as data blocks
 * followed by a padded final block.  So every transfer is fed into a
 * persistent context and a digest read returns the current state without
 * padding anything again.
 */
static void sha1_feed(IPodTouchSHA1State *s, const void *data, size_t len)
{
    if (s->ctx.bytes == 0) {
        memset(s->head, 0, sizeof(s->head));
        memcpy(s->head, data, MIN(len, sizeof(s->head)));
    }
    ipod_sha1_update(&s->ctx, data, len);
    s->hash_computed = false;
}

static void flush_hw_buffer(IPodTouchSHA1State *s) {
    uint8_t block[0x40];

    memcpy(block, s->hw_buffer, sizeof(block));
    sha1_feed(s, block, sizeof(block));
    memset(s->hw_buffer, 0, 0x40);
    s->hw_buffer_dirty = false;
}

static void sha1_reset(IPodTouchSHA1State *s)
{
	s->config = 0;
	s->memory_start = 0;
	s->memory_mode = 0;
	s->insize = 0;
	memset(&s->hw_buffer, 0, 0x10 * sizeof(uint32_t));
	memset(&s->head, 0, sizeof(s->head));
	memset(&s->hashout, 0, 0x14);
	s->hw_buffer_dirty = false;
	s->hash_computed = false;
	ipod_sha1_init(&s->ctx);
}

static uint64_t ipod_touch_sha1_read(void *opaque, hwaddr offset, unsigned size)
{
	IPodTouchSHA1State *s = (IPodTouchSHA1State *)opaque;

	switch(offset) {
		case SHA_CONFIG:
			return s->config;
		case SHA_RESET:
			return 0;
		case SHA_MEMORY_START:
			return s->memory_start;
		case SHA_MEMORY_MODE:
			return s->memory_mode;
		case SHA_INSIZE:
			return s->insize;
		/* Hash result ouput */
		case 0x20 ... 0x33:
			if (size != 4) {
				/* A partial digest read cannot be served from the
				 * 20-byte hashout without running past it (S7). */
				qemu_log_mask(LOG_GUEST_ERROR,
				              "sha1: %u-byte digest read at 0x%x\n",
				              size, (unsigned)offset);
				return 0;
			}
            if(!s->hash_computed) {
                int i;

                /*
                 * The guest has already padded the message: the block that
                 * arrives before it reads the digest is the SHA-1 padding
                 * block (0x80, zeros, length in bits).  Padding it again is
                 * what made the model's hash58 key cef09b89... instead of
                 * libgpod's 7eb88d2a....  A real accelerator only processes
                 * the blocks it is given, so return the state as it stands.
                 */
                for (i = 0; i < 5; i++) {
                    s->hashout[i * 4] = s->ctx.h[i] >> 24;
                    s->hashout[i * 4 + 1] = s->ctx.h[i] >> 16;
                    s->hashout[i * 4 + 2] = s->ctx.h[i] >> 8;
                    s->hashout[i * 4 + 3] = s->ctx.h[i];
                }
                s->hash_computed = true;

                /*
                 * SHA1TRACE=1: what the firmware hashes, for the first dozen
                 * digests.  The byte count, the head of the message and the
                 * digest are enough to tell the hash58 seed and the iTunesDB
                 * HMAC apart (an mhbd magic is unmistakable) and to compare a
                 * digest with libgpod's.
                 */
                if (sha1_trace_on()) {
                    static int n;
                    if (n++ < 12) {
                        fprintf(stderr,
                                "SHA1TRACE #%d: fed=%" PRIu64 " bytes, "
                                "memory_mode=%u start=0x%08x insize=%u\n",
                                n, s->ctx.bytes, s->memory_mode,
                                s->memory_start, s->insize);
                        fprintf(stderr, "  head: %02x %02x %02x %02x %02x %02x %02x %02x"
                                "   '%c%c%c%c'\n",
                                s->head[0], s->head[1], s->head[2], s->head[3],
                                s->head[4], s->head[5], s->head[6], s->head[7],
                                isprint(s->head[0]) ? s->head[0] : '.',
                                isprint(s->head[1]) ? s->head[1] : '.',
                                isprint(s->head[2]) ? s->head[2] : '.',
                                isprint(s->head[3]) ? s->head[3] : '.');
                        fprintf(stderr, "  digest:");
                        for (int j = 0; j < 20; j++) {
                            fprintf(stderr, "%02x", s->hashout[j]);
                        }
                        fprintf(stderr, "\n");
                    }
                }
            }

			return *(uint32_t *)&s->hashout[offset - 0x20];
	}

    return 0;
}

static void ipod_touch_sha1_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    IPodTouchSHA1State *s = (IPodTouchSHA1State *)opaque;

	switch(offset) {
		case SHA_CONFIG:
			if(value == 0x2 || value == 0xa)
			{
                if(s->hw_buffer_dirty) {
                    flush_hw_buffer(s);
                }

				if(s->memory_mode)
				{
					/*
					 * Feed the blocks as they arrive; the guest may update
					 * this engine several times for one HMAC (the key, the
					 * database in chunks, and the pre-padded final block).
					 */
					for(int i = 0; i < s->insize / 0x40; i++) {
						uint8_t block[0x40];
						cpu_physical_memory_read(s->memory_start + i * 0x40, block, 0x40);
						sha1_feed(s, block, 0x40);
					}
				}
			} else {
				s->config = value;
			}
			break;
		case SHA_RESET:
			sha1_reset(s);
			break;
		case SHA_MEMORY_START:
			s->memory_start = value;
			break;
		case SHA_MEMORY_MODE:
			s->memory_mode = value;
			break;
		case SHA_INSIZE:
            if (value > SHA1_INSIZE_MAX) {
                /* An assert would let the guest abort QEMU; refuse instead (Q2). */
                qemu_log_mask(LOG_GUEST_ERROR,
                              "sha1: insize 0x%" PRIx64 " is over the buffer\n",
                              value);
                break;
            }
			s->insize = value;
			break;
		/*
		 * The hash state registers.  The guest re-initialises SHA-1 by
		 * writing the five IV words here before each hash (the seed, the
		 * HMAC inner and the HMAC outer): they are the standard IV in
		 * little-endian word order -- 0x01234567 for 0x67452301.  The model
		 * used to ignore these writes, so every digest after the seed
		 * continued the previous message and hash58 could never match.
		 */
		case 0x20 ... 0x30:
			if ((offset & 3) == 0) {
				if (offset == 0x20) {
					ipod_sha1_init(&s->ctx);
				}
				s->ctx.h[(offset - 0x20) / 4] = bswap32((uint32_t)value);
				s->hash_computed = false;
			}
			break;
		case 0x40 ... 0x7f: {
            /*
             * Write to the hardware buffer.  Byte and halfword stores land at
             * their own lane of the word (see sha1_ops), not in its low byte.
             */
            unsigned shift = (offset & 3) * 8;
            uint64_t mask = size >= 4 ? 0xffffffffu : (1u << (size * 8)) - 1;

            s->hw_buffer[(offset - 0x40) / 4] |= (uint32_t)((value & mask)
                                                            << shift);
            s->hw_buffer_dirty = true;
			break;
		}
	}
}

static const MemoryRegionOps sha1_ops = {
    .read = ipod_touch_sha1_read,
    .write = ipod_touch_sha1_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    /*
     * The access sizes stay unrestricted: the 0x40..0x7c input buffer takes
     * byte and halfword stores, and a global 4-byte minimum turned those into
     * guest data aborts (S7).  The digest case checks its own size instead.
     */
};

static void ipod_touch_sha1_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(sbd);
    IPodTouchSHA1State *s = IPOD_TOUCH_SHA1(dev);

    memory_region_init_io(&s->iomem, obj, &sha1_ops, s, "sha1", 0x100);
    sysbus_init_mmio(sbd, &s->iomem);

    sha1_reset(s);
}

static void ipod_touch_sha1_reset(DeviceState *dev)
{
    sha1_reset(IPOD_TOUCH_SHA1(dev));
}

/*
 * The firmware hashes in several CONFIG kicks (the IV words, the key, the
 * database in chunks, the padded final block), so a snapshot can fall in the
 * middle of one: the running context goes with it, or the digest after a
 * restore is wrong and the library reads as empty.
 */
static const VMStateDescription vmstate_ipod_touch_sha1 = {
    .name = "ipodtouch.sha1",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(config, IPodTouchSHA1State),
        VMSTATE_UINT32(memory_start, IPodTouchSHA1State),
        VMSTATE_UINT32(memory_mode, IPodTouchSHA1State),
        VMSTATE_UINT32(insize, IPodTouchSHA1State),
        VMSTATE_UINT32_ARRAY(hw_buffer, IPodTouchSHA1State, 0x10),
        VMSTATE_UINT8_ARRAY(head, IPodTouchSHA1State, 8),
        VMSTATE_UINT8_ARRAY(hashout, IPodTouchSHA1State, 0x14),
        VMSTATE_BOOL(hw_buffer_dirty, IPodTouchSHA1State),
        VMSTATE_BOOL(hash_computed, IPodTouchSHA1State),
        VMSTATE_UINT32_ARRAY(ctx.h, IPodTouchSHA1State, 5),
        VMSTATE_UINT64(ctx.bytes, IPodTouchSHA1State),
        VMSTATE_UINT8_ARRAY(ctx.block, IPodTouchSHA1State, 64),
        VMSTATE_END_OF_LIST()
    }
};

static void ipod_touch_sha1_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = ipod_touch_sha1_reset;
    dc->vmsd = &vmstate_ipod_touch_sha1;
}

static const TypeInfo ipod_touch_sha1_info = {
    .name          = TYPE_IPOD_TOUCH_SHA1,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodTouchSHA1State),
    .instance_init = ipod_touch_sha1_init,
    .class_init    = ipod_touch_sha1_class_init,
};

static void ipod_touch_machine_types(void)
{
    type_register_static(&ipod_touch_sha1_info);
}

type_init(ipod_touch_machine_types)
