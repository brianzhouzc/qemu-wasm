#include "hw/arm/ipod_touch_sha1.h"

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

static void ipod_sha1_final(IPodTouchSHA1Ctx *ctx, uint8_t out[20])
{
    uint64_t bits = ctx->bytes * 8;
    size_t used = ctx->bytes % 64;
    int i;

    ctx->block[used++] = 0x80;
    if (used > 56) {
        memset(ctx->block + used, 0, 64 - used);
        sha1_transform(ctx->h, ctx->block);
        used = 0;
    }
    memset(ctx->block + used, 0, 56 - used);
    for (i = 0; i < 8; i++) {
        ctx->block[56 + i] = (uint8_t)(bits >> (56 - 8 * i));
    }
    sha1_transform(ctx->h, ctx->block);
    for (i = 0; i < 5; i++) {
        out[i * 4] = ctx->h[i] >> 24;
        out[i * 4 + 1] = ctx->h[i] >> 16;
        out[i * 4 + 2] = ctx->h[i] >> 8;
        out[i * 4 + 3] = ctx->h[i];
    }
}

static uint64_t swapLong(uint64_t x) {
    x = (x & 0x00000000FFFFFFFF) << 32 | (x & 0xFFFFFFFF00000000) >> 32;
    x = (x & 0x0000FFFF0000FFFF) << 16 | (x & 0xFFFF0000FFFF0000) >> 16;
    x = (x & 0x00FF00FF00FF00FF) << 8  | (x & 0xFF00FF00FF00FF00) >> 8;
    return x;
}

static void flush_hw_buffer(IPodTouchSHA1State *s) {
    // Flush the hardware buffer to the state buffer and clear the buffer.
    if (s->buffer_ind + 0x40 > SHA1_BUFFER_SIZE) {
        /* The guest kept hashing without a reset: drop, never overflow (Q2). */
        qemu_log_mask(LOG_GUEST_ERROR,
                      "sha1: buffer full, dropping a block\n");
        memset(s->hw_buffer, 0, 0x40);
        s->hw_buffer_dirty = false;
        return;
    }
    memcpy(s->buffer + s->buffer_ind, (uint8_t *)s->hw_buffer, 0x40);
    memset(s->hw_buffer, 0, 0x40);
    s->hw_buffer_dirty = false;
    s->buffer_ind += 0x40;
}

static void sha1_reset(IPodTouchSHA1State *s)
{
	s->config = 0;
	s->memory_start = 0;
	s->memory_mode = 0;
	s->insize = 0;
	memset(&s->buffer, 0, SHA1_BUFFER_SIZE);
	memset(&s->hw_buffer, 0, 0x10 * sizeof(uint32_t));
	s->buffer_ind = 0;
	memset(&s->hashout, 0, 0x14);
	s->hw_buffer_dirty = false;
	s->hash_computed = false;
	s->memory_fed = false;
}

static uint64_t ipod_touch_sha1_read(void *opaque, hwaddr offset, unsigned size)
{
	IPodTouchSHA1State *s = (IPodTouchSHA1State *)opaque;

    //fprintf(stderr, "%s: offset 0x%08x\n", __FUNCTION__, offset);

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
			//fprintf(stderr, "Hash out %08x\n",  *(uint32_t *)&s->hashout[offset - 0x20]);
            if(!s->hash_computed) {
                /*
                 * How many of the buffered bytes are the message?
                 *
                 * Two firmwares drive this block differently and the model has
                 * to cope with both:
                 *
                 *  - The iPod Touch code this was ported from pads the message
                 *    ITSELF, so the buffer ends with the standard SHA-1 trailer
                 *    and the real length is in its last eight bytes, in bits.
                 *    Hashing everything would then pad an already-padded
                 *    message and give the wrong digest.
                 *
                 *  - The Classic's retailOS does NOT pad. MEASURED with
                 *    SHA1TRACE=1: it hands over 64 and then 4096 bytes in memory
                 *    mode with no trailer at all, so the last eight bytes read
                 *    as zero. Trusting them made data_length 0, and the engine
                 *    answered da39a3ee5e6b4b0d3255bfef95601890afd80709 -- the
                 *    SHA-1 of the EMPTY STRING -- for every request. No database
                 *    could ever pass a check against that, which is exactly why
                 *    wiring this accelerator turned the music library empty.
                 *
                 * The two cases are told apart by HOW the data arrived, not by
                 * what the last bytes look like. A first attempt trusted the
                 * trailer whenever it was a plausible length, and immediately
                 * misread 34 out of 64 bytes of ordinary data as a length --
                 * these messages look random, so any 8 bytes of them can pass
                 * for one. Memory mode means the guest already said how much to
                 * hash, via SHA_INSIZE; only the block-at-a-time register path
                 * carries a message the guest padded itself.
                 */
                uint64_t data_length;
                if (s->memory_fed) {
                    data_length = s->buffer_ind;
                } else {
                    uint64_t declared = s->buffer_ind >= 8
                        ? swapLong(((uint64_t *)s->buffer)[s->buffer_ind / 8 - 1]) / 8
                        : 0;
                    data_length = (declared > 0 && declared <= s->buffer_ind)
                        ? declared : s->buffer_ind;
                }

                IPodTouchSHA1Ctx ctx;
                ipod_sha1_init(&ctx);
                ipod_sha1_update(&ctx, s->buffer, (size_t)data_length);
                ipod_sha1_final(&ctx, s->hashout);
                s->hash_computed = true;

                /*
                 * SHA1TRACE=1: what is the firmware actually hashing?
                 *
                 * This matters because with this engine wired the firmware
                 * rejects the iTunesDB outright (0 songs), and there are two
                 * very different reasons that could happen: the database really
                 * fails a signature check, or THIS MODEL returns the wrong
                 * digest and nothing could ever match. The lazy length above is
                 * an assumption -- that the guest writes its own SHA-1 padding
                 * and the "length in bits" trailer -- and a real accelerator
                 * usually pads itself. Printing the declared length, the length
                 * actually fed in, the head of the data and the digest says
                 * which of the two is happening, and whether the input is even
                 * the database (an mhbd magic would be unmistakable).
                 */
                if (getenv("SHA1TRACE")) {
                    static int n;
                    if (n++ < 12) {
                        fprintf(stderr,
                                "SHA1TRACE #%d: buffered=%u bytes, trailer says %"
                                PRIu64 ", memory_mode=%u start=0x%08x insize=%u\n",
                                n, s->buffer_ind, data_length,
                                s->memory_mode, s->memory_start, s->insize);
                        fprintf(stderr, "  head: %02x %02x %02x %02x %02x %02x %02x %02x"
                                "   '%c%c%c%c'\n",
                                s->buffer[0], s->buffer[1], s->buffer[2], s->buffer[3],
                                s->buffer[4], s->buffer[5], s->buffer[6], s->buffer[7],
                                isprint(s->buffer[0]) ? s->buffer[0] : '.',
                                isprint(s->buffer[1]) ? s->buffer[1] : '.',
                                isprint(s->buffer[2]) ? s->buffer[2] : '.',
                                isprint(s->buffer[3]) ? s->buffer[3] : '.');
                        fprintf(stderr, "  digest:");
                        for (int i = 0; i < 20; i++) {
                            fprintf(stderr, "%02x", s->hashout[i]);
                        }
                        fprintf(stderr, "\n");
                    }
                }
            }

			/*
			 * SHA1_ZERO=1: keep the device, but answer the digest as the
			 * unimplemented stub used to -- all zeros.
			 *
			 * This is the discriminator for the one difference left between the
			 * two configurations. With SHA1_OFF the library loads and playback
			 * starts; with the engine wired the boot is clean but the library is
			 * empty. Both now back the whole 1 MB window, so the only thing that
			 * still differs is what these registers answer. If zeroing just the
			 * digest brings the library back, the firmware is acting on the
			 * VALUE -- it compares it against something -- and the empty library
			 * is a verdict, not a side effect. If it does not, the difference is
			 * in another register and the digest is a red herring.
			 */
			if (getenv("SHA1_ZERO")) {
				return 0;
			}
			return *(uint32_t *)&s->hashout[offset - 0x20];
	}

    return 0;
}

static void ipod_touch_sha1_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    IPodTouchSHA1State *s = (IPodTouchSHA1State *)opaque;

    //fprintf(stderr, "%s: offset 0x%08x value 0x%08x\n", __FUNCTION__, offset, value);

	switch(offset) {
		case SHA_CONFIG:
			if(value == 0x2 || value == 0xa)
			{
                if(s->hw_buffer_dirty) {
                    flush_hw_buffer(s);
                }

				if(s->memory_mode)
				{
					// we are in memory mode - gradually add the memory to the buffer
					for(int i = 0; i < s->insize / 0x40; i++) {
						if (s->buffer_ind + 0x40 > SHA1_BUFFER_SIZE) {
							qemu_log_mask(LOG_GUEST_ERROR,
							              "sha1: buffer full, dropping input\n");
							break;
						}
						cpu_physical_memory_read(s->memory_start + i * 0x40, s->buffer + s->buffer_ind, 0x40);
						s->buffer_ind += 0x40;
					}
					/*
					 * Remember that the guest sized this message itself. The
					 * digest read needs to know: a message that arrived this
					 * way is NOT padded, so its last eight bytes are data, not
					 * a length.
					 */
					s->memory_fed = true;
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
            if (value > SHA1_BUFFER_SIZE) {
                /* An assert would let the guest abort QEMU; refuse instead (Q2). */
                qemu_log_mask(LOG_GUEST_ERROR,
                              "sha1: insize 0x%" PRIx64 " is over the buffer\n",
                              value);
                break;
            }
			s->insize = value;
			break;
		case 0x40 ... 0x7c:
            // write to the hardware buffer
            s->hw_buffer[(offset - 0x40) / 4] |= value;
            s->hw_buffer_dirty = true;
			break;
	}
}

static const MemoryRegionOps sha1_ops = {
    .read = ipod_touch_sha1_read,
    .write = ipod_touch_sha1_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    /*
     * The digest registers are four-byte words (0x20 .. 0x33); the handler
     * reads a uint32_t, so refuse unaligned or partial accesses that would
     * run past the 20-byte hashout (Q2).
     */
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
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

static void ipod_touch_sha1_class_init(ObjectClass *klass, void *data)
{

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
