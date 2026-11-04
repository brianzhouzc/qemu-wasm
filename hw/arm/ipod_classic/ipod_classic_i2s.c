/*
 * S5L8702 I2S transmitter, 0x3CA00000.
 *
 * See include/hw/arm/ipod_classic/ipod_classic_i2s.h for the register map and
 * wiki/14-classic6g-audio.md for the whole audio path and how it was measured.
 *
 * WHAT THIS MODEL DOES
 *
 *  - A register file with honest readback. The retailOS writes CLKCON=1,
 *    TXCON=0x0b100019, TXCOM=0x6 (start), RXCON=0x1000, CLKDIV=272 and reads
 *    I2STXCOM back to confirm the transmitter started; while this block was
 *    create_unimplemented_device every one of those readbacks was zero.
 *
 *  - A TX FIFO sink at +0x10. DMAC0 (a PL080) streams 16-bit PCM samples here.
 *    While the block was unimplemented ~6144 samples per boot fell to the floor.
 *
 *  - PCM capture. With AUDIOWAV=/path.wav set, every sample written to the FIFO
 *    is appended to a 16-bit-stereo WAV whose sample rate is taken from CLKDIV,
 *    and the first non-zero sample of the run is announced on stderr.
 *
 *  - Pacing. See ipod_classic_i2s_drain(): the FIFO is drained at the sample
 *    rate, in host time, so the PL080 hands over what has been clocked out and
 *    its terminal-count IRQ (16) reaches the firmware at real playback tempo
 *    instead of all at once, and what is drained goes to the machine's audio
 *    output for the page to play.
 *
 * INSTRUMENTATION: I2STRACE=1 prints every access with its value and guest PC,
 * same pattern as DISPTRACE=1/ATATRACE=1.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "hw/core/cpu.h"
#include "hw/arm/ipod_classic/ipod_classic_i2s.h"
#include "hw/arm/ipod_classic/ipod_classic_audio_out.h"

/* ------------------------------------------------------------- tracing */

static int i2strace = -1;

static bool i2s_trace_on(void)
{
    if (i2strace < 0) {
        const char *e = getenv("I2STRACE");
        i2strace = (e && *e && *e != '0') ? 1 : 0;
    }
    return i2strace == 1;
}

static uint32_t i2s_trace_pc(void)
{
    CPUState *cs = current_cpu;
    return cs ? (uint32_t)CPU_GET_CLASS(cs)->get_pc(cs) : 0;
}

static const char *i2s_reg_name(hwaddr off)
{
    switch (off) {
    case S5L8702_I2S_CLKCON: return "CLKCON";
    case S5L8702_I2S_TXCON:  return "TXCON";
    case S5L8702_I2S_TXCOM:  return "TXCOM";
    case S5L8702_I2S_TXDB0:  return "TXDB0";
    case S5L8702_I2S_RXCON:  return "RXCON";
    case S5L8702_I2S_RXCOM:  return "RXCOM";
    case S5L8702_I2S_RXDB:   return "RXDB";
    case S5L8702_I2S_STATUS: return "STATUS";
    case S5L8702_I2S_CLKDIV: return "CLKDIV";
    default:                 return "";
    }
}

/* --------------------------------------------------------- WAV capture */

/*
 * One WAV file for the run, opened lazily on the first FIFO write. A file-scope
 * pointer lets the atexit handler patch the RIFF/data sizes back into the
 * header; there is only ever one I2S instance, so a static is enough (same shape
 * as DISPTRACE's atexit totals).
 */
static FILE *i2s_wav;
static uint32_t i2s_wav_data_bytes;
static bool i2s_wav_announced;

static void i2s_wav_put_le32(uint8_t *p, uint32_t v)
{
    p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

/*
 * Snap 12e6/CLKDIV to the nearest standard sample rate. 272 gives 44117, and the
 * codec, which is the clock master here, does run from that 12 MHz at 44118 Hz.
 * But a song is 44100, and what matters is that it takes as long as it says it
 * does, so the machine plays the rate the firmware meant. CONVENIENCE: the 32 kHz
 * case, which the SoC clocks as 48 kHz with MCLK dropped to 8 MHz, comes out as
 * 48000 here because this helper assumes a 12 MHz MCLK, and so would play fast.
 * It takes Rockbox's "auto" frequency setting and a 32 kHz file.
 */
static uint32_t i2s_rate_from_div(uint32_t clkdiv)
{
    static const uint32_t std[] = {
        8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000
    };
    uint32_t hz, best = 44100, bestd = 0xffffffff;

    if (clkdiv == 0) {
        return 44100;
    }
    hz = S5L8702_I2S_MCLK_HZ / clkdiv;
    for (size_t i = 0; i < ARRAY_SIZE(std); i++) {
        uint32_t d = hz > std[i] ? hz - std[i] : std[i] - hz;
        if (d < bestd) {
            bestd = d;
            best = std[i];
        }
    }
    return best;
}

static void i2s_wav_close(void)
{
    uint8_t hdr[8];

    if (!i2s_wav) {
        return;
    }
    /* Patch RIFF chunk size (+4) and data chunk size (+40). */
    if (fseek(i2s_wav, 4, SEEK_SET) == 0) {
        i2s_wav_put_le32(hdr, 36 + i2s_wav_data_bytes);
        fwrite(hdr, 1, 4, i2s_wav);
    }
    if (fseek(i2s_wav, 40, SEEK_SET) == 0) {
        i2s_wav_put_le32(hdr, i2s_wav_data_bytes);
        fwrite(hdr, 1, 4, i2s_wav);
    }
    fclose(i2s_wav);
    i2s_wav = NULL;
    fprintf(stderr, "AUDIOWAV: wrote %u PCM bytes (%u stereo frames)\n",
            i2s_wav_data_bytes, i2s_wav_data_bytes / 4);
}

/*
 * Asked once: this runs for every sample, and getenv is a linear scan of the
 * environment.  An empty AUDIOWAV, or one that cannot be opened, is off.
 */
static int audiowav = -1;

static void i2s_wav_open(uint32_t rate)
{
    const char *path = getenv("AUDIOWAV");
    uint8_t h[44];
    uint32_t byte_rate = rate * 2 /*chan*/ * 2 /*bytes*/;

    if (!path || !*path) {
        return;
    }
    i2s_wav = fopen(path, "wb");
    if (!i2s_wav) {
        fprintf(stderr, "AUDIOWAV: cannot open %s\n", path);
        audiowav = 0;   /* once, not again for every sample */
        return;
    }
    /* Canonical 44-byte PCM WAV header; sizes patched at close. */
    memcpy(h + 0,  "RIFF", 4);
    i2s_wav_put_le32(h + 4, 36);            /* patched */
    memcpy(h + 8,  "WAVE", 4);
    memcpy(h + 12, "fmt ", 4);
    i2s_wav_put_le32(h + 16, 16);           /* fmt chunk size */
    h[20] = 1; h[21] = 0;                   /* PCM */
    h[22] = 2; h[23] = 0;                   /* stereo */
    i2s_wav_put_le32(h + 24, rate);
    i2s_wav_put_le32(h + 28, byte_rate);
    h[32] = 4; h[33] = 0;                   /* block align = 2ch * 2 bytes */
    h[34] = 16; h[35] = 0;                  /* bits per sample */
    memcpy(h + 36, "data", 4);
    i2s_wav_put_le32(h + 40, 0);            /* patched */
    fwrite(h, 1, sizeof(h), i2s_wav);
    i2s_wav_data_bytes = 0;
    atexit(i2s_wav_close);
    fprintf(stderr, "AUDIOWAV: capturing %u Hz 16-bit stereo to %s\n",
            rate, path);
}

static bool i2s_wav_on(void)
{
    if (audiowav < 0) {
        const char *e = getenv("AUDIOWAV");

        audiowav = e && *e;
    }
    return audiowav == 1;
}

/* Append the low `size` bytes of a FIFO write to the WAV, little-endian. */
static void i2s_wav_sample(IPodClassicI2SState *s, uint64_t value, unsigned size)
{
    uint8_t b[4];

    if (!i2s_wav_on()) {
        return;
    }
    if (!i2s_wav) {
        i2s_wav_open(i2s_rate_from_div(s->clkdiv));
        if (!i2s_wav) {
            return;
        }
    }
    if (size > 4) {
        size = 4;
    }
    i2s_wav_put_le32(b, (uint32_t)value);
    fwrite(b, 1, size, i2s_wav);
    i2s_wav_data_bytes += size;

    if (!i2s_wav_announced && value != 0) {
        i2s_wav_announced = true;
        fprintf(stderr, "s5l8702.i2s: FIRST NON-ZERO PCM sample 0x%0*x "
                "(pc~0x%08x) -- real audio is reaching the FIFO\n",
                size * 2, (uint32_t)value, i2s_trace_pc());
    }
}

/*
 * I2SCOUNT=1: tally accesses per register instead of printing every one.
 *
 * I2STRACE prints a line per access, which is unusable for exactly the question
 * that matters here -- whether the firmware SPINS on a register. The CLKCON bug
 * above was found that way and needed 161 million reads to show itself; a
 * per-line trace of that is not something anyone can read. The STATUS register
 * at +0x3c is the remaining unknown in this device: its bits are undocumented
 * and the model answers 0 forever, so if the audio driver polls it for a "FIFO
 * has room" or "TX ready" condition it would write one burst and wait for ever
 * -- which is the shape of the bug being chased. Counting answers that in one
 * run. The tally prints on a doubling schedule so a genuine spin is loud and a
 * handful of accesses stays quiet.
 */
static int i2scount = -1;

static bool i2s_count_on(void)
{
    if (i2scount < 0) {
        const char *e = getenv("I2SCOUNT");
        i2scount = (e && *e && *e != '0') ? 1 : 0;
    }
    return i2scount == 1;
}

static void i2s_count_hit(hwaddr off, const char *dir, uint32_t val)
{
    /* One slot per 4-byte register in the low 0x44 bytes; the rest share [17]. */
    static uint64_t rd[18], wr[18];
    uint64_t *tab = (*dir == 'r') ? rd : wr;
    unsigned slot = (off < 0x44) ? (unsigned)(off >> 2) : 17;
    uint64_t n = ++tab[slot];

    /* Powers of two: 1, 2, 4, ... plus every millionth once past that. */
    if ((n & (n - 1)) == 0 || n % 1000000 == 0) {
        fprintf(stderr, "I2SCOUNT %s +0x%02" HWADDR_PRIx " %-7s n=%llu "
                "last=0x%08x\n", dir, off, i2s_reg_name(off),
                (unsigned long long)n, val);
    }
}

/* -------------------------------------------------------------- pacing */

/*
 * The drain's timer, and the whole of what the guest's sample clock is here.
 *
 * It cannot be virtual time.  Under -icount sleep=off that clock warps to the
 * next deadline whenever the vCPU halts, so paced on it a song races while the
 * guest idles between buffers and crawls while it decodes.  The sample clock is
 * a wall-clock source like the RTC and Rockbox's tick, and takes the same rules
 * (docs/architecture.md, "Wall-clock sources are host time").
 *
 * The main loop runs this no more often than it can wake, which on Windows is
 * every 15.4 ms whatever is asked for, so nothing here assumes the period: each
 * tick works out what the time since the last one was worth.  A late tick pays
 * for more samples.
 *
 * The clock is QEMU_CLOCK_VIRTUAL_RT and not REALTIME because it stops with the
 * VM: this writes guest state, through the DMA and its interrupt, and must not
 * while a snapshot is being saved.  Resuming then finds no time to pay for.
 */
#define I2S_PACE_PERIOD_NS   (10 * SCALE_MS)
/*
 * How far behind the clock the transmitter may be and still catch up, in the
 * output's terms: a stall longer than this loses the time rather than sending a
 * burst nobody can play on time, like the host tick's backlog.
 */
#define I2S_PACE_BACKLOG_NS  (200 * SCALE_MS)
#define I2S_FRAME_BYTES      4

static uint32_t i2s_sample_rate(IPodClassicI2SState *s)
{
    return i2s_rate_from_div(s->clkdiv);
}

static bool i2s_sending(IPodClassicI2SState *s)
{
    return s->tx_running && (s->clkcon & 1);
}

/* What the FIFO receives, from the DMA as a block or a sample from the CPU. */
static void i2s_tx(void *opaque, const uint8_t *buf, unsigned bytes)
{
    IPodClassicI2SState *s = opaque;

    ipod_classic_audio_out_write(buf, bytes);
    if (i2s_wav_on()) {
        for (unsigned i = 0; i + 2 <= bytes; i += 2) {
            uint16_t sample = buf[i] | (buf[i + 1] << 8);

            i2s_wav_sample(s, sample, 2);
        }
    }
}

/*
 * One drain tick: let DMAC0 hand over what the sample clock has used since the
 * last one, then come back.  A tick that moves nothing leaves the timer off,
 * because a transmitter that is on with no DMA behind it must not wake the main
 * loop for nothing.  The PL080 kicks this when a channel next has work.
 */
static void ipod_classic_i2s_drain(void *opaque)
{
    IPodClassicI2SState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL_RT);
    uint64_t rate_bytes = (uint64_t)i2s_sample_rate(s) * I2S_FRAME_BYTES;
    int64_t elapsed = MIN(now - s->pace_last_ns, I2S_PACE_BACKLOG_NS);
    uint64_t want, moved;

    if (!i2s_sending(s) || !s->dmac0) {
        return;
    }
    /* Not below zero: this clock can step when a machine is restored. */
    s->pace_frac += MAX(elapsed, 0) * rate_bytes;
    s->pace_last_ns = now;
    s->pace_due += s->pace_frac / NANOSECONDS_PER_SECOND;
    s->pace_frac %= NANOSECONDS_PER_SECOND;
    s->pace_due = MIN(s->pace_due,
                      I2S_PACE_BACKLOG_NS * rate_bytes / NANOSECONDS_PER_SECOND);

    want = s->pace_due & ~(uint64_t)(I2S_FRAME_BYTES - 1);
    moved = pl080_pace_request(s->dmac0, S5L8702_I2S_DMA_IIS0_TX, want);
    s->pace_due -= moved;

    ipod_classic_audio_out_note_tick(moved < want);
    if (moved > 0 && i2s_trace_on()) {
        fprintf(stderr, "I2STRACE drain: %" PRIu64 " of %" PRIu64 " bytes "
                "(%u Hz)\n", moved, want, i2s_sample_rate(s));
    }
    if (moved || !want) {
        timer_mod(s->drain_timer, now + I2S_PACE_PERIOD_NS);
    }
}

/*
 * A DMA channel can move again.  Not only when the drain has stopped asking: a
 * channel that waited for the guest to take an interrupt carries on with what
 * this tick already paid for, or tasks shorter than a tick would play slow.  With
 * nothing owed the next tick will do, and this runs on any DMA in the controller.
 */
static void i2s_pace_kick(void *opaque)
{
    IPodClassicI2SState *s = opaque;

    if (i2s_sending(s) && (!timer_pending(s->drain_timer) ||
                           s->pace_due >= I2S_FRAME_BYTES)) {
        timer_mod_anticipate(s->drain_timer,
                             qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL_RT));
    }
}

/* Follow the transmitter starting or stopping. */
static void i2s_pace_update(IPodClassicI2SState *s, bool was_sending)
{
    bool sending = i2s_sending(s);

    ipod_classic_audio_out_set_running(sending);
    if (sending && !was_sending) {
        /* Nothing has been clocked out yet: start from nothing owed. */
        s->pace_last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL_RT);
        s->pace_frac = 0;
        s->pace_due = 0;
        timer_mod(s->drain_timer, s->pace_last_ns + I2S_PACE_PERIOD_NS);
    } else if (!sending && was_sending) {
        timer_del(s->drain_timer);
    }
}

void ipod_classic_i2s_set_dmac0(IPodClassicI2SState *s, PL080State *dmac0)
{
    s->dmac0 = dmac0;
    /* Mem->peri transfers to the FIFO wait for our drain ticks. */
    if (dmac0) {
        pl080_set_paced_line(dmac0, S5L8702_I2S_DMA_IIS0_TX, i2s_tx,
                             i2s_pace_kick, s);
    }
}

/* MMIO ------------------------------------------------------------------ */

static uint64_t ipod_classic_i2s_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    IPodClassicI2SState *s = opaque;
    uint32_t val;

    switch (offset) {
    case S5L8702_I2S_CLKCON:
        /*
         * Bit 0 is the clock-enable the retailOS writes (see the header). Bit 1
         * is a read-only hardware STATUS bit: "the I2S clock has stopped". The
         * retailOS's IRAM audio driver, when it (re)configures the clock for a
         * real play, DISABLES it first -- `str 0,[CLKCON]` -- and then spins
         *     loop: ldr r0,[CLKCON]; tst r0,#2; beq loop
         * waiting for that status bit before it reprograms CLKDIV/TXCON and
         * re-enables. On real silicon the bit rises as soon as the gated clock
         * settles; our register file only ever returned the written 0, so bit 1
         * stayed clear and the driver span forever at IRAM 0x2200881c -- the
         * exact reason a Shuffle-Songs play reached this driver but never got
         * past it to decode a single PCM frame (measured: 161e6 CLKCON reads,
         * all 0). Model the bit: the clock is "stopped" whenever the enable bit
         * is clear. Boot's silence-init uses the enable path and never polls it;
         * Rockbox drives its own I2S and never reads this bit -- both unaffected.
         */
        val = s->clkcon;
        if (!(s->clkcon & 1)) {
            val |= 2;
        }
        break;
    case S5L8702_I2S_TXCON:  val = s->txcon;  break;
    case S5L8702_I2S_TXCOM:  val = s->txcom;  break;
    case S5L8702_I2S_RXCON:  val = s->rxcon;  break;
    case S5L8702_I2S_RXCOM:  val = s->rxcom;  break;
    case S5L8702_I2S_RXDB:   val = s->rxdb;   break;
    case S5L8702_I2S_CLKDIV: val = s->clkdiv; break;
    case S5L8702_I2S_STATUS:
        /*
         * The status register. Its bit meanings are UNDETERMINED -- no source
         * we have documents them and Rockbox never reads it. Returning 0 (what
         * the unimplemented window did) never hung the boot, so we keep 0 as
         * the base and let I2STRACE during a real play tell us which bit, if
         * any, the Now Playing screen waits on. See wiki/14.
         */
        val = 0;
        break;
    default:
        val = 0;
        break;
    }

    if (i2s_count_on()) {
        i2s_count_hit(offset, "rd", val);
    }
    if (i2s_trace_on()) {
        fprintf(stderr, "I2STRACE rd +0x%02" HWADDR_PRIx " -> 0x%08x  %-7s "
                "pc~0x%08x\n", offset, val, i2s_reg_name(offset),
                i2s_trace_pc());
    }
    return val;
}

static void ipod_classic_i2s_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    IPodClassicI2SState *s = opaque;

    switch (offset) {
    case S5L8702_I2S_CLKCON: {
        bool was_sending = i2s_sending(s);

        s->clkcon = value;
        i2s_pace_update(s, was_sending);
        break;
    }
    case S5L8702_I2S_TXCON:  s->txcon = value;  break;
    case S5L8702_I2S_TXCOM: {
        bool was_sending = i2s_sending(s);

        s->txcom = value;
        s->tx_running = (value & S5L8702_I2S_TXCOM_RUN) != 0;
        i2s_pace_update(s, was_sending);
        break;
    }
    case S5L8702_I2S_RXCON:  s->rxcon = value;  break;
    case S5L8702_I2S_RXCOM:  s->rxcom = value;  break;
    case S5L8702_I2S_CLKDIV:
        s->clkdiv = value;
        ipod_classic_audio_out_set_rate(i2s_sample_rate(s));
        break;
    case S5L8702_I2S_TXDB0: {
        /* A PCM sample from the CPU, or from DMA that is not a plain block. */
        uint32_t le = cpu_to_le32(value);

        i2s_tx(s, (const uint8_t *)&le, MIN(size, sizeof(le)));
        break;
    }
    default:
        break;
    }

    if (i2s_count_on()) {
        i2s_count_hit(offset, "wr", (uint32_t)value);
    }
    if (i2s_trace_on() && offset != S5L8702_I2S_TXDB0) {
        /* The FIFO gets thousands of writes a boot; count, do not print. */
        fprintf(stderr, "I2STRACE wr +0x%02" HWADDR_PRIx " = 0x%08x  %-7s "
                "pc~0x%08x\n", offset, (uint32_t)value, i2s_reg_name(offset),
                i2s_trace_pc());
    }
}

static const MemoryRegionOps ipod_classic_i2s_ops = {
    .read = ipod_classic_i2s_read,
    .write = ipod_classic_i2s_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void ipod_classic_i2s_reset(DeviceState *dev)
{
    IPodClassicI2SState *s = IPOD_CLASSIC_I2S(dev);

    s->clkcon = 0;
    s->txcon = 0;
    s->txcom = 0;
    s->rxcon = 0;
    s->rxcom = 0;
    s->rxdb = 0;
    s->clkdiv = 0;
    s->tx_running = false;
    if (s->drain_timer) {
        timer_del(s->drain_timer);
    }
    ipod_classic_audio_out_set_running(false);
    ipod_classic_audio_out_flush();
}

static void ipod_classic_i2s_realize(DeviceState *dev, Error **errp)
{
    IPodClassicI2SState *s = IPOD_CLASSIC_I2S(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &ipod_classic_i2s_ops, s,
                          TYPE_IPOD_CLASSIC_I2S, S5L8702_I2S_MEM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    s->drain_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL_RT,
                                  ipod_classic_i2s_drain, s);
}

/*
 * The timer and the clock's arithmetic are not state: a restored machine owes
 * nothing, and starts sending again from now.  What was queued for the old one
 * must not play.
 */
static int ipod_classic_i2s_post_load(void *opaque, int version_id)
{
    IPodClassicI2SState *s = opaque;

    ipod_classic_audio_out_flush();
    ipod_classic_audio_out_set_rate(i2s_sample_rate(s));
    timer_del(s->drain_timer);
    i2s_pace_update(s, false);
    return 0;
}

static const VMStateDescription vmstate_ipod_classic_i2s = {
    .name = TYPE_IPOD_CLASSIC_I2S,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ipod_classic_i2s_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(clkcon, IPodClassicI2SState),
        VMSTATE_UINT32(txcon, IPodClassicI2SState),
        VMSTATE_UINT32(txcom, IPodClassicI2SState),
        VMSTATE_UINT32(rxcon, IPodClassicI2SState),
        VMSTATE_UINT32(rxcom, IPodClassicI2SState),
        VMSTATE_UINT32(rxdb, IPodClassicI2SState),
        VMSTATE_UINT32(clkdiv, IPodClassicI2SState),
        VMSTATE_BOOL(tx_running, IPodClassicI2SState),
        VMSTATE_END_OF_LIST()
    },
};

static void ipod_classic_i2s_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "S5L8702 I2S transmitter";
    dc->realize = ipod_classic_i2s_realize;
    dc->reset = ipod_classic_i2s_reset;
    dc->vmsd = &vmstate_ipod_classic_i2s;
    dc->user_creatable = false;
}

static const TypeInfo ipod_classic_i2s_info = {
    .name          = TYPE_IPOD_CLASSIC_I2S,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodClassicI2SState),
    .class_init    = ipod_classic_i2s_class_init,
};

static void ipod_classic_i2s_register_types(void)
{
    type_register_static(&ipod_classic_i2s_info);
}

type_init(ipod_classic_i2s_register_types)
