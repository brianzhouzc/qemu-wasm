/*
 * The machine's audio output, a ring of what the I2S clocked out.
 *
 * Under -display none the LCD publishes frames for the page to copy; this is the
 * same idea for sound, with the difference that a reader cannot pull at its own
 * pace and take whatever is newest.  The page plays this ring from an
 * AudioWorklet, on the audio thread, reading the module's memory directly.
 *
 * One writer and one reader, and the writer never waits: the DMA that feeds this
 * follows the guest's sample clock, and a page that is muted, not started or
 * behind must not be able to slow the guest down.  So the reader keeps its own
 * position, and one that falls a whole ring behind has lost those frames.
 *
 * The header's layout is a contract with src/audio-worklet.js, which reads it as
 * 32-bit words.  `write` counts frames and wraps; a frame's samples go in before
 * it moves.  `epoch` changes when what is queued must not be played, after a
 * reset or a restore, and the reader then starts again from `write`.
 */

#include "qemu/osdep.h"
#include "qemu/atomic.h"
#include "hw/arm/ipod_classic/ipod_classic_audio_out.h"

#if defined(EMSCRIPTEN)
#include <emscripten.h>
#endif

#define OUT_SAMPLES (IPOD_AUDIO_OUT_FRAMES * 2)

static struct {
    uint32_t write;         /* [0] frames written */
    uint32_t rate;          /* [1] Hz */
    uint32_t running;       /* [2] the transmitter is sending */
    uint32_t epoch;         /* [3] see above */
    uint32_t gain[2];       /* [4] [5] 16.16 */
    uint32_t frames;        /* [6] IPOD_AUDIO_OUT_FRAMES */
    uint32_t ticks;         /* [7] ticks of the sample clock */
    uint32_t short_ticks;   /* [8] ticks the guest could not fill */
    uint32_t reserved[7];
    int16_t pcm[OUT_SAMPLES];   /* at byte 64 */
} out = {
    .rate = 44100,
    .gain = { IPOD_AUDIO_OUT_UNITY, IPOD_AUDIO_OUT_UNITY },
    .frames = IPOD_AUDIO_OUT_FRAMES,
};

/*
 * In samples, not frames: DMA may stop on an odd one, and the pair is only
 * published once its other half has arrived.
 */
static uint32_t out_samples;

void ipod_classic_audio_out_write(const void *samples, size_t bytes)
{
    const uint8_t *from = samples;
    size_t left = bytes / 2;

    while (left) {
        uint32_t at = out_samples & (OUT_SAMPLES - 1);
        size_t n = MIN(left, OUT_SAMPLES - at);

        memcpy(&out.pcm[at], from, n * 2);
        out_samples += n;
        from += n * 2;
        left -= n;
    }
    qatomic_store_release(&out.write, out_samples / 2);
}

void ipod_classic_audio_out_set_rate(uint32_t hz)
{
    if (qatomic_read(&out.rate) != hz) {
        /* Frames still queued were sent at the old rate. */
        ipod_classic_audio_out_flush();
        qatomic_set(&out.rate, hz);
    }
}

void ipod_classic_audio_out_set_running(bool running)
{
    qatomic_set(&out.running, running);
}

void ipod_classic_audio_out_set_gain(uint32_t left, uint32_t right)
{
    qatomic_set(&out.gain[0], left);
    qatomic_set(&out.gain[1], right);
}

void ipod_classic_audio_out_flush(void)
{
    /* Line the next frame up with the left channel again. */
    out_samples = (out_samples + 1) & ~1u;
    qatomic_store_release(&out.write, out_samples / 2);
    qatomic_store_release(&out.epoch, qatomic_read(&out.epoch) + 1);
}

void ipod_classic_audio_out_note_tick(bool short_of_data)
{
    qatomic_set(&out.ticks, qatomic_read(&out.ticks) + 1);
    if (short_of_data) {
        qatomic_set(&out.short_ticks, qatomic_read(&out.short_ticks) + 1);
    }
}

#if defined(EMSCRIPTEN)
EMSCRIPTEN_KEEPALIVE uint32_t ipod_wasm_audio_ptr(void)
{
    return (uint32_t)(uintptr_t)&out;
}
#endif
