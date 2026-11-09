/*
 * The machine's audio output: what the I2S clocks out to the codec, published
 * for whatever plays it.  See ipod_classic_audio_out.c.
 */

#ifndef HW_ARM_IPOD_CLASSIC_AUDIO_OUT_H
#define HW_ARM_IPOD_CLASSIC_AUDIO_OUT_H

#define IPOD_AUDIO_OUT_FRAMES   (1u << 16)  /* a power of two */
#define IPOD_AUDIO_OUT_UNITY    65536u      /* a gain of 1 */

/* Queue 16-bit little-endian stereo samples, L first.  Never blocks. */
void ipod_classic_audio_out_write(const void *samples, size_t bytes);

/* The rate of the frames from now on, in Hz. */
void ipod_classic_audio_out_set_rate(uint32_t hz);

/* Whether the transmitter is sending, so that a dry ring is not an underrun. */
void ipod_classic_audio_out_set_running(bool running);

/* The codec's gain for each channel, IPOD_AUDIO_OUT_UNITY being 0 dB. */
void ipod_classic_audio_out_set_gain(uint32_t left, uint32_t right);

/* What is queued must not be played: the machine it came from is gone. */
void ipod_classic_audio_out_flush(void);

/* A tick of the sample clock, and whether the guest had less than it asked. */
void ipod_classic_audio_out_note_tick(bool short_of_data);

/*
 * What the transmitter knows about the frames it will send next: nothing (it
 * is not sending), only where its sample clock is, or how much the guest has
 * already queued for it.
 */
typedef enum {
    IPOD_AUDIO_OUT_IDLE = 0,
    IPOD_AUDIO_OUT_CLOCK = 1,
    IPOD_AUDIO_OUT_QUEUED = 2,
} IPodAudioOutAhead;

/*
 * The transmitter's answer: frames that will come out before the next one
 * the guest has not handed over yet (QUEUED), or that the clock has used
 * since the last were written (CLOCK).
 */
typedef IPodAudioOutAhead IPodAudioOutAheadFn(void *opaque, uint32_t *frames);
void ipod_classic_audio_out_set_ahead(IPodAudioOutAheadFn *fn, void *opaque);

/*
 * The ring's frame count at which what the guest writes next will come out
 * (QUEUED), or at which the sample clock is now (CLOCK).  See the .c file.
 */
IPodAudioOutAhead ipod_classic_audio_out_next(uint32_t *frame);

#endif
