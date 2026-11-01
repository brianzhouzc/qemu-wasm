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

#endif
