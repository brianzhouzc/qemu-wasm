/*
 * The audio decoders behind the SM1 engine.
 *
 * See ipod_classic_sm1.h for why these exist: the firmware hands SM1 one
 * compressed frame at a time and expects PCM back, and the engine's own
 * program cannot be run here.
 */

#ifndef HW_ARM_IPOD_CLASSIC_SM1_CODEC_H
#define HW_ARM_IPOD_CLASSIC_SM1_CODEC_H

#include <stddef.h>
#include <stdint.h>

/* The most stereo frames one compressed frame can decode to. */
#define IPOD_SM1_CODEC_MAX_FRAMES 1152

typedef enum {
    IPOD_SM1_CODEC_NONE,
    IPOD_SM1_CODEC_MP3,
    IPOD_SM1_CODEC_AAC,
} IPodClassicSM1CodecType;

typedef struct IPodClassicSM1Codec IPodClassicSM1Codec;

/*
 * A decoder for one stream.  `rate_index` is the AAC sample rate index the
 * firmware puts in the mailbox, and MP3 takes no parameters.  NULL if the
 * type cannot be decoded.
 */
IPodClassicSM1Codec *ipod_classic_sm1_codec_new(IPodClassicSM1CodecType type,
                                                unsigned rate_index);
void ipod_classic_sm1_codec_free(IPodClassicSM1Codec *c);

/*
 * Decode exactly one frame into interleaved stereo.  Returns the frames
 * written, always at least one frame's worth: a frame that yields nothing
 * (the start of a stream, a damaged frame) comes out as that much silence, so
 * that time keeps moving and the player does not take it for the end of the
 * stream.
 */
unsigned ipod_classic_sm1_codec_decode(IPodClassicSM1Codec *c,
                                       const uint8_t *frame, size_t bytes,
                                       int16_t *pcm);

#endif
