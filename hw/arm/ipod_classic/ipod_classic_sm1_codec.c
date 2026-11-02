/*
 * The audio decoders behind the SM1 engine.
 *
 * There are none yet.  With no decoder the engine takes a frame and gives
 * nothing back, the firmware calls that the end of the stream, and the track
 * ends where it started.
 */

#include "qemu/osdep.h"
#include "hw/arm/ipod_classic/ipod_classic_sm1_codec.h"

IPodClassicSM1Codec *ipod_classic_sm1_codec_new(IPodClassicSM1CodecType type,
                                                unsigned rate_index)
{
    return NULL;
}

void ipod_classic_sm1_codec_free(IPodClassicSM1Codec *c)
{
}

unsigned ipod_classic_sm1_codec_decode(IPodClassicSM1Codec *c,
                                       const uint8_t *frame, size_t bytes,
                                       int16_t *pcm)
{
    return 0;
}
