/*
 * The audio decoders behind the SM1 engine.
 *
 * minimp3 (CC0) and FAAD2 (GPL, built for AAC LC alone), both under codecs/.
 * Code from FAAD2 is copyright (c) Nero AG, www.nero.com
 *
 * The firmware's demuxer does the parsing and hands over one frame at a time,
 * MP3 with its header and AAC as a bare raw_data_block, so this is only the
 * step from one frame to PCM.
 */

#include "qemu/osdep.h"
#include "hw/arm/ipod_classic/ipod_classic_sm1_codec.h"

#include "codecs/minimp3/minimp3.h"
#include "codecs/faad2/neaacdec.h"

#define MP3_FRAME_FRAMES 1152
#define AAC_FRAME_FRAMES 1024

struct IPodClassicSM1Codec {
    IPodClassicSM1CodecType type;
    unsigned channels;
    mp3dec_t mp3;
    NeAACDecHandle aac;
    unsigned aac_rate_index;
    unsigned aac_channels;
    bool aac_decoded;
};

static bool aac_open(IPodClassicSM1Codec *c, unsigned channels)
{
    NeAACDecConfigurationPtr config;
    /* AudioSpecificConfig: AAC LC, the rate index, the channel count. */
    unsigned char asc[2] = {
        (2 << 3) | (c->aac_rate_index >> 1),
        ((c->aac_rate_index & 1) << 7) | (channels << 3),
    };
    unsigned long rate;
    unsigned char got;

    if (c->aac) {
        NeAACDecClose(c->aac);
    }
    c->aac = NeAACDecOpen();
    if (!c->aac) {
        return false;
    }
    config = NeAACDecGetCurrentConfiguration(c->aac);
    config->outputFormat = FAAD_FMT_16BIT;
    config->downMatrix = 1;
    NeAACDecSetConfiguration(c->aac, config);
    c->aac_channels = channels;
    return NeAACDecInit2(c->aac, asc, sizeof(asc), &rate, &got) == 0;
}

IPodClassicSM1Codec *ipod_classic_sm1_codec_new(IPodClassicSM1CodecType type,
                                                unsigned rate_index)
{
    IPodClassicSM1Codec *c = g_new0(IPodClassicSM1Codec, 1);

    c->type = type;
    c->channels = 2;
    switch (type) {
    case IPOD_SM1_CODEC_MP3:
        mp3dec_init(&c->mp3);
        return c;
    case IPOD_SM1_CODEC_AAC:
        c->aac_rate_index = rate_index & 0xf;
        if (aac_open(c, 2)) {
            return c;
        }
        break;
    default:
        break;
    }
    ipod_classic_sm1_codec_free(c);
    return NULL;
}

void ipod_classic_sm1_codec_free(IPodClassicSM1Codec *c)
{
    if (c) {
        if (c->aac) {
            NeAACDecClose(c->aac);
        }
        g_free(c);
    }
}

static unsigned silence(int16_t *pcm, unsigned frames)
{
    memset(pcm, 0, frames * 2 * sizeof(*pcm));
    return frames;
}

static unsigned decode_mp3(IPodClassicSM1Codec *c, const uint8_t *frame,
                           size_t bytes, int16_t *pcm)
{
    mp3dec_frame_info_t info;
    int frames;

    /*
     * minimp3 will only sync on a frame it can see the next header after, and
     * this is handed one frame and no more.  A header it already holds skips
     * that search, and that is all this does: the frame's own length is still
     * checked against what was delivered.  Reset on nothing here, or the bit
     * reservoir goes with it.
     */
    if (bytes >= 4 && frame[0] == 0xff && (frame[1] & 0xe0) == 0xe0) {
        memcpy(c->mp3.header, frame, sizeof(c->mp3.header));
        c->channels = (frame[3] >> 6) == 3 ? 1 : 2;
    }
    frames = mp3dec_decode_frame(&c->mp3, frame, bytes, pcm, &info);
    if (frames <= 0) {
        /* Layer 3 is 1152 frames in MPEG-1 and 576 below it, as is silence. */
        return silence(pcm, bytes >= 2 && !(frame[1] & 0x08)
                       ? MP3_FRAME_FRAMES / 2 : MP3_FRAME_FRAMES);
    }
    return frames;
}

static unsigned decode_aac(IPodClassicSM1Codec *c, const uint8_t *frame,
                           size_t bytes, int16_t *pcm)
{
    NeAACDecFrameInfo info;
    void *out;
    unsigned frames;

    out = NeAACDecDecode(c->aac, &info, (unsigned char *)frame, bytes);
    if ((!out || info.error) && !c->aac_decoded && c->aac_channels == 2) {
        /*
         * The mailbox has the rate and not the channels, and most streams are
         * stereo.  A mono one shows itself on the first frame.
         */
        if (aac_open(c, 1)) {
            out = NeAACDecDecode(c->aac, &info, (unsigned char *)frame, bytes);
        }
    }
    if (!info.error && info.channels) {
        c->channels = MIN(info.channels, 2);
    }
    if (!out || info.error || !info.samples || !info.channels) {
        /* The first frame of a stream is only ever overlap. */
        return silence(pcm, AAC_FRAME_FRAMES);
    }
    c->aac_decoded = true;
    frames = info.samples / info.channels;
    if (frames > IPOD_SM1_CODEC_MAX_FRAMES) {
        return silence(pcm, AAC_FRAME_FRAMES);
    }
    if (info.channels == 1) {
        memcpy(pcm, out, frames * sizeof(*pcm));
    } else if (info.channels == 2) {
        memcpy(pcm, out, frames * 2 * sizeof(*pcm));
    } else {
        const int16_t *from = out;

        /* downMatrix makes this stereo; this is the case it does not cover. */
        for (unsigned i = 0; i < frames; i++) {
            pcm[i * 2] = from[i * info.channels];
            pcm[i * 2 + 1] = from[i * info.channels + 1];
        }
    }
    return frames;
}

unsigned ipod_classic_sm1_codec_decode(IPodClassicSM1Codec *c,
                                       const uint8_t *frame, size_t bytes,
                                       int16_t *pcm, unsigned *channels)
{
    unsigned frames = c->type == IPOD_SM1_CODEC_MP3
                      ? decode_mp3(c, frame, bytes, pcm)
                      : decode_aac(c, frame, bytes, pcm);

    *channels = c->channels;
    return frames;
}
