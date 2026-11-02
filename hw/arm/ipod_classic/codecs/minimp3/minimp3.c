/*
 * minimp3's implementation, here and not in ipod_classic_sm1_codec.c so that it
 * builds with the codecs library's flags.  minimp3.h is upstream's, unchanged.
 */

/* wasm has no use for the x86 and ARM paths. */
#define MINIMP3_NO_SIMD
#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"
