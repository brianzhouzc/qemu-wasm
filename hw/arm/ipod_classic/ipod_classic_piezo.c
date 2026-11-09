/*
 * The piezo: the iPod classic's own speaker, inside the case, that the
 * clicker and the firmware's beeps play on.  It is not the headphone output:
 * the codec never sees it, and the firmware drives it with a timer.
 *
 * HOW BOTH FIRMWARES DRIVE IT.  Timer A's output (TOUT) toggles at a rate the
 * timer sets, and GPIO 0.6 and 0.7 are switched from plain outputs to the
 * timer's output (functions 3 and 5) for as long as the sound lasts:
 *
 *   Rockbox, piezo-6g.c: PCON0 = (PCON0 & 0x00ffffff) | 0x53000000, then
 *     timer A in PWM mode at 100 kHz with DATA1 = 2 * cycles as the period;
 *     its INT1 counts the periods and stops it, and PCON0 goes back to
 *     0xee (both low).  A key click is piezo_start(40, 4): 1.25 kHz, four
 *     periods, 3.2 ms.
 *
 *   Apple 2.0.5 (MEASURED with GPIOTRACE and TIMERTRACE on a detent at the
 *     Language screen, then read in the code): GPIOCMD 0x603 and 0x705
 *     (0x08364288), timer A in interval mode at 6 MHz with DATA0 =
 *     3,000,000 / Hz (0x08364648), so TOUT toggles twice a period; timer F
 *     as a one-shot of the duration at 1 kHz (0x083645e4), whose interrupt
 *     stops timer A; then GPIOCMD 0x60e and 0x70e, both low (0x08364260).
 *     The piezo task (0x0838a328) takes (Hz, ms) messages, and a click is
 *     500 Hz for 2 ms: one cycle.  0x082b2f24 is the same without the task,
 *     for beeps (1 kHz, and a 1.2/1.2/2.4 kHz sequence), timed with a sleep.
 *     On boards whose id is 0x10xxxx with a minor of 6 or more it uses timer
 *     D and gates with timer C; this machine's 0x13xxxx uses A and F.
 *
 * What pin 0.7's function 5 carries (TOUT inverted, for a bridge drive, or
 * another signal) is UNDETERMINED.  It does not matter here: the sound's
 * timing is TOUT's.
 *
 * WHAT THIS PUBLISHES is one record per drive, a stretch of time with the
 * pins routed and TOUT toggling at one period, once it is over: its period,
 * how long it lasted and how long the piezo was quiet before it, all in
 * QEMU_CLOCK_VIRTUAL, which is the firmware's clock, and the frame of the
 * audio output it goes with.  The page makes the drive's sound from its
 * edges (iPodWasm's src/effects.js).  Not the waveform, and not as it
 * happens:
 *
 *   - Under -icount sleep=off virtual time is not host time.  A click's
 *     2 ms gate, waited for by an idle guest, passes in microseconds; a
 *     busy guest takes longer than real time.  So the host-time instants
 *     of the edges mean nothing, and the page needs the drive's length in
 *     the firmware's time, which is only known at its end.  A click ends a
 *     few virtual milliseconds after it starts: the delay is that.
 *   - The gap lets the page keep a sequence of tones that arrived together
 *     (the guest warped through them) apart as the firmware timed them.
 *   - The frame is where the drive goes in the audio output's ring
 *     (ipod_classic_audio_out_next(), taken as it starts): the page plays the
 *     ring a queue behind, and plays the piezo when it plays that frame.  On
 *     2.0.5 that is where the clicker's other click, in the headphones,
 *     begins, and the two come out of the page's speakers as one.
 *
 * The record's layout is a contract with iPodWasm's src/page.html (32-bit
 * words; see `out` below).  One writer, any number of readers; a reader that
 * falls IPOD_PIEZO_RING drives behind has lost the oldest.  The writer
 * bumps `head` after the record is in, and wakes anyone waiting on it with
 * Atomics.waitAsync, so a click reaches the page as soon as it is over.
 *
 * Not modelled: TOUT's level (a drive is its period and its length), a piezo
 * bit-banged through GPIOCMD 0xe/0xf (Rockbox's bootloader does it; neither
 * firmware the page boots does), and timer D's route.
 *
 * PIEZOTRACE=1 prints each drive.
 */

#include "qemu/osdep.h"
#include "qemu/atomic.h"
#include "qemu/timer.h"
#include "hw/arm/ipod_classic/ipod_classic_piezo.h"
#include "hw/arm/ipod_classic/ipod_classic_audio_out.h"
#include "hw/arm/ipod_classic/ipod_classic_lat.h"

#if defined(EMSCRIPTEN)
#include <emscripten.h>
#include <emscripten/threading.h>
#endif

static struct {
    uint32_t head;          /* [0] drives published, wraps */
    uint32_t size;          /* [1] IPOD_PIEZO_RING */
    uint32_t words;         /* [2] IPOD_PIEZO_WORDS */
    uint32_t driving;       /* [3] the period of a drive now, 0 when quiet */
    uint32_t reserved[4];
    /*
     * At byte 32.  Drive n (counting from 1) is ring[(n - 1) % size]:
     *   [0] TOUT's period, ns
     *   [1] how long it lasted, ns, saturating
     *   [2] how long the piezo was quiet before it, ns, saturating
     *       (0xffffffff: no drive before it since the machine started)
     *   [3] n, written last, so a reader that reads it last can tell a
     *       record that was overwritten while it read it
     *   [4] the audio ring's frame count it goes with, wrapping
     *   [5] how [4] was found: IPodAudioOutAhead, 0 when the I2S was not
     *       sending and [4] means nothing
     */
    uint32_t ring[IPOD_PIEZO_RING][IPOD_PIEZO_WORDS];
} out = {
    .size = IPOD_PIEZO_RING,
    .words = IPOD_PIEZO_WORDS,
};

static bool routed;
static uint64_t tout_period;
static bool driving;
static uint64_t drive_period;
static int64_t drive_start;
static uint32_t drive_gap;
static uint32_t drive_frame;
static uint32_t drive_kind;
static bool ended_once;
static int64_t last_end;

static bool piezo_trace_on(void)
{
    static int on = -1;

    if (on < 0) {
        const char *e = getenv("PIEZOTRACE");
        on = e && *e && *e != '0';
    }
    return on;
}

static uint32_t saturate32(int64_t ns)
{
    if (ns <= 0) {
        return 0;
    }
    return ns >= UINT32_MAX ? UINT32_MAX : (uint32_t)ns;
}

static void piezo_publish(uint32_t period, uint32_t duration, uint32_t gap)
{
    uint32_t n = out.head + 1;
    uint32_t *r = out.ring[(n - 1) % IPOD_PIEZO_RING];

    qatomic_set(&r[0], period);
    qatomic_set(&r[1], duration);
    qatomic_set(&r[2], gap);
    qatomic_set(&r[4], drive_frame);
    qatomic_set(&r[5], drive_kind);
    qatomic_store_release(&r[3], n);
    qatomic_store_release(&out.head, n);
    ipod_lat_mark2(IPOD_LAT_PIEZO_OFF, n, drive_frame);
#if defined(EMSCRIPTEN)
    emscripten_futex_wake(&out.head, INT_MAX);
#endif
    if (piezo_trace_on()) {
        fprintf(stderr, "PIEZOTRACE: drive %u: %.1f Hz for %.3f ms, "
                "%.3f ms after the last, audio frame %u (%s)\n", n,
                period ? 1e9 / period : 0.0, duration / 1e6,
                gap == UINT32_MAX ? -1.0 : gap / 1e6, drive_frame,
                drive_kind == IPOD_AUDIO_OUT_QUEUED ? "after the queue"
                : drive_kind == IPOD_AUDIO_OUT_CLOCK ? "the clock" : "idle");
    }
}

static void piezo_update(void)
{
    uint64_t want = routed ? tout_period : 0;
    int64_t now;

    if (driving && want == drive_period) {
        return;
    }
    if (!driving && !want) {
        return;
    }
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (driving) {
        /* Over, or another period: that is a drive of its own. */
        piezo_publish(saturate32(drive_period), saturate32(now - drive_start),
                      drive_gap);
        driving = false;
        ended_once = true;
        last_end = now;
    }
    if (want) {
        driving = true;
        drive_period = want;
        drive_start = now;
        drive_gap = ended_once ? saturate32(now - last_end) : UINT32_MAX;
        drive_kind = ipod_classic_audio_out_next(&drive_frame);
        ipod_lat_mark2(IPOD_LAT_PIEZO_ON, saturate32(want), drive_frame);
    }
    qatomic_set(&out.driving, driving ? saturate32(drive_period) : 0);
}

void ipod_classic_piezo_set_route(bool on)
{
    routed = on;
    piezo_update();
}

void ipod_classic_piezo_set_tout(uint64_t period_ns)
{
    tout_period = period_ns;
    piezo_update();
}

#if defined(EMSCRIPTEN)
EMSCRIPTEN_KEEPALIVE uint32_t ipod_wasm_piezo_ptr(void)
{
    return (uint32_t)(uintptr_t)&out;
}
#endif
