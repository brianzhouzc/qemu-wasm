/*
 * The latency trace.  A press on the page goes through the main loop, the
 * wheel, the guest's interrupt and its tasks, the display pipe or the LCD
 * port, the compose, and the piezo or the audio ring before anyone sees or
 * hears it; this records when each of those happened, in host time and in
 * the guest's, so that the page can say where the time went.
 *
 * Instrumentation only: nothing here changes what the machine does.  Off by
 * default and costs one load per event while off.  LATTRACE=1 turns it on
 * from the start, ipod_wasm_lat_enable() at any time.
 *
 * The events go into a ring in the module's memory, as the piezo's drives
 * do, and the page reads it there (perf/input-latency.js in iPodWasm), so
 * recording one takes no lock, no message and no print: stderr is proxied to
 * the page's thread and a print would wait for it, which is the very delay
 * this is for measuring.  Any thread may write.  A writer takes a slot with
 * an atomic add and writes the slot's number last; a reader that sees the
 * number it expects has the whole event, and one that falls IPOD_LAT_RING
 * behind has lost the oldest.
 *
 * Each event is IPOD_LAT_WORDS 32-bit words:
 *   [0] [1] QEMU_CLOCK_REALTIME, ns, low and high: performance.timeOrigin +
 *           performance.now() on every thread of this build, so the page can
 *           put its own events on the same line
 *   [2] [3] QEMU_CLOCK_VIRTUAL, ns, low and high: the guest's clock
 *   [4] the event (ipod_classic_lat.h)
 *   [5] [6] its arguments
 *   [7] the event's number, counting from 1, written last
 *   [8] [9] guest instructions so far, low and high
 */

#include "qemu/osdep.h"
#include "qemu/atomic.h"
#include "qemu/timer.h"
#include "sysemu/cpu-timers.h"
#include "hw/core/cpu.h"
#include "hw/arm/ipod_classic/ipod_classic_lat.h"

#if defined(EMSCRIPTEN)
#include <emscripten.h>
#endif

static struct {
    uint32_t head;          /* [0] events recorded */
    uint32_t size;          /* [1] IPOD_LAT_RING */
    uint32_t words;         /* [2] IPOD_LAT_WORDS */
    uint32_t on;            /* [3] recording */
    uint32_t reserved[4];
    uint32_t ring[IPOD_LAT_RING][IPOD_LAT_WORDS];   /* at byte 32 */
} lat = {
    .size = IPOD_LAT_RING,
    .words = IPOD_LAT_WORDS,
};

static int lat_env = -1;

/* The main loop's sleeps, which util/ cannot report to this machine itself. */
extern void (*qemu_em_wait_hook)(int64_t asked_ns, int64_t took_ns);

static void lat_wait(int64_t asked_ns, int64_t took_ns)
{
    ipod_lat_mark2(IPOD_LAT_WAIT, asked_ns < 0 ? UINT32_MAX : asked_ns / 1000,
                   took_ns / 1000);
}

static void lat_set(bool on)
{
    qatomic_set(&lat.on, on);
#if defined(EMSCRIPTEN)
    qemu_em_wait_hook = on ? lat_wait : NULL;
#endif
}

bool ipod_lat_on(void)
{
    if (lat_env < 0) {
        const char *e = getenv("LATTRACE");

        lat_env = e && *e && *e != '0';
        if (lat_env) {
            lat_set(true);
        }
    }
    return qatomic_read(&lat.on);
}

void ipod_lat_mark2(uint32_t ev, uint32_t arg, uint32_t arg2)
{
    uint32_t n, *r;
    int64_t host, guest, insns;

    if (!ipod_lat_on()) {
        return;
    }
    host = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);
    guest = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    insns = icount_enabled() ? icount_get_raw() : 0;
    n = qatomic_fetch_inc(&lat.head) + 1;
    r = lat.ring[(n - 1) & (IPOD_LAT_RING - 1)];
    qatomic_set(&r[7], 0);
    qatomic_set(&r[0], (uint32_t)host);
    qatomic_set(&r[1], (uint32_t)(host >> 32));
    qatomic_set(&r[2], (uint32_t)guest);
    qatomic_set(&r[3], (uint32_t)(guest >> 32));
    qatomic_set(&r[4], ev);
    qatomic_set(&r[5], arg);
    qatomic_set(&r[6], arg2);
    qatomic_set(&r[8], (uint32_t)insns);
    qatomic_set(&r[9], (uint32_t)(insns >> 32));
    qatomic_store_release(&r[7], n);
}

/*
 * LATPC=1: where the guest is, every 2 ms of its time, while it is busy
 * (more than 60,000 instructions since the last look, about half of what
 * 2 ms holds): a long stretch of work and a delay loop look the same in the
 * trace otherwise.  A timer of guest time, so it runs on the guest's thread
 * between two blocks, where the pc is the guest's own.
 */
static QEMUTimer *lat_pc_timer;
static int64_t lat_pc_insns;

static void lat_pc_tick(void *opaque)
{
    int64_t insns = icount_get_raw();

    if (insns - lat_pc_insns > 60000 && current_cpu) {
        ipod_lat_mark(IPOD_LAT_PC,
                      CPU_GET_CLASS(current_cpu)->get_pc(current_cpu));
    }
    lat_pc_insns = insns;
    timer_mod(lat_pc_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 2000000);
}

void ipod_lat_init(void)
{
    const char *e = getenv("LATPC");

    if (e && *e && *e != '0' && icount_enabled() && !lat_pc_timer) {
        lat_pc_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lat_pc_tick, NULL);
        timer_mod(lat_pc_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 2000000);
    }
}

#if defined(EMSCRIPTEN)
EMSCRIPTEN_KEEPALIVE uint32_t ipod_wasm_lat_ptr(void)
{
    return (uint32_t)(uintptr_t)&lat;
}

EMSCRIPTEN_KEEPALIVE void ipod_wasm_lat_enable(int on)
{
    ipod_lat_on();
    lat_set(on != 0);
}
#endif
