/*
 * The latency trace: where the time between a press on the page and what
 * the guest does about it goes.  See ipod_classic_lat.c.
 */

#ifndef HW_ARM_IPOD_CLASSIC_LAT_H
#define HW_ARM_IPOD_CLASSIC_LAT_H

#define IPOD_LAT_RING       65536u  /* events kept; a power of two */
#define IPOD_LAT_WORDS      10u     /* 32-bit words per event */

/* The events, in the order a press makes them.  A contract with the page. */
enum {
    IPOD_LAT_KEY = 1,       /* the press reaches the machine: arg keycode */
    IPOD_LAT_PUSH = 2,      /* the wheel queues a word: arg the word */
    IPOD_LAT_IRQ = 3,       /* the wheel's line rises */
    IPOD_LAT_POP = 4,       /* the guest takes a word: arg the word */
    IPOD_LAT_TICK = 5,      /* the wheel's tick: arg 1 if host time made it,
                               arg2 the guest's pc, if on its thread */
    IPOD_LAT_RELEASE = 6,   /* the press's key goes up: arg keycode */
    IPOD_LAT_PIXELS = 7,    /* first pixel through the LCD port since a frame */
    IPOD_LAT_PIXELS_END = 8,/* the port's window filled: arg pixels so far */
    IPOD_LAT_LAYERS = 9,    /* first display pipe write since a frame */
    IPOD_LAT_FRAME = 10,    /* a changed frame published: arg its sequence */
    IPOD_LAT_PIEZO_ON = 11, /* a piezo drive starts: arg its period, ns */
    IPOD_LAT_PIEZO_OFF = 12,/* ...is over and published: arg its number */
    IPOD_LAT_AUDIO = 13,    /* frames into the audio ring: arg the write count */
    IPOD_LAT_WAIT = 14,     /* the main loop slept: arg asked, arg2 took, us */
    IPOD_LAT_COMPOSE = 15,  /* the compose ran, changed or not: arg2 took, us */
    IPOD_LAT_DISP = 16,     /* any display pipe write: arg offset, arg2 value */
    IPOD_LAT_PC = 17,       /* LATPC=1: the guest's pc while it is busy */
};

/* The machine is up: start what needs a timer (LATPC). */
void ipod_lat_init(void);

/* Whether the trace is on: LATTRACE=1, or ipod_wasm_lat_enable(1). */
bool ipod_lat_on(void);

/* Record an event now, from any thread.  Does nothing while off. */
void ipod_lat_mark2(uint32_t ev, uint32_t arg, uint32_t arg2);

static inline void ipod_lat_mark(uint32_t ev, uint32_t arg)
{
    ipod_lat_mark2(ev, arg, 0);
}

#endif
