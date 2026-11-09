/*
 * The iPod classic's piezo, the speaker inside the case that the clicker
 * and the firmware's beeps play on.  See ipod_classic_piezo.c.
 */

#ifndef HW_ARM_IPOD_CLASSIC_PIEZO_H
#define HW_ARM_IPOD_CLASSIC_PIEZO_H

#define IPOD_PIEZO_RING     16u     /* drives the page can fall behind by */
#define IPOD_PIEZO_WORDS    6u      /* 32-bit words per drive */

/*
 * Whether GPIO 0.6 or 0.7 is set to carry timer A's output (functions 3 and
 * 5), which is how both firmwares connect the piezo to the timer.
 */
void ipod_classic_piezo_set_route(bool routed);

/*
 * The period of timer A's output in ns while it toggles, 0 while it does
 * not (stopped, or in a mode with no output).
 */
void ipod_classic_piezo_set_tout(uint64_t period_ns);

#endif
