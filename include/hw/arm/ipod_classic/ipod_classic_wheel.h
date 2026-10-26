/*
 * S5L8702 click wheel, 0x3C200000 (WHEEL_BASE), IRQ 23.
 *
 * The wheel is not a set of buttons wired to GPIOs. It is a separate
 * microcontroller on a serial link; this block is that link's master. The
 * host writes a command word, kicks it, and the wheel answers -- and, once
 * running, the wheel volunteers a packet of its own every time a finger moves
 * or a button changes. That volunteered packet is the only thing the retailOS
 * user interface ever sees.
 *
 *
 * THE REGISTER MAP
 *
 * The base and the offsets are Rockbox's (firmware/export/s5l87xx.h:1642-1651:
 * WHEEL_BASE 0x3C200000, and IRQ_WHEEL 23 at :1901). The NAMES below are ours
 * except three: Rockbox calls the registers WHEEL00/04/08/0C/10, WHEELINT,
 * WHEELRX and WHEELTX, so only the last three are borrowed and WHEELCON,
 * WHEELGO, WHEELDIV, WHEELSTAT, WHEELMASK and WHEELPEND are names this port
 * invented to say what it believes each one does. The semantics are from
 * Apple's own code, disassembled with --adjust-vma=0x07FF51B8 over
 * assets/classic6g/osos_decrypted.bin.
 *
 *   +0x00 WHEELCON    control. VERIFIED as a sequence, UNDETERMINED bit by bit
 *   +0x04 WHEELGO     bit 0 starts the transfer queued in WHEELTX. VERIFIED
 *   +0x08 WHEELDIV    written 0x0003A980 once at init. Meaning UNDETERMINED
 *   +0x0C WHEELSTAT   bit 0 "a word is waiting", bit 3 "transmit in flight".
 *                     VERIFIED by use, see below
 *   +0x10 WHEELMASK   interrupt enable, three bits parallel to WHEELPEND.
 *                     Role INFERRED, and getting it wrong is fatal; see
 *                     "THE TRAP" below
 *   +0x14 WHEELPEND   pending, write-1-to-clear, three sources. VERIFIED
 *   +0x18 WHEELRX     the received word. VERIFIED
 *   +0x1C WHEELTX     the word to send. VERIFIED, and measured on the bus
 *
 * How each line was established, with the instruction that establishes it:
 *
 *   init, 0x08056198: WHEELCON &= 0xFFFF0000; &= ~0x70000; &= ~0x80000;
 *   &= ~0x400000; WHEELPEND = 7; WHEELMASK |= 1; WHEELDIV = 0x0003A980;
 *   WHEELCON |= 0x100000; WHEELCON |= 0x200000.  Before that, four calls to
 *   0x08325468 with (pin, 2, 0) for pins 0x72..0x75, which is Rockbox's
 *   PCON14 = (PCON14 & ~0x00ffff00) | 0x00222200 (button-clickwheel.c:386) --
 *   confirmed on the bus, the GPIO model logs writes of 0xe0202/0xe0302/
 *   0xe0402/0xe0502 to GPIO+0x200.
 *
 *   transmit, 0x08327940: WHEELCON &= ~0x200000; udelay(1);
 *   WHEELCON |= 0x200000; WHEELTX = 0x80000000 | (cmd >> 1); WHEELGO |= 1;
 *   then "while (WHEELSTAT & 8) if (timeout(1500)) break;" -- so bit 3 is
 *   transmit-busy and an implementation that never sets it leaves the wait at
 *   0x08327994 falling through on its first test, which is what we want.
 *
 *   one polled transaction, 0x0832777c: save WHEELMASK and write 0
 *   (0x080afb48 is exactly "r0 = WHEELMASK; WHEELMASK = 0; return r0");
 *   WHEELPEND = 1 twice; transmit; then spin for 2000 us on
 *   "if (WHEELPEND & 1) { if (WHEELSTAT & 1) break; WHEELPEND = 1; }";
 *   then "rx = (WHEELSTAT & 1) ? WHEELRX : 0"; WHEELPEND = 1; restore
 *   WHEELMASK.
 *
 *
 * THE TRAP, and it is the one that eats the first attempt
 *
 * WHEELMASK is saved and restored around every polled transaction. While the
 * block was create_unimplemented_device() every read returned zero, so what
 * got restored was zero -- and the boot log shows it: twelve writes of 0 to
 * +0x10 against a single write of 1. An implementation that does not read
 * back honestly leaves the interrupt masked for ever and no keypress can ever
 * arrive, however right the rest of it is.
 *
 * The same reasoning says: do NOT gate packet generation on any bit of
 * WHEELCON. What its bits mean is UNDETERMINED, and the firmware clears and
 * sets bit 21 around every single transmit. A model that demands "enabled"
 * there can go silent without saying why.
 *
 *
 * THE TWO PACKET FORMATS
 *
 * (a) The volunteered event packet, and this is the one that moves the user
 *     interface. Accepted by the ISR at 0x080479a8 if
 *     (rx & 0xBC0000FF) == 0x8000001A -- bit 31 set, bits 29..26 clear, low
 *     byte 0x1A. Rockbox uses the looser 0x800000FF (button-clickwheel.c:121)
 *     and the same 0x1A, which is an independent source agreeing.
 *
 *        31   30          25..16        12..8       7..0
 *       +---+------+----------------+----------+----------+
 *       | 1 | down |    position    | buttons  |   0x1A   |
 *       +---+------+----------------+----------+----------+
 *
 *     bit 30 is "a finger is on the wheel" (tst r0,#0x40000000 at 0x080479dc);
 *     buttons are (rx >> 8) & 0x1F (lsl #19 / lsr #27, 0x080479d8);
 *     position is (rx >> 16) & 0x3FF (mvn ip,#0xFC00 / and, 0x080479f0) and
 *     is stored as 0x40000000 | position. Apple masks ten bits, Rockbox seven;
 *     only 0..95 are ever used, see WHEEL_POSITIONS.
 *
 * (b) The answer to a polled command. Validated at 0x0832783c with
 *     bic #0x7F000000 + bic #0x00FF0000 + cmp, i.e. bit 31 and bits 15..0 must
 *     equal the command and bits 30..16 are free payload. Same as Rockbox's
 *     (status & 0x8000FFFF) == 0x8000023A (button-clickwheel.c:273) with more
 *     room.
 *
 * The five button bits are Rockbox's, and Apple's keycodes come from the five
 * arms of 0x0805c11c..0x0805c168:
 *
 *     packet bit 8  SELECT (centre)  -> keycode 3
 *     packet bit 9  RIGHT / next     -> keycode 1
 *     packet bit 10 LEFT / previous  -> keycode 2
 *     packet bit 11 PLAY             -> keycode 4
 *     packet bit 12 MENU             -> keycode 0
 *
 *
 * WHAT THE FIRMWARE DOES WITH A PACKET, because the model has to satisfy it
 *
 * The ISR writes the decoded state into the block at 0x089116d8 and posts
 * kernel event 5. The wheel task (0x08047a5c) wakes, calls the button edge
 * detector (0x0805c0e4) and, only if that found no edge, the scroll decoder
 * (0x0807a670). The scroll decoder, disassembled:
 *
 *     new = [+0x20]; old = [+0x18]; delta = new - old
 *     if (!(new & 0x40000000)) { if (old had the bit) reset; return; }
 *     if (!(old & 0x40000000)) { store and return; }        <- first touch
 *     if (delta >  72) delta -= 96                          <- 96 per turn
 *     if (delta < -72) delta += 96
 *     if (|delta| < [+0x04]) { [+0x08] += delta; goto tail; }
 *     if ((unsigned)(delta + 35) >= 71) goto tail;          <- window -35..35
 *     [+0x0C] += delta; [+0x08] = 0; deliver
 *   tail:
 *     if ((unsigned)([+0x08] + 3) >= 7) { [+0x04] = 0; [+0x08] = 0; }
 *
 * Three hard constraints fall out, and they are why this device paces itself
 * with a timer instead of dumping packets into the FIFO:
 *
 *   1. 96 positions to the turn.
 *   2. TWO packets with bit 30 set are needed before any delta is computed.
 *      A single "finger down and already moved" packet produces nothing, so
 *      a gesture has to start with an arming packet at the unchanged position
 *      and the guest has to see it as a separate interrupt.
 *   3. The threshold [+0x04] is put back to 6 by the ISR on every untouched
 *      packet, and a delivered event leaves it at 6. So the first accepted
 *      movement of a gesture must be between 6 and 35 positions.
 *
 * (The threshold does fall to 0 after four sub-threshold clicks accumulate --
 * that is Apple's acceleration -- but relying on it means the first four key
 * presses of a gesture do nothing, which is not what a test wants to measure.)
 *
 *
 * THE INTERRUPT PATH IS ALREADY ARMED, and this was checked rather than hoped
 *
 * The lesson from the 5G (hw/arm/ipod_video/ipod_video_intc.c: HI_IRQ gating
 * the whole high bank) says raising a line proves nothing. Measured here:
 * 0x08047a68 registers 0x080479a8 on IRQ 23, and VIC0 INTENABLE reads
 * 0x01A3010F with the machine running -- bit 23 set. The IRAM dispatcher
 * (0x22004DA0) reads both VICs with no cross-condition, takes the number from
 * VICADDRESS, calls the handler, and ends with 0x01010101 to VICADDRESS. It
 * services ONE interrupt per entry and returns, so a level that stays high is
 * re-taken immediately -- which is what drains a FIFO with more than one
 * packet in it.
 */

#ifndef HW_ARM_IPOD_CLASSIC_WHEEL_H
#define HW_ARM_IPOD_CLASSIC_WHEEL_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qom/object.h"
#include "qemu/timer.h"

#define TYPE_IPOD_CLASSIC_WHEEL "s5l8702.wheel"
OBJECT_DECLARE_SIMPLE_TYPE(IPodClassicWheelState, IPOD_CLASSIC_WHEEL)

/*
 * 4 KB, not the 1 MB the unimplemented device covered. Anything past this in
 * the 0x3C200000 block falls to the machine's io floor, so -d unimp will say
 * so if the guest ever reaches for it. It never has.
 */
#define S5L8702_WHEEL_MEM_SIZE  0x1000

#define S5L8702_WHEEL_CON       0x00
#define S5L8702_WHEEL_GO        0x04
#define S5L8702_WHEEL_DIV       0x08
#define S5L8702_WHEEL_STAT      0x0C
#define S5L8702_WHEEL_MASK      0x10
#define S5L8702_WHEEL_PEND      0x14
#define S5L8702_WHEEL_RX        0x18
#define S5L8702_WHEEL_TX        0x1C

#define S5L8702_WHEEL_GO_START      0x00000001
#define S5L8702_WHEEL_STAT_RX       0x00000001  /* a word is waiting */
#define S5L8702_WHEEL_STAT_TXBUSY   0x00000008  /* never set; see the header */
#define S5L8702_WHEEL_PEND_RX       0x00000001
#define S5L8702_WHEEL_PEND_ALL      0x00000007

/* Packet (a). See "THE TWO PACKET FORMATS". */
#define S5L8702_WHEEL_EVENT_ID      0x8000001a
#define S5L8702_WHEEL_EVENT_MASK    0xbc0000ff
#define S5L8702_WHEEL_DOWN          0x40000000
#define S5L8702_WHEEL_POS_SHIFT     16
#define S5L8702_WHEEL_POS_MASK      0x3ff
#define S5L8702_WHEEL_BTN_SHIFT     8

/* Packet (b): the three commands the retailOS ever sends. */
#define S5L8702_WHEEL_CMD_BUTTONS   0x8000023a
#define S5L8702_WHEEL_CMD_POSITION  0x8000063a
#define S5L8702_WHEEL_CMD_ACK       0x8000062a

/* Button bits inside the event packet, at S5L8702_WHEEL_BTN_SHIFT. */
#define S5L8702_WHEEL_BTN_SELECT    0x01
#define S5L8702_WHEEL_BTN_RIGHT     0x02
#define S5L8702_WHEEL_BTN_LEFT      0x04
#define S5L8702_WHEEL_BTN_PLAY      0x08
#define S5L8702_WHEEL_BTN_MENU      0x10

/* WHEELCLICKS_PER_ROTATION, button-clickwheel.c:64, and the +/-96 of the
 * firmware's own wrap-around at 0x0807a6a4. Two independent sources. */
#define S5L8702_WHEEL_POSITIONS     96

/*
 * How far one keystroke turns the wheel. 6 is not a round number picked for
 * looks: it is the threshold the firmware itself holds in 0x089116d8+0x04 at
 * rest, measured through the monitor, and the smallest movement it will
 * deliver at the start of a gesture. Tunable at run time with
 * IPOD_WHEEL_STEP for experiments; the window the firmware accepts is 6..35.
 */
#define S5L8702_WHEEL_KEY_STEP      6

/*
 * Packet pacing. The real wheel reports on its own clock; here the rate has
 * to be slow enough that the guest's ISR AND its wheel task both get to run
 * between two packets. That is not a comfort margin, it is constraint 2: the
 * ISR overwrites 0x089116d8+0x20 on every packet, so if two packets are taken
 * before the task runs once, the arming packet is gone and the movement is
 * read as a first touch and discarded. 40 ms of a machine that boots this
 * firmware in about 25x real time is roughly 1.6 ms of iPod, which is plenty
 * for an interrupt and a task wake and still a plausible rate for a wheel.
 *
 * The finger has to stay down long enough for the user interface to drain the
 * delta before it comes off, because the decoder's untouch path
 * (0x0807a724) sets "last reported" equal to "accumulated" -- anything the
 * interface had not read yet is thrown away. 1.5 s is many repaints.
 *
 * Both are tunable at run time, for experiments, with IPOD_WHEEL_TICK_MS and
 * IPOD_WHEEL_UNTOUCH_MS. Neither is a fact about the hardware: the real
 * wheel's reporting rate is UNDETERMINED.
 */
#define S5L8702_WHEEL_TICK_MS       40
#define S5L8702_WHEEL_UNTOUCH_MS    1500

/*
 * BUTTONS ARE REPEATED, and this is the same constraint 2 as above, applied to
 * the half of the wheel that used to be exempt from it.
 *
 * A press used to be exactly two packets: one with the bit set, one with it
 * clear. The retailOS acts on the RELEASE -- its rules are called
 * "button.menu.up" and "button.select.up" -- so it needs the wheel task to run
 * and see the bit SET, and then run again and see it CLEAR. Miss either and
 * the whole press is gone. And the ISR overwrites the same state block on
 * every packet, so a task that does not run between the two never sees the
 * press at all.
 *
 * MEASURED, five SELECT/MENU round trips into the Music screen and back, one
 * press every 13 s, with the model reporting every packet queued and none
 * dropped. Counting only presses that had somewhere to go:
 *
 *              SELECT from the main menu     MENU out of Music
 *   one shot         1 of 3                       2 of 3
 *   repeated         5 of 10                      5 of 5
 *
 * So the state is repeated one packet per tick while a button is held, and for
 * a while after the release so that the UP edge gets the same treatment.
 * Tunable with IPOD_WHEEL_BTN_REPEAT_MS; 400 ms is ten ticks, and a monitor
 * "sendkey ret 150" therefore lands about four DOWN packets and ten UP ones.
 *
 * 400 MS IS NOT A FACT ABOUT THE HARDWARE. It is a model choice, and it is
 * held to the same standard as the two constants above: the real wheel's
 * reporting rate is UNDETERMINED and so is whether it keeps reporting after a
 * button comes back up. What IS known is only that the state of every button
 * travels in every packet the wheel sends (button-clickwheel.c:273 decodes all
 * five bits out of one status word), so repeating the state is at worst
 * harmless and at best exactly what the part does. The window was picked long
 * enough that the guest's wheel task cannot miss both edges and short enough
 * that it is over before the next keystroke; the measurement it bought is the
 * MENU column of the table above, and nothing here claims more than that.
 *
 * AND IT IS NOT THE WHOLE STORY, which is why the table above is printed with
 * its bad column: SELECT is still lost about half the time and this did not
 * change that, so the loss has a second cause that has NOT been found. The
 * suspect and the experiment are written down in wiki/11 s.18.8; the short
 * version is that the key dispatcher reads the PMU hold switch over I2C on
 * every single key and returns without doing anything if the answer is not
 * zero. Do not read this repeat as "input is fixed".
 */
#define S5L8702_WHEEL_BTN_REPEAT_MS 400

/*
 * Deep enough that a burst of keystrokes cannot overrun it, shallow enough
 * that if the guest is NOT draining we find out quickly through the dropped
 * counter rather than by guessing.
 */
#define S5L8702_WHEEL_FIFO_LEN      16

struct IPodClassicWheelState {
#ifdef EMSCRIPTEN
    /* Browser input: releases a held key at a virtual deadline, or as soon
     * as the guest has consumed a packet carrying the button. */
    int wasm_keycode;
    int64_t wasm_release_at;
    int64_t wasm_release_cap;
    uint32_t wasm_press_mask;
    uint32_t wasm_consumed;      /* presses the guest has consumed */
    uint32_t wasm_consumed0;     /* counter value when the press started */
#endif
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    /* Registers that read back what was written. */
    uint32_t con;
    uint32_t go;
    uint32_t div;
    uint32_t tx;
    uint32_t mask;      /* WHEELMASK: honest readback is load bearing */
    uint32_t pend;      /* latched bits 1..2 only; bit 0 follows the FIFO */

    /* Words waiting to be read out of WHEELRX, oldest first. */
    uint32_t fifo[S5L8702_WHEEL_FIFO_LEN];
    unsigned fifo_head;
    unsigned fifo_len;

    /* The physical wheel. */
    uint32_t position;      /* 0 .. S5L8702_WHEEL_POSITIONS - 1 */
    uint32_t buttons;       /* five bits, S5L8702_WHEEL_BTN_* */
    bool touched;
    /*
     * Whether the guest has already been told about this gesture at the
     * current position. Constraint 2: the first packet of a gesture must not
     * move, or the firmware throws the movement away.
     */
    bool armed;
    int32_t pending_steps;  /* positions still owed to the guest, signed */
    int64_t untouch_at;     /* virtual ns at which the finger comes off */
    /*
     * Virtual ns until which the button state keeps being REPEATED, one packet
     * per tick, instead of being sent once and forgotten. See
     * S5L8702_WHEEL_BTN_REPEAT_MS: without it a whole press can vanish.
     */
    int64_t button_repeat_until;

    /* What the last packet we sent said, so we do not repeat ourselves. */
    uint32_t sent_buttons;

    QEMUTimer *tick;
    uint32_t key_step;
    int64_t tick_ns;
    int64_t untouch_ns;
    int64_t button_repeat_ns;
    bool irq_level;

    /*
     * The WHEELTRACE=1 counters are NOT here: they are file statics in the .c,
     * because atexit() has to reach them and there is exactly one wheel. Four
     * per-instance copies used to sit here and nothing ever wrote to them.
     */
};

#endif /* HW_ARM_IPOD_CLASSIC_WHEEL_H */
