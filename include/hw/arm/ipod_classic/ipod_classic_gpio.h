/*
 * S5L8702 GPIO, 0x3CF00000 (GPIO_BASE).
 *
 * Sixteen groups of eight pins, 0x20 bytes of registers each. Layout from
 * Rockbox, firmware/export/s5l87xx.h:1370-1384 (the GPIO_OFFSET_BITS==5 arm,
 * which is the S5L8702):
 *
 *   +0x00 PCON(i)   pin configuration, four bits per pin
 *   +0x04 PDAT(i)   pin data
 *   +0x08 PUNA(i)
 *   +0x0C PUNB(i)
 *   +0x10 PUNC(i)
 *
 * Pin n lives in group n>>3 at bit n&7. Apple's accessor is 0x08325584:
 *
 *   r2 = [GPIO_BASE + ((pin >> 3) << 5) + 4]   ; PDAT(pin >> 3)
 *   *out = (r2 >> (pin & 7)) & 1
 *
 * WHY THIS DEVICE EXISTS, and it is one bit:
 *
 * 0x080aeca4 is gpio_get_pin(86) and nothing else. 0x08328720 -- the body of
 * the RTXC task that owns storage arrival -- calls it as the last of four
 * guards and posts message type 0 ("mount") to queue 4 when it is 1, or type 1
 * ("no media") when it is 0:
 *
 *   bl 0x08294d7c ; cmp r0,#0 ; bne no_media     (PMU register 0x4b bit 2)
 *   bl 0x08295508 ; cmp r0,#0 ; bne no_media     (PMU 0x12/0x4b)
 *   bl 0x080a1474 ; cmp r0,#0 ; bne no_media     ([0x089120a1] != 100)
 *   bl 0x080aeca4 ; cmp r0,#0 ; movne r1,#0      <- gpio_get_pin(86)
 *   mov r0,#4 ; bl 0x080b2ce8                    ; post {r1} to queue 4
 *
 * MEASURED with a disk attached: the first three guards return 0 and the
 * fourth returned 0 as well, because GPIO was create_unimplemented_device() and
 * read as zero. So the guard chain posted "no media" when a disk was in fact
 * attached, which is plainly wrong and is what this device fixes. MEASURED
 * after the fix: GPIOTRACE shows PDAT(10) -> 0x40, i.e. pin 86 reads 1, and the
 * message becomes type 0.
 *
 * NECESSARY, NOT SUFFICIENT, and the earlier version of this note overstated
 * it. Fixing the pin does NOT make a boot with a disk draw: 0x080b2ce8
 * dispatches the announcement through [0x08911ad8], which is still 0 at 45 s
 * (measured through the monitor, "xp /1wx 0x08911ad8"), so the message is
 * dropped. The boot still parks in "while (!ready) wait();" at 0x081228c4 and
 * still screendumps 0 lit pixels. See wiki/11-classic6g-retailos.md sec.9 for
 * what actually blocks it.
 *
 * What pin 86 is CALLED is UNDETERMINED: Rockbox does not name it and no
 * datasheet we have covers it. What it DOES in Apple's firmware is verified
 * above, so this model drives it from the one fact it can defend -- whether a
 * drive is attached to the ATA controller. With no -drive it reads 0 and the
 * machine behaves exactly as it did before this device existed.
 *
 * Every other pin reads 0, which is also exactly what the unimplemented device
 * did. Two are worth naming because the firmware reads them:
 *
 *   pins 52,53 -> PDAT(6) bits 4,5, read by 0x0807ff94 (lcd_get_type). Zero
 *                 selects panel type 0, whose driver record is 0x08911fe0 and
 *                 whose command set is the 8-bit one -- consistent with the
 *                 LCD_CON = 0x80000c21 the guest actually writes. Leave at 0.
 *   PDAT(11)   -> read 2130 times in 30 s with no disk but only 8 times in 30 s
 *                 with one (measured with GPIOTRACE=1; the diskless figure is
 *                 inflated by the reboot cycle, which repeats the probe on
 *                 every attempt). Leaving it 0 is what the machine has always
 *                 done.
 *
 * GPIOCMD, +0x200, sets one pin's function: (group << 16) | (pin << 8) | cmd,
 * which is how Rockbox's gpio_init() writes it (0xe070e) and what Apple's
 * 0x0836cf7c builds on 2.0.5. cmd is the pin's PCON nibble, with 0xe and 0xf
 * being output low and high. The rest of its behaviour is UNDETERMINED: the
 * output level goes nowhere, as for a PDAT write.
 *
 * THE WHEEL'S LINK, and it is why GPIOCMD is modelled at all. The wheel talks
 * over E2..E5 (group 14), function 2 in both firmwares, and the hold switch
 * reaches it only through them. Nothing in the hardware path gates the wheel:
 * each firmware disconnects it by hand when the PMU says hold went on.
 *
 *   Rockbox, button-clickwheel.c:463:
 *       PCON14 = (PCON14 & ~0x00ffff00) | 0x000e0e00;   back to 0x00222200
 *   Apple 2.0.5, 0x08060ad0, on PMU event 0x200 (INT6 bit 1):
 *       hold on:  GPIOCMD 0xe040e, 0xe020e              back to 0xe0202, 0xe0402
 *
 * Both drive E2 and E4 low, so that is what cuts the link here. It is not "not
 * function 2": the machine starts with no bootrom and PCON14 at 0, and a guest
 * that never set the pins up must keep its wheel, as it always has.
 */

#ifndef HW_ARM_IPOD_CLASSIC_GPIO_H
#define HW_ARM_IPOD_CLASSIC_GPIO_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_IPOD_CLASSIC_GPIO "s5l8702.gpio"
OBJECT_DECLARE_SIMPLE_TYPE(IPodClassicGPIOState, IPOD_CLASSIC_GPIO)

#define S5L8702_GPIO_MEM_SIZE   0x1000
#define S5L8702_GPIO_GROUPS     16
#define S5L8702_GPIO_STRIDE     0x20

#define S5L8702_GPIO_PCON       0x00
#define S5L8702_GPIO_PDAT       0x04
#define S5L8702_GPIO_PUNA       0x08
#define S5L8702_GPIO_PUNB       0x0C
#define S5L8702_GPIO_PUNC       0x10
#define S5L8702_GPIO_CMD        0x200

/* The one pin this device exists for. See the note above. */
#define S5L8702_GPIO_PIN_MEDIA  86

/* The wheel's link. See the note above. */
#define S5L8702_GPIO_WHEEL_GROUP  14
#define S5L8702_GPIO_WHEEL_PIN_A  2
#define S5L8702_GPIO_WHEEL_PIN_B  4

struct IPodClassicGPIOState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    /* 1 while the guest holds the wheel's pins low; goes to the wheel. */
    qemu_irq wheel_cut;

    uint32_t pcon[S5L8702_GPIO_GROUPS];
    uint32_t puna[S5L8702_GPIO_GROUPS];
    uint32_t punb[S5L8702_GPIO_GROUPS];
    uint32_t punc[S5L8702_GPIO_GROUPS];
    /* What the pins read as. Inputs only; see the note on writes in the .c. */
    uint32_t pdat[S5L8702_GPIO_GROUPS];

    /* Property: drives pin 86. Set by the machine from drive_get(IF_IDE). */
    bool media_present;
};

#endif /* HW_ARM_IPOD_CLASSIC_GPIO_H */
