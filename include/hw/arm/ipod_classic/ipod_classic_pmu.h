/*
 * NXP PCF50635 power management unit of the iPod classic 6G.
 *
 * Register numbers from Rockbox firmware/export/pcf5063x.h; the three
 * undocumented ones from firmware/target/arm/s5l8702/ipod6g/pmu-target.h,
 * where they are introduced as "undocummented PMU registers".
 *
 * The I2C address is 0xe6 as both drivers write it, i.e. the 8-bit form with
 * the read/write bit already in place. Rockbox says so in pmu-6g.c:
 * "i2c_read(0, 0xe6, ...)". The 7-bit address QEMU's i2c core wants is 0x73.
 */

#ifndef HW_ARM_IPOD_CLASSIC_PMU_H
#define HW_ARM_IPOD_CLASSIC_PMU_H

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_IPOD_CLASSIC_PMU "pcf50635"
OBJECT_DECLARE_SIMPLE_TYPE(IPodClassicPMUState, IPOD_CLASSIC_PMU)

#define S5L8702_PMU_I2C_ADDR   0x73   /* 0xe6 >> 1 */
#define S5L8702_PMU_NUM_REGS   0x100

#define PCF5063X_REG_VERSION   0x00
#define PCF5063X_REG_VARIANT   0x01
#define PCF5063X_REG_INT1      0x02
#define PCF5063X_REG_INT2      0x03
#define PCF5063X_REG_INT3      0x04
#define PCF5063X_REG_INT4      0x05
#define PCF5063X_REG_INT5      0x06
#define PCF5063X_REG_INT1M     0x07
#define PCF5063X_REG_INT2M     0x08
#define PCF5063X_REG_INT3M     0x09
#define PCF5063X_REG_INT4M     0x0a
#define PCF5063X_REG_INT5M     0x0b
#define PCF5063X_REG_OOCSHDWN  0x0c
#define PCF5063X_REG_OOCSTAT   0x12
#define PCF5063X_REG_GPIOCTL   0x13
#define PCF5063X_REG_DCDCSTAT  0x41
#define PCF5063X_REG_LDOSTAT   0x42
#define PCF5063X_REG_MBCS1     0x4b
#define PCF5063X_REG_MBCS2     0x4c
#define PCF5063X_REG_MBCS3     0x4d
#define PCF5063X_REG_ADCC3     0x52
#define PCF5063X_REG_ADCC2     0x53
#define PCF5063X_REG_ADCC1     0x54
#define PCF5063X_REG_ADCS1     0x55
#define PCF5063X_REG_ADCS2     0x56
#define PCF5063X_REG_ADCS3     0x57
#define PCF5063X_REG_RTCSC     0x59
#define PCF5063X_REG_RTCMT     0x5e
#define PCF50635_REG_INT6      0x85
#define PCF50635_REG_INT6M     0x86
#define PCF50635_REG_GPIOSTAT  0x87

#define PCF5063X_ADCC1_ADCSTART   0x01
#define PCF5063X_ADCS3_ADCRDY     0x80

/*
 * Power-status registers, and the healthy values a running iPod reports.
 *
 * OOCSTAT (0x12) is the on/off-controller status. Bits, from Rockbox's
 * pcf5063x.h: ONKEY 0x01, EXTON1..3 0x02/0x04/0x08, BUBPRES 0x10, SYSOK 0x20,
 * BATOK 0x40, TMPOK 0x80. Apple's power manager polls it and, when BATOK/SYSOK
 * read clear, decides the battery is empty -> "battery low" -> software reset
 * (see the note in ipod_classic_pmu.c). A plain register file leaves this at a
 * stale/zero value with those bits clear, which is why it gives up.
 *
 * MBCS1 (0x4b) is the main-battery-charger status: USBPRES 0x01, USBOK 0x02,
 * ADAPTPRES 0x04, ADAPTOK 0x08, TBAT in bits 4..5 (00 = OK, 20 = below range),
 * PREWDTEXP 0x40, WDTEXP 0x80. The stale value 0x20 reads as TBAT_BELOW, a
 * spurious temperature fault.
 *
 * CONVENIENCE VALUES, marked as such: this machine has no PMU analogue state to
 * derive them from, so the honest report is "powered on, battery/system/temp
 * OK, running on battery with no charger fault". OOCSTAT_HEALTHY sets
 * ONKEY|SYSOK|BATOK|TMPOK; MBCS1_HEALTHY is TBAT_OK with nothing else, matching
 * the emulated "no USB/adapter" reality (Rockbox reads MBCS1 for USB/FireWire
 * presence and this is the state it already expects).
 */
#define PCF5063X_OOCSTAT_HEALTHY  0xe1   /* ONKEY|SYSOK|BATOK|TMPOK */
#define PCF5063X_MBCS1_USBPRES    0x01
#define PCF5063X_MBCS1_USBOK      0x02
#define PCF5063X_MBCS1_ADAPTPRES  0x04
#define PCF5063X_MBCS1_ADAPTOK    0x08
#define PCF5063X_MBCS1_HEALTHY    0x00   /* TBAT_OK, no charger, no WDT expiry */

/*
 * GPIOSTAT bit 1, PCF50635_GPIOSTAT_GPIO2 in Rockbox's pmu-target.h:43. It is
 * the hold switch, inverted: pmu-6g.c:244 reads it as
 * "holdswitch = !(GPIOSTAT & GPIO2)", so SET means hold is OFF. Apple's
 * keycode dispatcher reaches the same conclusion from the other direction --
 * see the long note at the reset function. Clear it and no button on the
 * click wheel does anything at all.
 */
#define PCF50635_GPIOSTAT_HOLD_OFF  0x02

/*
 * ASSUMED. 844 of 1023 is 3900 mV through Rockbox's own conversion in
 * pmu-6g.c (raw * 2000 / 1023 + 2250). Nothing in our sources says what a real
 * cell reads; this is chosen to be an ordinary value rather than an
 * interesting one.
 */
#define S5L8702_PMU_ADC_BATTERY_RAW  844

struct IPodClassicPMUState {
    I2CSlave parent_obj;

    uint8_t reg[S5L8702_PMU_NUM_REGS];
    uint8_t addr;         /* the register pointer, which survives a STOP */
    bool addr_pending;    /* next byte written is the register number */

    /*
     * Support for un-clobbering the register pointer after a register WRITE.
     * addr_saved is the pointer as it stood before the current transaction; it
     * is restored at STOP if, and only if, the transaction actually wrote a
     * data byte (wrote_data). See the long note in ipod_classic_pmu.c.
     */
    uint8_t addr_saved;
    bool wrote_data;
};

#endif
