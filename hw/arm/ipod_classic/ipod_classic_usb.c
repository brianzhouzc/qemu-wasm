/*
 * S5L8702 USB: the DesignWare OTG core as a device, its PHY, and a host.
 *
 * What both firmwares do with it (MEASURED with a probe build, qemu 0243833):
 *
 *  - VBUS is the PMU's EXTON2 (ipod_classic_pmu_set_vbus()).  On its rising
 *    edge Apple's 2.0.5 powers and tunes the PHY at 0x3C400000, resets the
 *    core, reads GSNPSID and GHWCFG1-4, and sets the core up as a device with
 *    buffer DMA: GAHBCFG 0x2b, GUSBCFG 0x1408, DCFG 0x40804, GINTMSK
 *    0xa01c1806, OUT0 armed for a SETUP (DOEPTSIZ0 0x60080040, DOEPDMA0 in
 *    DRAM, DOEPCTL0 0x88000000), then DCTL 0xd00 to leave soft-disconnect.
 *    On the falling edge it tears the core down.
 *  - Rockbox (firmware/drivers/usb-designware.c, DMA mode, "core 2.60a")
 *    does the same in its own words, and panics in usb_dw_check_hw() unless
 *    GHWCFG2-4 describe a core with internal DMA, dedicated Tx FIFOs, at
 *    least USB_NUM_ENDPOINTS (6) endpoints and 0x580 words of FIFO memory.
 *    With every register reading zero it drew its logf dump and stopped.
 *
 * So this model is the core as those two drivers use it, nothing more:
 *
 *  - The identity registers describe a 2.60a core with internal DMA,
 *    dedicated FIFOs, 7 endpoints each way and 0x820 words of FIFO, the
 *    numbers in Rockbox's usb-s5l8702.c ("Available FIFO memory: 0x820
 *    words").  GHWCFG values beyond what the drivers check are ASSUMED.
 *  - Buffer DMA only: a transfer reads or writes guest memory at DxEPDMAn
 *    and updates DxEPTSIZn the way the drivers read them back (residual
 *    XferSize, PktCnt, SUPCnt).  Slave mode, the FIFO windows, descriptor
 *    DMA, host mode, isochronous scheduling and LPM are not modelled.
 *  - The handshakes the drivers spin on with short timeouts (EPDIS ->
 *    EPDisbld, SNAK -> NAKSts and INEPNakEff, SGOUTNak -> GOUTNakEff, FIFO
 *    flushes, core reset, AHB idle) complete inside the register write.
 *  - The PHY is a register file: the drivers write it and read it back.
 *
 * The host.  A page cannot hand the emulated iPod to the real computer (WebUSB
 * is host-side only), so the other end of the cable is emulated too.  Once
 * the device pulls up, the host resets the bus, then enumerates (device
 * descriptor, address, configuration, strings) and sets configuration 1,
 * logging what the firmware answers.  Then it mounts the disk: the
 * configuration's mass-storage interface (08/06/50) gets GET_MAX_LUN, then
 * INQUIRY, TEST UNIT READY until the firmware has handed its disk over, and
 * READ CAPACITY, in Bulk-Only Transport over the two bulk endpoints.  From
 * then on it runs the page's requests (ipod_wasm_usb_io(): READ(10),
 * WRITE(10), eject), one at a time, through a buffer the page fills or reads.
 * It reacts on QEMU_CLOCK_VIRTUAL as soon as the driver arms an endpoint, and
 * never acts inside a guest register access; but it waits, polls and times out
 * in host time as well (usb_host_poll, usb_host_deadline), because the virtual
 * clock warps while the vCPU sleeps and Rockbox's tick runs on host time.
 *
 * MEASURED (Node, JIT): Rockbox's disk is 4096-byte blocks, the whole disk;
 * its first TEST UNIT READY says "medium not present" (2/3A/00), the next one
 * is ready; a 256 KB read takes 31 ms (16 ms a request in Chrome).  Apple's
 * 2.0.5 says "becoming ready" (2/04/01) for 80 s of guest time, about 9 s of
 * host time, then exports only its FAT32 data partition, in 4096-byte blocks;
 * it keeps written blocks in RAM until the eject, which it answers with "OK to
 * Disconnect".
 *
 * USBTRACE=1 logs host events, descriptors and the mass-storage setup; 2 adds
 * the page's SCSI commands and register writes; 3 adds reads.
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "exec/address-spaces.h"
#include "exec/memory.h"
#include "hw/core/cpu.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "hw/arm/ipod_classic/ipod_classic_usb.h"

#ifdef EMSCRIPTEN
#include <emscripten.h>
#endif

void ipod_classic_set_usb_cable(bool on);

/* ---- identity: what the drivers check ---- */

#define USB_GSNPSID  0x4f54260au   /* "OT2", 2.60a */
#define USB_GHWCFG1  0x00000000u   /* every endpoint bidirectional */
/*
 * OtgMode 0, OtgArch 2 (internal DMA), HSPhyType 1 (UTMI+), NumDevEps 6 (so 7
 * with EP0), NumHstChnl 15, PerioSupport, DynFifoSizing, NPTxQDepth 1,
 * PTxQDepth 2, TknQDepth 8.
 */
#define USB_GHWCFG2  ((2u << 3) | (1u << 6) | ((S5L8702_USB_NUM_EPS - 1u) << 10) | \
                      (15u << 14) | (1u << 18) | (1u << 19) | (1u << 22) | \
                      (2u << 24) | (8u << 26))
/* XferSizeWidth 8, PktSizeWidth 6, OtgEn, DfifoDepth 0x820 words. */
#define USB_GHWCFG3  ((8u << 0) | (6u << 4) | (1u << 7) | (0x820u << 16))
/* PhyDataWidth 16 bit, DedFifoMode, NumInEps 6. */
#define USB_GHWCFG4  ((1u << 14) | (1u << 25) | (6u << 26))
/* DAINTMSK bits that exist: IN and OUT 0..6. */
#define USB_EP_MASK  (((1u << S5L8702_USB_NUM_EPS) - 1u) * 0x10001u)

/* ---- the host's script ---- */

#define USB_REQ_GET_DESCRIPTOR     6
#define USB_REQ_SET_ADDRESS        5
#define USB_REQ_SET_CONFIGURATION  9
#define USB_REQ_CLEAR_FEATURE      1
#define USB_DT_DEVICE              1
#define USB_DT_CONFIG              2
#define USB_DT_STRING              3
/* Mass-storage class requests and Bulk-Only Transport. */
#define USB_REQ_GET_MAX_LUN        0xfe
#define USB_BOT_CBW_SIG            0x43425355u   /* "USBC" */
#define USB_BOT_CSW_SIG            0x53425355u   /* "USBS" */
#define SCSI_TEST_UNIT_READY       0x00
#define SCSI_REQUEST_SENSE         0x03
#define SCSI_INQUIRY               0x12
#define SCSI_START_STOP_UNIT       0x1b
#define SCSI_SYNCHRONIZE_CACHE_10  0x35
#define SCSI_READ_CAPACITY_10      0x25
#define SCSI_READ_10               0x28
#define SCSI_WRITE_10              0x2a
/*
 * How long one command may take.  Hosts allow about 30 s; stock 2.0.5 is still
 * busy after 5 s of guest time when it is ejected (MEASURED: no status by
 * then, yet it showed "OK to Disconnect").
 */
#define USB_BOT_TIMEOUT_NS         (30 * NANOSECONDS_PER_SECOND)
#define USB_CTRL_TIMEOUT_NS        (5 * NANOSECONDS_PER_SECOND)
/* How long the firmware may take to hand its disk over (TEST UNIT READY). */
#define USB_MSC_RETRY_NS           (100 * SCALE_MS)

/* The page's transfers; see USB_IO_MAX. */
static uint8_t usb_io_buf[USB_IO_MAX];

/* Steps, in order; usb_host_next() builds each SETUP. */
enum {
    STEP_GET_DEVICE = 0,
    STEP_SET_ADDRESS,
    STEP_GET_CONFIG_HEAD,
    STEP_GET_CONFIG,
    STEP_GET_LANGS,
    STEP_GET_MANUFACTURER,
    STEP_GET_PRODUCT,
    STEP_GET_SERIAL,
    STEP_SET_CONFIGURATION,
    STEP_GET_MAX_LUN,
    STEP_DONE,
    /* Not in the script: clearing a halted bulk pipe, mid-command. */
    STEP_CLEAR_HALT,
};

static const char *const usb_host_state_names[] = {
    "detached", "attaching", "reset", "setup", "data-in", "status-out",
    "status-in", "configured", "failed", "bulk-out", "bulk-in",
};

static int usb_trace_level(void)
{
    static int level = -1;

    if (level < 0) {
        const char *e = getenv("USBTRACE");

        level = e && *e ? atoi(e) : 0;
    }
    return level;
}

#define USB_TRACE(lvl, ...) do { \
        if (usb_trace_level() >= (lvl)) { \
            fprintf(stderr, "s5l8702.usb: " __VA_ARGS__); \
        } \
    } while (0)

static inline uint32_t *usb_reg(IPodClassicUSBState *s, hwaddr off)
{
    return &s->reg[off / 4];
}

static inline uint32_t *usb_ep(IPodClassicUSBState *s, bool in, int n,
                               hwaddr r)
{
    return usb_reg(s, (in ? DWC_DIEP(n) : DWC_DOEP(n)) + r);
}

/* EP0's MPS field is a code: 0 = 64, 1 = 32, 2 = 16, 3 = 8. */
static uint32_t usb_ep0_mps(IPodClassicUSBState *s)
{
    static const uint32_t mps[4] = { 64, 32, 16, 8 };

    return mps[*usb_ep(s, true, 0, DWC_EPCTL) & 3];
}

/* ---- interrupts ---- */

static uint32_t usb_daint(IPodClassicUSBState *s)
{
    uint32_t daint = 0;

    for (int n = 0; n < S5L8702_USB_MAX_EPS; n++) {
        if (*usb_ep(s, true, n, DWC_EPINT) & *usb_reg(s, DWC_DIEPMSK)) {
            daint |= 1u << n;
        }
        if (*usb_ep(s, false, n, DWC_EPINT) & *usb_reg(s, DWC_DOEPMSK)) {
            daint |= 1u << (16 + n);
        }
    }
    return daint;
}

/*
 * Start of frame.  Once the host has reset the bus it sends a SOF every
 * microframe (125 us at high speed): DSTS.SOFFN counts them and GINTSTS.Sof
 * is set again in each new microframe after the driver clears it.  Computed
 * when read, from the virtual clock; nothing is ticking.
 */
static bool usb_sof_running(IPodClassicUSBState *s)
{
    return s->sof_base_ns && s->host_state >= USB_HOST_SETUP;
}

static uint32_t usb_microframe(IPodClassicUSBState *s)
{
    if (!usb_sof_running(s)) {
        return 0;
    }
    return (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->sof_base_ns) / 125000;
}

static uint32_t usb_gintsts(IPodClassicUSBState *s)
{
    uint32_t v = s->gintsts;
    uint32_t daint = usb_daint(s) & *usb_reg(s, DWC_DAINTMSK);

    if (usb_sof_running(s) && usb_microframe(s) != s->sof_cleared) {
        v |= GINTSTS_SOF;
    }
    if (daint & 0xffff) {
        v |= GINTSTS_IEPINT;
    }
    if (daint >> 16) {
        v |= GINTSTS_OEPINT;
    }
    if (*usb_reg(s, DWC_GOTGINT)) {
        v |= GINTSTS_OTGINT;
    }
    return v;   /* CurMod 0: device */
}

static void usb_update_irq(IPodClassicUSBState *s)
{
    bool level = (*usb_reg(s, DWC_GAHBCFG) & GAHBCFG_GINT) &&
                 (usb_gintsts(s) & *usb_reg(s, DWC_GINTMSK));

    qemu_set_irq(s->irq, level);
}

/* ---- the host ---- */

static void usb_host_arm(IPodClassicUSBState *s, int64_t delay_ns)
{
    timer_mod_ns(s->host_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + delay_ns);
}

/*
 * Look again later, in host time.  Under -icount sleep=off the virtual clock
 * warps to the next virtual timer whenever the vCPU sleeps, and Rockbox's
 * kernel tick runs on host time (docs/architecture.md, "Wall-clock sources
 * are host time").  So while its USB thread slept a tick or two, a 1 ms
 * virtual poll warped through seconds of "guest time" in a few milliseconds,
 * spinning the emulator: MEASURED in Chrome, the host gave up on
 * SET_CONFIGURATION after 5 s of virtual time and 60 ms of wall time.  The
 * driver's own register writes still kick the virtual timer (usb_host_arm),
 * so a transfer the device is ready for goes at once.
 */
static void usb_host_poll(IPodClassicUSBState *s, int64_t delay_ns)
{
    timer_mod_ns(s->host_timer_rt,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL_RT) + delay_ns);
}

/*
 * A wait for the device ends after `ns` of guest time and `ns` of host time
 * both: the warp cannot end it early, and neither can a slow emulation.
 */
static void usb_host_deadline(IPodClassicUSBState *s, int64_t ns)
{
    s->host_deadline_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns;
    s->host_deadline_rt = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL_RT) + ns;
}

static bool usb_host_overdue(IPodClassicUSBState *s, int64_t now)
{
    return s->host_deadline_ns && now > s->host_deadline_ns &&
           qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL_RT) > s->host_deadline_rt;
}

static void usb_host_set_state(IPodClassicUSBState *s, int state)
{
    if (s->host_state != state) {
        bool bulk = state >= USB_HOST_BULK_OUT ||
                    s->host_state >= USB_HOST_BULK_OUT;

        USB_TRACE(bulk ? 3 : 1, "host: %s -> %s\n",
                  usb_host_state_names[s->host_state],
                  usb_host_state_names[state]);
    }
    s->host_state = state;
    /*
     * Every wait for the device has a limit: 5 s for a control transfer,
     * Linux's USB_CTRL_SET_TIMEOUT (see usb_host_deadline).
     */
    usb_host_deadline(s, USB_CTRL_TIMEOUT_NS);
}

/* The device is attached when VBUS is up and it is not soft-disconnected. */
static bool usb_pulled_up(IPodClassicUSBState *s)
{
    return s->vbus && !(*usb_reg(s, DWC_DCTL) & DCTL_SFTDISCON) &&
           (*usb_reg(s, DWC_GAHBCFG) & GAHBCFG_DMAEN);
}

static void usb_host_detach(IPodClassicUSBState *s)
{
    if (s->host_state != USB_HOST_DETACHED) {
        usb_host_set_state(s, USB_HOST_DETACHED);
    }
    s->host_deadline_ns = 0;
    s->host_step = 0;
    s->address = 0;
    s->configuration = 0;
    s->sof_base_ns = 0;
    s->msc_iface = -1;
    s->msc_ready = false;
    s->msc_blocks = 0;
    s->msc_block_size = 0;
    s->bot_phase = USB_BOT_IDLE;
    if (qatomic_read(&s->io_status) == USB_IO_BUSY) {
        qatomic_set(&s->io_status, USB_IO_FAILED);
    }
    timer_del(s->host_timer);
    timer_del(s->host_timer_rt);
}

/* Call after anything that may attach or detach the device. */
static void usb_host_check_attach(IPodClassicUSBState *s)
{
    if (usb_pulled_up(s)) {
        if (s->host_state == USB_HOST_DETACHED) {
            usb_host_set_state(s, USB_HOST_ATTACHING);
            /* USB 2.0 7.1.7.3: at least 100 ms of debounce before reset. */
            usb_host_arm(s, 100 * SCALE_MS);
        }
    } else if (s->host_state != USB_HOST_DETACHED) {
        usb_host_detach(s);
    }
}

/* The next SETUP of the script, or false when it is done. */
static bool usb_host_next(IPodClassicUSBState *s)
{
    uint8_t *r = s->setup;
    uint8_t req_type = 0x80, req = USB_REQ_GET_DESCRIPTOR;
    uint16_t value = 0, index = 0, length = 0;

    for (;;) {
        switch (s->host_step) {
        case STEP_GET_DEVICE:
            value = USB_DT_DEVICE << 8;
            length = 64;     /* what Windows asks first */
            break;
        case STEP_SET_ADDRESS:
            req_type = 0;
            req = USB_REQ_SET_ADDRESS;
            value = 1;
            break;
        case STEP_GET_CONFIG_HEAD:
            value = USB_DT_CONFIG << 8;
            length = 9;
            break;
        case STEP_GET_CONFIG:
            value = USB_DT_CONFIG << 8;
            length = MIN(s->config_len, sizeof(s->config));
            break;
        case STEP_GET_LANGS:
            value = USB_DT_STRING << 8;
            length = 255;
            break;
        case STEP_GET_MANUFACTURER:
        case STEP_GET_PRODUCT:
        case STEP_GET_SERIAL: {
            uint8_t idx = s->dev_desc[14 + s->host_step - STEP_GET_MANUFACTURER];

            if (!idx) {
                s->host_step++;
                continue;
            }
            value = (USB_DT_STRING << 8) | idx;
            index = 0x0409;
            length = 255;
            break;
        }
        case STEP_SET_CONFIGURATION:
            req_type = 0;
            req = USB_REQ_SET_CONFIGURATION;
            value = s->config_len >= 9 ? s->config[5] : 1;
            break;
        case STEP_CLEAR_HALT:
            /* CLEAR_FEATURE(ENDPOINT_HALT) to the endpoint. */
            req_type = 0x02;
            req = USB_REQ_CLEAR_FEATURE;
            index = s->bot_halted;
            break;
        case STEP_GET_MAX_LUN:
            if (s->msc_iface < 0) {
                s->host_step++;
                continue;
            }
            /* Class, interface, IN: a single LUN may stall it. */
            req_type = 0xa1;
            req = USB_REQ_GET_MAX_LUN;
            index = s->msc_iface;
            length = 1;
            break;
        default:
            return false;
        }
        break;
    }
    r[0] = req_type;
    r[1] = req;
    r[2] = value;
    r[3] = value >> 8;
    r[4] = index;
    r[5] = index >> 8;
    r[6] = length;
    r[7] = length >> 8;
    s->want = length;
    s->got = 0;
    return true;
}

static void usb_host_log_string(IPodClassicUSBState *s, const char *what)
{
    char text[128];
    size_t n = 0;

    for (uint32_t i = 2; i + 1 < s->got && n + 1 < sizeof(text); i += 2) {
        uint16_t c = s->buf[i] | (s->buf[i + 1] << 8);

        text[n++] = c >= 0x20 && c < 0x7f ? c : '?';
    }
    text[n] = 0;
    USB_TRACE(1, "host: %s \"%s\"\n", what, text);
}

/* A control read finished: keep what the script needs. */
static void usb_host_data_done(IPodClassicUSBState *s)
{
    switch (s->host_step) {
    case STEP_GET_DEVICE:
        memcpy(s->dev_desc, s->buf, MIN(s->got, sizeof(s->dev_desc)));
        if (s->got >= 12) {
            s->vid_pid = (lduw_le_p(s->dev_desc + 8) << 16) |
                         lduw_le_p(s->dev_desc + 10);
        }
        USB_TRACE(1, "host: device descriptor, %u bytes: USB %x.%02x class "
                  "%u ep0 %u, %04x:%04x rel %x.%02x, %u configuration(s)\n",
                  s->got, s->dev_desc[3], s->dev_desc[2], s->dev_desc[4],
                  s->dev_desc[7], s->vid_pid >> 16, s->vid_pid & 0xffff,
                  s->dev_desc[13], s->dev_desc[12], s->dev_desc[17]);
        break;
    case STEP_GET_CONFIG_HEAD:
        s->config_len = s->got >= 4 ? lduw_le_p(s->buf + 2) : 9;
        break;
    case STEP_GET_CONFIG: {
        memcpy(s->config, s->buf, MIN(s->got, sizeof(s->config)));
        USB_TRACE(1, "host: configuration %u, %u bytes, %u interface(s)\n",
                  s->config[5], s->got, s->config[4]);
        /*
         * Every interface and endpoint, for the log; and the mass-storage
         * interface (class 8, SCSI transparent, Bulk-Only) with its two bulk
         * endpoints, which the host talks to once configured.
         */
        {
            bool in_msc = false;

            s->msc_iface = -1;
            s->msc_in = s->msc_out = 0;
            for (uint32_t at = 0; at + 2 <= s->got && s->config[at]; ) {
                const uint8_t *d = s->config + at;

                if (d[1] == 4 && d[0] >= 9) {
                    USB_TRACE(1, "host:   interface %u alt %u class "
                              "%02x/%02x/%02x, %u endpoint(s)\n", d[2], d[3],
                              d[5], d[6], d[7], d[4]);
                    in_msc = d[5] == 8 && d[6] == 6 && d[7] == 0x50 &&
                             d[3] == 0 && s->msc_iface < 0;
                    if (in_msc) {
                        s->msc_iface = d[2];
                    }
                } else if (d[1] == 5 && d[0] >= 7) {
                    USB_TRACE(1, "host:     endpoint 0x%02x type %u mps %u\n",
                              d[2], d[3] & 3, lduw_le_p(d + 4));
                    if (in_msc && (d[3] & 3) == 2) {
                        if (d[2] & 0x80) {
                            s->msc_in = d[2] & 0x0f;
                        } else {
                            s->msc_out = d[2] & 0x0f;
                        }
                    }
                }
                at += d[0];
            }
            if (s->msc_iface >= 0 && (!s->msc_in || !s->msc_out)) {
                s->msc_iface = -1;
            }
        }
        break;
    }
    case STEP_GET_MAX_LUN:
        USB_TRACE(1, "host: max LUN %d\n", s->stalled ? 0 : s->buf[0]);
        break;
    case STEP_GET_MANUFACTURER:
        usb_host_log_string(s, "manufacturer");
        break;
    case STEP_GET_PRODUCT:
        usb_host_log_string(s, "product");
        break;
    case STEP_GET_SERIAL:
        usb_host_log_string(s, "serial");
        break;
    }
}

/* Deliver the SETUP into OUT0's DMA buffer, if the device has armed it. */
static bool usb_host_try_setup(IPodClassicUSBState *s)
{
    uint32_t *ctl = usb_ep(s, false, 0, DWC_EPCTL);
    uint32_t *tsiz = usb_ep(s, false, 0, DWC_EPTSIZ);
    uint32_t *dma = usb_ep(s, false, 0, DWC_EPDMA);
    uint32_t sup;

    if (!(*ctl & EPCTL_EPENA)) {
        return false;
    }
    /* A SETUP clears a stall on EP0, as the core does. */
    *ctl &= ~EPCTL_STALL;
    *usb_ep(s, true, 0, DWC_EPCTL) &= ~EPCTL_STALL;
    s->stalled = false;
    s->setup_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    address_space_write(&address_space_memory, *dma, MEMTXATTRS_UNSPECIFIED,
                        s->setup, sizeof(s->setup));
    USB_TRACE(1, "host: SETUP %02x %02x %04x %04x %04x -> 0x%08x\n",
              s->setup[0], s->setup[1], lduw_le_p(s->setup + 2),
              lduw_le_p(s->setup + 4), lduw_le_p(s->setup + 6), *dma);
    *dma += sizeof(s->setup);
    sup = EPTSIZ_SUPCNT(*tsiz);
    if (sup) {
        *tsiz = deposit32(*tsiz, 29, 2, sup - 1);
    }
    /*
     * A SETUP takes 8 bytes off XferSize, as data would.  2.0.5's interrupt
     * handler (0x08076938) takes 64 - XferSize as the bytes received and only
     * accepts a SETUP when that is a non-zero multiple of 8.
     */
    if (EPTSIZ_XFERSIZE(*tsiz) >= sizeof(s->setup)) {
        *tsiz -= sizeof(s->setup);
    }
    /* The SETUP phase ends the transfer: OUT0 is disabled until re-armed. */
    *ctl &= ~EPCTL_EPENA;
    *usb_ep(s, false, 0, DWC_EPINT) |= EPINT_SETUP;
    usb_update_irq(s);
    return true;
}

/*
 * IN0 has been armed with data: take what the control read still wants.  A
 * short packet, or all of wLength, ends the data stage.
 */
static bool usb_host_try_in(IPodClassicUSBState *s, bool *done)
{
    uint32_t *ctl = usb_ep(s, true, 0, DWC_EPCTL);
    uint32_t *tsiz = usb_ep(s, true, 0, DWC_EPTSIZ);
    uint32_t *dma = usb_ep(s, true, 0, DWC_EPDMA);
    uint32_t mps = usb_ep0_mps(s);
    uint32_t size, n, room;

    *done = false;
    if (*ctl & EPCTL_STALL) {
        /* The device refused the request: no data, no status stage. */
        s->stalled = true;
        *done = true;
        return true;
    }
    if (!(*ctl & EPCTL_EPENA) || (*ctl & EPCTL_NAKSTS)) {
        return false;
    }
    size = *tsiz & 0x7f;                 /* EP0: 7 bits */
    room = s->want > s->got ? s->want - s->got : 0;
    n = MIN(size, room);
    n = MIN(n, (uint32_t)sizeof(s->buf) - s->got);
    if (n) {
        address_space_read(&address_space_memory, *dma,
                           MEMTXATTRS_UNSPECIFIED, s->buf + s->got, n);
    }
    s->got += n;
    *dma += n;
    /* Residual size and packets, as the drivers read them back. */
    *tsiz = (*tsiz & ~0x7fu & ~(3u << 19)) | (size - n);
    if (size - n) {
        *tsiz |= (DIV_ROUND_UP(size - n, mps) & 3) << 19;
    }
    if (size == n) {
        *ctl &= ~EPCTL_EPENA;
        *usb_ep(s, true, 0, DWC_EPINT) |= EPINT_XFERCOMPL;
    }
    /* A short (or empty) last packet, or wLength reached, ends the stage. */
    *done = s->got >= s->want || n == 0 || (n % mps) != 0;
    usb_update_irq(s);
    return true;
}

/* The zero-length packet that ends a control transfer. */
static bool usb_host_try_zlp(IPodClassicUSBState *s, bool in)
{
    uint32_t *ctl = usb_ep(s, in, 0, DWC_EPCTL);
    uint32_t *tsiz = usb_ep(s, in, 0, DWC_EPTSIZ);

    if (!(*ctl & EPCTL_EPENA) || (in && (*ctl & EPCTL_NAKSTS))) {
        return false;
    }
    if (in) {
        if (*tsiz & 0x7f) {
            /* Data where the host expects none: take it, ignore it. */
            USB_TRACE(1, "host: %u unexpected byte(s) in a status stage\n",
                      *tsiz & 0x7f);
        }
        *tsiz &= ~(0x7fu | (3u << 19));
    } else {
        /* Nothing received: XferSize stays as programmed, one packet used. */
        uint32_t pk = EPTSIZ_PKTCNT(*tsiz);

        *tsiz = deposit32(*tsiz, 19, 2, pk ? pk - 1 : 0);
    }
    *ctl &= ~EPCTL_EPENA;
    *usb_ep(s, in, 0, DWC_EPINT) |= EPINT_XFERCOMPL;
    usb_update_irq(s);
    return true;
}

/* ---- bulk transfers ---- */

/*
 * Host to device on bulk OUT endpoint `n`: up to `len` bytes from `src`.
 * Returns the bytes the device took, -1 while it is not armed, -2 when it
 * stalled the pipe.  The
 * transfer ends, as on the wire, when the device's XferSize runs out or a
 * packet is short.
 */
static int usb_host_bulk_out(IPodClassicUSBState *s, int n,
                             const uint8_t *src, uint32_t len)
{
    uint32_t *ctl = usb_ep(s, false, n, DWC_EPCTL);
    uint32_t *tsiz = usb_ep(s, false, n, DWC_EPTSIZ);
    uint32_t *dma = usb_ep(s, false, n, DWC_EPDMA);
    uint32_t mps = (*ctl & EPCTL_MPS_MASK) ? (*ctl & EPCTL_MPS_MASK) : 512;
    uint32_t size, take, pk, used;

    if (*ctl & EPCTL_STALL) {
        return -2;
    }
    if (!(*ctl & EPCTL_EPENA) || (*ctl & EPCTL_NAKSTS)) {
        return -1;
    }
    size = EPTSIZ_XFERSIZE(*tsiz);
    take = MIN(size, len);
    if (take) {
        address_space_write(&address_space_memory, *dma,
                            MEMTXATTRS_UNSPECIFIED, src, take);
    }
    *dma += take;
    pk = EPTSIZ_PKTCNT(*tsiz);
    used = take ? DIV_ROUND_UP(take, mps) : 1;
    pk = pk > used ? pk - used : 0;
    *tsiz = (*tsiz & ~(0x7ffffu | (0x3ffu << 19))) | (size - take) |
            (pk << 19);
    if (size == take || (take % mps) != 0 || take == 0) {
        *ctl &= ~EPCTL_EPENA;
        *usb_ep(s, false, n, DWC_EPINT) |= EPINT_XFERCOMPL;
        usb_update_irq(s);
    }
    return take;
}

/*
 * Device to host on bulk IN endpoint `n`: up to `room` bytes into `dst`.
 * Returns the bytes taken (0 for a zero-length packet), -1 while the device
 * has nothing armed, -2 when it stalled the pipe.  *end says the device's
 * transfer ended on a short packet, which ends a data phase early.
 */
static int usb_host_bulk_in(IPodClassicUSBState *s, int n, uint8_t *dst,
                            uint32_t room, bool *end)
{
    uint32_t *ctl = usb_ep(s, true, n, DWC_EPCTL);
    uint32_t *tsiz = usb_ep(s, true, n, DWC_EPTSIZ);
    uint32_t *dma = usb_ep(s, true, n, DWC_EPDMA);
    uint32_t mps = (*ctl & EPCTL_MPS_MASK) ? (*ctl & EPCTL_MPS_MASK) : 512;
    uint32_t size, take, pk, used;

    *end = false;
    if (*ctl & EPCTL_STALL) {
        return -2;
    }
    if (!(*ctl & EPCTL_EPENA) || (*ctl & EPCTL_NAKSTS)) {
        return -1;
    }
    size = EPTSIZ_XFERSIZE(*tsiz);
    take = MIN(size, room);
    if (take < size) {
        take -= take % mps;   /* whole packets, unless it is the last */
    }
    if (take) {
        address_space_read(&address_space_memory, *dma,
                           MEMTXATTRS_UNSPECIFIED, dst, take);
    }
    *dma += take;
    pk = EPTSIZ_PKTCNT(*tsiz);
    used = take ? DIV_ROUND_UP(take, mps) : 1;
    pk = pk > used ? pk - used : 0;
    *tsiz = (*tsiz & ~(0x7ffffu | (0x3ffu << 19))) | (size - take) |
            (pk << 19);
    if (size == take) {
        *ctl &= ~EPCTL_EPENA;
        *usb_ep(s, true, n, DWC_EPINT) |= EPINT_XFERCOMPL;
        usb_update_irq(s);
        *end = (size % mps) != 0 || size == 0;
    }
    return take;
}

/* ---- Bulk-Only Transport ---- */

static uint8_t *usb_bot_data(IPodClassicUSBState *s)
{
    return s->bot_page ? usb_io_buf : s->bot_buf;
}

/* Send one command block; its data and status follow from the tick. */
static void usb_bot_start(IPodClassicUSBState *s, const uint8_t *cdb,
                          int cdb_len, uint32_t len, bool in, bool page)
{
    uint8_t *w = s->bot_cbw;

    memset(w, 0, sizeof(s->bot_cbw));
    stl_le_p(w, USB_BOT_CBW_SIG);
    stl_le_p(w + 4, ++s->bot_tag);
    stl_le_p(w + 8, len);
    w[12] = in ? 0x80 : 0;
    w[13] = 0;                       /* LUN 0 */
    w[14] = cdb_len;
    memcpy(w + 15, cdb, cdb_len);
    s->bot_len = len;
    s->bot_done = 0;
    s->bot_retried = false;
    s->bot_in = in;
    s->bot_page = page;
    s->bot_phase = USB_BOT_CBW;
    USB_TRACE(page || cdb[0] == SCSI_TEST_UNIT_READY ||
              cdb[0] == SCSI_REQUEST_SENSE ? 2 : 1,
              "host: SCSI %02x, %u byte(s) %s\n", cdb[0], len,
              len ? (in ? "in" : "out") : "");
    usb_host_set_state(s, USB_HOST_BULK_OUT);
    usb_host_deadline(s, USB_BOT_TIMEOUT_NS);
    usb_host_arm(s, 50 * SCALE_US);
}

/* The page's request, if there is one and the device is free for it. */
static bool usb_msc_page_start(IPodClassicUSBState *s)
{
    uint8_t cdb[10] = { 0 };
    uint32_t len = 0;
    bool in = false;

    if (!s->msc_ready || s->bot_phase != USB_BOT_IDLE ||
        qatomic_read(&s->io_status) != USB_IO_BUSY || !s->io_op) {
        return false;
    }
    switch (s->io_op) {
    case USB_IO_EJECT + 0x10:
        /* The eject after a refused flush and its auto-sense. */
        cdb[0] = SCSI_START_STOP_UNIT;
        cdb[4] = 0x02;
        break;
    case USB_IO_READ:
    case USB_IO_WRITE:
        cdb[0] = s->io_op == USB_IO_READ ? SCSI_READ_10 : SCSI_WRITE_10;
        stl_be_p(cdb + 2, s->io_lba);
        stw_be_p(cdb + 7, s->io_count);
        len = s->io_count * s->msc_block_size;
        in = s->io_op == USB_IO_READ;
        break;
    case USB_IO_EJECT:
    case USB_IO_FLUSH:
        /*
         * What a computer does before it lets go: write back the device's
         * cache (stock keeps writes in RAM: MEASURED, a written block read
         * back over USB was not yet on its disk), then eject.
         */
        cdb[0] = SCSI_SYNCHRONIZE_CACHE_10;   /* LBA 0, 0 blocks: all */
        s->io_then_eject = s->io_op == USB_IO_EJECT;
        break;
    }
    s->io_op = 0;
    usb_bot_start(s, cdb, cdb[0] == SCSI_START_STOP_UNIT ? 6 : 10, len, in,
                  true);
    return true;
}

/* The init sequence's next command; false when it is over. */
static bool usb_msc_init_next(IPodClassicUSBState *s)
{
    static const uint8_t inquiry[6] = { SCSI_INQUIRY, 0, 0, 0, 36, 0 };
    static const uint8_t tur[6] = { SCSI_TEST_UNIT_READY, 0, 0, 0, 0, 0 };
    static const uint8_t sense[6] = { SCSI_REQUEST_SENSE, 0, 0, 0, 18, 0 };
    static const uint8_t capacity[10] = { SCSI_READ_CAPACITY_10 };

    switch (s->msc_init) {
    case 0:
        usb_bot_start(s, inquiry, 6, 36, true, false);
        return true;
    case 1:
        usb_bot_start(s, tur, 6, 0, false, false);
        return true;
    case 2:
        usb_bot_start(s, sense, 6, 18, true, false);
        return true;
    case 3:
        usb_bot_start(s, capacity, 10, 8, true, false);
        return true;
    default:
        return false;
    }
}

/* A command's status is in: the init sequence or the page moves on. */
static void usb_bot_finish(IPodClassicUSBState *s, bool ok)
{
    uint8_t *d = s->bot_buf;
    uint8_t op = s->bot_cbw[15];

    s->bot_phase = USB_BOT_IDLE;
    usb_host_set_state(s, USB_HOST_CONFIGURED);
    s->host_deadline_ns = 0;
    if (s->bot_page) {
        USB_TRACE(2, "host: SCSI %02x %s\n", op, ok ? "done" : "failed");
        if (op == SCSI_SYNCHRONIZE_CACHE_10 && !ok) {
            /*
             * Rockbox has no SYNCHRONIZE CACHE (it writes through): a
             * refusal means nothing is held back.  The sense is logged.
             */
            s->msc_autosense = true;
            s->msc_failed_op = op;
            ok = true;
        }
        if (op == SCSI_SYNCHRONIZE_CACHE_10 && s->io_then_eject) {
            static const uint8_t eject[6] = {
                SCSI_START_STOP_UNIT, 0, 0, 0, 0x02, 0,   /* LoEj, not Start */
            };

            s->io_then_eject = false;
            if (s->msc_autosense) {
                /* The sense first; the eject follows from the tick. */
                s->io_op = USB_IO_EJECT + 0x10;
                usb_host_arm(s, 50 * SCALE_US);
            } else {
                usb_bot_start(s, eject, 6, 0, false, true);
            }
            return;
        }
        if (op == SCSI_START_STOP_UNIT && ok) {
            /* The medium is gone, as far as this host is concerned. */
            USB_TRACE(1, "host: ejected\n");
            s->msc_ready = false;
        }
        /* Ask why it failed, as hosts do, before the next request. */
        if (!ok) {
            s->msc_autosense = op != SCSI_REQUEST_SENSE;
            s->msc_failed_op = op;
        }
        qatomic_set(&s->io_status, ok ? USB_IO_DONE : USB_IO_FAILED);
        /* A request that came in meanwhile is taken from the tick. */
        usb_host_arm(s, 50 * SCALE_US);
        return;
    }
    switch (op) {
    case SCSI_INQUIRY:
        if (ok) {
            USB_TRACE(1, "host: INQUIRY \"%.8s\" \"%.16s\" \"%.4s\", "
                      "type %u%s\n", d + 8, d + 16, d + 32, d[0] & 0x1f,
                      d[1] & 0x80 ? ", removable" : "");
        }
        s->msc_init = 1;
        break;
    case SCSI_TEST_UNIT_READY:
        if (ok) {
            s->msc_init = 3;
        } else {
            /* As a real host does: ask why, and ask again, while plugged. */
            s->msc_tries++;
            s->msc_init = 2;
        }
        break;
    case SCSI_REQUEST_SENSE: {
        uint32_t sense = ((d[2] & 0x0f) << 16) | (d[12] << 8) | d[13];

        if (s->msc_ready) {
            /* The auto-sense after a page command failed. */
            USB_TRACE(1, "host: SCSI %02x failed: sense %x/%02x/%02x\n",
                      s->msc_failed_op, sense >> 16, (sense >> 8) & 0xff,
                      sense & 0xff);
            break;
        }

        /* Each new reason, and every 10 s of guest time the same one. */
        if (sense != s->msc_sense || s->msc_tries % 100 == 0) {
            USB_TRACE(1, "host: not ready (sense %x/%02x/%02x), try %d, "
                      "%.1f s\n", sense >> 16, (sense >> 8) & 0xff,
                      sense & 0xff, s->msc_tries,
                      (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                       s->sof_base_ns) / 1e9);
        }
        s->msc_sense = sense;
        s->msc_init = 1;
        /*
         * Hosts poll TEST UNIT READY every so often in their own time: let
         * the firmware get on with it.  The driver re-arming its OUT endpoint
         * kicks the virtual timer at once (see usb_write_epctl), so the pause
         * is kept as a time, not only a timer, and in host time for the same
         * reason as usb_host_poll.
         */
        s->msc_retry_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL_RT) +
                          USB_MSC_RETRY_NS;
        usb_host_poll(s, USB_MSC_RETRY_NS);
        return;
    }
    case SCSI_READ_CAPACITY_10:
        s->msc_init = 99;
        if (ok) {
            s->msc_blocks = ldl_be_p(d) + 1;
            s->msc_block_size = ldl_be_p(d + 4);
            s->msc_ready = s->msc_block_size > 0;
            USB_TRACE(1, "host: mass storage ready: %u blocks of %u bytes "
                      "(%u MB), after %d TEST UNIT READY, %.1f s\n",
                      s->msc_blocks, s->msc_block_size,
                      (uint32_t)((uint64_t)s->msc_blocks *
                                 s->msc_block_size >> 20),
                      s->msc_tries + 1,
                      (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                       s->sof_base_ns) / 1e9);
        }
        break;
    default:
        break;
    }
    usb_host_arm(s, 50 * SCALE_US);
}

/*
 * The device halted a bulk pipe, as it does for a command it rejects with
 * data to move: clear the halt with CLEAR_FEATURE on EP0, then read the
 * status (BOT 6.7.2 and 6.7.3); usb_host_tick() comes back to USB_BOT_CSW.
 */
static void usb_bot_clear_halt(IPodClassicUSBState *s, bool in)
{
    s->bot_halted = in ? (0x80 | s->msc_in) : s->msc_out;
    USB_TRACE(1, "host: endpoint 0x%02x halted in SCSI %02x, clearing it\n",
              s->bot_halted, s->bot_cbw[15]);
    s->host_step = STEP_CLEAR_HALT;
    usb_host_next(s);
    usb_host_set_state(s, USB_HOST_SETUP);
}

/* One step of the command in flight; true when something moved. */
static bool usb_bot_tick(IPodClassicUSBState *s)
{
    uint8_t *data = usb_bot_data(s);
    bool end;
    int n;

    switch (s->bot_phase) {
    case USB_BOT_CBW:
        n = usb_host_bulk_out(s, s->msc_out, s->bot_cbw, sizeof(s->bot_cbw));
        if (n == -2) {
            /* It refuses commands: only a reset recovery would help. */
            USB_TRACE(1, "host: the device stalled a command block\n");
            usb_bot_finish(s, false);
            return true;
        }
        if (n < 0) {
            return false;
        }
        if (!s->bot_len) {
            s->bot_phase = USB_BOT_CSW;
        } else {
            s->bot_phase = s->bot_in ? USB_BOT_DATA_IN : USB_BOT_DATA_OUT;
        }
        usb_host_set_state(s, s->bot_phase == USB_BOT_DATA_OUT
                              ? USB_HOST_BULK_OUT : USB_HOST_BULK_IN);
        return true;
    case USB_BOT_DATA_OUT:
        n = usb_host_bulk_out(s, s->msc_out, data + s->bot_done,
                              s->bot_len - s->bot_done);
        if (n == -2) {
            usb_bot_clear_halt(s, false);
            return true;
        }
        if (n < 0) {
            return false;
        }
        s->bot_done += n;
        if (s->bot_done >= s->bot_len) {
            s->bot_phase = USB_BOT_CSW;
            usb_host_set_state(s, USB_HOST_BULK_IN);
        }
        return true;
    case USB_BOT_DATA_IN:
        n = usb_host_bulk_in(s, s->msc_in, data + s->bot_done,
                             s->bot_len - s->bot_done, &end);
        if (n == -2) {
            usb_bot_clear_halt(s, true);
            return true;
        }
        if (n < 0) {
            return false;
        }
        s->bot_done += n;
        if (s->bot_done >= s->bot_len || end || n == 0) {
            s->bot_phase = USB_BOT_CSW;
        }
        return true;
    case USB_BOT_CSW:
        n = usb_host_bulk_in(s, s->msc_in, s->bot_csw, sizeof(s->bot_csw),
                             &end);
        if (n == -2 && !s->bot_retried) {
            /* A stalled status is cleared and read once more (BOT 5.3.3). */
            s->bot_retried = true;
            usb_bot_clear_halt(s, true);
            return true;
        }
        if (n == -2) {
            usb_bot_finish(s, false);
            return true;
        }
        if (n < 0) {
            return false;
        }
        if (n != sizeof(s->bot_csw) ||
            ldl_le_p(s->bot_csw) != USB_BOT_CSW_SIG ||
            ldl_le_p(s->bot_csw + 4) != s->bot_tag) {
            USB_TRACE(1, "host: bad status wrapper (%d bytes) for SCSI %02x\n",
                      n, s->bot_cbw[15]);
            usb_bot_finish(s, false);
        } else {
            /* A residue means less moved than asked: not a whole request. */
            uint32_t residue = ldl_le_p(s->bot_csw + 8);

            if (residue && s->bot_page) {
                USB_TRACE(1, "host: SCSI %02x left %u byte(s) unmoved\n",
                          s->bot_cbw[15], residue);
            }
            usb_bot_finish(s, s->bot_csw[12] == 0 &&
                              !(residue && s->bot_page));
        }
        return true;
    default:
        return false;
    }
}

/* Begin the next request, or settle once the script is done. */
static void usb_host_begin(IPodClassicUSBState *s)
{
    if (!usb_host_next(s)) {
        usb_host_set_state(s, USB_HOST_CONFIGURED);
        s->host_deadline_ns = 0;
        USB_TRACE(1, "host: enumeration done, configuration %u\n",
                  s->configuration);
        if (s->msc_iface >= 0) {
            /* Mass storage: what any host asks before it mounts a disk. */
            s->msc_init = 0;
            s->msc_tries = 0;
            s->msc_sense = 0;
            s->msc_retry_ns = 0;
            usb_host_arm(s, SCALE_MS);
        }
        return;
    }
    usb_host_set_state(s, USB_HOST_SETUP);
    usb_host_arm(s, 0);
}

static void usb_host_tick(void *opaque)
{
    IPodClassicUSBState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    bool progress = false, done;

    switch (s->host_state) {
    case USB_HOST_ATTACHING:
        /* Bus reset: SE0 for at least 10 ms, then the speed handshake. */
        USB_TRACE(1, "host: device pulled up, resetting the bus\n");
        s->gintsts |= GINTSTS_USBRST;
        s->address = 0;
        s->configuration = 0;
        usb_update_irq(s);
        usb_host_set_state(s, USB_HOST_RESET);
        usb_host_arm(s, 20 * SCALE_MS);
        return;
    case USB_HOST_RESET:
        /* Enumerated at high speed: DSTS.EnumSpd reads 0. */
        s->gintsts |= GINTSTS_ENUMDNE;
        /* SOFs from here on; the reset is over. */
        s->sof_base_ns = now ? now : 1;
        s->sof_cleared = UINT32_MAX;
        usb_update_irq(s);
        s->host_step = STEP_GET_DEVICE;
        usb_host_begin(s);
        /* Give the driver its ENUMDNE before the first SETUP. */
        usb_host_arm(s, 2 * SCALE_MS);
        return;
    case USB_HOST_SETUP:
        if (usb_host_try_setup(s)) {
            progress = true;
            usb_host_set_state(s, s->want ? USB_HOST_DATA_IN
                                          : USB_HOST_STATUS_IN);
        }
        break;
    case USB_HOST_DATA_IN:
        if (usb_host_try_in(s, &done)) {
            progress = true;
            usb_host_deadline(s, USB_CTRL_TIMEOUT_NS);
            if (done && s->stalled) {
                /* Refused: no status stage; the script goes on. */
                usb_host_data_done(s);
                s->host_step++;
                usb_host_begin(s);
            } else if (done) {
                usb_host_data_done(s);
                usb_host_set_state(s, USB_HOST_STATUS_OUT);
            }
        }
        break;
    case USB_HOST_STATUS_OUT:
        if (usb_host_try_zlp(s, false)) {
            progress = true;
            s->host_step++;
            usb_host_begin(s);
        }
        break;
    case USB_HOST_STATUS_IN:
        if (usb_host_try_zlp(s, true)) {
            progress = true;
            if (now - s->setup_ns > 100 * SCALE_MS) {
                USB_TRACE(1, "host: request %02x took %" PRId64 " ms of guest "
                          "time\n", s->setup[1],
                          (now - s->setup_ns) / SCALE_MS);
            }
            if (s->setup[1] == USB_REQ_SET_ADDRESS) {
                s->address = s->setup[2];
                USB_TRACE(1, "host: address %u (DCFG.DevAddr %u)\n",
                          s->address,
                          (*usb_reg(s, DWC_DCFG) >> 4) & 0x7f);
            } else if (s->setup[1] == USB_REQ_SET_CONFIGURATION) {
                s->configuration = s->setup[2];
            }
            if (s->host_step == STEP_CLEAR_HALT) {
                /* The pipe is clear: on to the command's status. */
                s->bot_phase = USB_BOT_CSW;
                usb_host_set_state(s, USB_HOST_BULK_IN);
                break;
            }
            s->host_step++;
            usb_host_begin(s);
        }
        break;
    case USB_HOST_CONFIGURED:
        if (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL_RT) < s->msc_retry_ns) {
            /*
             * MEASURED with stock 2.0.5: without this, its re-armed OUT
             * endpoint brought the next TEST UNIT READY in a millisecond, 300
             * of them in 0.1 s of guest time, and it never became ready.
             */
            timer_mod_ns(s->host_timer_rt, s->msc_retry_ns);
            return;
        }
        /* Between commands: the init sequence, then the page's requests. */
        if (s->msc_iface >= 0 && !s->msc_ready && s->msc_init < 99) {
            usb_msc_init_next(s);
        } else if (s->msc_autosense) {
            static const uint8_t sense[6] = { SCSI_REQUEST_SENSE, 0, 0, 0,
                                              18, 0 };

            s->msc_autosense = false;
            usb_bot_start(s, sense, 6, 18, true, false);
        } else {
            usb_msc_page_start(s);
        }
        return;
    case USB_HOST_BULK_OUT:
    case USB_HOST_BULK_IN:
        if (usb_bot_tick(s)) {
            progress = true;
            usb_host_deadline(s, USB_BOT_TIMEOUT_NS);
        }
        break;
    default:
        return;   /* detached or failed: nothing to do */
    }

    if (s->host_state == USB_HOST_CONFIGURED) {
        return;
    }
    if (progress) {
        /* The next packet goes in the next microframe or so. */
        usb_host_arm(s, 50 * SCALE_US);
        return;
    }
    if (usb_host_overdue(s, now)) {
        bool bulk = s->host_state >= USB_HOST_BULK_OUT;
        int in = bulk ? s->msc_in : 0, out = bulk ? s->msc_out : 0;

        USB_TRACE(1, "host: the device stopped answering in %s (step %d, "
                  "SCSI %02x phase %d); DOEPCTL%d 0x%08x DIEPCTL%d 0x%08x "
                  "DIEPTSIZ%d 0x%08x\n",
                  usb_host_state_names[s->host_state], s->host_step,
                  bulk ? s->bot_cbw[15] : 0, s->bot_phase,
                  out, *usb_ep(s, false, out, DWC_EPCTL),
                  in, *usb_ep(s, true, in, DWC_EPCTL),
                  in, *usb_ep(s, true, in, DWC_EPTSIZ));
        usb_host_set_state(s, USB_HOST_FAILED);
        s->host_deadline_ns = 0;
        s->msc_ready = false;
        if (qatomic_read(&s->io_status) == USB_IO_BUSY) {
            qatomic_set(&s->io_status, USB_IO_FAILED);
        }
        return;
    }
    /* Not ready yet: look again in a millisecond (a USB frame) of host time. */
    usb_host_poll(s, SCALE_MS);
}

/* ---- the cable ---- */

void ipod_classic_usb_set_vbus(IPodClassicUSBState *s, bool on)
{
    if (s->vbus == on) {
        return;
    }
    s->vbus = on;
    USB_TRACE(1, "cable %s\n", on ? "plugged in" : "pulled out");
    if (on) {
        s->gintsts |= GINTSTS_SESSREQINT;
    } else {
        *usb_reg(s, DWC_GOTGINT) |= GOTGINT_SESENDDET;
        s->gintsts |= GINTSTS_DISCONNINT;
    }
    usb_host_check_attach(s);
    usb_update_irq(s);
}

/* ---- registers ---- */

static void usb_core_reset(IPodClassicUSBState *s)
{
    /* CSRST: interrupts, transfers and the bus state go; configuration stays. */
    s->gintsts = 0;
    *usb_reg(s, DWC_GOTGINT) = 0;
    for (int n = 0; n < S5L8702_USB_MAX_EPS; n++) {
        *usb_ep(s, true, n, DWC_EPCTL) &= ~EPCTL_EPENA;
        *usb_ep(s, false, n, DWC_EPCTL) &= ~EPCTL_EPENA;
        *usb_ep(s, true, n, DWC_EPINT) = 0;
        *usb_ep(s, false, n, DWC_EPINT) = 0;
    }
    *usb_reg(s, DWC_DCTL) &= ~(DCTL_GINNAKSTS | DCTL_GOUTNAKSTS);
}

/* For the register trace: the head of the running translation block. */
static uint32_t usb_trace_pc(void)
{
    CPUState *cs = current_cpu;

    return cs ? (uint32_t)CPU_GET_CLASS(cs)->get_pc(cs) : 0;
}

static uint64_t usb_read(void *opaque, hwaddr off, unsigned size)
{
    IPodClassicUSBState *s = opaque;
    uint32_t v;

    if (off >= S5L8702_USB_REGS_SIZE) {
        qemu_log_mask(LOG_UNIMP, "s5l8702.usb: read of FIFO window +0x%"
                      HWADDR_PRIx " (slave mode is not modelled)\n", off);
        return 0;
    }
    switch (off & ~3) {
    case DWC_GOTGCTL:
        v = (*usb_reg(s, DWC_GOTGCTL) & 0xffff) | GOTGCTL_CONIDSTS |
            (s->vbus ? GOTGCTL_BSESVLD : 0);
        break;
    case DWC_GRSTCTL:
        v = (*usb_reg(s, DWC_GRSTCTL) &
             ~(GRSTCTL_CSRST | GRSTCTL_RXFFLSH | GRSTCTL_TXFFLSH)) |
            GRSTCTL_AHBIDL;
        break;
    case DWC_GINTSTS:
        v = usb_gintsts(s);
        break;
    case DWC_GSNPSID:
        v = USB_GSNPSID;
        break;
    case DWC_GHWCFG1:
        v = USB_GHWCFG1;
        break;
    case DWC_GHWCFG2:
        v = USB_GHWCFG2;
        break;
    case DWC_GHWCFG3:
        v = USB_GHWCFG3;
        break;
    case DWC_GHWCFG4:
        v = USB_GHWCFG4;
        break;
    case DWC_HPRT0:
        v = 0;
        break;
    case DWC_DSTS:
        /* Not suspended, EnumSpd 0 (high speed), and the (micro)frame. */
        v = (usb_microframe(s) & 0x3fff) << 8;
        break;
    case DWC_DAINT:
        v = usb_daint(s);
        break;
    default:
        v = *usb_reg(s, off & ~3);
        break;
    }
    USB_TRACE(3, "rd +0x%03" HWADDR_PRIx " -> 0x%08x  pc~0x%08x\n", off, v,
              usb_trace_pc());
    return extract32(v, (off & 3) * 8, size * 8);
}

/* A write to DxEPCTLn: the handshakes happen here, at once. */
static void usb_write_epctl(IPodClassicUSBState *s, bool in, int n,
                            uint32_t v)
{
    uint32_t *ctl = usb_ep(s, in, n, DWC_EPCTL);
    uint32_t *intr = usb_ep(s, in, n, DWC_EPINT);
    uint32_t keep = *ctl & (EPCTL_EPENA | EPCTL_NAKSTS);
    uint32_t c = (v & ~(EPCTL_EPENA | EPCTL_EPDIS | EPCTL_CNAK | EPCTL_SNAK |
                        EPCTL_SETD0PID | EPCTL_SETD1PID | EPCTL_NAKSTS)) |
                 keep;

    if (v & EPCTL_SNAK) {
        c |= EPCTL_NAKSTS;
        if (in) {
            *intr |= EPINT_INEPNAKEFF;
        }
    }
    if (v & EPCTL_CNAK) {
        c &= ~EPCTL_NAKSTS;
    }
    if (v & EPCTL_EPENA) {
        c |= EPCTL_EPENA;
    }
    if ((v & EPCTL_EPDIS) && (c & EPCTL_EPENA)) {
        /* Disabled now: the drivers wait for EPDIS to clear and EPDisbld. */
        c &= ~EPCTL_EPENA;
        *intr |= EPINT_EPDISBLD;
    }
    *ctl = c;
    if ((c & EPCTL_EPENA) && s->host_state != USB_HOST_DETACHED) {
        /* Something for the host, perhaps: never act inside this access. */
        usb_host_arm(s, 20 * SCALE_US);
    }
}

static void usb_write(void *opaque, hwaddr off, uint64_t value,
                      unsigned size)
{
    IPodClassicUSBState *s = opaque;
    uint32_t old, v, *r;
    hwaddr a = off & ~3;

    if (off >= S5L8702_USB_REGS_SIZE) {
        qemu_log_mask(LOG_UNIMP, "s5l8702.usb: write of FIFO window +0x%"
                      HWADDR_PRIx " (slave mode is not modelled)\n", off);
        return;
    }
    r = usb_reg(s, a);
    old = *r;
    v = size == 4 ? (uint32_t)value
                  : deposit32(old, (off & 3) * 8, size * 8, value);
    USB_TRACE(2, "wr +0x%03" HWADDR_PRIx " = 0x%08x  pc~0x%08x\n", a, v,
              usb_trace_pc());

    if (a >= DWC_DIEP(0) && a < DWC_DIEP(S5L8702_USB_MAX_EPS)) {
        int n = (a - DWC_DIEP(0)) / 0x20;
        hwaddr reg = (a - DWC_DIEP(0)) % 0x20;

        if (reg == DWC_EPCTL) {
            usb_write_epctl(s, true, n, v);
        } else if (reg == DWC_EPINT) {
            *r &= ~v;
        } else {
            *r = v;
        }
        usb_update_irq(s);
        return;
    }
    if (a >= DWC_DOEP(0) && a < DWC_DOEP(S5L8702_USB_MAX_EPS)) {
        int n = (a - DWC_DOEP(0)) / 0x20;
        hwaddr reg = (a - DWC_DOEP(0)) % 0x20;

        if (reg == DWC_EPCTL) {
            usb_write_epctl(s, false, n, v);
        } else if (reg == DWC_EPINT) {
            *r &= ~v;
        } else {
            *r = v;
        }
        usb_update_irq(s);
        return;
    }

    switch (a) {
    case DWC_GOTGINT:
        *r &= ~v;
        break;
    case DWC_GINTSTS:
        s->gintsts &= ~v;
        if (v & GINTSTS_SOF) {
            s->sof_cleared = usb_microframe(s);
        }
        break;
    case DWC_GRSTCTL:
        *r = v & ~(GRSTCTL_CSRST | GRSTCTL_RXFFLSH | GRSTCTL_TXFFLSH);
        if (v & GRSTCTL_CSRST) {
            usb_core_reset(s);
        }
        break;
    case DWC_GSNPSID:
    case DWC_GHWCFG1:
    case DWC_GHWCFG2:
    case DWC_GHWCFG3:
    case DWC_GHWCFG4:
    case DWC_DSTS:
    case DWC_DAINT:
        break;   /* read-only */
    case DWC_DAINTMSK:
        *r = v & USB_EP_MASK;
        break;
    case DWC_DCTL: {
        uint32_t c = v & ~(DCTL_SGNPINNAK | DCTL_CGNPINNAK | DCTL_SGOUTNAK |
                           DCTL_CGOUTNAK | DCTL_GINNAKSTS | DCTL_GOUTNAKSTS);

        c |= old & (DCTL_GINNAKSTS | DCTL_GOUTNAKSTS);
        if (v & DCTL_SGNPINNAK) {
            c |= DCTL_GINNAKSTS;
            s->gintsts |= GINTSTS_GINNAKEFF;
        }
        if (v & DCTL_CGNPINNAK) {
            c &= ~DCTL_GINNAKSTS;
            s->gintsts &= ~GINTSTS_GINNAKEFF;
        }
        if (v & DCTL_SGOUTNAK) {
            c |= DCTL_GOUTNAKSTS;
            s->gintsts |= GINTSTS_GOUTNAKEFF;
        }
        if (v & DCTL_CGOUTNAK) {
            c &= ~DCTL_GOUTNAKSTS;
            s->gintsts &= ~GINTSTS_GOUTNAKEFF;
        }
        *r = c;
        if ((old ^ c) & DCTL_SFTDISCON) {
            USB_TRACE(1, "device %s\n", c & DCTL_SFTDISCON ?
                      "soft-disconnected" : "pulled up");
        }
        break;
    }
    default:
        *r = v;
        break;
    }
    usb_host_check_attach(s);
    usb_update_irq(s);
}

static const MemoryRegionOps usb_ops = {
    .read = usb_read,
    .write = usb_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* The PHY: a register file the drivers write and read back. */
static uint64_t usb_phy_read(void *opaque, hwaddr off, unsigned size)
{
    IPodClassicUSBState *s = opaque;
    uint32_t v;

    if (off >= S5L8702_USBPHY_REGS_SIZE) {
        return 0;
    }
    v = s->phy[off / 4];
    if ((off & ~3) == S5L8702_USBPHY_STAT) {
        /*
         * OPHYSTAT bit 0: VBUS is valid at the PHY.  2.0.5's session check
         * (0x080fcb7c in the running image) wants GOTGCTL.BSesVld AND this
         * bit (0x08364b98); with it clear, a bus reset made the driver
         * decide nothing was there and soft-disconnect (MEASURED).
         */
        v = (v & ~1u) | (s->vbus ? 1 : 0);
    }
    return extract32(v, (off & 3) * 8, size * 8);
}

static void usb_phy_write(void *opaque, hwaddr off, uint64_t value,
                          unsigned size)
{
    IPodClassicUSBState *s = opaque;

    if (off >= S5L8702_USBPHY_REGS_SIZE) {
        return;
    }
    USB_TRACE(2, "phy wr +0x%03" HWADDR_PRIx " = 0x%08" PRIx64 "\n", off,
              value);
    s->phy[off / 4] = deposit32(s->phy[off / 4], (off & 3) * 8, size * 8,
                                value);
}

static const MemoryRegionOps usb_phy_ops = {
    .read = usb_phy_read,
    .write = usb_phy_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

/* ---- the page's side ---- */

static IPodClassicUSBState *usb_state;

/* The page's request (ipod_wasm_usb_io), taken with the BQL held. */
static void usb_io_bh(void *opaque)
{
    IPodClassicUSBState *s = opaque;
    int req = qatomic_xchg(&s->io_request, 0);

    if (!req) {
        return;
    }
    s->io_op = (req >> 28) & 0xf;
    s->io_count = req & 0xfffffff;
    s->io_lba = qatomic_read(&s->io_request_lba);
    if (s->host_state == USB_HOST_CONFIGURED && !s->msc_autosense) {
        usb_msc_page_start(s);
    } else {
        usb_host_arm(s, 50 * SCALE_US);   /* the tick takes it */
    }
}

static void usb_cable_bh(void *opaque)
{
    IPodClassicUSBState *s = opaque;
    int req = qatomic_xchg(&s->cable_request, -1);

    if (req >= 0) {
        ipod_classic_set_usb_cable(req);
    }
}

#ifdef EMSCRIPTEN
/*
 * Plug the cable in (1) or pull it out (0).  Off the page's thread like every
 * other input: the bottom half does it with the BQL held.
 */
EMSCRIPTEN_KEEPALIVE void ipod_wasm_usb_cable(int on)
{
    IPodClassicUSBState *s = usb_state;

    if (!s || !s->cable_bh) {
        return;
    }
    qatomic_set(&s->cable_request, on ? 1 : 0);
    qemu_bh_schedule(s->cable_bh);
    qemu_notify_event();
}

/*
 * bits 0-3: the host's state (IPodClassicUSBHostState); bit 4 VBUS; bit 5 the
 * device pulled up; bit 6 its disk ready for ipod_wasm_usb_io(); bits 8-15
 * the address; bits 16-23 the configuration.
 */
EMSCRIPTEN_KEEPALIVE int ipod_wasm_usb_state(void)
{
    IPodClassicUSBState *s = usb_state;

    if (!s) {
        return 0;
    }
    return (s->host_state & 0xf) | (s->vbus ? 0x10 : 0) |
           (usb_pulled_up(s) ? 0x20 : 0) | (s->msc_ready ? 0x40 : 0) |
           (s->address << 8) |
           (s->configuration << 16);
}

/* VID << 16 | PID from the device descriptor, 0 before enumeration. */
EMSCRIPTEN_KEEPALIVE unsigned ipod_wasm_usb_id(void)
{
    return usb_state ? usb_state->vid_pid : 0;
}

/*
 * Mass storage, for the page.  Once ipod_wasm_usb_state() says the disk is
 * ready (bit 6), ipod_wasm_usb_io() asks the iPod, through its own USB
 * driver, to read or write `count` blocks at `lba` (data in the buffer at
 * ipod_wasm_usb_buffer(), at most USB_IO_MAX bytes) or to eject.  It returns
 * 0 when the request is taken, -1 when the disk is not ready or still busy;
 * ipod_wasm_usb_io_status() then goes from USB_IO_BUSY to USB_IO_DONE or
 * USB_IO_FAILED.  One request at a time.
 */
EMSCRIPTEN_KEEPALIVE int ipod_wasm_usb_io(int op, unsigned lba,
                                          unsigned count)
{
    IPodClassicUSBState *s = usb_state;

    if (!s || !s->io_bh || !s->msc_ready ||
        qatomic_read(&s->io_status) == USB_IO_BUSY ||
        op < USB_IO_READ || op > USB_IO_FLUSH ||
        ((op == USB_IO_READ || op == USB_IO_WRITE) &&
         (!count || (uint64_t)count * s->msc_block_size > USB_IO_MAX ||
          (uint64_t)lba + count > s->msc_blocks))) {
        return -1;
    }
    qatomic_set(&s->io_status, USB_IO_BUSY);
    qatomic_set(&s->io_request_lba, lba);
    qatomic_set(&s->io_request,
                (op << 28) | (op == USB_IO_READ || op == USB_IO_WRITE
                              ? count : 0));
    qemu_bh_schedule(s->io_bh);
    qemu_notify_event();
    return 0;
}

EMSCRIPTEN_KEEPALIVE int ipod_wasm_usb_io_status(void)
{
    return usb_state ? qatomic_read(&usb_state->io_status) : USB_IO_IDLE;
}

EMSCRIPTEN_KEEPALIVE unsigned ipod_wasm_usb_buffer(void)
{
    return (unsigned)(uintptr_t)usb_io_buf;
}

EMSCRIPTEN_KEEPALIVE unsigned ipod_wasm_usb_buffer_size(void)
{
    return USB_IO_MAX;
}

/* The disk as READ CAPACITY gave it: blocks, and bytes per block. */
EMSCRIPTEN_KEEPALIVE unsigned ipod_wasm_usb_blocks(void)
{
    return usb_state ? usb_state->msc_blocks : 0;
}

EMSCRIPTEN_KEEPALIVE unsigned ipod_wasm_usb_block_size(void)
{
    return usb_state ? usb_state->msc_block_size : 0;
}
#endif

/* ---- device ---- */

static void ipod_classic_usb_reset(DeviceState *dev)
{
    IPodClassicUSBState *s = IPOD_CLASSIC_USB(dev);

    memset(s->reg, 0, sizeof(s->reg));
    memset(s->phy, 0, sizeof(s->phy));
    s->gintsts = 0;
    /* The core comes out of reset soft-disconnected. */
    *usb_reg(s, DWC_DCTL) = DCTL_SFTDISCON;
    /* VBUS is where the cable is: a reset does not move it. */
    usb_host_detach(s);
    s->vid_pid = 0;
    s->config_len = 0;
    memset(s->dev_desc, 0, sizeof(s->dev_desc));
    qemu_set_irq(s->irq, 0);
}

static void ipod_classic_usb_realize(DeviceState *dev, Error **errp)
{
    IPodClassicUSBState *s = IPOD_CLASSIC_USB(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &usb_ops, s,
                          TYPE_IPOD_CLASSIC_USB, S5L8702_USB_MEM_SIZE);
    memory_region_init_io(&s->phy_iomem, OBJECT(dev), &usb_phy_ops, s,
                          "s5l8702.usb-phy", S5L8702_USBPHY_MEM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->phy_iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->host_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, usb_host_tick, s);
    s->host_timer_rt = timer_new_ns(QEMU_CLOCK_VIRTUAL_RT, usb_host_tick, s);
    s->cable_bh = qemu_bh_new(usb_cable_bh, s);
    s->cable_request = -1;
    s->io_bh = qemu_bh_new(usb_io_bh, s);
    s->msc_iface = -1;
    usb_state = s;
}

static int ipod_classic_usb_post_load(void *opaque, int version_id)
{
    usb_update_irq(opaque);
    return 0;
}

static const VMStateDescription vmstate_ipod_classic_usb = {
    .name = TYPE_IPOD_CLASSIC_USB,
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = ipod_classic_usb_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(reg, IPodClassicUSBState,
                             S5L8702_USB_REGS_SIZE / 4),
        VMSTATE_UINT32_ARRAY(phy, IPodClassicUSBState,
                             S5L8702_USBPHY_REGS_SIZE / 4),
        VMSTATE_UINT32(gintsts, IPodClassicUSBState),
        VMSTATE_BOOL(vbus, IPodClassicUSBState),
        VMSTATE_TIMER_PTR(host_timer, IPodClassicUSBState),
        VMSTATE_INT32(host_state, IPodClassicUSBState),
        VMSTATE_INT32(host_step, IPodClassicUSBState),
        VMSTATE_INT64(host_deadline_ns, IPodClassicUSBState),
        VMSTATE_UINT8_ARRAY(setup, IPodClassicUSBState, 8),
        VMSTATE_UINT32(want, IPodClassicUSBState),
        VMSTATE_UINT32(got, IPodClassicUSBState),
        VMSTATE_UINT8_ARRAY(buf, IPodClassicUSBState, 512),
        VMSTATE_UINT8_ARRAY(dev_desc, IPodClassicUSBState, 18),
        VMSTATE_UINT8_ARRAY(config, IPodClassicUSBState, 256),
        VMSTATE_UINT32(config_len, IPodClassicUSBState),
        VMSTATE_UINT8(address, IPodClassicUSBState),
        VMSTATE_UINT8(configuration, IPodClassicUSBState),
        VMSTATE_UINT32(vid_pid, IPodClassicUSBState),
        VMSTATE_INT64(sof_base_ns, IPodClassicUSBState),
        VMSTATE_UINT32(sof_cleared, IPodClassicUSBState),
        VMSTATE_INT32_V(msc_iface, IPodClassicUSBState, 2),
        VMSTATE_UINT8_V(msc_in, IPodClassicUSBState, 2),
        VMSTATE_UINT8_V(msc_out, IPodClassicUSBState, 2),
        VMSTATE_BOOL_V(stalled, IPodClassicUSBState, 2),
        VMSTATE_BOOL_V(msc_ready, IPodClassicUSBState, 2),
        VMSTATE_INT32_V(msc_init, IPodClassicUSBState, 2),
        VMSTATE_INT32_V(msc_tries, IPodClassicUSBState, 2),
        VMSTATE_UINT32_V(msc_sense, IPodClassicUSBState, 2),
        VMSTATE_BOOL_V(msc_autosense, IPodClassicUSBState, 2),
        VMSTATE_UINT8_V(msc_failed_op, IPodClassicUSBState, 2),
        VMSTATE_BOOL_V(io_then_eject, IPodClassicUSBState, 2),
        VMSTATE_INT64_V(setup_ns, IPodClassicUSBState, 2),
        VMSTATE_UINT32_V(msc_blocks, IPodClassicUSBState, 2),
        VMSTATE_UINT32_V(msc_block_size, IPodClassicUSBState, 2),
        VMSTATE_INT32_V(bot_phase, IPodClassicUSBState, 2),
        VMSTATE_UINT8_ARRAY_V(bot_cbw, IPodClassicUSBState, 31, 2),
        VMSTATE_UINT8_ARRAY_V(bot_csw, IPodClassicUSBState, 13, 2),
        VMSTATE_UINT32_V(bot_tag, IPodClassicUSBState, 2),
        VMSTATE_UINT32_V(bot_len, IPodClassicUSBState, 2),
        VMSTATE_UINT32_V(bot_done, IPodClassicUSBState, 2),
        VMSTATE_BOOL_V(bot_in, IPodClassicUSBState, 2),
        VMSTATE_BOOL_V(bot_page, IPodClassicUSBState, 2),
        VMSTATE_UINT8_ARRAY_V(bot_buf, IPodClassicUSBState, 64, 2),
        VMSTATE_BOOL_V(bot_retried, IPodClassicUSBState, 2),
        VMSTATE_UINT8_V(bot_halted, IPodClassicUSBState, 2),
        VMSTATE_END_OF_LIST()
    },
};

static void ipod_classic_usb_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "S5L8702 USB OTG (device mode) and its PHY";
    dc->realize = ipod_classic_usb_realize;
    dc->reset = ipod_classic_usb_reset;
    dc->vmsd = &vmstate_ipod_classic_usb;
}

static const TypeInfo ipod_classic_usb_info = {
    .name          = TYPE_IPOD_CLASSIC_USB,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodClassicUSBState),
    .class_init    = ipod_classic_usb_class_init,
};

static void ipod_classic_usb_register_types(void)
{
    type_register_static(&ipod_classic_usb_info);
}

type_init(ipod_classic_usb_register_types)
