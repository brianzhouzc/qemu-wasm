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
 * is host-side only), so the other end of the cable is emulated too.  For now
 * it is a script: once the device pulls up, the host resets the bus, then
 * enumerates (device descriptor, address, configuration, strings) and sets
 * configuration 1, logging what the firmware answers.  Mass storage comes
 * next.  It runs on QEMU_CLOCK_VIRTUAL, so it waits through the guest's idle
 * like any other device, and it never acts inside a guest register access.
 *
 * USBTRACE=1 logs host events and descriptors; 2 adds register writes; 3 adds
 * reads.
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
#define USB_DT_DEVICE              1
#define USB_DT_CONFIG              2
#define USB_DT_STRING              3

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
    STEP_DONE,
};

static const char *const usb_host_state_names[] = {
    "detached", "attaching", "reset", "setup", "data-in", "status-out",
    "status-in", "configured", "failed",
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

static void usb_host_set_state(IPodClassicUSBState *s, int state)
{
    if (s->host_state != state) {
        USB_TRACE(1, "host: %s -> %s\n", usb_host_state_names[s->host_state],
                  usb_host_state_names[state]);
    }
    s->host_state = state;
    /* Every wait for the device has a limit: 2 s of guest time. */
    s->host_deadline_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          2 * NANOSECONDS_PER_SECOND;
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
    timer_del(s->host_timer);
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
        /* Every interface and endpoint, for the log. */
        for (uint32_t at = 0; at + 2 <= s->got && s->config[at]; ) {
            const uint8_t *d = s->config + at;

            if (d[1] == 4 && d[0] >= 9) {
                USB_TRACE(1, "host:   interface %u alt %u class %02x/%02x/%02x"
                          ", %u endpoint(s)\n", d[2], d[3], d[5], d[6], d[7],
                          d[4]);
            } else if (d[1] == 5 && d[0] >= 7) {
                USB_TRACE(1, "host:     endpoint 0x%02x type %u mps %u\n",
                          d[2], d[3] & 3, lduw_le_p(d + 4));
            }
            at += d[0];
        }
        break;
    }
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

/* Begin the next request, or settle once the script is done. */
static void usb_host_begin(IPodClassicUSBState *s)
{
    if (!usb_host_next(s)) {
        usb_host_set_state(s, USB_HOST_CONFIGURED);
        s->host_deadline_ns = 0;
        USB_TRACE(1, "host: enumeration done, configuration %u\n",
                  s->configuration);
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
            s->host_deadline_ns = now + 2 * NANOSECONDS_PER_SECOND;
            if (done) {
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
            if (s->setup[1] == USB_REQ_SET_ADDRESS) {
                s->address = s->setup[2];
                USB_TRACE(1, "host: address %u (DCFG.DevAddr %u)\n",
                          s->address,
                          (*usb_reg(s, DWC_DCFG) >> 4) & 0x7f);
            } else if (s->setup[1] == USB_REQ_SET_CONFIGURATION) {
                s->configuration = s->setup[2];
            }
            s->host_step++;
            usb_host_begin(s);
        }
        break;
    default:
        return;   /* detached, configured or failed: nothing to do */
    }

    if (s->host_state == USB_HOST_CONFIGURED) {
        return;
    }
    if (progress) {
        /* The next packet goes in the next microframe or so. */
        usb_host_arm(s, 50 * SCALE_US);
        return;
    }
    if (s->host_deadline_ns && now > s->host_deadline_ns) {
        USB_TRACE(1, "host: the device stopped answering in %s (step %d); "
                  "DOEPCTL0 0x%08x DIEPCTL0 0x%08x DIEPTSIZ0 0x%08x\n",
                  usb_host_state_names[s->host_state], s->host_step,
                  *usb_ep(s, false, 0, DWC_EPCTL),
                  *usb_ep(s, true, 0, DWC_EPCTL),
                  *usb_ep(s, true, 0, DWC_EPTSIZ));
        usb_host_set_state(s, USB_HOST_FAILED);
        s->host_deadline_ns = 0;
        return;
    }
    /* Not ready yet: look again in a millisecond (a USB frame). */
    usb_host_arm(s, SCALE_MS);
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
 * device pulled up; bits 8-15 the address; bits 16-23 the configuration.
 */
EMSCRIPTEN_KEEPALIVE int ipod_wasm_usb_state(void)
{
    IPodClassicUSBState *s = usb_state;

    if (!s) {
        return 0;
    }
    return (s->host_state & 0xf) | (s->vbus ? 0x10 : 0) |
           (usb_pulled_up(s) ? 0x20 : 0) | (s->address << 8) |
           (s->configuration << 16);
}

/* VID << 16 | PID from the device descriptor, 0 before enumeration. */
EMSCRIPTEN_KEEPALIVE unsigned ipod_wasm_usb_id(void)
{
    return usb_state ? usb_state->vid_pid : 0;
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
    s->cable_bh = qemu_bh_new(usb_cable_bh, s);
    s->cable_request = -1;
    usb_state = s;
}

static int ipod_classic_usb_post_load(void *opaque, int version_id)
{
    usb_update_irq(opaque);
    return 0;
}

static const VMStateDescription vmstate_ipod_classic_usb = {
    .name = TYPE_IPOD_CLASSIC_USB,
    .version_id = 1,
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
