/*
 * S5L8702 USB: the Synopsys DesignWare USB 2.0 OTG core (device mode, buffer
 * DMA) at 0x38400000, its PHY control block at 0x3C400000, and a small
 * emulated host on the far end of the cable.
 *
 * See ipod_classic_usb.c for what is modelled and why.
 */
#ifndef HW_ARM_IPOD_CLASSIC_USB_H
#define HW_ARM_IPOD_CLASSIC_USB_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_IPOD_CLASSIC_USB "s5l8702.usb-otg"
OBJECT_DECLARE_SIMPLE_TYPE(IPodClassicUSBState, IPOD_CLASSIC_USB)

/* Both windows are 1 MB; the registers are at the start of each. */
#define S5L8702_USB_MEM_SIZE        0x100000
#define S5L8702_USB_REGS_SIZE       0x1000   /* CSRs; the FIFOs (0x1000+) are not used in DMA mode */
#define S5L8702_USBPHY_MEM_SIZE     0x100000
#define S5L8702_USBPHY_REGS_SIZE    0x200
#define S5L8702_USBPHY_STAT         0x28     /* OPHYSTAT: bit 0 VBUS valid */

/* Endpoints the core reports, EP0 included (GHWCFG2 NumDevEps + 1). */
#define S5L8702_USB_NUM_EPS         7
/* Register space exists for 16 of each direction. */
#define S5L8702_USB_MAX_EPS         16

/* Global registers. */
#define DWC_GOTGCTL     0x000
#define DWC_GOTGINT     0x004
#define DWC_GAHBCFG     0x008
#define DWC_GUSBCFG     0x00c
#define DWC_GRSTCTL     0x010
#define DWC_GINTSTS     0x014
#define DWC_GINTMSK     0x018
#define DWC_GRXSTSR     0x01c
#define DWC_GRXSTSP     0x020
#define DWC_GRXFSIZ     0x024
#define DWC_GNPTXFSIZ   0x028
#define DWC_GNPTXSTS    0x02c
#define DWC_GSNPSID     0x040
#define DWC_GHWCFG1     0x044
#define DWC_GHWCFG2     0x048
#define DWC_GHWCFG3     0x04c
#define DWC_GHWCFG4     0x050
#define DWC_HPRT0       0x440
/* Device registers. */
#define DWC_DCFG        0x800
#define DWC_DCTL        0x804
#define DWC_DSTS        0x808
#define DWC_DIEPMSK     0x810
#define DWC_DOEPMSK     0x814
#define DWC_DAINT       0x818
#define DWC_DAINTMSK    0x81c
#define DWC_DIEP(n)     (0x900 + 0x20 * (n))
#define DWC_DOEP(n)     (0xb00 + 0x20 * (n))
#define DWC_EPCTL       0x00
#define DWC_EPINT       0x08
#define DWC_EPTSIZ      0x10
#define DWC_EPDMA       0x14
#define DWC_PCGCCTL     0xe00

/* GAHBCFG */
#define GAHBCFG_GINT        (1u << 0)
#define GAHBCFG_DMAEN       (1u << 5)
/* GRSTCTL */
#define GRSTCTL_CSRST       (1u << 0)
#define GRSTCTL_RXFFLSH     (1u << 4)
#define GRSTCTL_TXFFLSH     (1u << 5)
#define GRSTCTL_AHBIDL      (1u << 31)
/* GOTGCTL */
#define GOTGCTL_CONIDSTS    (1u << 16)   /* 1: B-device */
#define GOTGCTL_BSESVLD     (1u << 19)
/* GOTGINT */
#define GOTGINT_SESENDDET   (1u << 2)
/* GINTSTS / GINTMSK */
#define GINTSTS_OTGINT      (1u << 2)
#define GINTSTS_SOF         (1u << 3)
#define GINTSTS_GINNAKEFF   (1u << 6)
#define GINTSTS_GOUTNAKEFF  (1u << 7)
#define GINTSTS_USBSUSP     (1u << 11)
#define GINTSTS_USBRST      (1u << 12)
#define GINTSTS_ENUMDNE     (1u << 13)
#define GINTSTS_IEPINT      (1u << 18)
#define GINTSTS_OEPINT      (1u << 19)
#define GINTSTS_DISCONNINT  (1u << 29)
#define GINTSTS_SESSREQINT  (1u << 30)
/* DCTL */
#define DCTL_SFTDISCON      (1u << 1)
#define DCTL_GINNAKSTS      (1u << 2)
#define DCTL_GOUTNAKSTS     (1u << 3)
#define DCTL_SGNPINNAK      (1u << 7)
#define DCTL_CGNPINNAK      (1u << 8)
#define DCTL_SGOUTNAK       (1u << 9)
#define DCTL_CGOUTNAK       (1u << 10)
/* DxEPCTL */
#define EPCTL_MPS_MASK      0x7ffu
#define EPCTL_USBACTEP      (1u << 15)
#define EPCTL_NAKSTS        (1u << 17)
#define EPCTL_STALL         (1u << 21)
#define EPCTL_CNAK          (1u << 26)
#define EPCTL_SNAK          (1u << 27)
#define EPCTL_SETD0PID      (1u << 28)
#define EPCTL_SETD1PID      (1u << 29)
#define EPCTL_EPDIS         (1u << 30)
#define EPCTL_EPENA         (1u << 31)
/* DxEPINT */
#define EPINT_XFERCOMPL     (1u << 0)
#define EPINT_EPDISBLD      (1u << 1)
#define EPINT_SETUP         (1u << 3)    /* OUT */
#define EPINT_INEPNAKEFF    (1u << 6)    /* IN */
/* DxEPTSIZ */
#define EPTSIZ_XFERSIZE(v)  ((v) & 0x7ffffu)
#define EPTSIZ_PKTCNT(v)    (((v) >> 19) & 0x3ffu)
#define EPTSIZ_SUPCNT(v)    (((v) >> 29) & 0x3u)

/* What the emulated host is doing; ipod_wasm_usb_state() reports it. */
typedef enum {
    USB_HOST_DETACHED = 0,   /* no cable, or the device is not pulled up */
    USB_HOST_ATTACHING,      /* debounce, then bus reset */
    USB_HOST_RESET,          /* reset signalled, enumeration next */
    USB_HOST_SETUP,          /* a SETUP waits for OUT0 to be armed */
    USB_HOST_DATA_IN,        /* collecting a control read's data */
    USB_HOST_STATUS_OUT,     /* the ZLP that ends a control read */
    USB_HOST_STATUS_IN,      /* the ZLP that ends a no-data request */
    USB_HOST_CONFIGURED,     /* configured; between mass-storage commands */
    USB_HOST_FAILED,         /* the device stopped answering */
    USB_HOST_BULK_OUT,       /* a command block or data, to the device */
    USB_HOST_BULK_IN,        /* data or a status, from the device */
} IPodClassicUSBHostState;

/* Bulk-Only Transport, one command at a time. */
typedef enum {
    USB_BOT_IDLE = 0,
    USB_BOT_CBW,
    USB_BOT_DATA_IN,
    USB_BOT_DATA_OUT,
    USB_BOT_CSW,
} IPodClassicUSBBotPhase;

/* The page's block requests (ipod_wasm_usb_io) and their outcome. */
#define USB_IO_READ     1
#define USB_IO_WRITE    2
#define USB_IO_EJECT    3       /* flushes first, as computers do */
#define USB_IO_FLUSH    4       /* SYNCHRONIZE CACHE */
#define USB_IO_IDLE     0
#define USB_IO_BUSY     1
#define USB_IO_DONE     2
#define USB_IO_FAILED   3
/* The page's transfers go through this buffer, a pointer the page reads. */
#define USB_IO_MAX      (256 * 1024)

struct IPodClassicUSBState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion phy_iomem;
    qemu_irq irq;

    uint32_t reg[S5L8702_USB_REGS_SIZE / 4];
    uint32_t phy[S5L8702_USBPHY_REGS_SIZE / 4];
    /* Latched GINTSTS bits; IEPINT/OEPINT/OTGINT are computed. */
    uint32_t gintsts;
    bool vbus;

    /* The emulated host. */
    QEMUTimer *host_timer;
    QEMUTimer *host_timer_rt;   /* idle polls in host time: usb_host_poll */
    int32_t host_state;
    int32_t host_step;          /* index into the enumeration script */
    int64_t host_deadline_ns;   /* QEMU_CLOCK_VIRTUAL; 0 = none */
    int64_t host_deadline_rt;   /* QEMU_CLOCK_VIRTUAL_RT; not migrated */
    uint8_t setup[8];
    uint32_t want;              /* wLength of the current control read */
    uint32_t got;               /* bytes of it received */
    uint8_t buf[512];
    uint8_t dev_desc[18];
    uint8_t config[256];
    uint32_t config_len;
    uint8_t address;
    uint8_t configuration;
    uint32_t vid_pid;           /* 0 until the device descriptor is in */
    /* Mass storage: the interface, its bulk endpoints, what it reported. */
    int32_t msc_iface;          /* -1: none */
    uint8_t msc_in;             /* endpoint numbers */
    uint8_t msc_out;
    bool stalled;               /* the last control read was stalled */
    bool msc_ready;
    int32_t msc_init;           /* the init sequence's next command */
    int32_t msc_tries;          /* TEST UNIT READY attempts */
    uint32_t msc_sense;         /* the last not-ready reason: key/ASC/ASCQ */
    int64_t msc_retry_ns;       /* next TEST UNIT READY; VIRTUAL_RT, unsaved */
    bool msc_autosense;         /* a page command failed: REQUEST SENSE next */
    uint8_t msc_failed_op;      /* its opcode, for the log */
    bool io_then_eject;         /* the flush before an eject is running */
    int64_t setup_ns;           /* when the last SETUP went in */
    uint32_t msc_blocks;
    uint32_t msc_block_size;
    /* The command in flight. */
    int32_t bot_phase;
    uint8_t bot_cbw[31];
    uint8_t bot_csw[13];
    uint32_t bot_tag;
    uint32_t bot_len;
    uint32_t bot_done;
    bool bot_in;
    bool bot_page;              /* the page's command, not the init sequence */
    bool bot_retried;           /* the status was stalled once already */
    uint8_t bot_halted;         /* the endpoint being cleared */
    uint8_t bot_buf[64];        /* the init sequence's data */
    /* The page's request, handed over by ipod_wasm_usb_io(). */
    int32_t io_op;
    uint32_t io_lba;
    uint32_t io_count;
    int io_status;              /* USB_IO_*; atomic */
    int io_request;             /* op << 28 | count, posted; atomic */
    uint32_t io_request_lba;    /* atomic */
    QEMUBH *io_bh;

    int64_t sof_base_ns;        /* when SOFs began; 0 = none (see usb_microframe) */
    uint32_t sof_cleared;       /* the microframe in which GINTSTS.Sof was cleared */

    /* The page's requests, taken in a bottom half (see the exports). */
    QEMUBH *cable_bh;
    int cable_request;          /* -1 none, else 0/1; atomic */
};

/* Plug or pull the cable: VBUS for the core; the PMU's EXTON2 is the caller's. */
void ipod_classic_usb_set_vbus(IPodClassicUSBState *s, bool on);

#endif
