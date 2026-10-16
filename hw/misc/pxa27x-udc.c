/*
 * Intel PXA27x USB device controller (UDC), with a built-in host.
 *
 * Copyright (c) 2026 The RetroPane OS contributors
 *
 * This code is licensed under the GNU GPL v2 or later.
 *
 * Follows RetroPane's clean-room spec (retropane-os docs/hw/udc.md). QEMU has
 * no "USB gadget" side, so the model contains the host too: once the guest
 * enables the controller it plays what Windows does when a device is plugged
 * in (bus resets, the descriptor reads, SET_CONFIGURATION, which the PXA27x
 * answers in hardware, Get Max LUN for a mass storage interface), then:
 *
 *  - the CDC serial function: when a client connects to the chardev
 *    (`-global pxa27x-udc.chardev=<id>`), the requests a terminal program
 *    sends on open, then bytes from the chardev become bulk OUT packets and
 *    bulk IN packets go to the chardev (raw bytes);
 *  - a USB Mass Storage function (RetroPane ADR-0014), if the configuration
 *    has one: its bulk pair is bridged to a second chardev
 *    (`-global pxa27x-udc.msc=<id>`), framed so transfers keep their
 *    boundaries: the client sends each OUT transfer as a 4-byte
 *    little-endian length and the bytes; each IN packet reaches the client
 *    as a 1-byte length and the bytes (a zero length is a zero-length
 *    packet).
 *
 * Like a real host, it reads the configuration descriptor to learn which
 * endpoint belongs to which function (by interface class), and both
 * functions run at the same time; the serial port's open requests don't hold
 * up the drive.
 *
 * Bulk endpoints as measured on the real X51v (retropane-os item 74,
 * docs/hw/udc.md, 2026-10-16): one packet buffer, or two with UDCCRx.DE
 * (double buffering); an OUT packet shorter than the maximum sets UDCCSRx.SP
 * while it is the one being read; an IN endpoint reads "not full" while a
 * buffer is free and no short packet waits (none is queued behind one);
 * each packet the host takes or delivers sets the slot's packet-complete
 * event in UDCISR0 (bit 2x); with UDCCSRx.DME the slot drives its DMA
 * request (named GPIO "dreq", slot x: the PXA27x's request 24 + x): OUT
 * while the packet being read has bytes left, and a packet read to its end
 * by DMA is released without the guest's PC; IN while a buffer has room.
 *
 * The X51v's D+ pull-up is hard-wired (retropane-os docs/hw/udc.md, measured
 * 2026-10-11 and traced on a PC 2026-10-16): once the host has enumerated
 * the device, turning the controller off and on again is invisible to it,
 * and it keeps the old device (`fixed-pullup`, on by default). The host
 * sees an unplug only when the connector is driven to SE0: port 2's
 * transceiver given to the PXA27x's USB host controller (UP2OCR.HXS) and a
 * root-hub port reset there (named GPIO "port2-reset", wired by the x51v
 * machine; rp-unplug run 3), or a board reset. The next time the controller
 * is on with port 2 back on it, the host enumerates it as a new device.
 *
 * The host is strict where the hardware is: a packet waits in its FIFO until
 * the host's next IN token (one host step), a full FIFO is sent without IPR,
 * a host-to-device request ends with the guest's zero-length status IN (IPR,
 * measured on the real X51v), only endpoints of the active interface work,
 * and writing into a FIFO that still holds a packet is logged as a guest
 * error. Problems are logged with -d guest_errors.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qemu/fifo8.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "chardev/char-fe.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_PXA27X_UDC "pxa27x-udc"
OBJECT_DECLARE_SIMPLE_TYPE(Pxa27xUdcState, PXA27X_UDC)

/* Registers (offsets from 0x40600000). */
#define UDCCR       0x000
#define UDCICR0     0x004
#define UDCICR1     0x008
#define UDCISR0     0x00C
#define UDCISR1     0x010
#define UDCFNR      0x014
#define UP2OCR      0x020
#define UP3OCR      0x024
#define CSR(x)      (0x100 + 4 * (x))
#define BCR(x)      (0x200 + 4 * (x))
#define DR(x)       (0x300 + 4 * (x))
#define CR(x)       (0x400 + 4 * (x))
#define NEP         24

#define UDCCR_UDE   (1 << 0)
#define UP2OCR_HXS  (1 << 16)
#define UDCCR_EMCE  (1 << 3)
#define UDCCR_SMAC  (1 << 4)
#define UDCCR_ACN_SHIFT 11

#define ISR1_RESET  (1u << 27)
#define ISR1_CC     (1u << 31)
#define ISR0_EP0    (1u << 0)
/* Packet complete of endpoint slot x (1 = A). */
#define ISR0_PKT(x) (1u << (2 * (x)))

#define CR_DE       (1 << 1)

#define CSR0_OPC    (1 << 0)
#define CSR0_IPR    (1 << 1)
#define CSR0_FTF    (1 << 2)
#define CSR0_SST    (1 << 4)
#define CSR0_FST    (1 << 5)
#define CSR0_RNE    (1 << 6)
#define CSR0_SA     (1 << 7)
#define CSR0_AREN   (1 << 8)

#define CSR_PC      (1 << 1)
#define CSR_DME     (1 << 3)
#define CSR_BNX     (1 << 6)
#define CSR_SP      (1 << 7)
#define CSR_FEF     (1 << 8)

#define EP0_SIZE    16
#define BULK_SIZE   64

#define STEP_NS     (100 * 1000)
/* Give up on a control transfer after this many steps: 5 s, the USB 2.0
 * limit for a whole control transfer (Windows is as patient). */
#define XFER_LIMIT  50000

/* Reset recovery: a host waits 10 ms after a bus reset before the next
 * request (USB 2.0, TRSTRCY). Without it, a driver that handles the reset
 * after the first SETUP arrived clears that SETUP's flags as it cleans up,
 * and the request is lost (seen as intermittent first-request timeouts). */
#define RESET_RECOVERY_STEPS 100

/* Mass storage OUT transfers queued from the chardev. */
#define MSC_XFERS   32
#define MSC_RX      4096

enum host_state {
    H_IDLE,         /* controller off: detached */
    H_ATTACH,       /* attached, about to reset */
    H_SCRIPT,       /* running the enumeration script */
    H_RUN,          /* configured: functions running */
    H_FAILED,
};

enum xfer_phase {
    X_NONE,
    X_SETUP,        /* SETUP placed; waiting for the guest to take it */
    X_IN,           /* collecting IN packets */
    X_STATUS_OUT,   /* zero-length OUT placed; waiting for OPC to clear */
    X_OUT,          /* OUT data placed; waiting for OPC to clear */
    X_STATUS_IN,    /* host-to-device request: waiting for the guest's
                     * zero-length status IN (IPR), as on the real X51v */
    X_WAIT_ACK,     /* SET_CONFIGURATION: waiting for AREN */
};

/* One packet buffer. */
typedef struct BulkPkt {
    uint8_t d[BULK_SIZE];
    uint32_t len, pos;
} BulkPkt;

/* One bulk endpoint slot, as the host and the guest see it: IN, `fill` is
 * the buffer being written and `q` the committed packets waiting for the
 * host's IN tokens; OUT, `q` holds the packets received, the front one
 * being read. One buffer, or two with UDCCRx.DE. */
typedef struct BulkFifo {
    BulkPkt fill;
    BulkPkt q[2];
    uint32_t head, count;
} BulkFifo;

struct Pxa27xUdcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    /* DMA requests, by slot (the PXA27x's request 24 + slot). */
    qemu_irq dreq[NEP];
    bool dreq_level[NEP];
    CharBackend chr;
    CharBackend msc;
    QEMUTimer *timer;

    uint32_t udccr;
    uint32_t icr0, icr1, isr0, isr1;
    uint32_t up2ocr, up3ocr;
    uint32_t csr[NEP];
    uint32_t cr[NEP];

    /* Endpoint 0: host->device bytes, device->host packet being built and
     * the packet waiting for the host's IN token. */
    Fifo8 ep0_rx;
    uint8_t ep0_tx[EP0_SIZE];
    uint32_t ep0_tx_len;
    bool ep0_ready;

    /* Bulk FIFOs, one per endpoint slot. */
    BulkFifo bulk[NEP];
    /* Bytes from the serial chardev not yet sent as OUT packets. */
    Fifo8 host_rx;

    /* Mass storage: OUT bytes from the chardev, the lengths of the
     * transfers they belong to, and the frame header being parsed. */
    Fifo8 msc_rx;
    uint32_t msc_xfer[MSC_XFERS];
    uint32_t msc_xfer_head, msc_xfer_count;
    uint8_t msc_hdr[4];
    uint32_t msc_hdr_len;
    uint32_t msc_frame_left;
    bool msc_open;

    /* What the configuration descriptor says (endpoint numbers, 0 = none). */
    uint8_t cdc_in_ep, cdc_out_ep;
    uint8_t msc_in_ep, msc_out_ep;
    int msc_iface;      /* -1: no mass storage interface */
    uint8_t max_lun;

    /* The host. */
    enum host_state state;
    uint32_t step;          /* position in the script */
    enum xfer_phase phase;
    uint8_t setup[8];
    uint8_t out_data[EP0_SIZE];
    uint32_t out_data_len;
    uint8_t resp[256];
    uint32_t resp_len;
    bool stalled;
    uint32_t xfer_steps;
    uint32_t attach_steps;
    uint32_t wait_steps;    /* idle steps before the next script step */
    uint8_t config_value;
    uint32_t smac_count;
    /* A program has the serial port open: a client on the chardev (or no
     * chardev at all). Like Windows, the host sends the "open" requests
     * (line coding, DTR) only then, so nothing the guest sends is lost. */
    bool chr_open;
    /* The open requests: 0 not sent, 1/2 in progress, 3 done. */
    uint32_t open_step;
    /* The host accepts composite (IAD, class 0xEF) devices; off plays a
     * host that doesn't, to test a guest's fallback layout. */
    bool accept_composite;
    /* The pull-up is hard-wired (property, on): see the top of the file. */
    bool fixed_pullup;
    /* The host has seen the device plugged in, and no unplug since. */
    bool seen;
};

static void udc_update_irq(Pxa27xUdcState *s)
{
    qemu_set_irq(s->irq, (s->isr0 & s->icr0) || (s->isr1 & s->icr1));
}

/* Whether slot x is an enabled bulk endpoint of the active interface. Only
 * the active interface's endpoints work (measured on the real X51v); a CDC
 * host never sends SET_INTERFACE, so that's 0. */
static bool slot_active_bulk(Pxa27xUdcState *s, int x)
{
    uint32_t c = s->cr[x];
    return (c & 1) && ((c >> 13) & 3) == 2 &&
           ((c >> 22) & 7) == ((s->udccr >> 8) & 7);
}

static bool slot_is_in(Pxa27xUdcState *s, int x)
{
    return (s->cr[x] >> 12) & 1;
}

/* Packet buffers of slot x: two with double buffering (UDCCRx.DE). */
static unsigned slot_depth(Pxa27xUdcState *s, int x)
{
    return (s->cr[x] & CR_DE) ? 2 : 1;
}

static BulkPkt *bulk_front(BulkFifo *f)
{
    return f->count ? &f->q[f->head] : NULL;
}

static void bulk_pop(BulkFifo *f)
{
    if (f->count) {
        f->head = (f->head + 1) % 2;
        f->count--;
    }
}

static bool bulk_push(BulkFifo *f, unsigned depth, const uint8_t *b,
                      uint32_t n)
{
    if (f->count >= depth) {
        return false;
    }
    BulkPkt *p = &f->q[(f->head + f->count) % 2];
    memcpy(p->d, b, n);
    p->len = n;
    p->pos = 0;
    f->count++;
    return true;
}

/* IN: a committed short packet waits (measured: nothing goes in behind
 * one, the endpoint reads full until it has gone). */
static bool bulk_short_waits(BulkFifo *f)
{
    for (uint32_t i = 0; i < f->count; i++) {
        if (f->q[(f->head + i) % 2].len < BULK_SIZE) {
            return true;
        }
    }
    return false;
}

/* IN: room for the guest's next bytes ("buffer not full"). */
static bool in_room(Pxa27xUdcState *s, int x)
{
    BulkFifo *f = &s->bulk[x];
    return f->count < slot_depth(s, x) && !bulk_short_waits(f) &&
           f->fill.len < BULK_SIZE;
}

/* Drives each bulk slot's DMA request from its state. */
static void udc_update_dreq(Pxa27xUdcState *s)
{
    for (int x = 1; x < NEP; x++) {
        bool on = false;
        if (slot_active_bulk(s, x) && (s->csr[x] & CSR_DME)) {
            if (slot_is_in(s, x)) {
                on = in_room(s, x);
            } else {
                BulkPkt *p = bulk_front(&s->bulk[x]);
                on = p && p->pos < p->len;
            }
        }
        if (on != s->dreq_level[x]) {
            s->dreq_level[x] = on;
            qemu_set_irq(s->dreq[x], on);
        }
    }
}

/* IN: the buffer being written becomes a packet for the host. */
static void in_commit(Pxa27xUdcState *s, int x)
{
    BulkFifo *f = &s->bulk[x];
    if (!bulk_push(f, slot_depth(s, x), f->fill.d, f->fill.len)) {
        qemu_log_mask(LOG_GUEST_ERROR, "pxa27x-udc: bulk IN slot %d "
                      "committed a packet with no free buffer\n", x);
    }
    f->fill.len = 0;
}

/* The active bulk slot for endpoint `number` in direction `in`; with
 * number 0, the first active bulk slot in that direction. */
static int find_slot(Pxa27xUdcState *s, uint8_t number, bool in)
{
    for (int x = 1; x < NEP; x++) {
        if (slot_active_bulk(s, x) && slot_is_in(s, x) == in &&
            (number == 0 || ((s->cr[x] >> 15) & 0xF) == number)) {
            return x;
        }
    }
    return -1;
}

/* ---- host side ------------------------------------------------------- */

static void host_reset_bus(Pxa27xUdcState *s)
{
    fifo8_reset(&s->ep0_rx);
    s->ep0_tx_len = 0;
    s->ep0_ready = false;
    s->csr[0] = 0;
    s->udccr &= ~(3u << UDCCR_ACN_SHIFT);
    memset(s->bulk, 0, sizeof(s->bulk));
    s->phase = X_NONE;
    s->open_step = 0;
    s->isr1 |= ISR1_RESET;
    udc_update_irq(s);
    udc_update_dreq(s);
}

static void host_fail(Pxa27xUdcState *s, const char *why)
{
    qemu_log_mask(LOG_GUEST_ERROR, "pxa27x-udc: host gave up at step %u: %s\n",
                  s->step, why);
    s->state = H_FAILED;
    s->phase = X_NONE;
}

static void host_setup(Pxa27xUdcState *s, uint8_t type, uint8_t req,
                       uint16_t value, uint16_t index, uint16_t length)
{
    uint8_t p[8] = { type, req, value & 0xff, value >> 8,
                     index & 0xff, index >> 8, length & 0xff, length >> 8 };
    memcpy(s->setup, p, 8);
    fifo8_reset(&s->ep0_rx);
    fifo8_push_all(&s->ep0_rx, p, 8);
    s->csr[0] |= CSR0_SA | CSR0_OPC;
    s->resp_len = 0;
    s->stalled = false;
    s->xfer_steps = 0;
    s->phase = X_SETUP;
}

/* One step of the current control transfer. Returns true when it ended. */
static bool host_xfer_step(Pxa27xUdcState *s)
{
    if (++s->xfer_steps > XFER_LIMIT) {
        host_fail(s, "control transfer timed out");
        return false;
    }
    if (s->csr[0] & CSR0_FST) {
        /* The guest stalled the request: the host sees a STALL. */
        s->csr[0] = (s->csr[0] & ~CSR0_FST) | CSR0_SST;
        fifo8_reset(&s->ep0_rx);
        s->stalled = true;
        s->phase = X_NONE;
        return true;
    }
    switch (s->phase) {
    case X_SETUP:
        if (s->csr[0] & CSR0_SA) {
            return false; /* not taken yet */
        }
        if (s->setup[0] & 0x80) {
            s->phase = X_IN;
        } else if (s->out_data_len) {
            fifo8_push_all(&s->ep0_rx, s->out_data, s->out_data_len);
            s->csr[0] |= CSR0_OPC;
            s->phase = X_OUT;
        } else {
            /* No data stage: the guest sends the status IN. */
            s->phase = X_STATUS_IN;
        }
        return false;
    case X_IN:
        if (s->ep0_ready) {
            uint32_t n = s->ep0_tx_len;
            if (s->resp_len + n <= sizeof(s->resp)) {
                memcpy(s->resp + s->resp_len, s->ep0_tx, n);
                s->resp_len += n;
            }
            s->ep0_tx_len = 0;
            s->ep0_ready = false;
            s->csr[0] &= ~CSR0_IPR;
            s->isr0 |= ISR0_EP0;
            udc_update_irq(s);
            uint16_t want = s->setup[6] | s->setup[7] << 8;
            if (n < EP0_SIZE || s->resp_len >= want) {
                s->csr[0] |= CSR0_OPC; /* zero-length status OUT */
                s->phase = X_STATUS_OUT;
            }
        }
        return false;
    case X_STATUS_OUT:
        if (!(s->csr[0] & CSR0_OPC)) {
            s->phase = X_NONE;
            return true;
        }
        return false;
    case X_OUT:
        if (!(s->csr[0] & CSR0_OPC)) {
            s->phase = X_STATUS_IN; /* data taken: now the status IN */
        }
        return false;
    case X_STATUS_IN:
        if (s->ep0_ready) {
            if (s->ep0_tx_len) {
                qemu_log_mask(LOG_GUEST_ERROR, "pxa27x-udc: status IN carried "
                              "%u bytes\n", s->ep0_tx_len);
            }
            s->ep0_tx_len = 0;
            s->ep0_ready = false;
            s->csr[0] &= ~CSR0_IPR;
            s->isr0 |= ISR0_EP0;
            udc_update_irq(s);
            s->phase = X_NONE;
            return true;
        }
        return false;
    case X_WAIT_ACK:
        return false; /* ended by the AREN write */
    case X_NONE:
        return true;
    }
    return true;
}

/* Learns the functions' endpoints from the configuration descriptor, as a
 * host's driver matching does: bulk endpoints of a CDC data interface
 * (class 0x0A) carry the serial port, those of a mass storage interface
 * (class 0x08, SCSI transparent, Bulk-Only) the drive. */
static void host_parse_config(Pxa27xUdcState *s)
{
    uint8_t cls = 0, sub = 0, proto = 0;
    int iface = -1;
    s->cdc_in_ep = s->cdc_out_ep = 0;
    s->msc_in_ep = s->msc_out_ep = 0;
    s->msc_iface = -1;
    for (uint32_t i = 0; i + 2 <= s->resp_len;) {
        uint8_t len = s->resp[i], type = s->resp[i + 1];
        if (len < 2 || i + len > s->resp_len) {
            break;
        }
        if (type == 4 && len >= 9) {
            iface = s->resp[i + 2];
            cls = s->resp[i + 5];
            sub = s->resp[i + 6];
            proto = s->resp[i + 7];
        } else if (type == 5 && len >= 7 && (s->resp[i + 3] & 3) == 2) {
            uint8_t addr = s->resp[i + 2];
            bool in = addr & 0x80;
            uint8_t num = addr & 0xF;
            if (cls == 0x0A) {
                *(in ? &s->cdc_in_ep : &s->cdc_out_ep) = num;
            } else if (cls == 0x08 && sub == 0x06 && proto == 0x50) {
                *(in ? &s->msc_in_ep : &s->msc_out_ep) = num;
                s->msc_iface = iface;
            }
        }
        i += len;
    }
}

/* The enumeration script: one control transfer per step number. */
static void host_script_start(Pxa27xUdcState *s)
{
    s->out_data_len = 0;
    switch (s->step) {
    case 0:
    case 2:                                                   /* reset (2: then address) */
        host_reset_bus(s);
        s->phase = X_NONE;
        s->wait_steps = RESET_RECOVERY_STEPS;
        break;
    case 1: host_setup(s, 0x80, 6, 0x0100, 0, 64); break;     /* device, 64 */
    case 3: host_setup(s, 0x80, 6, 0x0100, 0, 18); break;     /* device */
    case 4: host_setup(s, 0x80, 6, 0x0200, 0, 9); break;      /* config head */
    case 5: host_setup(s, 0x80, 6, 0x0200, 0, 255); break;    /* config */
    case 6: host_setup(s, 0x80, 6, 0x0300, 0, 255); break;    /* languages */
    case 7: host_setup(s, 0x80, 6, 0x0302, 0x409, 255); break;/* product */
    case 8: host_setup(s, 0x80, 6, 0x0600, 0, 10); break;     /* qualifier */
    case 9:
        /* SET_CONFIGURATION: answered by the hardware; the guest sees a
         * configuration-change interrupt and acknowledges with AREN. */
        s->udccr = (s->udccr & ~(3u << UDCCR_ACN_SHIFT))
                   | (uint32_t)(s->config_value & 3) << UDCCR_ACN_SHIFT;
        s->isr1 |= ISR1_CC;
        udc_update_irq(s);
        s->xfer_steps = 0;
        s->phase = X_WAIT_ACK;
        break;
    case 10:
        /* Get Max LUN, as Windows' storage driver asks first. */
        if (s->msc_iface >= 0) {
            host_setup(s, 0xA1, 0xFE, 0, s->msc_iface, 1);
        } else {
            s->phase = X_NONE;
        }
        break;
    default:
        s->state = H_RUN;
        s->phase = X_NONE;
        s->open_step = 0;
        qemu_log_mask(LOG_UNIMP, "pxa27x-udc: enumerated and configured "
                      "(serial %s, drive %s)\n",
                      s->cdc_in_ep ? "yes" : "no",
                      s->msc_iface >= 0 ? "yes" : "no");
        break;
    }
}

static void host_script_check(Pxa27xUdcState *s)
{
    switch (s->step) {
    case 1: case 3:
        if (s->resp_len < 8 || s->resp[1] != 1) {
            host_fail(s, "bad device descriptor");
        } else if (s->step == 3 && !s->accept_composite &&
                   s->resp_len >= 5 && s->resp[4] == 0xEF) {
            host_fail(s, "this host doesn't take composite devices");
        }
        break;
    case 4:
        if (s->resp_len >= 6 && s->resp[1] == 2) {
            s->config_value = s->resp[5];
        } else {
            host_fail(s, "bad configuration descriptor");
        }
        break;
    case 5:
        host_parse_config(s);
        break;
    case 8:
        if (!s->stalled) {
            host_fail(s, "a full-speed device must stall the qualifier");
        }
        break;
    case 10:
        if (s->msc_iface >= 0) {
            if (s->stalled || s->resp_len != 1) {
                /* Allowed by the spec for single-LUN devices. */
                s->max_lun = 0;
            } else {
                s->max_lun = s->resp[0];
            }
        }
        break;
    }
}

/* The serial port's open requests, run while the functions are live. */
static void host_open_step(Pxa27xUdcState *s)
{
    if (s->open_step == 0) {
        if (!s->chr_open) {
            return;
        }
        /* SET_LINE_CODING 115200 8N1, as usbser.sys does on open. */
        static const uint8_t coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 };
        memcpy(s->out_data, coding, 7);
        s->out_data_len = 7;
        host_setup(s, 0x21, 0x20, 0, 0, 7);
        s->open_step = 1;
        return;
    }
    if (s->open_step >= 3 || !host_xfer_step(s)) {
        return;
    }
    if (s->open_step == 1) {
        s->out_data_len = 0;
        host_setup(s, 0x21, 0x22, 3, 0, 0);        /* DTR+RTS */
        s->open_step = 2;
    } else {
        s->open_step = 3;
    }
}

/* IN token on slot x: the oldest waiting packet is taken; returns its
 * length or -1. */
static int host_take_in(Pxa27xUdcState *s, int x, uint8_t *out)
{
    if (x <= 0) {
        return -1;
    }
    BulkPkt *p = bulk_front(&s->bulk[x]);
    if (!p) {
        return -1;
    }
    int n = p->len;
    memcpy(out, p->d, n);
    bulk_pop(&s->bulk[x]);
    s->csr[x] |= CSR_PC;
    s->isr0 |= ISR0_PKT(x);
    udc_update_irq(s);
    udc_update_dreq(s);
    return n;
}

/* OUT token on slot x: accepted while a packet buffer is free. */
static bool host_can_out(Pxa27xUdcState *s, int x)
{
    return x > 0 && s->bulk[x].count < slot_depth(s, x);
}

static void host_put_out(Pxa27xUdcState *s, int x, const uint8_t *b,
                         uint32_t n)
{
    bulk_push(&s->bulk[x], slot_depth(s, x), b, n);
    s->isr0 |= ISR0_PKT(x);
    udc_update_irq(s);
    udc_update_dreq(s);
}

static void host_bridge_serial(Pxa27xUdcState *s)
{
    if (s->open_step < 3) {
        return; /* Windows polls the data pipe only once the port is open */
    }
    int in = find_slot(s, s->cdc_in_ep, true);
    int out = find_slot(s, s->cdc_out_ep, false);
    uint8_t p[BULK_SIZE];
    int n = host_take_in(s, in, p);
    if (n >= 0) {
        qemu_chr_fe_write_all(&s->chr, p, n);
    }
    if (host_can_out(s, out) && !fifo8_is_empty(&s->host_rx)) {
        uint32_t m = MIN(fifo8_num_used(&s->host_rx), BULK_SIZE);
        for (uint32_t i = 0; i < m; i++) {
            p[i] = fifo8_pop(&s->host_rx);
        }
        host_put_out(s, out, p, m);
        qemu_chr_fe_accept_input(&s->chr);
    }
}

static void host_bridge_msc(Pxa27xUdcState *s)
{
    if (s->msc_iface < 0 || !s->msc_open) {
        return;
    }
    int in = find_slot(s, s->msc_in_ep, true);
    int out = find_slot(s, s->msc_out_ep, false);
    uint8_t p[BULK_SIZE + 1];
    int n = host_take_in(s, in, p + 1);
    if (n >= 0) {
        p[0] = n;
        qemu_chr_fe_write_all(&s->msc, p, n + 1);
    }
    /* OUT: the next packet of the oldest transfer, once all of it (up to a
     * packet) has arrived, so packets never split early. */
    if (host_can_out(s, out) && s->msc_xfer_count) {
        uint32_t *left = &s->msc_xfer[s->msc_xfer_head];
        uint32_t need = MIN(*left, BULK_SIZE);
        if (fifo8_num_used(&s->msc_rx) >= need) {
            for (uint32_t i = 0; i < need; i++) {
                p[i] = fifo8_pop(&s->msc_rx);
            }
            host_put_out(s, out, p, need);
            *left -= need;
            if (*left == 0) {
                s->msc_xfer_head = (s->msc_xfer_head + 1) % MSC_XFERS;
                s->msc_xfer_count--;
            }
            qemu_chr_fe_accept_input(&s->msc);
        }
    }
}

/* The controller is on and the connector is ours: a host that hasn't seen
 * the device (first plug-in, or after an unplug) enumerates it; one that
 * has keeps its old device and does nothing (the pull-up never went). */
static void host_plug(Pxa27xUdcState *s)
{
    if (s->up2ocr & UP2OCR_HXS) {
        return;     /* the transceiver belongs to the host controller */
    }
    if (s->fixed_pullup && s->seen) {
        qemu_log_mask(LOG_UNIMP, "pxa27x-udc: controller on again, but the "
                      "pull-up never went away: the host keeps the old "
                      "device\n");
        return;
    }
    s->seen = true;
    s->state = H_ATTACH;
    s->attach_steps = 0;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + STEP_NS);
}

/* The host controller reset port 2 (named GPIO "port2-reset"): with the
 * transceiver on it, that drives SE0 and the host sees an unplug. */
static void udc_port2_reset(void *opaque, int n, int level)
{
    Pxa27xUdcState *s = opaque;

    if (level && (s->up2ocr & UP2OCR_HXS)) {
        s->seen = false;
        s->state = H_IDLE;
        s->phase = X_NONE;
        timer_del(s->timer);
        qemu_log_mask(LOG_UNIMP, "pxa27x-udc: SE0 on the connector: the host "
                      "sees an unplug\n");
    }
}

static void host_step(void *opaque)
{
    Pxa27xUdcState *s = opaque;

    switch (s->state) {
    case H_IDLE:
    case H_FAILED:
        break;
    case H_ATTACH:
        if (++s->attach_steps >= 100) { /* 10 ms after the pull-up */
            s->state = H_SCRIPT;
            s->step = 0;
            host_script_start(s);
        }
        break;
    case H_SCRIPT:
        if (s->wait_steps) {
            s->wait_steps--;
            break;
        }
        if (host_xfer_step(s)) {
            host_script_check(s);
            if (s->state == H_SCRIPT) {
                s->step++;
                host_script_start(s);
            }
        }
        break;
    case H_RUN:
        host_open_step(s);
        host_bridge_serial(s);
        host_bridge_msc(s);
        break;
    }
    if (s->state != H_IDLE) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + STEP_NS);
    }
}

/* ---- chardevs --------------------------------------------------------- */

static int udc_can_receive(void *opaque)
{
    Pxa27xUdcState *s = opaque;
    return s->state == H_RUN ? fifo8_num_free(&s->host_rx) : 0;
}

static void udc_event(void *opaque, QEMUChrEvent event)
{
    Pxa27xUdcState *s = opaque;

    if (event == CHR_EVENT_OPENED) {
        s->chr_open = true;
    } else if (event == CHR_EVENT_CLOSED) {
        s->chr_open = false;
    }
}

static void udc_receive(void *opaque, const uint8_t *buf, int size)
{
    Pxa27xUdcState *s = opaque;
    fifo8_push_all(&s->host_rx, buf, MIN((uint32_t)size,
                                         fifo8_num_free(&s->host_rx)));
}

static int msc_can_receive(void *opaque)
{
    Pxa27xUdcState *s = opaque;
    if (s->state != H_RUN || s->msc_xfer_count >= MSC_XFERS - 1) {
        return 0;
    }
    /* Header bytes take no FIFO space, so this never overfills it. */
    return fifo8_num_free(&s->msc_rx);
}

static void msc_event(void *opaque, QEMUChrEvent event)
{
    Pxa27xUdcState *s = opaque;

    if (event == CHR_EVENT_OPENED || event == CHR_EVENT_CLOSED) {
        s->msc_open = event == CHR_EVENT_OPENED;
        fifo8_reset(&s->msc_rx);
        s->msc_xfer_head = s->msc_xfer_count = 0;
        s->msc_hdr_len = 0;
        s->msc_frame_left = 0;
    }
}

static void msc_receive(void *opaque, const uint8_t *buf, int size)
{
    Pxa27xUdcState *s = opaque;
    for (int i = 0; i < size; i++) {
        if (s->msc_frame_left == 0) {
            s->msc_hdr[s->msc_hdr_len++] = buf[i];
            if (s->msc_hdr_len == 4) {
                uint32_t len = ldl_le_p(s->msc_hdr);
                s->msc_hdr_len = 0;
                if (len == 0 || s->msc_xfer_count >= MSC_XFERS) {
                    continue; /* nothing to send */
                }
                s->msc_frame_left = len;
                uint32_t tail = (s->msc_xfer_head + s->msc_xfer_count)
                                % MSC_XFERS;
                s->msc_xfer[tail] = len;
                s->msc_xfer_count++;
            }
            continue;
        }
        if (!fifo8_is_full(&s->msc_rx)) {
            fifo8_push(&s->msc_rx, buf[i]);
        }
        s->msc_frame_left--;
    }
}

/* ---- guest side ------------------------------------------------------- */

static void ep0_push(Pxa27xUdcState *s, const uint8_t *b, unsigned n)
{
    if (s->ep0_ready) {
        qemu_log_mask(LOG_GUEST_ERROR, "pxa27x-udc: wrote endpoint 0's FIFO "
                      "while a packet waits for the host\n");
        return;
    }
    for (unsigned i = 0; i < n && s->ep0_tx_len < EP0_SIZE; i++) {
        s->ep0_tx[s->ep0_tx_len++] = b[i];
    }
    if (s->ep0_tx_len >= EP0_SIZE) {
        s->ep0_ready = true;
    }
}

static void in_push(Pxa27xUdcState *s, int x, const uint8_t *b, unsigned n)
{
    BulkFifo *f = &s->bulk[x];
    if (f->count >= slot_depth(s, x) || bulk_short_waits(f)) {
        qemu_log_mask(LOG_GUEST_ERROR, "pxa27x-udc: wrote bulk IN slot %d "
                      "with no free buffer\n", x);
        return;
    }
    for (unsigned i = 0; i < n && f->fill.len < BULK_SIZE; i++) {
        f->fill.d[f->fill.len++] = b[i];
    }
    if (f->fill.len >= BULK_SIZE) {
        in_commit(s, x);
    }
    udc_update_dreq(s);
}

/* The active bulk slot a CSR/BCR/DR address belongs to, or -1. */
static int bulk_slot(Pxa27xUdcState *s, hwaddr addr, hwaddr base)
{
    if (addr < base + 4 || addr >= base + 4 * NEP) {
        return -1;
    }
    int x = (addr - base) / 4;
    return slot_active_bulk(s, x) ? x : -1;
}

static uint64_t udc_read(void *opaque, hwaddr addr, unsigned size)
{
    Pxa27xUdcState *s = opaque;
    int x;

    switch (addr) {
    case UDCCR:   return s->udccr;
    case UDCICR0: return s->icr0;
    case UDCICR1: return s->icr1;
    case UDCISR0: return s->isr0;
    case UDCISR1: return s->isr1;
    case UDCFNR:  return 0;
    case UP2OCR:  return s->up2ocr;
    case UP3OCR:  return s->up3ocr;
    }
    if (addr == CSR(0)) {
        return s->csr[0] | (fifo8_is_empty(&s->ep0_rx) ? 0 : CSR0_RNE);
    }
    if (addr >= CSR(1) && addr < CSR(NEP)) {
        int y = (addr - CSR(0)) / 4;
        uint32_t v = s->csr[y];
        if (slot_active_bulk(s, y)) {
            if (slot_is_in(s, y)) {
                if (in_room(s, y)) {
                    v |= CSR_BNX; /* not full */
                }
            } else {
                /* The packet being read: complete, short, not empty. */
                BulkPkt *p = bulk_front(&s->bulk[y]);
                v &= ~(CSR_PC | CSR_SP);
                if (p) {
                    v |= CSR_PC;
                    if (p->len < BULK_SIZE) {
                        v |= CSR_SP;
                    }
                    if (p->pos < p->len) {
                        v |= CSR_BNX;
                    }
                }
            }
        }
        return v;
    }
    if (addr == BCR(0)) {
        return fifo8_num_used(&s->ep0_rx);
    }
    if ((x = bulk_slot(s, addr, BCR(0))) > 0 && !slot_is_in(s, x)) {
        BulkPkt *p = bulk_front(&s->bulk[x]);
        return p ? p->len - p->pos : 0;
    }
    if (addr == DR(0)) {
        uint32_t v = 0;
        for (unsigned i = 0; i < size; i++) {
            uint8_t b = fifo8_is_empty(&s->ep0_rx) ? 0 : fifo8_pop(&s->ep0_rx);
            v |= (uint32_t)b << (8 * i);
        }
        return v;
    }
    if ((x = bulk_slot(s, addr, DR(0))) > 0 && !slot_is_in(s, x)) {
        BulkPkt *p = bulk_front(&s->bulk[x]);
        uint32_t v = 0;
        for (unsigned i = 0; i < size; i++) {
            uint8_t b = p && p->pos < p->len ? p->d[p->pos++] : 0;
            v |= (uint32_t)b << (8 * i);
        }
        /* Read to its end by DMA: released without the guest's PC (the
         * X51v's DMA moved 64 KiB with no CPU per packet). */
        if (p && p->pos >= p->len && (s->csr[x] & CSR_DME)) {
            bulk_pop(&s->bulk[x]);
        }
        udc_update_dreq(s);
        return v;
    }
    if (addr >= CR(1) && addr < CR(NEP)) {
        return s->cr[(addr - CR(0)) / 4];
    }
    return 0;
}

static void udc_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    Pxa27xUdcState *s = opaque;
    int x;

    switch (addr) {
    case UDCCR: {
        bool was = s->udccr & UDCCR_UDE;
        if (val & UDCCR_SMAC) {
            s->smac_count++;
        }
        s->udccr = (s->udccr & ~UDCCR_UDE) | (val & UDCCR_UDE);
        if (!was && (val & UDCCR_UDE)) {
            host_plug(s);
        } else if (was && !(val & UDCCR_UDE)) {
            s->state = H_IDLE;
            timer_del(s->timer);
        }
        return;
    }
    case UDCICR0: s->icr0 = val; udc_update_irq(s); return;
    case UDCICR1: s->icr1 = val; udc_update_irq(s); return;
    case UDCISR0: s->isr0 &= ~val; udc_update_irq(s); return;
    case UDCISR1: s->isr1 &= ~val; udc_update_irq(s); return;
    case UP2OCR: {
        bool to_host = s->up2ocr & UP2OCR_HXS;
        s->up2ocr = val;
        /* Port 2 back on the device controller after an unplug. */
        if (to_host && !(val & UP2OCR_HXS) && (s->udccr & UDCCR_UDE)) {
            host_plug(s);
        }
        return;
    }
    case UP3OCR:  s->up3ocr = val; return;
    }
    if (addr == CSR(0)) {
        s->csr[0] &= ~(val & (CSR0_OPC | CSR0_SA | CSR0_SST));
        if (val & CSR0_FTF) {
            s->ep0_tx_len = 0;
            s->ep0_ready = false;
        }
        if (val & CSR0_FST) {
            s->csr[0] |= CSR0_FST;
        }
        if (val & CSR0_IPR) {
            s->csr[0] |= CSR0_IPR;
            s->ep0_ready = true; /* short (or empty) packet */
        }
        if ((val & CSR0_AREN) && s->phase == X_WAIT_ACK) {
            s->phase = X_NONE; /* SET_CONFIGURATION acknowledged */
        }
        return;
    }
    if (addr >= CSR(1) && addr < CSR(NEP)) {
        int y = (addr - CSR(0)) / 4;
        bool bulk = slot_active_bulk(s, y);
        bool in = slot_is_in(s, y);
        /* DME takes the value of every write (drivers carry it). */
        s->csr[y] = (s->csr[y] & ~CSR_DME) | (val & CSR_DME);
        if (val & CSR_PC) {
            s->csr[y] &= ~CSR_PC;
            if (bulk && !in) {
                bulk_pop(&s->bulk[y]); /* packet released */
            }
        }
        if (bulk && in && (val & CSR_SP)) {
            in_commit(s, y);
        }
        if (bulk && (val & CSR_FEF)) {
            memset(&s->bulk[y], 0, sizeof(s->bulk[y]));
        }
        udc_update_dreq(s);
        return;
    }
    if (addr == DR(0)) {
        uint8_t b[4];
        for (unsigned i = 0; i < size; i++) {
            b[i] = val >> (8 * i);
        }
        ep0_push(s, b, size);
        return;
    }
    if ((x = bulk_slot(s, addr, DR(0))) > 0 && slot_is_in(s, x)) {
        uint8_t b[4];
        for (unsigned i = 0; i < size; i++) {
            b[i] = val >> (8 * i);
        }
        in_push(s, x, b, size);
        return;
    }
    if (addr >= CR(1) && addr < CR(NEP)) {
        if (s->udccr & UDCCR_UDE) {
            qemu_log_mask(LOG_GUEST_ERROR, "pxa27x-udc: endpoint table "
                          "written while enabled\n");
        }
        s->cr[(addr - CR(0)) / 4] = val;
        return;
    }
    qemu_log_mask(LOG_UNIMP, "pxa27x-udc: write 0x%03" HWADDR_PRIx
                  " = 0x%08" PRIx64 "\n", addr, val);
}

static const MemoryRegionOps udc_ops = {
    .read = udc_read,
    .write = udc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void udc_reset(DeviceState *dev)
{
    Pxa27xUdcState *s = PXA27X_UDC(dev);

    s->udccr = 0;
    s->icr0 = s->icr1 = s->isr0 = s->isr1 = 0;
    s->up2ocr = s->up3ocr = 0;
    memset(s->csr, 0, sizeof(s->csr));
    memset(s->cr, 0, sizeof(s->cr));
    fifo8_reset(&s->ep0_rx);
    fifo8_reset(&s->host_rx);
    fifo8_reset(&s->msc_rx);
    s->msc_xfer_head = s->msc_xfer_count = 0;
    s->msc_hdr_len = 0;
    s->msc_frame_left = 0;
    s->ep0_tx_len = 0;
    s->ep0_ready = false;
    memset(s->bulk, 0, sizeof(s->bulk));
    udc_update_dreq(s);
    s->msc_iface = -1;
    s->cdc_in_ep = s->cdc_out_ep = s->msc_in_ep = s->msc_out_ep = 0;
    s->state = H_IDLE;
    s->phase = X_NONE;
    s->open_step = 0;
    /* A board reset takes the pull-up's supply away (measured: the PC saw
     * a 14 s unplug): a fresh plug-in next time. */
    s->seen = false;
    timer_del(s->timer);
    qemu_set_irq(s->irq, 0);
}

static void udc_realize(DeviceState *dev, Error **errp)
{
    Pxa27xUdcState *s = PXA27X_UDC(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &udc_ops, s, "pxa27x-udc",
                          0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    qdev_init_gpio_out_named(dev, s->dreq, "dreq", NEP);
    qdev_init_gpio_in_named(dev, udc_port2_reset, "port2-reset", 1);
    fifo8_create(&s->ep0_rx, 64);
    fifo8_create(&s->host_rx, 256);
    fifo8_create(&s->msc_rx, MSC_RX);
    s->msc_iface = -1;
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, host_step, s);
    qemu_chr_fe_set_handlers(&s->chr, udc_can_receive, udc_receive, udc_event,
                             NULL, s, NULL, true);
    qemu_chr_fe_set_handlers(&s->msc, msc_can_receive, msc_receive, msc_event,
                             NULL, s, NULL, true);
    /* Without a chardev there is nothing to wait for. */
    s->chr_open = !qemu_chr_fe_backend_connected(&s->chr);
    s->msc_open = false;
}

static Property udc_properties[] = {
    DEFINE_PROP_CHR("chardev", Pxa27xUdcState, chr),
    DEFINE_PROP_CHR("msc", Pxa27xUdcState, msc),
    DEFINE_PROP_BOOL("accept-composite", Pxa27xUdcState, accept_composite,
                     true),
    DEFINE_PROP_BOOL("fixed-pullup", Pxa27xUdcState, fixed_pullup, true),
    DEFINE_PROP_END_OF_LIST(),
};

static void udc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = udc_realize;
    dc->reset = udc_reset;
    device_class_set_props(dc, udc_properties);
    dc->desc = "Intel PXA27x USB device controller (with a built-in host)";
}

static const TypeInfo udc_info = {
    .name = TYPE_PXA27X_UDC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Pxa27xUdcState),
    .class_init = udc_class_init,
};

static void udc_register_types(void)
{
    type_register_static(&udc_info);
}

type_init(udc_register_types)
