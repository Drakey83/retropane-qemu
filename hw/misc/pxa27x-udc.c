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
 * answers in hardware, then the CDC requests a terminal program sends on
 * open), and from then on bridges the guest's bulk endpoints to a chardev
 * (`-global pxa27x-udc.chardev=<id>`): bytes from the chardev become bulk OUT
 * packets, bulk IN packets go to the chardev.
 *
 * The host is strict where the hardware is: a packet waits in its FIFO until
 * the host's next IN token (one host step), a full FIFO is sent without IPR,
 * a host-to-device request ends with the guest's zero-length status IN (IPR,
 * measured on the real X51v), and writing into a FIFO that still holds a
 * packet is logged as a guest error. Problems are logged with
 * -d guest_errors.
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

/* Registers (docs/hw/udc.md). */
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
#define UDCCR_EMCE  (1 << 3)
#define UDCCR_SMAC  (1 << 4)
#define UDCCR_ACN_SHIFT 11

#define ISR1_RESET  (1u << 27)
#define ISR1_CC     (1u << 31)
#define ISR0_EP0    (1u << 0)

#define CSR0_OPC    (1 << 0)
#define CSR0_IPR    (1 << 1)
#define CSR0_FTF    (1 << 2)
#define CSR0_SST    (1 << 4)
#define CSR0_FST    (1 << 5)
#define CSR0_RNE    (1 << 6)
#define CSR0_SA     (1 << 7)
#define CSR0_AREN   (1 << 8)

#define CSR_PC      (1 << 1)
#define CSR_BNX     (1 << 6)
#define CSR_SP      (1 << 7)
#define CSR_FEF     (1 << 8)

#define EP0_SIZE    16
#define BULK_SIZE   64
/* Virtual time between host steps (one IN/OUT token per step). */
#define STEP_NS     (100 * 1000)
/* Steps a control transfer may take before the host gives up. */
#define XFER_LIMIT  2000

enum host_state {
    H_IDLE,         /* controller off: detached */
    H_ATTACH,       /* attached, about to reset */
    H_SCRIPT,       /* running the enumeration script */
    H_RUN,          /* configured, bridging bulk data */
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
    X_WAIT_OPEN,    /* waiting for a client on the chardev */
};

struct Pxa27xUdcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    CharBackend chr;
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

    /* Bulk: the OUT packet the guest is reading, the IN packet it is
     * building, and whether a built IN packet waits for the host. */
    uint8_t out_pkt[BULK_SIZE];
    uint32_t out_len, out_pos;
    uint8_t in_pkt[BULK_SIZE];
    uint32_t in_len;
    bool in_ready;
    /* Bytes from the chardev not yet sent as OUT packets. */
    Fifo8 host_rx;

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
    uint8_t config_value;
    uint32_t smac_count;
    /* A program has the port open: a client on the chardev (or no chardev
     * at all). Like Windows, the host sends the "open" requests (line
     * coding, DTR) only then, so nothing the guest sends is lost. */
    bool chr_open;
};

static void udc_update_irq(Pxa27xUdcState *s)
{
    qemu_set_irq(s->irq, (s->isr0 & s->icr0) || (s->isr1 & s->icr1));
}

static int find_slot(Pxa27xUdcState *s, bool in)
{
    for (int x = 1; x < NEP; x++) {
        uint32_t c = s->cr[x];
        bool enabled = c & 1;
        bool bulk = ((c >> 13) & 3) == 2;
        bool is_in = (c >> 12) & 1;
        if (enabled && bulk && is_in == in) {
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
    s->out_len = s->out_pos = 0;
    s->in_len = 0;
    s->in_ready = false;
    s->phase = X_NONE;
    s->isr1 |= ISR1_RESET;
    udc_update_irq(s);
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
    case X_WAIT_OPEN:
        s->xfer_steps = 0; /* no time limit: a person may open it any time */
        if (s->chr_open) {
            s->phase = X_NONE;
            s->step--; /* run the open requests now */
            return true;
        }
        return false;
    case X_NONE:
        return true;
    }
    return true;
}

/* The enumeration script: one control transfer per step number. */
static void host_script_start(Pxa27xUdcState *s)
{
    s->out_data_len = 0;
    switch (s->step) {
    case 0: host_reset_bus(s); s->phase = X_NONE; break;
    case 1: host_setup(s, 0x80, 6, 0x0100, 0, 64); break;     /* device, 64 */
    case 2: host_reset_bus(s); s->phase = X_NONE; break;      /* reset, address */
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
    case 10: {
        if (!s->chr_open) {
            /* Wait for a program to open the port. */
            s->phase = X_WAIT_OPEN;
            return;
        }
        /* SET_LINE_CODING 115200 8N1, as usbser.sys does on open. */
        static const uint8_t coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 };
        memcpy(s->out_data, coding, 7);
        s->out_data_len = 7;
        host_setup(s, 0x21, 0x20, 0, 0, 7);
        break;
    }
    case 11: host_setup(s, 0x21, 0x22, 3, 0, 0); break;        /* DTR+RTS */
    default:
        s->state = H_RUN;
        s->phase = X_NONE;
        qemu_log_mask(LOG_UNIMP, "pxa27x-udc: enumerated and configured\n");
        break;
    }
}

static void host_script_check(Pxa27xUdcState *s)
{
    switch (s->step) {
    case 1: case 3:
        if (s->resp_len < 8 || s->resp[1] != 1) {
            host_fail(s, "bad device descriptor");
        }
        break;
    case 4:
        if (s->resp_len >= 6 && s->resp[1] == 2) {
            s->config_value = s->resp[5];
        } else {
            host_fail(s, "bad configuration descriptor");
        }
        break;
    case 8:
        if (!s->stalled) {
            host_fail(s, "a full-speed device must stall the qualifier");
        }
        break;
    }
}

static void host_bridge_step(Pxa27xUdcState *s)
{
    int in = find_slot(s, true), out = find_slot(s, false);

    /* IN token: a waiting packet goes to the chardev. */
    if (in > 0 && s->in_ready) {
        qemu_chr_fe_write_all(&s->chr, s->in_pkt, s->in_len);
        s->in_len = 0;
        s->in_ready = false;
        s->csr[in] |= CSR_PC;
    }
    /* OUT token: the next chunk of chardev bytes, when the guest has
     * released the previous packet. */
    if (out > 0 && !(s->csr[out] & CSR_PC) && !fifo8_is_empty(&s->host_rx)) {
        uint32_t n = MIN(fifo8_num_used(&s->host_rx), BULK_SIZE);
        for (uint32_t i = 0; i < n; i++) {
            s->out_pkt[i] = fifo8_pop(&s->host_rx);
        }
        s->out_len = n;
        s->out_pos = 0;
        s->csr[out] |= CSR_PC;
        qemu_chr_fe_accept_input(&s->chr);
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
        if (host_xfer_step(s)) {
            host_script_check(s);
            if (s->state == H_SCRIPT) {
                s->step++;
                host_script_start(s);
            }
        }
        break;
    case H_RUN:
        host_bridge_step(s);
        break;
    }
    if (s->state != H_IDLE) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + STEP_NS);
    }
}

/* ---- chardev ---------------------------------------------------------- */

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

static void in_push(Pxa27xUdcState *s, const uint8_t *b, unsigned n)
{
    if (s->in_ready) {
        qemu_log_mask(LOG_GUEST_ERROR, "pxa27x-udc: wrote the bulk IN FIFO "
                      "while a packet waits for the host\n");
        return;
    }
    for (unsigned i = 0; i < n && s->in_len < BULK_SIZE; i++) {
        s->in_pkt[s->in_len++] = b[i];
    }
    if (s->in_len >= BULK_SIZE) {
        s->in_ready = true;
    }
}

static uint64_t udc_read(void *opaque, hwaddr addr, unsigned size)
{
    Pxa27xUdcState *s = opaque;
    int in = find_slot(s, true), out = find_slot(s, false);

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
        int x = (addr - CSR(0)) / 4;
        uint32_t v = s->csr[x];
        if (x == in && !s->in_ready && s->in_len < BULK_SIZE) {
            v |= CSR_BNX; /* not full */
        }
        if (x == out && s->out_pos < s->out_len) {
            v |= CSR_BNX; /* not empty */
        }
        return v;
    }
    if (addr == BCR(0)) {
        return fifo8_num_used(&s->ep0_rx);
    }
    if (out > 0 && addr == BCR(out)) {
        return s->out_len - s->out_pos;
    }
    if (addr == DR(0)) {
        uint32_t v = 0;
        for (unsigned i = 0; i < size; i++) {
            uint8_t b = fifo8_is_empty(&s->ep0_rx) ? 0 : fifo8_pop(&s->ep0_rx);
            v |= (uint32_t)b << (8 * i);
        }
        return v;
    }
    if (out > 0 && addr == DR(out)) {
        uint32_t v = 0;
        for (unsigned i = 0; i < size; i++) {
            uint8_t b = s->out_pos < s->out_len ? s->out_pkt[s->out_pos++] : 0;
            v |= (uint32_t)b << (8 * i);
        }
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
    int in = find_slot(s, true), out = find_slot(s, false);

    switch (addr) {
    case UDCCR: {
        bool was = s->udccr & UDCCR_UDE;
        if (val & UDCCR_SMAC) {
            s->smac_count++;
        }
        s->udccr = (s->udccr & ~UDCCR_UDE) | (val & UDCCR_UDE);
        if (!was && (val & UDCCR_UDE)) {
            s->state = H_ATTACH;
            s->attach_steps = 0;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + STEP_NS);
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
    case UP2OCR:  s->up2ocr = val; return;
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
        int x = (addr - CSR(0)) / 4;
        if (val & CSR_PC) {
            s->csr[x] &= ~CSR_PC;
            if (x == out) {
                s->out_len = s->out_pos = 0; /* packet released */
            }
        }
        if (x == in && (val & CSR_SP)) {
            s->in_ready = true;
        }
        if (val & CSR_FEF) {
            if (x == in) {
                s->in_len = 0;
                s->in_ready = false;
            }
            if (x == out) {
                s->out_len = s->out_pos = 0;
            }
        }
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
    if (in > 0 && addr == DR(in)) {
        uint8_t b[4];
        for (unsigned i = 0; i < size; i++) {
            b[i] = val >> (8 * i);
        }
        in_push(s, b, size);
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
    s->ep0_tx_len = 0;
    s->ep0_ready = false;
    s->out_len = s->out_pos = s->in_len = 0;
    s->in_ready = false;
    s->state = H_IDLE;
    s->phase = X_NONE;
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
    fifo8_create(&s->ep0_rx, 64);
    fifo8_create(&s->host_rx, 256);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, host_step, s);
    qemu_chr_fe_set_handlers(&s->chr, udc_can_receive, udc_receive, udc_event,
                             NULL, s, NULL, true);
    /* Without a chardev there is nothing to wait for. */
    s->chr_open = !qemu_chr_fe_backend_connected(&s->chr);
}

static Property udc_properties[] = {
    DEFINE_PROP_CHR("chardev", Pxa27xUdcState, chr),
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
