/*
 * Intel 2700G ("Marathon") multimedia accelerator: display subsystem model.
 *
 * Copyright (c) 2026 The RetroPane OS contributors
 *
 * This code is licensed under the GNU GPL v2 or later.
 *
 * Models what RetroPane OS needs: local memory, the register file, and
 * scan-out of the graphics and video planes, mixed by the display controller
 * (layer slots and blend modes), to a QEMU console. The register semantics
 * follow RetroPane's clean-room spec (retropane-os docs/hw/2700g.md, "Plane
 * composition"), which draws on the public Linux mbxfb register definitions.
 * The mixing arithmetic is the spec's exactly, so RetroPane's tests can
 * compare screendumps against a reference bit for bit.
 *
 * Not modelled (the registers just read back what was written): the MBX 3D
 * core, the MVED1 decoder, the hardware cursor, colour keys, gamma, the
 * video scaler (the video plane is shown 1:1) and the PLLs.
 *
 * The panel's backlight is not the 2700G's, but it decides what the screen
 * shows: the "backlight" GPIO input (the X51v's CPLD 0x18 bit 2) low shows
 * black with the picture kept in memory, as on the device. Tests read it
 * with `qom-get /machine/peripheral/mbx2700g backlight`.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/display/intel-2700g.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define REG_WINDOW      0x20000
#define NREGS           (0x4000 / 4)

/* Registers (byte offsets in the register block). */
#define R_PLLSTAT       0x0040
#define R_VOVRCLK       0x0044
#define R_ODFBPWR       0x0064
#define R_ODFBSTAT      0x0068
#define R_ID            0x0FF0
#define R_LMPWR         0x1008
#define R_LMPWRSTAT     0x100C
#define R_GSCTRL        0x2000
#define R_VSCTRL        0x2004
#define R_GBBASE        0x2020
#define R_VBBASE        0x2024
#define R_GSCADR        0x2060
#define R_VSCADR        0x2064
#define R_VUBASE        0x2084
#define R_VVBASE        0x20A4
#define R_GSADR         0x20C0
#define R_VSADR         0x20C4
#define R_DSCTRL        0x2154
#define R_BGCOLOR       0x2174
#define R_DHDET         0x2314
#define R_DVDET         0x2318

/* One layout for both planes: xSCTRL, xSCADR, xSADR, xBBASE. */
#define SCTRL_FMT(v)        (((v) >> 27) & 0xF)
#define SCTRL_WIDTH(v)      ((((v) >> 11) & 0x7FF) + 1)
#define SCTRL_HEIGHT(v)     (((v) & 0x7FF) + 1)
#define VSCTRL_CSC_EN       (1u << 25)
#define SCADR_STR_EN        (1u << 31)
#define SCADR_COLKEY_EN     (1u << 30)
#define SCADR_BLEND_M(v)    (((v) >> 27) & 0x3)
#define SCADR_BLEND_POS(v)  (((v) >> 24) & 0x3)
#define SCADR_BASE(v)       (((v) & 0x7FFFFF) * 16)
#define SADR_STRIDE(v)      (((((v) >> 22) & 0x3FF) + 1) * 16)
#define SADR_X(v)           (((v) >> 11) & 0x7FF)
#define SADR_Y(v)           ((v) & 0x7FF)
#define BBASE_GLALPHA(v)    (((v) >> 24) & 0xFF)
#define VUBASE_UVHALFSTR    (1u << 31)
#define CBASE(v)            ((uint64_t)((v) & 0xFFFFFF) * 8)
#define VOVRCLK_EN          (1u << 0)
#define DSCTRL_SYNCGEN_EN   (1u << 31)

enum {
    FMT_ARGB4444 = 0x4,
    FMT_ARGB1555 = 0x5,
    FMT_RGB888   = 0x6,
    FMT_RGB565   = 0x7,
    FMT_ARGB8888 = 0x8,
    /* Video plane. */
    FMT_YUV_PLANAR = 0x9,               /* 4:2:0 with UVHALFSTR, else 4:2:2 */
    FMT_UY0VY1   = 0xC,
    FMT_VY0UY1   = 0xD,
    FMT_Y0UY1V   = 0xE,
    FMT_Y0VY1U   = 0xF,
};

enum { BLEND_NONE, BLEND_INV, BLEND_GLOB, BLEND_PIX };

/* Panel default until the guest programs the plane: 3.7" VGA, portrait. */
#define DEFAULT_WIDTH   480
#define DEFAULT_HEIGHT  640

OBJECT_DECLARE_SIMPLE_TYPE(Intel2700GState, INTEL_2700G)

struct Intel2700GState {
    SysBusDevice parent_obj;

    MemoryRegion vram;
    MemoryRegion regs_mr;
    QemuConsole *con;
    uint32_t regs[NREGS];
    bool dark;      /* backlight input low; an input level, kept on reset */
};

static uint64_t i2700g_read(void *opaque, hwaddr addr, unsigned size)
{
    Intel2700GState *s = opaque;

    if (addr >= sizeof(s->regs)) {
        qemu_log_mask(LOG_UNIMP, "intel-2700g: read of 0x%05" HWADDR_PRIx
                      " (outside modelled registers)\n", addr);
        return 0;
    }
    switch (addr) {
    case R_PLLSTAT:
        return 0xFFFFFFFF;                  /* every PLL reports locked */
    case R_ODFBSTAT:
        return s->regs[R_ODFBPWR / 4];      /* power state follows request */
    case R_LMPWRSTAT:
        return s->regs[R_LMPWR / 4];
    default:
        return s->regs[addr / 4];
    }
}

static void i2700g_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    Intel2700GState *s = opaque;

    if (addr >= sizeof(s->regs)) {
        qemu_log_mask(LOG_UNIMP, "intel-2700g: write 0x%08" PRIx64 " to 0x%05"
                      HWADDR_PRIx " (outside modelled registers)\n", val, addr);
        return;
    }
    if (addr == R_ID) {
        return;                             /* read-only */
    }
    s->regs[addr / 4] = val;
}

static const MemoryRegionOps i2700g_ops = {
    .read = i2700g_read,
    .write = i2700g_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * One plane as scan-out sees it. A pixel is read as (r, g, b, a), 8 bits
 * each; then layers are mixed bottom (slot 0) to top over BGCOLOR:
 *   out = (src * k + out * (255 - k) + 127) / 255
 * with k = 255 (none), a (per-pixel), 255 - a (inverse) or the plane's
 * global alpha (xBBASE[31:24]).
 */
typedef struct Plane {
    bool on;
    int fmt;
    int bpp;                            /* bytes per pixel (Y for planar) */
    int x, y, width, height;            /* on screen */
    uint64_t base, stride;
    uint64_t ubase, vbase;              /* planar chroma */
    bool uvhalf;
    bool csc;
    int mode;
    int slot;
    int glalpha;
} Plane;

static int clamp8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* BT.601, studio range: Y 16-235, Cb/Cr 16-240. */
static void yuv_to_rgb(int y, int u, int v, int *r, int *g, int *b)
{
    int c = y - 16, d = u - 128, e = v - 128;

    *r = clamp8((298 * c + 409 * e + 128) >> 8);
    *g = clamp8((298 * c - 100 * d - 208 * e + 128) >> 8);
    *b = clamp8((298 * c + 516 * d + 128) >> 8);
}

static int bytes_per_pixel(int fmt)
{
    switch (fmt) {
    case FMT_ARGB4444:
    case FMT_ARGB1555:
    case FMT_RGB565:
    case FMT_UY0VY1:
    case FMT_VY0UY1:
    case FMT_Y0UY1V:
    case FMT_Y0VY1U:
        return 2;
    case FMT_RGB888:
        return 3;
    case FMT_ARGB8888:
        return 4;
    case FMT_YUV_PLANAR:
        return 1;                       /* the Y plane */
    default:
        return 0;
    }
}

static Plane plane_from_regs(Intel2700GState *s, bool video)
{
    uint32_t sctrl = s->regs[(video ? R_VSCTRL : R_GSCTRL) / 4];
    uint32_t scadr = s->regs[(video ? R_VSCADR : R_GSCADR) / 4];
    uint32_t sadr = s->regs[(video ? R_VSADR : R_GSADR) / 4];
    uint32_t bbase = s->regs[(video ? R_VBBASE : R_GBBASE) / 4];
    uint32_t vubase = s->regs[R_VUBASE / 4];
    Plane p = {
        .fmt = SCTRL_FMT(sctrl),
        .x = SADR_X(sadr),
        .y = SADR_Y(sadr),
        .width = SCTRL_WIDTH(sctrl),
        .height = SCTRL_HEIGHT(sctrl),
        .base = SCADR_BASE(scadr),
        .stride = SADR_STRIDE(sadr),
        .ubase = CBASE(vubase),
        .vbase = CBASE(s->regs[R_VVBASE / 4]),
        .uvhalf = vubase & VUBASE_UVHALFSTR,
        .csc = sctrl & VSCTRL_CSC_EN,
        .mode = SCADR_BLEND_M(scadr),
        .slot = SCADR_BLEND_POS(scadr),
        .glalpha = BBASE_GLALPHA(bbase),
    };
    bool video_fmt = p.fmt == FMT_YUV_PLANAR || p.fmt >= FMT_UY0VY1;
    uint64_t size = memory_region_size(&s->vram);

    p.bpp = bytes_per_pixel(p.fmt);
    p.on = (scadr & SCADR_STR_EN) && p.bpp != 0 && video_fmt == video;
    if (video && !(s->regs[R_VOVRCLK / 4] & VOVRCLK_EN)) {
        p.on = false;                   /* the overlay's clock is off */
    }
    if (p.on && (scadr & SCADR_COLKEY_EN)) {
        qemu_log_mask(LOG_UNIMP, "intel-2700g: colour keys not modelled\n");
    }
    /* Everything the plane reads must lie in local memory. */
    if (p.on && p.base + (uint64_t)(p.height - 1) * p.stride +
                (uint64_t)p.width * p.bpp > size) {
        p.on = false;
    }
    if (p.on && p.fmt == FMT_YUV_PLANAR) {
        uint64_t rows = p.uvhalf ? (p.height + 1) / 2 : p.height;
        uint64_t end = (rows - 1) * (p.stride / 2) + (p.width + 1) / 2;
        if (p.ubase + end > size || p.vbase + end > size) {
            p.on = false;
        }
    }
    return p;
}

/* Reads pixel (px, py) of the plane, in plane coordinates. */
static void plane_pixel(const Plane *p, const uint8_t *vram, int px, int py,
                        int *r, int *g, int *b, int *a)
{
    const uint8_t *q = vram + p->base + (uint64_t)py * p->stride;
    uint32_t v;
    int y, u, w;

    *a = 255;
    switch (p->fmt) {
    case FMT_ARGB4444:
        v = lduw_le_p(q + px * 2);
        *a = ((v >> 12) & 0xF) * 17;
        *r = ((v >> 8) & 0xF) * 17;
        *g = ((v >> 4) & 0xF) * 17;
        *b = (v & 0xF) * 17;
        return;
    case FMT_ARGB1555:
        v = lduw_le_p(q + px * 2);
        *a = (v & 0x8000) ? 255 : 0;
        *r = ((v >> 10) & 0x1F) << 3;
        *g = ((v >> 5) & 0x1F) << 3;
        *b = (v & 0x1F) << 3;
        return;
    case FMT_RGB565:
        v = lduw_le_p(q + px * 2);
        *r = ((v >> 11) & 0x1F) << 3;
        *g = ((v >> 5) & 0x3F) << 2;
        *b = (v & 0x1F) << 3;
        return;
    case FMT_RGB888:
        q += px * 3;
        *r = q[2];
        *g = q[1];
        *b = q[0];
        return;
    case FMT_ARGB8888:
        v = ldl_le_p(q + px * 4);
        *a = v >> 24;
        *r = (v >> 16) & 0xFF;
        *g = (v >> 8) & 0xFF;
        *b = v & 0xFF;
        return;
    case FMT_YUV_PLANAR: {
        uint64_t c = (uint64_t)(p->uvhalf ? py / 2 : py) * (p->stride / 2) +
                     px / 2;
        y = q[px];
        u = vram[p->ubase + c];
        w = vram[p->vbase + c];
        break;
    }
    default: {                          /* packed 4:2:2: two pixels a word */
        const uint8_t *m = q + (px & ~1) * 2;
        int odd = px & 1;

        switch (p->fmt) {
        case FMT_UY0VY1:
            u = m[0]; y = m[1 + 2 * odd]; w = m[2];
            break;
        case FMT_VY0UY1:
            w = m[0]; y = m[1 + 2 * odd]; u = m[2];
            break;
        case FMT_Y0UY1V:
            y = m[2 * odd]; u = m[1]; w = m[3];
            break;
        default:                        /* FMT_Y0VY1U */
            y = m[2 * odd]; w = m[1]; u = m[3];
            break;
        }
        break;
    }
    }
    if (p->csc) {
        yuv_to_rgb(y, u, w, r, g, b);
    } else {
        *r = y;                         /* no conversion: the raw bytes */
        *g = u;
        *b = w;
    }
}

static void i2700g_update(void *opaque)
{
    Intel2700GState *s = opaque;
    bool sync = s->regs[R_DSCTRL / 4] & DSCTRL_SYNCGEN_EN;
    uint32_t dhdet = s->regs[R_DHDET / 4], dvdet = s->regs[R_DVDET / 4];
    uint32_t bg = s->regs[R_BGCOLOR / 4];
    Plane planes[2] = { plane_from_regs(s, false), plane_from_regs(s, true) };
    const Plane *order[2];
    int n = 0;
    int width = DEFAULT_WIDTH, height = DEFAULT_HEIGHT;
    int hact = (int)(dhdet & 0xFFF) - (int)((dhdet >> 16) & 0xFFF);
    int vact = (int)(dvdet & 0xFFF) - (int)((dvdet >> 16) & 0xFFF);

    /*
     * The panel's active area comes from the display-enable window; failing
     * that, the graphics plane's size (a guest that only set up a plane).
     */
    if (hact > 0 && vact > 0 && hact <= 2048 && vact <= 2048) {
        width = hact;
        height = vact;
    } else if (planes[0].on) {
        width = planes[0].width;
        height = planes[0].height;
    }
    /* Bottom slot first; graphics before video within one slot. */
    for (int slot = 0; slot < 4; slot++) {
        for (int i = 0; i < 2; i++) {
            if (planes[i].on && planes[i].slot == slot) {
                order[n++] = &planes[i];
            }
        }
    }

    DisplaySurface *surface = qemu_console_surface(s->con);
    if (!surface || surface_width(surface) != width ||
        surface_height(surface) != height) {
        qemu_console_resize(s->con, width, height);
        surface = qemu_console_surface(s->con);
    }

    uint8_t *dst = surface_data(surface);
    int dst_stride = surface_stride(surface);
    uint8_t *vram = memory_region_get_ram_ptr(&s->vram);

    for (int y = 0; y < height; y++) {
        uint32_t *row = (uint32_t *)(dst + y * dst_stride);
        for (int x = 0; x < width; x++) {
            int r = (bg >> 16) & 0xFF, g = (bg >> 8) & 0xFF, b = bg & 0xFF;

            if (!sync || s->dark) {
                row[x] = 0;             /* display or backlight off: black */
                continue;
            }
            for (int i = 0; i < n; i++) {
                const Plane *p = order[i];
                int px = x - p->x, py = y - p->y;
                int sr, sg, sb, sa, k;

                if (px < 0 || py < 0 || px >= p->width || py >= p->height) {
                    continue;
                }
                plane_pixel(p, vram, px, py, &sr, &sg, &sb, &sa);
                switch (p->mode) {
                case BLEND_PIX:
                    k = sa;
                    break;
                case BLEND_INV:
                    k = 255 - sa;
                    break;
                case BLEND_GLOB:
                    k = p->glalpha;
                    break;
                default:
                    k = 255;
                    break;
                }
                r = (sr * k + r * (255 - k) + 127) / 255;
                g = (sg * k + g * (255 - k) + 127) / 255;
                b = (sb * k + b * (255 - k) + 127) / 255;
            }
            row[x] = rgb_to_pixel32(r, g, b);
        }
    }
    dpy_gfx_update_full(s->con);
}

static void i2700g_set_backlight(void *opaque, int n, int level)
{
    Intel2700GState *s = opaque;

    s->dark = !level;
}

static bool i2700g_get_backlight(Object *obj, Error **errp)
{
    return !INTEL_2700G(obj)->dark;
}

static void i2700g_invalidate(void *opaque)
{
}

static const GraphicHwOps i2700g_gfx_ops = {
    .invalidate = i2700g_invalidate,
    .gfx_update = i2700g_update,
};

static void i2700g_reset(DeviceState *dev)
{
    Intel2700GState *s = INTEL_2700G(dev);

    memset(s->regs, 0, sizeof(s->regs));
}

static void i2700g_realize(DeviceState *dev, Error **errp)
{
    Intel2700GState *s = INTEL_2700G(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (!memory_region_init_ram(&s->vram, OBJECT(dev), "intel-2700g.vram",
                                INTEL_2700G_VRAM_SIZE, errp)) {
        return;
    }
    memory_region_init_io(&s->regs_mr, OBJECT(dev), &i2700g_ops, s,
                          "intel-2700g.regs", REG_WINDOW);
    sysbus_init_mmio(sbd, &s->vram);
    sysbus_init_mmio(sbd, &s->regs_mr);
    s->con = graphic_console_init(dev, 0, &i2700g_gfx_ops, s);
    qdev_init_gpio_in_named(dev, i2700g_set_backlight, "backlight", 1);
    object_property_add_bool(OBJECT(dev), "backlight", i2700g_get_backlight,
                             NULL);
}

static const VMStateDescription vmstate_i2700g = {
    .name = "intel-2700g",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Intel2700GState, NREGS),
        VMSTATE_BOOL(dark, Intel2700GState),
        VMSTATE_END_OF_LIST()
    },
};

static void i2700g_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = i2700g_realize;
    dc->reset = i2700g_reset;
    dc->vmsd = &vmstate_i2700g;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo i2700g_info = {
    .name = TYPE_INTEL_2700G,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Intel2700GState),
    .class_init = i2700g_class_init,
};

static void i2700g_register_types(void)
{
    type_register_static(&i2700g_info);
}

type_init(i2700g_register_types)
