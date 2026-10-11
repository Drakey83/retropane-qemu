/*
 * Intel 2700G ("Marathon") multimedia accelerator: display subsystem model.
 *
 * Copyright (c) 2026 The RetroPane OS contributors
 *
 * This code is licensed under the GNU GPL v2 or later.
 *
 * Models what RetroPane OS needs first: local memory, the register file, and
 * scan-out of the graphics plane to a QEMU console. The register semantics
 * follow RetroPane's clean-room spec (retropane-os docs/hw/2700g.md), which
 * draws on the public Linux mbxfb register definitions:
 *
 *   GSCTRL  0x2000  [30:27] pixel format, [21:11] width-1, [10:0] height-1
 *   GSCADR  0x2060  [31] stream enable, [22:0] base address / 16
 *   GSADR   0x20C0  [31:22] stride / 16 - 1
 *   DSCTRL  0x2154  [31] sync generator enable
 *
 * The MBX 3D core, 2D blitter, video plane, cursor and MVED1 decoder are not
 * modelled yet; their registers just read back what was written.
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
#define R_ODFBPWR       0x0064
#define R_ODFBSTAT      0x0068
#define R_ID            0x0FF0
#define R_LMPWR         0x1008
#define R_LMPWRSTAT     0x100C
#define R_GSCTRL        0x2000
#define R_GSCADR        0x2060
#define R_GSADR         0x20C0
#define R_DSCTRL        0x2154

#define GSCTRL_FMT(v)       (((v) >> 27) & 0xF)
#define GSCTRL_WIDTH(v)     ((((v) >> 11) & 0x7FF) + 1)
#define GSCTRL_HEIGHT(v)    (((v) & 0x7FF) + 1)
#define GSCADR_STR_EN       (1u << 31)
#define GSCADR_BASE(v)      (((v) & 0x7FFFFF) * 16)
#define GSADR_STRIDE(v)     (((((v) >> 22) & 0x3FF) + 1) * 16)
#define DSCTRL_SYNCGEN_EN   (1u << 31)

enum {
    FMT_ARGB1555 = 0x5,
    FMT_RGB888   = 0x6,
    FMT_RGB565   = 0x7,
    FMT_ARGB8888 = 0x8,
};

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

static uint32_t pixel_to_rgb(int fmt, const uint8_t *p)
{
    uint32_t v;

    switch (fmt) {
    case FMT_RGB565:
        v = lduw_le_p(p);
        return rgb_to_pixel32(((v >> 11) & 0x1F) << 3, ((v >> 5) & 0x3F) << 2,
                              (v & 0x1F) << 3);
    case FMT_ARGB1555:
        v = lduw_le_p(p);
        return rgb_to_pixel32(((v >> 10) & 0x1F) << 3, ((v >> 5) & 0x1F) << 3,
                              (v & 0x1F) << 3);
    case FMT_RGB888:
        return rgb_to_pixel32(p[2], p[1], p[0]);
    case FMT_ARGB8888:
    default:
        v = ldl_le_p(p);
        return rgb_to_pixel32((v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF);
    }
}

static int bytes_per_pixel(int fmt)
{
    switch (fmt) {
    case FMT_RGB565:
    case FMT_ARGB1555:
        return 2;
    case FMT_RGB888:
        return 3;
    case FMT_ARGB8888:
        return 4;
    default:
        return 0;
    }
}

static void i2700g_update(void *opaque)
{
    Intel2700GState *s = opaque;
    uint32_t gsctrl = s->regs[R_GSCTRL / 4];
    uint32_t gscadr = s->regs[R_GSCADR / 4];
    uint32_t gsadr = s->regs[R_GSADR / 4];
    bool on = (s->regs[R_DSCTRL / 4] & DSCTRL_SYNCGEN_EN) &&
              (gscadr & GSCADR_STR_EN);
    int fmt = GSCTRL_FMT(gsctrl);
    int bpp = bytes_per_pixel(fmt);
    int width = on ? GSCTRL_WIDTH(gsctrl) : DEFAULT_WIDTH;
    int height = on ? GSCTRL_HEIGHT(gsctrl) : DEFAULT_HEIGHT;
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint8_t *vram = memory_region_get_ram_ptr(&s->vram);
    uint64_t vram_size = memory_region_size(&s->vram);

    if (!surface || surface_width(surface) != width ||
        surface_height(surface) != height) {
        qemu_console_resize(s->con, width, height);
        surface = qemu_console_surface(s->con);
    }

    uint8_t *dst = surface_data(surface);
    int dst_stride = surface_stride(surface);
    uint64_t base = GSCADR_BASE(gscadr);
    uint64_t stride = GSADR_STRIDE(gsadr);

    for (int y = 0; y < height; y++) {
        uint32_t *row = (uint32_t *)(dst + y * dst_stride);
        for (int x = 0; x < width; x++) {
            uint64_t off = base + y * stride + (uint64_t)x * bpp;
            if (!on || bpp == 0 || off + bpp > vram_size) {
                row[x] = 0;                 /* display off: black */
            } else {
                row[x] = pixel_to_rgb(fmt, vram + off);
            }
        }
    }
    dpy_gfx_update_full(s->con);
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
}

static const VMStateDescription vmstate_i2700g = {
    .name = "intel-2700g",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Intel2700GState, NREGS),
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
