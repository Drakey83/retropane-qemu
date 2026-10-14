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
 * The MBX core's 2D engine runs solid fills and copies (clipped,
 * colour-keyed, in either direction) written to its command window
 * (see "The MBX core" below); reading an MBX offset that doesn't exist, or
 * one on the wrong block, is a bus error, as it hangs the device.
 *
 * Not modelled (the registers just read back what was written): the MBX 3D
 * core, the 2D engine's other operations (format conversion, scaling,
 * rotation, palettes, masks, blending), the MVED1 decoder, the hardware cursor, colour keys, gamma, the
 * video scaler (the video plane is shown 1:1) and the PLLs.
 *
 * The panel's backlight is not the 2700G's, but it decides what the screen
 * shows: the "backlight" GPIO input (the X51v's CPLD 0x18 bit 2) low shows
 * black with the picture kept in memory, as on the device. Tests read it
 * with `qom-get /machine/peripheral/mbx2700g backlight`.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/bitops.h"
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

    /* The MBX core: its registers and the 2D engine's command parser. */
    uint32_t mbx[0x1000 / 4];
    uint32_t mbx_state;
    int32_t mbx_skip;               /* words left to drop; -1: until the end */
    uint32_t mbx_dest, mbx_addr, mbx_colour, mbx_xy0, mbx_xy1;
    uint32_t mbx_src, mbx_src_addr, mbx_origin, mbx_op;
    uint32_t mbx_clip_x, mbx_clip_y, mbx_key, mbx_keymask;
};

/*
 * The MBX core (retropane-os docs/hw/2700g.md, "The MBX 2D engine"):
 * its register block at +0x4000 and the 2D engine's command window at
 * +0x15000. Commands run the moment their last word arrives, so the FIFO
 * always reads empty and a fence is written at once.
 */
#define MBX_BASE        0x4000
#define MBX_SIZE        0x1000
#define MBX_RESET       0x080
#define MBX_STATUS      0x12C
#define MBX_ENABLE      0x130
#define MBX_CLEAR       0x134
#define MBX_ID          0xF00
#define MBX_REV         0xF10
#define MBX_ID_VALUE    0x01020000      /* measured on the X51v */
#define MBX_REV_VALUE   0x00010000
#define MBX_2D_PORT     0x15000
#define MBX_2D_SIZE     0x1000

/* 2D command words. */
#define CMD_DEST        0xA             /* top nibble: destination surface */
#define CMD_END         0x70000000      /* end of batch */
#define CMD_KEY         0x20000001      /* colour key and mask follow */
#define ROP_PATCOPY     0xF0F0

/*
 * Offsets of the MBX block that exist (the ones Dell's driver touches).
 * Reading any other one is a bus error here; on the device unimplemented
 * addresses in the 2700G's register block hang the bus.
 */
static bool mbx_known(hwaddr off)
{
    return off == MBX_RESET || off == MBX_STATUS || off == MBX_ENABLE ||
           off == MBX_CLEAR || (off >= 0x300 && off <= 0x30C) ||
           (off >= 0x600 && off <= 0x6FC) || (off >= 0x800 && off <= 0x85C) ||
           off == MBX_ID || off == MBX_REV || off >= 0xFE0;
}

/*
 * Offsets of the MBX block that froze Windows Mobile when read on the
 * system block (rp-wmgpu v3-v6): a wrong base is caught, not read as 0.
 */
static bool system_block_freezes(hwaddr addr)
{
    return addr == MBX_STATUS || addr == 0x30C || addr == 0x85C ||
           addr == MBX_ID;
}

static int mbx_bpp(uint32_t dest)
{
    switch ((dest >> 15) & 0x1F) {
    case 0x6:
        return 1;
    case 0xA:
        return 2;                       /* RGB565 */
    case 0xB:
        return 3;
    case 0xC:
        return 4;
    default:
        return 0;
    }
}

/*
 * The command parser. A command starts with a word whose top nibble names
 * it; some take more words. State words (destination, source, origin,
 * clip, key) stay set until replaced; the operation header, its colour
 * (unless bit 16) and the destination rectangle draw.
 */
enum {
    MBX_IDLE,
    MBX_DEST_ADDR,
    MBX_SRC_ADDR,
    MBX_CLIP_X,
    MBX_CLIP_Y,
    MBX_KEY,
    MBX_KEY_MASK,
    MBX_COLOUR,
    MBX_XY0,
    MBX_XY1,
    MBX_SKIP,       /* drop mbx_skip words; -1: until CMD_END */
};

#define OP_NO_COLOUR    (1u << 16)
#define OP_CLIP         (1u << 18)
#define OP_KEY          (1u << 20)
/*
 * Bit 23 is Dell's driver's "y not positive", but on the X51v it does not
 * make a copy walk bottom-up (retropane-os docs/hw/2700g.md, measured
 * 2026-10-14: an 8 px downward overlap smears exactly as a top-down copy),
 * and the driver never sends it with a real overlap. Rows always run top
 * down here; the bit is accepted and has no effect.
 */
#define OP_BOTTOM_UP    (1u << 23)
#define OP_RIGHT_LEFT   (1u << 24)
#define OP_ROTATE       (3u << 25)
#define ROP_SRCCOPY     0xCCCC
#define SCALE_1_TO_1    0x60800200

static void mbx_skip(Intel2700GState *s, int words, const char *what, uint32_t w)
{
    if (what) {
        qemu_log_mask(LOG_UNIMP, "intel-2700g: 2D %s 0x%08x\n", what, w);
    }
    s->mbx_skip = words;
    s->mbx_state = words ? MBX_SKIP : MBX_IDLE;
}

/*
 * The engine decodes 32 MiB of addresses, wrapping (measured 2026-10-14):
 * local memory at 0-16 MiB, then 16 MiB that read as this constant in
 * every word; 32 MiB mirrors 0. No address reaches system RAM.
 */
#define MBX_DECODE      0x01FFFFFFu
#define MBX_UNMAPPED    0x1000008Cu

/*
 * The stride is 15-bit two's complement: a negative one walks up through
 * memory, which is how a copy down over itself runs bottom first in one
 * command (measured 2026-10-14: exact at shifts 1 and 8; strides
 * 0x7F80 = -128 and 0x7C40 = -960).
 */
static int32_t mbx_stride(uint32_t surf)
{
    return sextract32(surf, 0, 15);
}

static uint32_t mbx_at(uint32_t surf, uint32_t addr, int x, int y)
{
    return ((addr & 0x03FFFFFC) + (uint32_t)(y * mbx_stride(surf))
            + (uint32_t)x * mbx_bpp(surf)) & MBX_DECODE;
}

static uint8_t *mbx_pixel(uint8_t *vram, uint32_t surf, uint32_t addr,
                          int x, int y)
{
    uint32_t at = mbx_at(surf, addr, x, y);

    if (at + mbx_bpp(surf) > INTEL_2700G_VRAM_SIZE) {
        return NULL;
    }
    return vram + at;
}

/* A source pixel: local memory, or the constant past it. */
static uint32_t mbx_read(uint8_t *vram, uint32_t surf, uint32_t addr,
                         int x, int y)
{
    uint32_t at = mbx_at(surf, addr, x, y);
    uint32_t v = 0;

    for (int k = 0; k < mbx_bpp(surf); k++) {
        uint32_t b = at + k < INTEL_2700G_VRAM_SIZE ? vram[at + k]
                     : (MBX_UNMAPPED >> (8 * ((at + k) & 3))) & 0xFF;
        v |= b << (8 * k);
    }
    return v;
}

static void mbx_draw(Intel2700GState *s)
{
    uint8_t *vram = memory_region_get_ram_ptr(&s->vram);
    uint32_t op = s->mbx_op;
    int bpp = mbx_bpp(s->mbx_dest);
    int x0 = (s->mbx_xy0 >> 16) & 0x1FFF, y0 = s->mbx_xy0 & 0x1FFF;
    int x1 = (s->mbx_xy1 >> 16) & 0x1FFF, y1 = s->mbx_xy1 & 0x1FFF;
    int cx0 = x0, cy0 = y0, cx1 = x1, cy1 = y1;
    bool copy = (op & 0xFFFF) == ROP_SRCCOPY;

    if (!bpp) {
        qemu_log_mask(LOG_GUEST_ERROR, "intel-2700g: 2D destination 0x%08x:"
                      " unknown format\n", s->mbx_dest);
        return;
    }
    if ((op & 0xFFFF) != ROP_PATCOPY && !copy) {
        qemu_log_mask(LOG_UNIMP, "intel-2700g: 2D ROP 0x%04x\n", op & 0xFFFF);
        return;
    }
    if (op & OP_ROTATE) {
        qemu_log_mask(LOG_UNIMP, "intel-2700g: 2D rotation 0x%08x\n", op);
        return;
    }
    if (copy && mbx_bpp(s->mbx_src) != bpp) {
        qemu_log_mask(LOG_UNIMP, "intel-2700g: 2D copy with format conversion"
                      " (0x%08x to 0x%08x)\n", s->mbx_src, s->mbx_dest);
        return;
    }
    if (op & OP_CLIP) {                 /* right and bottom exclusive */
        cx0 = MAX(x0, (int)(s->mbx_clip_x & 0x1FFF));
        cx1 = MIN(x1, (int)((s->mbx_clip_x >> 16) & 0x1FFF));
        cy0 = MAX(y0, (int)(s->mbx_clip_y & 0x1FFF));
        cy1 = MIN(y1, (int)((s->mbx_clip_y >> 16) & 0x1FFF));
    }
    /*
     * Pixel by pixel, rows top down (OP_BOTTOM_UP has no effect, as on
     * the chip), right to left with OP_RIGHT_LEFT, reading the source as
     * it is at that moment: an overlapping copy in the wrong direction
     * smears, as it does on the chip.
     */
    for (int j = 0; j < cy1 - cy0; j++) {
        int y = cy0 + j;

        for (int i = 0; i < cx1 - cx0; i++) {
            int x = op & OP_RIGHT_LEFT ? cx1 - 1 - i : cx0 + i;
            uint8_t *d = mbx_pixel(vram, s->mbx_dest, s->mbx_addr, x, y);
            uint32_t v = s->mbx_colour;

            if (copy) {
                int sx = ((s->mbx_origin >> 14) & 0x1FFF) + (x - x0);
                int sy = (s->mbx_origin & 0x1FFF) + (y - y0);

                v = mbx_read(vram, s->mbx_src, s->mbx_src_addr, sx, sy);
                if ((op & OP_KEY) && (v & s->mbx_keymask) == s->mbx_key) {
                    continue;       /* the key colour: left as it was */
                }
            }
            if (!d) {
                /* Writes there were never measured: dropped, logged. */
                qemu_log_mask(LOG_UNIMP, "intel-2700g: 2D write past the end"
                              " of local memory\n");
                return;
            }
            for (int k = 0; k < bpp; k++) {
                d[k] = v >> (8 * k);
            }
        }
    }
}

static void mbx_command(Intel2700GState *s, uint32_t w)
{
    switch (s->mbx_state) {
    case MBX_IDLE:
        switch (w >> 28) {
        case 0x0:
            if (w == 1) {
                s->mbx_state = MBX_CLIP_X;
            } else {
                mbx_skip(s, -1, "command", w);
            }
            return;
        case 0x2:
            if (w == CMD_KEY) {
                s->mbx_state = MBX_KEY;
            } else {
                mbx_skip(s, (w & 0xF) ? 1 : 0, "state", w);
            }
            return;
        case 0x3:
            s->mbx_origin = w;
            return;
        case 0x6:
            if (w != SCALE_1_TO_1) {
                qemu_log_mask(LOG_UNIMP, "intel-2700g: 2D scaling 0x%08x\n", w);
            }
            return;
        case 0x7:                       /* end of batch */
            return;
        case 0x8:
            s->mbx_op = w;
            s->mbx_state = w & OP_NO_COLOUR ? MBX_XY0 : MBX_COLOUR;
            return;
        case 0x9:
            s->mbx_src = w;
            s->mbx_state = MBX_SRC_ADDR;
            return;
        case CMD_DEST:
            s->mbx_dest = w;
            s->mbx_state = MBX_DEST_ADDR;
            return;
        case 0xC:                       /* palettes, a mask: one address */
        case 0xD:
        case 0xE:
            mbx_skip(s, 1, "palette or mask", w);
            return;
        default:
            mbx_skip(s, -1, "command", w);
            return;
        }
    case MBX_DEST_ADDR:
        s->mbx_addr = w;
        break;
    case MBX_SRC_ADDR:
        s->mbx_src_addr = w;
        break;
    case MBX_CLIP_X:
        s->mbx_clip_x = w;
        s->mbx_state = MBX_CLIP_Y;
        return;
    case MBX_CLIP_Y:
        s->mbx_clip_y = w;
        break;
    case MBX_KEY:
        s->mbx_key = w;
        s->mbx_state = MBX_KEY_MASK;
        return;
    case MBX_KEY_MASK:
        s->mbx_keymask = w;
        s->mbx_key &= w;
        break;
    case MBX_COLOUR:
        s->mbx_colour = w;
        s->mbx_state = MBX_XY0;
        return;
    case MBX_XY0:
        s->mbx_xy0 = w;
        s->mbx_state = MBX_XY1;
        return;
    case MBX_XY1:
        s->mbx_xy1 = w;
        s->mbx_state = MBX_IDLE;
        mbx_draw(s);
        return;
    default:                            /* MBX_SKIP */
        if (s->mbx_skip > 0) {
            if (--s->mbx_skip == 0) {
                s->mbx_state = MBX_IDLE;
            }
        } else if (w == CMD_END) {
            s->mbx_state = MBX_IDLE;
        }
        return;
    }
    s->mbx_state = MBX_IDLE;
}

static MemTxResult i2700g_read(void *opaque, hwaddr addr, uint64_t *data,
                               unsigned size, MemTxAttrs attrs)
{
    Intel2700GState *s = opaque;

    if (addr >= MBX_BASE && addr < MBX_BASE + MBX_SIZE) {
        hwaddr off = addr - MBX_BASE;

        if (!mbx_known(off)) {
            qemu_log_mask(LOG_GUEST_ERROR, "intel-2700g: read of MBX 0x%03"
                          HWADDR_PRIx " (not implemented: the device's bus"
                          " hangs)\n", off);
            return MEMTX_ERROR;
        }
        switch (off) {
        case MBX_ID:
            *data = MBX_ID_VALUE;
            break;
        case MBX_REV:
            *data = MBX_REV_VALUE;
            break;
        case MBX_STATUS:                /* FIFO level 0: commands run at once */
            *data = s->mbx[off / 4] & ~0x40FF0000u;
            break;
        default:
            *data = off >= 0xFE0 ? 0 : s->mbx[off / 4];
            break;
        }
        return MEMTX_OK;
    }
    if (addr >= MBX_2D_PORT && addr < MBX_2D_PORT + MBX_2D_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "intel-2700g: read of the 2D command"
                      " window (write-only)\n");
        return MEMTX_ERROR;
    }
    if (system_block_freezes(addr)) {
        qemu_log_mask(LOG_GUEST_ERROR, "intel-2700g: read of 0x%03" HWADDR_PRIx
                      ": an MBX offset on the system block (froze the"
                      " device)\n", addr);
        return MEMTX_ERROR;
    }
    if (addr >= sizeof(s->regs)) {
        qemu_log_mask(LOG_UNIMP, "intel-2700g: read of 0x%05" HWADDR_PRIx
                      " (outside modelled registers)\n", addr);
        *data = 0;
        return MEMTX_OK;
    }
    switch (addr) {
    case R_PLLSTAT:
        *data = 0xFFFFFFFF;             /* every PLL reports locked */
        break;
    case R_ODFBSTAT:
        *data = s->regs[R_ODFBPWR / 4]; /* power state follows request */
        break;
    case R_LMPWRSTAT:
        *data = s->regs[R_LMPWR / 4];
        break;
    default:
        *data = s->regs[addr / 4];
        break;
    }
    return MEMTX_OK;
}

static MemTxResult i2700g_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size, MemTxAttrs attrs)
{
    Intel2700GState *s = opaque;

    if (addr >= MBX_BASE && addr < MBX_BASE + MBX_SIZE) {
        hwaddr off = addr - MBX_BASE;

        if (off == MBX_CLEAR) {
            s->mbx[MBX_STATUS / 4] &= ~(uint32_t)val;
        } else if (off != MBX_ID && off != MBX_REV && off != MBX_STATUS) {
            s->mbx[off / 4] = val;
        }
        return MEMTX_OK;
    }
    if (addr >= MBX_2D_PORT && addr < MBX_2D_PORT + MBX_2D_SIZE) {
        mbx_command(s, val);            /* any offset: the window is a FIFO */
        return MEMTX_OK;
    }
    if (addr >= sizeof(s->regs)) {
        qemu_log_mask(LOG_UNIMP, "intel-2700g: write 0x%08" PRIx64 " to 0x%05"
                      HWADDR_PRIx " (outside modelled registers)\n", val, addr);
        return MEMTX_OK;
    }
    if (addr == R_ID) {
        return MEMTX_OK;                /* read-only */
    }
    s->regs[addr / 4] = val;
    return MEMTX_OK;
}

static const MemoryRegionOps i2700g_ops = {
    .read_with_attrs = i2700g_read,
    .write_with_attrs = i2700g_write,
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
    memset(s->mbx, 0, sizeof(s->mbx));
    s->mbx_state = 0;
    s->mbx_skip = 0;
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
    .version_id = 3,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, Intel2700GState, NREGS),
        VMSTATE_BOOL(dark, Intel2700GState),
        VMSTATE_UINT32_ARRAY(mbx, Intel2700GState, 0x1000 / 4),
        VMSTATE_UINT32(mbx_state, Intel2700GState),
        VMSTATE_INT32(mbx_skip, Intel2700GState),
        VMSTATE_UINT32(mbx_dest, Intel2700GState),
        VMSTATE_UINT32(mbx_addr, Intel2700GState),
        VMSTATE_UINT32(mbx_colour, Intel2700GState),
        VMSTATE_UINT32(mbx_xy0, Intel2700GState),
        VMSTATE_UINT32(mbx_xy1, Intel2700GState),
        VMSTATE_UINT32(mbx_src, Intel2700GState),
        VMSTATE_UINT32(mbx_src_addr, Intel2700GState),
        VMSTATE_UINT32(mbx_origin, Intel2700GState),
        VMSTATE_UINT32(mbx_op, Intel2700GState),
        VMSTATE_UINT32(mbx_clip_x, Intel2700GState),
        VMSTATE_UINT32(mbx_clip_y, Intel2700GState),
        VMSTATE_UINT32(mbx_key, Intel2700GState),
        VMSTATE_UINT32(mbx_keymask, Intel2700GState),
        VMSTATE_END_OF_LIST()
    },
};

/* Present from rp-v0.12.0: tells tests the MBX 2D engine is modelled. */
static bool i2700g_get_mbx_2d(Object *obj, Error **errp)
{
    return true;
}

static void i2700g_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = i2700g_realize;
    dc->reset = i2700g_reset;
    dc->vmsd = &vmstate_i2700g;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    object_class_property_add_bool(klass, "mbx-2d", i2700g_get_mbx_2d, NULL);
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
