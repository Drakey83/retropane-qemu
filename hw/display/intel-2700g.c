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
 * The MBX core's 2D engine runs solid fills, copies (clipped, colour-keyed,
 * in either direction, converting between formats) and blends, written to
 * its command window (see "The MBX core" below); reading an MBX offset that
 * doesn't exist, or one on the wrong block, is a bus error, as it hangs the
 * device. The display counts scan lines, sets its line and frame events and
 * raises its interrupt output ("irq", GPIO85 on the X51v) as measured.
 * The MBX 3D core renders its direct path (objects, lists and region
 * arrays the CPU writes) as measured (see "The MBX 3D core" below).
 *
 * Not modelled (the registers just read back what was written): the MBX 3D
 * core's tile accelerator, the 2D engine's other operations (scaling, rotation, palettes,
 * masks, ROPs other than copy and pattern copy), the MVED1 decoder, the
 * hardware cursor, colour keys, gamma, the video scaler (the video plane is
 * shown 1:1), scan-out under/overruns and the PLLs.
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
#include "qemu/timer.h"
#include "hw/irq.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/display/intel-2700g.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include <math.h>

#define REG_WINDOW      0x20000
#define NREGS           (0x4000 / 4)

/* Registers (byte offsets in the register block). */
#define R_MINTENABLE    0x0018
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
#define R_DINTRS        0x2178
#define R_DINTRE        0x217C
#define R_DVLNUM        0x2308
#define R_DVECTRL       0x2310
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

    /* The display's events: scan lines counted from the virtual clock. */
    qemu_irq irq;                   /* asserted high; GPIO85 is its inverse */
    QEMUTimer *ev_timer;
    int64_t ev_epoch;               /* virtual ns at line 0 of frame 0 */
    int64_t ev_line;                /* lines since then, events applied */

    /* The MBX core: its registers and the 2D engine's command parser. */
    uint32_t mbx[0x1000 / 4];
    uint32_t mbx_state;
    int32_t mbx_skip;               /* words left to drop; -1: until the end */
    uint32_t mbx_dest, mbx_addr, mbx_colour, mbx_xy0, mbx_xy1;
    uint32_t mbx_src, mbx_src_addr, mbx_origin, mbx_op;
    uint32_t mbx_clip_x, mbx_clip_y, mbx_key, mbx_keymask;
    uint32_t mbx_blend;

    /* The 3D core (see "The MBX 3D core" below). */
    int64_t m3_done_ns;             /* render done due; INT64_MIN: none */
    int64_t m3_ready_ns;            /* memory manager ready due */
    bool m3_pm_set;                 /* the memory manager was started */
    bool m3_hung;                   /* until a reset */
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
#define CMD_BLEND       0x20000004      /* the blend state word follows */
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

/*
 * 2D format codes (surface words, bits 15-19; retropane-os docs/hw/2700g.md,
 * measured 2026-10-14 by rp-mbx-blend).
 */
#define MBX_FMT(surf)       (((surf) >> 15) & 0x1F)
#define MBX_COVERAGE4       0x5         /* 4 bpp coverage, high nibble first */
#define MBX_ARGB4444        0x7
#define MBX_XRGB1555        0x8         /* 8 and 9: 1:5:5:5, read with alpha 0 */
#define MBX_XRGB1555_9      0x9
#define MBX_RGB565          0xA
#define MBX_ARGB8888        0xC         /* 13-15 act as 12 */

static int mbx_bpp(uint32_t dest)
{
    switch (MBX_FMT(dest)) {
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
    MBX_BLEND_WORD,
    MBX_COLOUR,
    MBX_XY0,
    MBX_XY1,
    MBX_SKIP,       /* drop mbx_skip words; -1: until CMD_END */
};

#define OP_NO_COLOUR    (1u << 16)
#define OP_BLEND        (1u << 17)      /* apply the blend state word */
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

/* Bits per source pixel (sources know more formats than destinations). */
static int mbx_src_bits(uint32_t surf)
{
    switch (MBX_FMT(surf)) {
    case MBX_COVERAGE4:
        return 4;
    case MBX_ARGB4444:
    case MBX_XRGB1555:
    case MBX_XRGB1555_9:
        return 16;
    case 0xD:
    case 0xE:
    case 0xF:
        return 32;
    default:
        return mbx_bpp(surf) * 8;
    }
}

/* A source pixel: local memory, or the constant past it. */
static uint32_t mbx_read(uint8_t *vram, uint32_t surf, uint32_t addr,
                         int x, int y)
{
    int bits = mbx_src_bits(surf);
    uint32_t at = ((addr & 0x03FFFFFC) + (uint32_t)(y * mbx_stride(surf))
                   + (uint32_t)x * bits / 8) & MBX_DECODE;
    uint32_t v = 0;

    for (int k = 0; k < MAX(bits / 8, 1); k++) {
        uint32_t b = at + k < INTEL_2700G_VRAM_SIZE ? vram[at + k]
                     : (MBX_UNMAPPED >> (8 * ((at + k) & 3))) & 0xFF;
        v |= b << (8 * k);
    }
    if (bits == 4) {
        v = x & 1 ? v & 0xF : v >> 4;   /* high nibble first (measured) */
    }
    return v;
}

/*
 * A pixel as 8-bit (a, r, g, b), channels widened by bit replication (as
 * measured: a 5-bit 31 blends as 255). A coverage pixel takes its colour
 * from the colour word, xRGB8888. False: a format with no known meaning.
 */
static bool mbx_argb(int fmt, uint32_t v, uint32_t colour, int c[4])
{
    switch (fmt) {
    case MBX_COVERAGE4:
        c[0] = v * 17;
        c[1] = (colour >> 16) & 0xFF;
        c[2] = (colour >> 8) & 0xFF;
        c[3] = colour & 0xFF;
        return true;
    case MBX_ARGB4444:
        for (int k = 0; k < 4; k++) {
            c[k] = ((v >> (12 - 4 * k)) & 0xF) * 17;
        }
        return true;
    case MBX_XRGB1555:
    case MBX_XRGB1555_9:
        c[0] = 0;
        for (int k = 1; k < 4; k++) {
            int f = (v >> (15 - 5 * k)) & 0x1F;
            c[k] = f << 3 | f >> 2;
        }
        return true;
    case MBX_RGB565: {
        int r = (v >> 11) & 0x1F, g = (v >> 5) & 0x3F, b = v & 0x1F;

        c[0] = 255;
        c[1] = r << 3 | r >> 2;
        c[2] = g << 2 | g >> 4;
        c[3] = b << 3 | b >> 2;
        return true;
    }
    case MBX_ARGB8888:
    case 0xD:
    case 0xE:
    case 0xF:
        for (int k = 0; k < 4; k++) {
            c[k] = (v >> (24 - 8 * k)) & 0xFF;
        }
        return true;
    default:
        return false;
    }
}

/* (a, r, g, b) to a destination pixel, truncated to its bits (measured). */
static bool mbx_pack(int fmt, const int c[4], uint32_t *v)
{
    switch (fmt) {
    case MBX_RGB565:
        *v = (c[1] >> 3) << 11 | (c[2] >> 2) << 5 | c[3] >> 3;
        return true;
    case MBX_ARGB8888:
        *v = (uint32_t)c[0] << 24 | c[1] << 16 | c[2] << 8 | c[3];
        return true;
    default:
        return false;
    }
}

/*
 * The blend (header bit 17), as measured on the X51v 2026-10-14 (rp-mbx-blend
 * B1-B3, every pixel exact): result = (s * Fs + d * Fd) >> 8 per channel,
 * saturated at 255, then truncated to the destination's bits. The blend
 * word's bits 27-24 pick Fd and 23-20 Fs; other bits changed nothing. Alpha
 * is 8-bit (4-bit values times 17) and weighs a + (a >> 7) out of 256.
 */
enum { F_ONE, F_A, F_ZERO, F_INV_A };
static const uint8_t mbx_factors[16] = {
    F_ONE, F_A, F_ZERO, F_A,            /* 3: "A" from another source? */
    F_ZERO, F_ZERO, F_ZERO, F_ZERO, F_ZERO,
    F_INV_A, F_ONE, F_INV_A, F_ONE, F_ONE, F_ONE, F_ONE,
};

static int mbx_factor(int code, int a)
{
    int w = a + (a >> 7);

    switch (mbx_factors[code & 0xF]) {
    case F_ONE:
        return 256;
    case F_A:
        return w;
    case F_ZERO:
        return 0;
    default:
        return 256 - w;
    }
}

static void mbx_blend(uint32_t word, const int src[4], int dst[4])
{
    int fs = mbx_factor(word >> 20, src[0]);
    int fd = mbx_factor(word >> 24, src[0]);

    /* The destination's own alpha (32 bpp) is kept: not measured. */
    for (int k = 1; k < 4; k++) {
        dst[k] = MIN((src[k] * fs + dst[k] * fd) >> 8, 255);
    }
}

/* How a pixel gets from the source (or colour word) to the destination. */
enum { MBX_SOLID, MBX_RAW, MBX_CONVERT, MBX_BLEND };

static void mbx_draw(Intel2700GState *s)
{
    uint8_t *vram = memory_region_get_ram_ptr(&s->vram);
    uint32_t op = s->mbx_op;
    uint32_t dest = s->mbx_dest;
    int bpp = mbx_bpp(dest);
    int x0 = (s->mbx_xy0 >> 16) & 0x1FFF, y0 = s->mbx_xy0 & 0x1FFF;
    int x1 = (s->mbx_xy1 >> 16) & 0x1FFF, y1 = s->mbx_xy1 & 0x1FFF;
    int cx0 = x0, cy0 = y0, cx1 = x1, cy1 = y1;
    int sfmt = MBX_FMT(s->mbx_src), dfmt = MBX_FMT(dest);
    int c[4], d[4];
    uint32_t packed;
    int how;

    if (!bpp) {
        qemu_log_mask(LOG_GUEST_ERROR, "intel-2700g: 2D destination 0x%08x:"
                      " unknown format\n", s->mbx_dest);
        return;
    }
    if ((op & 0xFFFF) != ROP_PATCOPY && (op & 0xFFFF) != ROP_SRCCOPY) {
        qemu_log_mask(LOG_UNIMP, "intel-2700g: 2D ROP 0x%04x\n", op & 0xFFFF);
        return;
    }
    if (op & OP_ROTATE) {
        qemu_log_mask(LOG_UNIMP, "intel-2700g: 2D rotation 0x%08x\n", op);
        return;
    }
    /*
     * With bit 17 the engine reads the source whatever the ROP (measured: a
     * fill with it blended the last source surface, not its colour).
     * Without it, copies convert between formats and ignore alpha, except
     * that a coverage source writes 32-bit pixels, coverage x 17 as alpha
     * over the colour word, whatever the destination's format (measured:
     * the 16 bpp test row read back as eight such pixels).
     */
    if (op & OP_BLEND) {
        how = MBX_BLEND;
        if (!mbx_argb(sfmt, 0, 0, c) || !mbx_pack(dfmt, c, &packed)) {
            qemu_log_mask(LOG_UNIMP, "intel-2700g: 2D blend from 0x%08x to"
                          " 0x%08x\n", s->mbx_src, dest);
            return;
        }
    } else if ((op & 0xFFFF) != ROP_SRCCOPY) {
        how = MBX_SOLID;
    } else if (sfmt == MBX_COVERAGE4) {
        how = MBX_CONVERT;
        dest = (dest & ~(0x1Fu << 15)) | MBX_ARGB8888 << 15;
        dfmt = MBX_ARGB8888;
        bpp = 4;
    } else if (sfmt == dfmt) {
        how = MBX_RAW;
    } else if (mbx_argb(sfmt, 0, 0, c) && mbx_pack(dfmt, c, &packed)) {
        how = MBX_CONVERT;
    } else if (mbx_src_bits(s->mbx_src) == bpp * 8) {
        how = MBX_RAW;                  /* formats not known: copied as bits */
    } else {
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
            uint8_t *p = mbx_pixel(vram, dest, s->mbx_addr, x, y);
            uint32_t v = s->mbx_colour;

            if (how != MBX_SOLID) {
                int sx = ((s->mbx_origin >> 14) & 0x1FFF) + (x - x0);
                int sy = (s->mbx_origin & 0x1FFF) + (y - y0);

                v = mbx_read(vram, s->mbx_src, s->mbx_src_addr, sx, sy);
                if ((op & OP_KEY) && (v & s->mbx_keymask) == s->mbx_key) {
                    continue;       /* the key colour: left as it was */
                }
            }
            if (!p) {
                /* Writes there were never measured: dropped, logged. */
                qemu_log_mask(LOG_UNIMP, "intel-2700g: 2D write past the end"
                              " of local memory\n");
                return;
            }
            if (how == MBX_CONVERT || how == MBX_BLEND) {
                mbx_argb(sfmt, v, s->mbx_colour, c);
                if (how == MBX_BLEND) {
                    uint32_t old = 0;

                    for (int k = 0; k < bpp; k++) {
                        old |= (uint32_t)p[k] << (8 * k);
                    }
                    mbx_argb(dfmt, old, 0, d);
                    mbx_blend(s->mbx_blend, c, d);
                    memcpy(c, d, sizeof(c));
                }
                mbx_pack(dfmt, c, &v);
            }
            for (int k = 0; k < bpp; k++) {
                p[k] = v >> (8 * k);
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
            } else if (w == CMD_BLEND) {
                s->mbx_state = MBX_BLEND_WORD;
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
    case MBX_BLEND_WORD:                /* kept until replaced, as on the chip */
        s->mbx_blend = w;
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

/*
 * The display's events (retropane-os docs/hw/2700g.md, measured on the X51v
 * 2026-10-14 by rp-mbx-blend): 645 lines a frame (0-644) at 27.46 us a line,
 * 56.46 Hz. DINTRS stays a plain read/write register (a 1 written sets a
 * bit, as on the chip); the hardware sets HBLNK0 (bit 0) as each line
 * starts and VEVENT0 (bit 2) as the line in DVECTRL bits 16-27 starts.
 * HBLNK1 and VEVENT1 never set; neither do the under/overrun bits here.
 * The interrupt (GPIO85 on the X51v, low when asserted) is asserted while
 * an event enabled in DINTRE is set and MINTENABLE bit 15 is on.
 * Events are applied when the registers are touched or a timer, armed only
 * for an enabled event not yet set, fires: no per-line timer otherwise.
 */
#define LINE_NS             27460
#define FRAME_LINES         645
#define EV_HBLNK0           (1u << 0)
#define EV_VEVENT0          (1u << 2)
#define DVECTRL_VLINE(v)    (((v) >> 16) & 0xFFF)
#define MINT_DISPLAY        (1u << 15)

static int64_t i2700g_line(Intel2700GState *s)
{
    return (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->ev_epoch) / LINE_NS;
}

/* The first line after `after` (lines since the epoch) that is `vline`. */
static int64_t i2700g_next_vline(int64_t after, uint32_t vline)
{
    int64_t at = after - after % FRAME_LINES + vline;

    if (vline >= FRAME_LINES) {
        return -1;                      /* never reached */
    }
    return at > after ? at : at + FRAME_LINES;
}

static void i2700g_events(Intel2700GState *s)
{
    int64_t now = i2700g_line(s);
    int64_t v = i2700g_next_vline(s->ev_line,
                                  DVECTRL_VLINE(s->regs[R_DVECTRL / 4]));

    if (now <= s->ev_line) {
        return;
    }
    s->regs[R_DINTRS / 4] |= EV_HBLNK0;
    if (v >= 0 && v <= now) {
        s->regs[R_DINTRS / 4] |= EV_VEVENT0;
    }
    s->ev_line = now;
}

static void i2700g_irq_update(Intel2700GState *s)
{
    uint32_t status = s->regs[R_DINTRS / 4], enable = s->regs[R_DINTRE / 4];
    bool on = s->regs[R_MINTENABLE / 4] & MINT_DISPLAY;
    uint32_t awaited = on ? enable & ~status : 0;
    int64_t next = -1;

    qemu_set_irq(s->irq, on && (status & enable));
    if (awaited & EV_HBLNK0) {
        next = s->ev_line + 1;
    }
    if (awaited & EV_VEVENT0) {
        int64_t v = i2700g_next_vline(s->ev_line,
                                      DVECTRL_VLINE(s->regs[R_DVECTRL / 4]));

        if (v >= 0 && (next < 0 || v < next)) {
            next = v;
        }
    }
    if (next >= 0) {
        timer_mod(s->ev_timer, s->ev_epoch + next * LINE_NS);
    } else {
        timer_del(s->ev_timer);
    }
}

static void i2700g_ev_tick(void *opaque)
{
    Intel2700GState *s = opaque;

    i2700g_events(s);
    i2700g_irq_update(s);
}

/*
 * The MBX 3D core's direct path (retropane-os ADR-0024/ADR-0026,
 * docs/hw/2700g.md "The scene bytes"), as measured on the X51v by rp-mbx3d
 * versions 3-23 and rp-mbx3d-api runs 1-3. A port of rp-mbx3d-render's
 * host model (drivers/mbx3d-render/src/model.rs): the two must agree, so a
 * change here goes there too.
 *
 * Writing 1 to 0x680 renders the region array at 0x608: two words a tile
 * (x | y << 8, bit 31 on the last; the tile's object list), each 8 x 16
 * tile written whole into the target at 0x65C (stride 0x660 words),
 * uncovered pixels white, unlisted tiles untouched. Lists hold strip
 * pointers (0x40800000 flat 17-word objects, 0x41000000 textured 26-word
 * ones, offsets in words from 0x60C) up to an 0xF... end word. Flat shading
 * takes the first vertex's colour, TSP bit 22 Gouraud; textures are
 * twiddled, point sampled, TEX0 bits 15-13 the format, 27-24/23-20 the
 * sides, 31-28 the address modes (wrap, mirror, clamp); TSP bit 13 blends
 * with the factors in bits 31-26 (each term value x factor >> 8, alpha
 * factors a and 256 - a); TSP bits 11-9 compare depth (larger z nearer;
 * "greater", "greater or equal", "always"), bit 7 set = no write; each
 * tile's depth starts at 0x6DC as a float if its bit 0 is set, else
 * undefined (no compare passes but "always"). Render done (status bit 2)
 * comes ~10 us + 5.25 us a tile + a quad's cost a tile later (fitted to
 * run 2); the parameter memory manager says ready (bit 6) 164 us after
 * 0x838 = 1, only with the 3D clock on (MBXCLK bits 1-0 = 2); without both
 * a start does nothing. TSP bits 16 and 19 hang the core until a reset.
 *
 * Assumed (not measured): pixel centres at +0.5 with a top-left fill rule,
 * Gouraud and modulate rounding, both windings drawn, 0x654/0x658 not
 * clipping. The core keeps its state across the 3D clock off (T9, measured:
 * rp-mbx3d-api run 4).
 * "Less" and "less or equal" never passed on the device (and hid objects
 * in partial renders, not modelled): here they never pass. Not modelled:
 * the tile accelerator, the interrupt line (render done never edged
 * GPIO85), filtering, depth across renders (0x674).
 */
#define R_MBXCLK            0x0054
#define M3_START            0x680
#define M3_PM_GO            0x838
#define M3_RENDER_DONE      (1u << 2)
#define M3_PM_READY         (1u << 6)
#define M3_LAST             (1u << 31)
#define M3_ADDR_MASK        0x01FFFFFCu
#define M3_STRIP_FLAT       0x40800000u
#define M3_STRIP_TEX        0x41000000u
#define M3_TSP_GOURAUD      (1u << 22)
#define M3_TSP_BLEND        (1u << 13)
#define M3_TSP_NO_WRITE     (1u << 7)
#define M3_TSP_HANGS        ((1u << 16) | (1u << 19))
#define M3_TILE_W           8
#define M3_TILE_H           16
#define M3_NONE             INT64_MIN

typedef struct M3Vertex {
    double x, y, z, u, v;
    uint32_t argb;
} M3Vertex;

typedef struct M3Obj {
    uint32_t tsp;
    bool textured;
    uint32_t tex[3];
    M3Vertex v[3];
} M3Obj;

typedef struct M3Tile {
    uint32_t x, y;
    uint32_t c[M3_TILE_W * M3_TILE_H];
    double z[M3_TILE_W * M3_TILE_H];
} M3Tile;

static uint32_t m3_word(Intel2700GState *s, uint32_t a)
{
    a &= M3_ADDR_MASK;
    if (a + 4 > INTEL_2700G_VRAM_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "intel-2700g: 3D read past local"
                      " memory at 0x%08x\n", a);
        return 0;
    }
    return ldl_le_p((uint8_t *)memory_region_get_ram_ptr(&s->vram) + a);
}

static double m3_float(uint32_t w)
{
    float f;

    memcpy(&f, &w, sizeof(f));
    return f;
}

static void m3_object(Intel2700GState *s, uint32_t at, bool textured,
                      M3Obj *o)
{
    int k;

    o->textured = textured;
    if (textured) {
        o->tsp = m3_word(s, at + 4 * 4);
        for (k = 0; k < 3; k++) {
            o->tex[k] = m3_word(s, at + 4 * (1 + k));
            o->v[k].x = m3_float(m3_word(s, at + 4 * (5 + 2 * k)));
            o->v[k].y = m3_float(m3_word(s, at + 4 * (6 + 2 * k)));
            o->v[k].z = m3_float(m3_word(s, at + 4 * (11 + k)));
            o->v[k].argb = m3_word(s, at + 4 * (17 + 3 * k));
            o->v[k].u = m3_float(m3_word(s, at + 4 * (18 + 3 * k)));
            o->v[k].v = m3_float(m3_word(s, at + 4 * (19 + 3 * k)));
        }
    } else {
        o->tsp = m3_word(s, at + 4);
        for (k = 0; k < 3; k++) {
            o->v[k].x = m3_float(m3_word(s, at + 4 * (2 + 2 * k)));
            o->v[k].y = m3_float(m3_word(s, at + 4 * (3 + 2 * k)));
            o->v[k].z = m3_float(m3_word(s, at + 4 * (8 + k)));
            o->v[k].argb = m3_word(s, at + 4 * (14 + k));
            o->v[k].u = o->v[k].v = 0;
        }
    }
}

static uint32_t m3_twiddle(uint32_t x, uint32_t y)
{
    uint32_t r = 0;
    int b;

    for (b = 0; b < 16; b++) {
        r |= ((y >> b) & 1) << (2 * b) | ((x >> b) & 1) << (2 * b + 1);
    }
    return r;
}

static int64_t m3_address(double c, int64_t side, uint32_t mode)
{
    int64_t i = (int64_t)floor(c * side), p;

    switch (mode) {
    case 0:                             /* wrap */
        return ((i % side) + side) % side;
    case 1:                             /* mirror */
        p = ((i % (2 * side)) + 2 * side) % (2 * side);
        return p >= side ? 2 * side - 1 - p : p;
    case 2:                             /* clamp */
        return i < 0 ? 0 : i > side - 1 ? side - 1 : i;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "intel-2700g: texture address mode 3"
                      " (broke sampling on the device)\n");
        return 0;
    }
}

static uint32_t m3_wide(uint32_t c, int bits)
{
    switch (bits) {
    case 5:
        return c << 3 | c >> 2;
    case 6:
        return c << 2 | c >> 4;
    case 4:
        return c * 17;
    case 3:
        return c << 5 | c << 2 | c >> 1;
    default:                            /* 2 */
        return c * 85;
    }
}

static uint32_t m3_argb(uint32_t a, uint32_t r, uint32_t g, uint32_t b)
{
    return a << 24 | r << 16 | g << 8 | b;
}

static uint32_t m3_sample(Intel2700GState *s, const uint32_t tex[3],
                          double u, double v)
{
    uint32_t tex0 = tex[0];
    int64_t su = 8ll << ((tex0 >> 24) & 15), sv = 8ll << ((tex0 >> 20) & 15);
    uint32_t x = m3_address(u, su, tex0 >> 30);
    uint32_t y = m3_address(v, sv, (tex0 >> 28) & 3);
    uint32_t a = ((tex[1] & 0x01FFFFFF) << 7) + 2 * m3_twiddle(x, y);
    uint32_t t = m3_word(s, a & ~3u) >> (8 * (a & 2)) & 0xFFFF;

    switch ((tex0 >> 13) & 7) {
    case 0:                             /* ARGB1555 */
        return m3_argb(t & 0x8000 ? 255 : 0, m3_wide((t >> 10) & 31, 5),
                       m3_wide((t >> 5) & 31, 5), m3_wide(t & 31, 5));
    case 2:                             /* RGB565 */
        return m3_argb(255, m3_wide(t >> 11, 5), m3_wide((t >> 5) & 63, 6),
                       m3_wide(t & 31, 5));
    case 4:                             /* ARGB4444 */
        return m3_argb(m3_wide(t >> 12, 4), m3_wide((t >> 8) & 15, 4),
                       m3_wide((t >> 4) & 15, 4), m3_wide(t & 15, 4));
    case 6:                             /* RGB332 in the low byte */
        return m3_argb(255, m3_wide((t >> 5) & 7, 3),
                       m3_wide((t >> 2) & 7, 3), m3_wide(t & 3, 2));
    default:
        qemu_log_mask(LOG_UNIMP, "intel-2700g: texel format %u not decoded"
                      " on the device\n", (tex0 >> 13) & 7);
        return 0xFF000000;
    }
}

static uint32_t m3_modulate(uint32_t t, uint32_t c)
{
    uint32_t r = 0;
    int i;

    for (i = 0; i < 32; i += 8) {
        r |= ((((t >> i) & 0xFF) * ((c >> i) & 0xFF) + 127) / 255) << i;
    }
    return r;
}

static uint32_t m3_blend(uint32_t tsp, uint32_t src, uint32_t dst)
{
    uint32_t fs = (tsp >> 29) & 7, fd = (tsp >> 26) & 7, r = 0;
    bool on = tsp & M3_TSP_BLEND;
    uint32_t sa = on ? src >> 24 : 256, da = on ? dst >> 24 : 256;
    int i;

    for (i = 0; i < 32; i += 8) {
        uint32_t sc = (src >> i) & 0xFF, dc = (dst >> i) & 0xFF;
        uint32_t f[2], code[2] = { fs, fd }, other[2] = { dc, sc }, v;
        int k;

        for (k = 0; k < 2; k++) {
            switch (code[k]) {
            case 0:
                f[k] = 0;
                break;
            case 1:
                f[k] = 256;
                break;
            case 2:
                f[k] = other[k];
                break;
            case 3:
                f[k] = 256 - other[k];
                break;
            case 4:
                f[k] = sa;
                break;
            case 5:
                f[k] = 256 - sa;
                break;
            case 6:
                f[k] = da;
                break;
            default:
                f[k] = 256 - da;
                break;
            }
        }
        v = ((sc * f[0]) >> 8) + ((dc * f[1]) >> 8);
        r |= (v > 255 ? 255 : v) << i;
    }
    return r;
}

static double m3_edge(const M3Vertex *a, const M3Vertex *b, double x,
                      double y)
{
    return (b->x - a->x) * (y - a->y) - (b->y - a->y) * (x - a->x);
}

static void m3_draw(Intel2700GState *s, const M3Obj *o, M3Tile *t)
{
    const M3Vertex *v = o->v;
    double area = m3_edge(&v[0], &v[1], v[2].x, v[2].y), sg;
    uint32_t mode = (o->tsp >> 9) & 7;
    int px, py, i;

    if (area == 0) {
        return;
    }
    sg = area > 0 ? 1 : -1;
    for (py = 0; py < M3_TILE_H; py++) {
        for (px = 0; px < M3_TILE_W; px++) {
            double x = t->x + px + 0.5, y = t->y + py + 0.5, w[3], z;
            bool inside = true, pass;
            int k = py * M3_TILE_W + px;
            uint32_t src;

            for (i = 0; i < 3; i++) {
                const M3Vertex *a = &v[(i + 1) % 3], *b = &v[(i + 2) % 3];
                double ei = m3_edge(a, b, x, y) * sg;
                double dx = (b->x - a->x) * sg, dy = (b->y - a->y) * sg;
                bool tie = dy > 0 || (dy == 0 && dx < 0);

                inside &= ei > 0 || (ei == 0 && tie);
                w[i] = ei * sg / area;
            }
            if (!inside) {
                continue;
            }
            z = w[0] * v[0].z + w[1] * v[1].z + w[2] * v[2].z;
            switch (mode) {
            case 4:
                pass = z > t->z[k];
                break;
            case 6:
                pass = z >= t->z[k];
                break;
            case 7:
                pass = true;
                break;
            case 1:
            case 3:                     /* never passed on the device */
                pass = false;
                break;
            default:
                qemu_log_mask(LOG_UNIMP, "intel-2700g: depth mode %u never"
                              " measured\n", mode);
                pass = false;
                break;
            }
            if (!pass) {
                continue;
            }
            if (o->tsp & M3_TSP_GOURAUD) {
                int sh;

                src = 0;
                for (sh = 0; sh < 32; sh += 8) {
                    double c = 0;

                    for (i = 0; i < 3; i++) {
                        c += w[i] * ((v[i].argb >> sh) & 0xFF);
                    }
                    c = floor(c + 0.5);
                    src |= (uint32_t)(c < 0 ? 0 : c > 255 ? 255 : c) << sh;
                }
            } else {
                src = v[0].argb;
            }
            if (o->textured) {
                double u = w[0] * v[0].u + w[1] * v[1].u + w[2] * v[2].u;
                double vv = w[0] * v[0].v + w[1] * v[1].v + w[2] * v[2].v;

                src = m3_modulate(m3_sample(s, o->tex, u, vv), src);
            }
            t->c[k] = m3_blend(o->tsp, src, t->c[k]);
            if (!(o->tsp & M3_TSP_NO_WRITE)) {
                t->z[k] = z;
            }
        }
    }
}

/* What a quad adds to a tile, quarter-microseconds (rp-mbx3d-api run 2). */
static uint32_t m3_quad_cost(const M3Obj *o)
{
    return o->tsp & M3_TSP_BLEND ? 20 : o->textured ? 4 : 2;
}

/*
 * Renders the region array; returns the render time in microseconds, or
 * -1 if an object hung the core.
 */
static int64_t m3_render(Intel2700GState *s)
{
    uint8_t *vram = memory_region_get_ram_ptr(&s->vram);
    uint32_t regions = s->mbx[0x608 / 4], params = s->mbx[0x60C / 4];
    uint32_t target = s->mbx[0x65C / 4], stride = s->mbx[0x660 / 4] * 4;
    uint32_t r6dc = s->mbx[0x6DC / 4], tiles = 0, cost = 0, k, j;
    double clear_z = r6dc & 1 ? m3_float(r6dc & ~1u) : NAN;
    M3Tile *t = g_new(M3Tile, 1);

    for (k = 0; k < 4096; k++) {
        uint32_t w0 = m3_word(s, regions + 8 * k);
        uint32_t list = m3_word(s, regions + 8 * k + 4) & M3_ADDR_MASK;
        int px, py;

        t->x = (w0 & 127) * M3_TILE_W;
        t->y = ((w0 >> 8) & 127) * M3_TILE_H;
        for (j = 0; j < M3_TILE_W * M3_TILE_H; j++) {
            t->c[j] = 0xFFFFFFFF;
            t->z[j] = clear_z;
        }
        for (j = 0; j < 512; j++) {
            uint32_t p = m3_word(s, list + 4 * j), flags = p & 0xFFE00000;
            M3Obj o;

            if (p >> 28 == 0xF) {
                break;
            }
            if (flags != M3_STRIP_FLAT && flags != M3_STRIP_TEX) {
                qemu_log_mask(LOG_GUEST_ERROR, "intel-2700g: strip pointer"
                              " flags 0x%08x never measured\n", flags);
                break;
            }
            m3_object(s, params + (p & 0x001FFFFF) * 4,
                      flags == M3_STRIP_TEX, &o);
            if (o.tsp & M3_TSP_HANGS) {
                g_free(t);
                return -1;              /* as on the device: until a reset */
            }
            m3_draw(s, &o, t);
            if (j > 0) {
                cost += m3_quad_cost(&o);
            }
        }
        for (py = 0; py < M3_TILE_H; py++) {
            for (px = 0; px < M3_TILE_W; px++) {
                uint64_t a = target + (uint64_t)(t->y + py) * stride +
                             4 * (t->x + px);

                if (a + 4 <= INTEL_2700G_VRAM_SIZE) {
                    stl_le_p(vram + a, t->c[py * M3_TILE_W + px]);
                }
            }
        }
        tiles++;
        if (w0 & M3_LAST) {
            break;
        }
    }
    g_free(t);
    memory_region_set_dirty(&s->vram, 0, INTEL_2700G_VRAM_SIZE);
    /* Two objects a quad: cost / 2 quarters, then quarters to us. */
    return 10 + tiles * 21 / 4 + cost / 2 / 4;
}

static bool m3_passed(int64_t t)
{
    return t != M3_NONE && qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) >= t;
}

/* Latches the 3D core's status bits whose time has come. */
static void m3_tick(Intel2700GState *s)
{
    if (m3_passed(s->m3_done_ns)) {
        s->m3_done_ns = M3_NONE;
        s->mbx[MBX_STATUS / 4] |= M3_RENDER_DONE;
    }
    if (m3_passed(s->m3_ready_ns)) {
        s->m3_ready_ns = M3_NONE;
        s->mbx[MBX_STATUS / 4] |= M3_PM_READY;
    }
}

static bool m3_clock_on(Intel2700GState *s)
{
    return (s->regs[R_MBXCLK / 4] & 3) == 2;
}

/* A write to the MBX block that concerns the 3D core. */
static void m3_write(Intel2700GState *s, hwaddr off, uint32_t val)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), us;

    if (off == MBX_RESET && val == 0x111) {
        s->m3_done_ns = s->m3_ready_ns = M3_NONE;
        s->m3_pm_set = s->m3_hung = false;
        s->mbx[MBX_STATUS / 4] &= ~(M3_RENDER_DONE | M3_PM_READY);
    } else if (off == M3_PM_GO && val == 1 && m3_clock_on(s)) {
        s->m3_pm_set = true;
        s->m3_ready_ns = now + 164 * SCALE_US;
    } else if (off == M3_START && val == 1) {
        /* Nothing without the clock and the manager (version 1), or hung. */
        if (!m3_clock_on(s) || !s->m3_pm_set || s->m3_hung) {
            return;
        }
        us = m3_render(s);
        if (us < 0) {
            s->m3_hung = true;
            return;
        }
        s->m3_done_ns = now + us * SCALE_US;
    }
}

static bool i2700g_event_reg(hwaddr addr)
{
    return addr == R_DINTRS || addr == R_DINTRE || addr == R_DVECTRL ||
           addr == R_MINTENABLE;
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
            m3_tick(s);
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
    case R_DINTRS:
        i2700g_events(s);
        *data = s->regs[R_DINTRS / 4];
        break;
    case R_DVLNUM:
        *data = i2700g_line(s) % FRAME_LINES;
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
            m3_tick(s);
            s->mbx[MBX_STATUS / 4] &= ~(uint32_t)val;
        } else if (off != MBX_ID && off != MBX_REV && off != MBX_STATUS) {
            s->mbx[off / 4] = val;
            m3_write(s, off, val);
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
    if (i2700g_event_reg(addr)) {
        i2700g_events(s);               /* what happened before the write */
        s->regs[addr / 4] = val;
        i2700g_irq_update(s);
        return MEMTX_OK;
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
    s->mbx_blend = 0;
    s->m3_done_ns = s->m3_ready_ns = M3_NONE;
    s->m3_pm_set = s->m3_hung = false;
    s->ev_epoch = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->ev_line = 0;
    i2700g_irq_update(s);
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
    qdev_init_gpio_out_named(dev, &s->irq, "irq", 1);
    s->ev_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, i2700g_ev_tick, s);
    object_property_add_bool(OBJECT(dev), "backlight", i2700g_get_backlight,
                             NULL);
}

static const VMStateDescription vmstate_i2700g = {
    .name = "intel-2700g",
    .version_id = 5,
    .minimum_version_id = 5,
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
        VMSTATE_UINT32(mbx_blend, Intel2700GState),
        VMSTATE_INT64(ev_epoch, Intel2700GState),
        VMSTATE_INT64(ev_line, Intel2700GState),
        VMSTATE_TIMER_PTR(ev_timer, Intel2700GState),
        VMSTATE_INT64(m3_done_ns, Intel2700GState),
        VMSTATE_INT64(m3_ready_ns, Intel2700GState),
        VMSTATE_BOOL(m3_pm_set, Intel2700GState),
        VMSTATE_BOOL(m3_hung, Intel2700GState),
        VMSTATE_END_OF_LIST()
    },
};

/*
 * mbx-2d (from rp-v0.12.0): the MBX 2D engine is modelled; mbx-blend (from
 * rp-v0.13.0): its blending and the display's events as measured; mbx-3d
 * (from rp-v0.15.0): the 3D core's direct path.
 */
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
    object_class_property_add_bool(klass, "mbx-blend", i2700g_get_mbx_2d,
                                   NULL);
    object_class_property_add_bool(klass, "mbx-3d", i2700g_get_mbx_2d, NULL);
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
