/*
 * Dell Axim X51v board CPLD (power rails, board identification).
 *
 * Copyright (c) 2026 The RetroPane OS contributors
 *
 * This code is licensed under the GNU GPL v2 or later.
 *
 * Follows RetroPane's clean-room spec (retropane-os docs/hw/cpld.md): sixteen
 * 16-bit registers on a 2-byte stride, measured on the owner's X51v:
 *
 * - Writes go to output latches that do NOT read back. Reads return status
 *   inputs: the values Windows Mobile left (0x00 = 0x0048, 0x04 = 0x00a0,
 *   0x06 = 0x0002, 0x14 = 0x0002, the rest 0), with 0x04 bit 1 clear while a
 *   CompactFlash card is in, and 0x1E bits 1:0 the LCD type while latch bit 3
 *   is set. A driver must keep its own shadow copy of every register.
 * - Latch bits with a modelled effect leave as GPIO outputs ("cf-power":
 *   0x16 bit 1 and 0x10 bit 12 both set; "cf-reset": 0x18 bit 1;
 *   "backlight": 0x18 bit 2). The machine wires them to the CF socket and
 *   the panel. Other bits are logged (-d unimp) and have no effect.
 * - The latches start as Windows Mobile hands over, its shared shadow
 *   table as measured on the X51v (retropane-os rp-handover, 2026-10-16):
 *   02=00FD 10=1000 12=000F 14=0008 16=0003 18=0004 1A=0006 1C=000C,
 *   the rest 0. So the backlight is on (0x18 bit 2), the CF slot powered
 *   (0x16 bit 1, 0x10 bit 12) and out of reset, SD power on (0x16 bit 0);
 *   a driver that writes 0x18 without the backlight bit darkens the screen,
 *   and one that writes 0x16 without bit 1 cuts the CF slot, as on the
 *   device.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_X51V_CPLD "x51v-cpld"
OBJECT_DECLARE_SIMPLE_TYPE(X51vCpldState, X51V_CPLD)

#define NREGS           16
#define R_STATUS        0x04
#define R_PWR10         0x10
#define R_PWR16         0x16
#define R_CTL18         0x18
#define R_LCD_ID        0x1E
#define LCD_ID_ENABLE   0x0008
#define CF_ABSENT       0x0002  /* 0x04 (read): set while the CF slot is empty */
#define CF_POWER_1      0x0002  /* 0x16 */
#define CF_POWER_2      0x1000  /* 0x10 */
#define CF_RESET        0x0002  /* 0x18 */
#define BACKLIGHT_EN    0x0004  /* 0x18 */

/* The latches as Windows Mobile hands over (its shadow table, measured). */
static const uint16_t cpld_wm_latches[NREGS] = {
    [0x02 / 2] = 0x00FD,
    [0x10 / 2] = 0x1000,
    [0x12 / 2] = 0x000F,
    [0x14 / 2] = 0x0008,
    [0x16 / 2] = 0x0003,
    [0x18 / 2] = 0x0004,
    [0x1A / 2] = 0x0006,
    [0x1C / 2] = 0x000C,
};

/* What reads return (status inputs), as measured with Windows Mobile's
 * state; 0x04 bit 1 follows the CF card. */
static const uint16_t cpld_status[NREGS] = {
    [0x00 / 2] = 0x0048,
    [0x04 / 2] = 0x00a0,
    [0x06 / 2] = 0x0002,
    [0x14 / 2] = 0x0002,
};

enum { OUT_CF_POWER, OUT_CF_RESET, OUT_BACKLIGHT, NOUTS };

struct X51vCpldState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint16_t regs[NREGS];   /* the output latches (write-only) */
    qemu_irq out[NOUTS];
    uint8_t panel_id;   /* 0 = 3.7" Sharp VGA (default), 2 = 3.7" Samsung VGA */
    bool cf_present;    /* "cf-present" input: a CF card is in (a level) */
};

static void cpld_update_outputs(X51vCpldState *s)
{
    qemu_set_irq(s->out[OUT_CF_POWER],
                 (s->regs[R_PWR16 / 2] & CF_POWER_1) &&
                 (s->regs[R_PWR10 / 2] & CF_POWER_2));
    qemu_set_irq(s->out[OUT_CF_RESET], !!(s->regs[R_CTL18 / 2] & CF_RESET));
    qemu_set_irq(s->out[OUT_BACKLIGHT],
                 !!(s->regs[R_CTL18 / 2] & BACKLIGHT_EN));
}

static void cpld_set_cf_present(void *opaque, int n, int level)
{
    X51V_CPLD(opaque)->cf_present = level;
}

static uint64_t cpld_read(void *opaque, hwaddr addr, unsigned size)
{
    X51vCpldState *s = opaque;
    uint16_t v = cpld_status[addr / 2];

    if (addr == R_STATUS && !s->cf_present) {
        v |= CF_ABSENT;
    }
    if (addr == R_LCD_ID && (s->regs[addr / 2] & LCD_ID_ENABLE)) {
        v = s->panel_id & 3;
    }
    return v;
}

static void cpld_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    X51vCpldState *s = opaque;

    if (s->regs[addr / 2] != (uint16_t)val) {
        qemu_log_mask(LOG_UNIMP, "x51v-cpld: reg 0x%02" HWADDR_PRIx
                      " 0x%04x -> 0x%04x\n", addr, s->regs[addr / 2],
                      (uint16_t)val);
    }
    s->regs[addr / 2] = val;
    cpld_update_outputs(s);
}

static const MemoryRegionOps cpld_ops = {
    .read = cpld_read,
    .write = cpld_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 2, .max_access_size = 2 },
};

static void cpld_reset(DeviceState *dev)
{
    X51vCpldState *s = X51V_CPLD(dev);

    memcpy(s->regs, cpld_wm_latches, sizeof(s->regs));
    cpld_update_outputs(s);
}

static void cpld_realize(DeviceState *dev, Error **errp)
{
    X51vCpldState *s = X51V_CPLD(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &cpld_ops, s,
                          "x51v-cpld", NREGS * 2);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    qdev_init_gpio_in_named(dev, cpld_set_cf_present, "cf-present", 1);
    qdev_init_gpio_out_named(dev, &s->out[OUT_CF_POWER], "cf-power", 1);
    qdev_init_gpio_out_named(dev, &s->out[OUT_CF_RESET], "cf-reset", 1);
    qdev_init_gpio_out_named(dev, &s->out[OUT_BACKLIGHT], "backlight", 1);
}

static Property cpld_properties[] = {
    DEFINE_PROP_UINT8("panel-id", X51vCpldState, panel_id, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_cpld = {
    .name = TYPE_X51V_CPLD,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, X51vCpldState, NREGS),
        VMSTATE_BOOL(cf_present, X51vCpldState),
        VMSTATE_END_OF_LIST()
    },
};

static void cpld_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = cpld_realize;
    dc->reset = cpld_reset;
    dc->vmsd = &vmstate_cpld;
    device_class_set_props(dc, cpld_properties);
}

static const TypeInfo cpld_info = {
    .name = TYPE_X51V_CPLD,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(X51vCpldState),
    .class_init = cpld_class_init,
};

static void cpld_register_types(void)
{
    type_register_static(&cpld_info);
}

type_init(cpld_register_types)
