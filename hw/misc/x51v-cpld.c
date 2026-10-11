/*
 * Dell Axim X51v board CPLD (power rails, board identification).
 *
 * Copyright (c) 2026 The RetroPane OS contributors
 *
 * This code is licensed under the GNU GPL v2 or later.
 *
 * Follows RetroPane's clean-room spec (retropane-os docs/hw/cpld.md): sixteen
 * 16-bit registers on a 2-byte stride. Power bits are stored and logged
 * (-d unimp) but have no effect yet. Register 0x1E reports the LCD type in
 * bits 1:0 while its bit 3 is set.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_X51V_CPLD "x51v-cpld"
OBJECT_DECLARE_SIMPLE_TYPE(X51vCpldState, X51V_CPLD)

#define NREGS           16
#define R_LCD_ID        0x1E
#define LCD_ID_ENABLE   0x0008

struct X51vCpldState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    uint16_t regs[NREGS];
    uint8_t panel_id;   /* 0 = 3.7" Sharp VGA (default), 2 = 3.7" Samsung VGA */
};

static uint64_t cpld_read(void *opaque, hwaddr addr, unsigned size)
{
    X51vCpldState *s = opaque;
    uint16_t v = s->regs[addr / 2];

    if (addr == R_LCD_ID && (v & LCD_ID_ENABLE)) {
        v = (v & ~3) | (s->panel_id & 3);
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

    memset(s->regs, 0, sizeof(s->regs));
}

static void cpld_realize(DeviceState *dev, Error **errp)
{
    X51vCpldState *s = X51V_CPLD(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &cpld_ops, s,
                          "x51v-cpld", NREGS * 2);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static Property cpld_properties[] = {
    DEFINE_PROP_UINT8("panel-id", X51vCpldState, panel_id, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_cpld = {
    .name = TYPE_X51V_CPLD,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, X51vCpldState, NREGS),
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
