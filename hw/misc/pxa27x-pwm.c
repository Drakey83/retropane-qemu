/*
 * Intel PXA27x pulse-width modulators (PWM0-3).
 *
 * Copyright (c) 2026 The RetroPane OS contributors
 *
 * This code is licensed under the GNU GPL v2 or later.
 *
 * Follows RetroPane's clean-room notes (retropane-os docs/hw/backlight.md):
 * four channels, each with a control (prescale), duty-cycle and period
 * register; PWM0 and PWM2 at 0x40B00000 (+0x10), PWM1 and PWM3 at
 * 0x40C00000 (+0x10). On the Dell Axim X51v PWM0 drives the backlight. The
 * model stores the registers (no waveform) and exposes them as read-only
 * properties (`qom-get /machine/peripheral/pwm dcr0`) so tests can check
 * what the guest set; changes are logged with -d unimp.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "qapi/visitor.h"

#define TYPE_PXA27X_PWM "pxa27x-pwm"
OBJECT_DECLARE_SIMPLE_TYPE(Pxa27xPwmState, PXA27X_PWM)

#define NCH 4

struct Pxa27xPwmState {
    SysBusDevice parent_obj;
    MemoryRegion iomem[2];
    uint32_t cr[NCH];
    uint32_t dcr[NCH];
    uint32_t pcr[NCH];
};

/* Region r (0: 0x40B00000, 1: 0x40C00000), offset: channel and register. */
static uint32_t *pwm_reg(Pxa27xPwmState *s, int r, hwaddr addr, int *ch)
{
    *ch = r + 2 * (addr >= 0x10);
    switch (addr & 0xF) {
    case 0x0: return &s->cr[*ch];
    case 0x4: return &s->dcr[*ch];
    case 0x8: return &s->pcr[*ch];
    }
    return NULL;
}

static uint64_t pwm_read(Pxa27xPwmState *s, int r, hwaddr addr)
{
    int ch;
    uint32_t *p = pwm_reg(s, r, addr, &ch);
    return p ? *p : 0;
}

static void pwm_write(Pxa27xPwmState *s, int r, hwaddr addr, uint64_t val)
{
    static const uint32_t mask[3] = { 0x7F, 0x7FF, 0x3FF };
    int ch;
    uint32_t *p = pwm_reg(s, r, addr, &ch);
    if (!p) {
        qemu_log_mask(LOG_GUEST_ERROR, "pxa27x-pwm: bad offset 0x%02"
                      HWADDR_PRIx "\n", addr);
        return;
    }
    uint32_t v = val & mask[(addr & 0xF) / 4];
    if (*p != v) {
        qemu_log_mask(LOG_UNIMP, "pxa27x-pwm: PWM%d reg 0x%x 0x%x -> 0x%x\n",
                      ch, (unsigned)(addr & 0xF), *p, v);
    }
    *p = v;
}

static uint64_t pwm_read0(void *o, hwaddr a, unsigned sz) { return pwm_read(o, 0, a); }
static uint64_t pwm_read1(void *o, hwaddr a, unsigned sz) { return pwm_read(o, 1, a); }
static void pwm_write0(void *o, hwaddr a, uint64_t v, unsigned sz) { pwm_write(o, 0, a, v); }
static void pwm_write1(void *o, hwaddr a, uint64_t v, unsigned sz) { pwm_write(o, 1, a, v); }

static const MemoryRegionOps pwm_ops[2] = {
    {
        .read = pwm_read0, .write = pwm_write0,
        .endianness = DEVICE_LITTLE_ENDIAN,
        .valid = { .min_access_size = 4, .max_access_size = 4 },
    },
    {
        .read = pwm_read1, .write = pwm_write1,
        .endianness = DEVICE_LITTLE_ENDIAN,
        .valid = { .min_access_size = 4, .max_access_size = 4 },
    },
};

static void pwm_reset(DeviceState *dev)
{
    Pxa27xPwmState *s = PXA27X_PWM(dev);
    memset(s->cr, 0, sizeof(s->cr));
    memset(s->dcr, 0, sizeof(s->dcr));
    memset(s->pcr, 0, sizeof(s->pcr));
}

static void pwm_init(Object *obj)
{
    Pxa27xPwmState *s = PXA27X_PWM(obj);
    for (int r = 0; r < 2; r++) {
        memory_region_init_io(&s->iomem[r], obj, &pwm_ops[r], s,
                              r ? "pxa27x-pwm1" : "pxa27x-pwm0", 0x20);
        sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem[r]);
    }
    for (int ch = 0; ch < NCH; ch++) {
        char name[8];
        snprintf(name, sizeof(name), "cr%d", ch);
        object_property_add_uint32_ptr(obj, name, &s->cr[ch], OBJ_PROP_FLAG_READ);
        snprintf(name, sizeof(name), "dcr%d", ch);
        object_property_add_uint32_ptr(obj, name, &s->dcr[ch], OBJ_PROP_FLAG_READ);
        snprintf(name, sizeof(name), "pcr%d", ch);
        object_property_add_uint32_ptr(obj, name, &s->pcr[ch], OBJ_PROP_FLAG_READ);
    }
}

static const VMStateDescription vmstate_pwm = {
    .name = TYPE_PXA27X_PWM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(cr, Pxa27xPwmState, NCH),
        VMSTATE_UINT32_ARRAY(dcr, Pxa27xPwmState, NCH),
        VMSTATE_UINT32_ARRAY(pcr, Pxa27xPwmState, NCH),
        VMSTATE_END_OF_LIST()
    },
};

static void pwm_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = pwm_reset;
    dc->vmsd = &vmstate_pwm;
    dc->desc = "Intel PXA27x PWM0-3";
}

static const TypeInfo pwm_info = {
    .name = TYPE_PXA27X_PWM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Pxa27xPwmState),
    .instance_init = pwm_init,
    .class_init = pwm_class_init,
};

static void pwm_register_types(void)
{
    type_register_static(&pwm_info);
}

type_init(pwm_register_types)
