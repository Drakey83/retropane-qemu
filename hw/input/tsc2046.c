/*
 * TI TSC2046 (ADS7846-compatible) touch screen controller, cycle-accurate at
 * the byte level.
 *
 * Copyright (c) 2026 The RetroPane OS contributors
 *
 * This code is licensed under the GNU GPL v2 or later.
 *
 * Unlike QEMU's ads7846 model, which answers in the same transfer as the
 * command, this follows the real device (TI TSC2046 datasheet): with 8-bit
 * SSP frames, a conversion is three transfers. Byte 0 is the control byte
 * (the device returns 0); byte 1 returns a busy bit (0) and D11..D5; byte 2
 * returns D4..D0 followed by three zero bits. In 8-bit mode (MODE=1) byte 1
 * returns 0 then D7..D1 and byte 2 returns D0 then zeros.
 *
 * Control byte: [7] S (start), [6:4] A2..A0 channel, [3] MODE, [2] SER/DFR,
 * [1:0] PD. Channels used: 5 = X position, 1 = Y position, 3 = Z1, 4 = Z2.
 *
 * PENIRQ (GPIO output 0) is low while the screen is touched.
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "ui/console.h"
#include "qom/object.h"

#define TYPE_TSC2046 "tsc2046"
OBJECT_DECLARE_SIMPLE_TYPE(TSC2046State, TSC2046)

#define CB_START    0x80
#define CB_MODE8    0x08

/* Raw 12-bit range reported across the panel (typical for a 3.7" panel). */
#define RAW_MIN     200
#define RAW_MAX     3900

struct TSC2046State {
    SSIPeripheral ssidev;
    qemu_irq penirq;

    uint16_t value[8];  /* per-channel 12-bit conversion results */
    bool touched;
    uint32_t phase;     /* 0 = expecting control byte, 1-2 = result bytes */
    uint16_t result;    /* current conversion, left-aligned per mode */
};

static void tsc2046_update_penirq(TSC2046State *s)
{
    qemu_set_irq(s->penirq, !s->touched);
}

static uint32_t tsc2046_transfer(SSIPeripheral *dev, uint32_t tx)
{
    TSC2046State *s = TSC2046(dev);
    uint32_t rx = 0;

    switch (s->phase) {
    case 0:
        if (tx & CB_START) {
            uint16_t v = s->touched || ((tx >> 4) & 7) == 0 ||
                         ((tx >> 4) & 7) >= 6 ? s->value[(tx >> 4) & 7] : 0;
            /* 16 result clocks: busy bit, then the data MSB first. */
            s->result = (tx & CB_MODE8) ? (uint16_t)((v >> 4) << 7)
                                        : (uint16_t)(v << 3);
            s->phase = 1;
        }
        break;
    case 1:
        rx = s->result >> 8;
        s->phase = 2;
        break;
    case 2:
        rx = s->result & 0xFF;
        s->phase = 0;
        break;
    }
    return rx;
}

static int tsc2046_set_cs(SSIPeripheral *dev, bool select)
{
    TSC2046State *s = TSC2046(dev);

    /* Deselecting aborts any partial conversion. */
    if (!select) {
        s->phase = 0;
    }
    return 0;
}

static void tsc2046_mouse_event(void *opaque, int x, int y, int z,
                                int buttons_state)
{
    TSC2046State *s = opaque;
    bool touched = buttons_state & MOUSE_EVENT_LBUTTON;

    if (touched) {
        /* x, y arrive in 0..0x7FFF across the screen. */
        int rx = RAW_MIN + ((RAW_MAX - RAW_MIN) * x) / 0x7FFF;
        int ry = RAW_MIN + ((RAW_MAX - RAW_MIN) * y) / 0x7FFF;
        /*
         * The X51v's panel is wired with X and Y swapped (retropane-os
         * docs/hw/touch.md): the X channel follows the screen's vertical
         * axis. Mirror that so drivers calibrate the same as on hardware.
         */
        s->value[5] = ry;                   /* X channel: screen vertical */
        s->value[1] = rx;                   /* Y channel: screen horizontal */
        s->value[3] = 600;                  /* Z1 */
        s->value[4] = 600 + 6000 * 4096 / (rx * 16 + 1); /* Z2 */
    }
    if (touched != s->touched) {
        s->touched = touched;
        tsc2046_update_penirq(s);
    }
}

static void tsc2046_reset(DeviceState *dev)
{
    TSC2046State *s = TSC2046(dev);

    /* Drive PENIRQ now that board wiring is in place. */
    s->phase = 0;
    tsc2046_update_penirq(s);
}

static void tsc2046_realize(SSIPeripheral *d, Error **errp)
{
    DeviceState *dev = DEVICE(d);
    TSC2046State *s = TSC2046(d);

    qdev_init_gpio_out(dev, &s->penirq, 1);
    s->value[0] = 2000;     /* TEMP0 */
    s->value[2] = 2000;     /* VBAT */
    s->value[6] = 2000;     /* AUX */
    s->value[7] = 3000;     /* TEMP1 */
    s->touched = false;
    tsc2046_update_penirq(s);
    qemu_add_mouse_event_handler(tsc2046_mouse_event, s, 1,
                                 "TSC2046 touch screen");
}

static const VMStateDescription vmstate_tsc2046 = {
    .name = TYPE_TSC2046,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(ssidev, TSC2046State),
        VMSTATE_UINT16_ARRAY(value, TSC2046State, 8),
        VMSTATE_BOOL(touched, TSC2046State),
        VMSTATE_UINT32(phase, TSC2046State),
        VMSTATE_UINT16(result, TSC2046State),
        VMSTATE_END_OF_LIST()
    }
};

static void tsc2046_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = tsc2046_realize;
    k->transfer = tsc2046_transfer;
    k->set_cs = tsc2046_set_cs;
    k->cs_polarity = SSI_CS_LOW;
    dc->vmsd = &vmstate_tsc2046;
    dc->reset = tsc2046_reset;
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo tsc2046_info = {
    .name = TYPE_TSC2046,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(TSC2046State),
    .class_init = tsc2046_class_init,
};

static void tsc2046_register_types(void)
{
    type_register_static(&tsc2046_info);
}

type_init(tsc2046_register_types)
