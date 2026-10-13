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
#define CB_PD       0x03

/*
 * The owner's X51v as its align screen measured it (2026-10-13,
 * retropane-os rp_touch_cal::Calibration::X51V_DEFAULT): screen =
 * ((a rx + b ry + c) / 65536, (d rx + e ry + f) / 65536). The model is
 * its exact inverse, so a tap here reads what it reads on the device: the
 * X channel follows the vertical (~195 at the top to ~3207 at the bottom),
 * the Y channel the horizontal mirrored (~3696 left to ~330 right), with
 * the panel's slight skew.
 */
#define CAL_A  (-43.0)
#define CAL_B  (-9345.0)
#define CAL_C  34538826.0
#define CAL_D  13925.0
#define CAL_E  (-54.0)
#define CAL_F  (-2709680.0)
#define SCREEN_W 480
#define SCREEN_H 640

/*
 * Settling, as measured: after a power-down (PD = 00) or a change of
 * channel, a position conversion starts far from the pen's value (the X
 * plate near 0, the Y plate near 4095) and each further conversion with
 * the converter on moves 30% of the remaining way, so drivers need 10-20
 * conversions to agree. Drivers reading once got every tap at an edge.
 */
#define SETTLE_START_X  150
#define SETTLE_START_Y  3850

struct TSC2046State {
    SSIPeripheral ssidev;
    qemu_irq penirq;

    uint16_t value[8];  /* per-channel 12-bit conversion results */
    bool touched;
    uint32_t phase;     /* 0 = expecting control byte, 1-2 = result bytes */
    uint16_t result;    /* current conversion, left-aligned per mode */
    uint8_t last_cmd;   /* previous control byte (its PD bits and channel) */
    int level;          /* the settling line's value for that channel */
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
            int ch = (tx >> 4) & 7;
            uint16_t v = s->touched || ch == 0 || ch >= 6 ? s->value[ch] : 0;
            if (s->touched && (ch == 5 || ch == 1)) {
                /* A fresh start after power-down or another channel. */
                bool fresh = (s->last_cmd & CB_PD) == 0 ||
                             ((s->last_cmd >> 4) & 7) != ch;
                if (fresh) {
                    s->level = ch == 5 ? SETTLE_START_X : SETTLE_START_Y;
                }
                s->level = v + (s->level - (int)v) * 7 / 10;
                v = s->level;
            }
            s->last_cmd = tx;
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
        double sx = (double)x * (SCREEN_W - 1) / 0x7FFF;
        double sy = (double)y * (SCREEN_H - 1) / 0x7FFF;
        /* Invert the calibration: [a b; d e] (rx ry)' = 65536 (sx sy)' - (c f)'. */
        double u = 65536.0 * sx - CAL_C, v = 65536.0 * sy - CAL_F;
        double det = CAL_A * CAL_E - CAL_B * CAL_D;
        int vert = (int)((CAL_E * u - CAL_B * v) / det + 0.5);   /* X channel */
        int horiz = (int)((CAL_A * v - CAL_D * u) / det + 0.5);  /* Y channel */
        int rx;
        vert = vert < 0 ? 0 : vert > 4095 ? 4095 : vert;
        horiz = horiz < 0 ? 0 : horiz > 4095 ? 4095 : horiz;
        rx = horiz;
        s->value[5] = vert;                 /* X channel: screen vertical */
        s->value[1] = horiz;                /* Y channel: screen horizontal */
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
        VMSTATE_UINT8(last_cmd, TSC2046State),
        VMSTATE_INT32(level, TSC2046State),
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
