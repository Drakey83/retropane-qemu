/*
 * Dell Axim X51v power button.
 *
 * Copyright (c) 2026 The RetroPane OS contributors
 *
 * This code is licensed under the GNU GPL v2 or later.
 *
 * The button is a level on a GPIO (GPIO0, RetroPane docs/hw/README.md):
 * low at rest, high while pressed. Tests press it through the monitor:
 * `qom-set /machine/peripheral/power pressed true` (then false).
 */

#include "qemu/osdep.h"
#include "hw/qdev-core.h"
#include "hw/irq.h"
#include "qapi/visitor.h"
#include "qom/object.h"

#define TYPE_X51V_BUTTON "x51v-button"
OBJECT_DECLARE_SIMPLE_TYPE(X51vButtonState, X51V_BUTTON)

struct X51vButtonState {
    DeviceState parent_obj;
    qemu_irq out;
    bool pressed;
};

static bool button_get(Object *obj, Error **errp)
{
    return X51V_BUTTON(obj)->pressed;
}

static void button_set(Object *obj, bool value, Error **errp)
{
    X51vButtonState *s = X51V_BUTTON(obj);
    s->pressed = value;
    qemu_set_irq(s->out, value);
}

static void button_init(Object *obj)
{
    X51vButtonState *s = X51V_BUTTON(obj);
    qdev_init_gpio_out(DEVICE(obj), &s->out, 1);
    object_property_add_bool(obj, "pressed", button_get, button_set);
}

static const TypeInfo button_info = {
    .name = TYPE_X51V_BUTTON,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(X51vButtonState),
    .instance_init = button_init,
};

static void button_register_types(void)
{
    type_register_static(&button_info);
}

type_init(button_register_types)
