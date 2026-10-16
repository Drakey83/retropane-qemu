/*
 * Intel XScale PXA255/270 PC Card and CompactFlash Interface.
 *
 * Copyright (c) 2006 Openedhand Ltd.
 * Written by Andrzej Zaborowski <balrog@zabor.org>
 *
 * This code is licensed under the GPLv2.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/pcmcia.h"
#include "hw/arm/pxa.h"
#include "hw/qdev-properties.h"

struct PXA2xxPCMCIAState {
    SysBusDevice parent_obj;

    PCMCIASocket slot;
    MemoryRegion container_mem;
    MemoryRegion common_iomem;
    MemoryRegion attr_iomem;
    MemoryRegion iomem;

    qemu_irq irq;
    qemu_irq cd_irq;

    PCMCIACardState *card;
    /* Board switched the socket's power off (or holds the card in reset):
     * the card doesn't answer, reads float to 0. Off only on boards that
     * call pxa2xx_pcmcia_set_power(). */
    bool off;
    /* 32-bit accesses to common memory become two 16-bit card cycles. */
    bool word_cycles;
};

/*
 * The card's bus is 16 bits wide: the PXA27x memory controller splits a
 * 32-bit access to a socket into two 16-bit card cycles, low half first
 * (PXA27x Developer's Manual, PC Card interface). With a CF card answering
 * its data register across 0x400-0x7FF in memory mode, a 32-bit load (or
 * a DMA word) there brings two data words: measured byte-exact on the X51v
 * (retropane-os rp-cftime, 2026-10-15, docs/research/storage-speed.md).
 * Upstream passed the access to the card as one cycle: one word. The
 * "word-cycles" property (on by default) switches the model off, for
 * checking that a test fails without it.
 */
static uint64_t pxa2xx_pcmcia_common_read(void *opaque,
                hwaddr offset, unsigned size)
{
    PXA2xxPCMCIAState *s = (PXA2xxPCMCIAState *) opaque;
    PCMCIACardClass *pcc;

    if (s->slot.attached && !s->off) {
        pcc = PCMCIA_CARD_GET_CLASS(s->card);
        if (size == 4 && s->word_cycles) {
            uint64_t lo = pcc->common_read(s->card, offset) & 0xffff;
            uint64_t hi = pcc->common_read(s->card, offset + 2) & 0xffff;
            return lo | hi << 16;
        }
        return pcc->common_read(s->card, offset);
    }

    return 0;
}

static void pxa2xx_pcmcia_common_write(void *opaque, hwaddr offset,
                                       uint64_t value, unsigned size)
{
    PXA2xxPCMCIAState *s = (PXA2xxPCMCIAState *) opaque;
    PCMCIACardClass *pcc;

    if (s->slot.attached && !s->off) {
        pcc = PCMCIA_CARD_GET_CLASS(s->card);
        if (size == 4 && s->word_cycles) {
            pcc->common_write(s->card, offset, value & 0xffff);
            pcc->common_write(s->card, offset + 2, (value >> 16) & 0xffff);
            return;
        }
        pcc->common_write(s->card, offset, value);
    }
}

static uint64_t pxa2xx_pcmcia_attr_read(void *opaque,
                hwaddr offset, unsigned size)
{
    PXA2xxPCMCIAState *s = (PXA2xxPCMCIAState *) opaque;
    PCMCIACardClass *pcc;

    if (s->slot.attached && !s->off) {
        pcc = PCMCIA_CARD_GET_CLASS(s->card);
        return pcc->attr_read(s->card, offset);
    }

    return 0;
}

static void pxa2xx_pcmcia_attr_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    PXA2xxPCMCIAState *s = (PXA2xxPCMCIAState *) opaque;
    PCMCIACardClass *pcc;

    if (s->slot.attached && !s->off) {
        pcc = PCMCIA_CARD_GET_CLASS(s->card);
        pcc->attr_write(s->card, offset, value);
    }
}

static uint64_t pxa2xx_pcmcia_io_read(void *opaque,
                hwaddr offset, unsigned size)
{
    PXA2xxPCMCIAState *s = (PXA2xxPCMCIAState *) opaque;
    PCMCIACardClass *pcc;

    if (s->slot.attached && !s->off) {
        pcc = PCMCIA_CARD_GET_CLASS(s->card);
        return pcc->io_read(s->card, offset);
    }

    return 0;
}

static void pxa2xx_pcmcia_io_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    PXA2xxPCMCIAState *s = (PXA2xxPCMCIAState *) opaque;
    PCMCIACardClass *pcc;

    if (s->slot.attached && !s->off) {
        pcc = PCMCIA_CARD_GET_CLASS(s->card);
        pcc->io_write(s->card, offset, value);
    }
}

static const MemoryRegionOps pxa2xx_pcmcia_common_ops = {
    .read = pxa2xx_pcmcia_common_read,
    .write = pxa2xx_pcmcia_common_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static const MemoryRegionOps pxa2xx_pcmcia_attr_ops = {
    .read = pxa2xx_pcmcia_attr_read,
    .write = pxa2xx_pcmcia_attr_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static const MemoryRegionOps pxa2xx_pcmcia_io_ops = {
    .read = pxa2xx_pcmcia_io_read,
    .write = pxa2xx_pcmcia_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void pxa2xx_pcmcia_set_irq(void *opaque, int line, int level)
{
    PXA2xxPCMCIAState *s = (PXA2xxPCMCIAState *) opaque;
    if (!s->irq)
        return;

    qemu_set_irq(s->irq, level);
}

static void pxa2xx_pcmcia_initfn(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    PXA2xxPCMCIAState *s = PXA2XX_PCMCIA(obj);

    memory_region_init(&s->container_mem, obj, "container", 0x10000000);
    sysbus_init_mmio(sbd, &s->container_mem);

    /* Socket I/O Memory Space */
    memory_region_init_io(&s->iomem, obj, &pxa2xx_pcmcia_io_ops, s,
                          "pxa2xx-pcmcia-io", 0x04000000);
    memory_region_add_subregion(&s->container_mem, 0x00000000,
                                &s->iomem);

    /* Then next 64 MB is reserved */

    /* Socket Attribute Memory Space */
    memory_region_init_io(&s->attr_iomem, obj, &pxa2xx_pcmcia_attr_ops, s,
                          "pxa2xx-pcmcia-attribute", 0x04000000);
    memory_region_add_subregion(&s->container_mem, 0x08000000,
                                &s->attr_iomem);

    /* Socket Common Memory Space */
    memory_region_init_io(&s->common_iomem, obj, &pxa2xx_pcmcia_common_ops, s,
                          "pxa2xx-pcmcia-common", 0x04000000);
    memory_region_add_subregion(&s->container_mem, 0x0c000000,
                                &s->common_iomem);

    s->slot.irq = qemu_allocate_irq(pxa2xx_pcmcia_set_irq, s, 0);

    object_property_add_link(obj, "card", TYPE_PCMCIA_CARD,
                             (Object **)&s->card,
                             NULL, /* read-only property */
                             0);
}

/* Insert a new card into a slot */
int pxa2xx_pcmcia_attach(void *opaque, PCMCIACardState *card)
{
    PXA2xxPCMCIAState *s = (PXA2xxPCMCIAState *) opaque;
    PCMCIACardClass *pcc;

    if (s->slot.attached) {
        return -EEXIST;
    }

    if (s->cd_irq) {
        qemu_irq_raise(s->cd_irq);
    }

    s->card = card;
    pcc = PCMCIA_CARD_GET_CLASS(s->card);

    s->slot.attached = true;
    s->card->slot = &s->slot;
    pcc->attach(s->card);

    return 0;
}

/* Eject card from the slot */
int pxa2xx_pcmcia_detach(void *opaque)
{
    PXA2xxPCMCIAState *s = (PXA2xxPCMCIAState *) opaque;
    PCMCIACardClass *pcc;

    if (!s->slot.attached) {
        return -ENOENT;
    }

    pcc = PCMCIA_CARD_GET_CLASS(s->card);
    pcc->detach(s->card);
    s->card->slot = NULL;
    s->card = NULL;

    s->slot.attached = false;

    if (s->irq) {
        qemu_irq_lower(s->irq);
    }
    if (s->cd_irq) {
        qemu_irq_lower(s->cd_irq);
    }

    return 0;
}

/* Board power switch (and reset) for the socket: while off, the card's
 * windows read 0 and ignore writes. Card state across power-up is the
 * board's business (it resets the card). */
void pxa2xx_pcmcia_set_power(void *opaque, bool on)
{
    PXA2xxPCMCIAState *s = (PXA2xxPCMCIAState *) opaque;
    s->off = !on;
}

/* Who to notify on card events */
void pxa2xx_pcmcia_set_irq_cb(void *opaque, qemu_irq irq, qemu_irq cd_irq)
{
    PXA2xxPCMCIAState *s = (PXA2xxPCMCIAState *) opaque;
    s->irq = irq;
    s->cd_irq = cd_irq;
}

static Property pxa2xx_pcmcia_properties[] = {
    DEFINE_PROP_BOOL("word-cycles", PXA2xxPCMCIAState, word_cycles, true),
    DEFINE_PROP_END_OF_LIST(),
};

static void pxa2xx_pcmcia_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, pxa2xx_pcmcia_properties);
}

static const TypeInfo pxa2xx_pcmcia_type_info = {
    .name = TYPE_PXA2XX_PCMCIA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PXA2xxPCMCIAState),
    .instance_init = pxa2xx_pcmcia_initfn,
    .class_init = pxa2xx_pcmcia_class_init,
};

static void pxa2xx_pcmcia_register_types(void)
{
    type_register_static(&pxa2xx_pcmcia_type_info);
}

type_init(pxa2xx_pcmcia_register_types)
