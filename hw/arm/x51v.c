/*
 * Dell Axim X51v (Intel PXA270 + Intel 2700G) machine for RetroPane OS.
 *
 * Copyright (c) 2026 The RetroPane OS contributors
 *
 * This code is licensed under the GNU GPL v2 or later.
 *
 * Hardware facts come from RetroPane's clean-room specifications
 * (https://github.com/Drakey83/retropane-os, docs/hw/ and
 * docs/research/hardware-inventory.md). Blocks that are not modelled yet are
 * mapped as "unimplemented" devices so that every guest access to them is
 * logged with `-d unimp`, which doubles as a reverse-engineering aid.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/arm/pxa.h"
#include "hw/arm/boot.h"
#include "hw/boards.h"
#include "hw/misc/unimp.h"
#include "hw/display/intel-2700g.h"
#include "monitor/qdev.h"
#include "hw/ssi/ssi.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/pcmcia.h"
#include "hw/i2c/i2c.h"
#include "hw/audio/wm8750.h"
#include "sysemu/blockdev.h"
#include "sysemu/reset.h"
#include "exec/address-spaces.h"

/* Static memory chip selects (PXA27x Developer's Manual, memory map). */
#define X51V_CS0_BOOT       0x00000000  /* boot ROM / DiskOnChip G3 */
#define X51V_CS3_2700G      0x0c000000  /* Intel 2700G: 16 MiB memory + regs */
#define X51V_CS4_CPLD       0x10000000  /* board CPLD: power rails, LCD ID */
#define PXA27X_UDC_BASE     0x40600000  /* USB device controller */
#define PXA27X_UHC_BASE     0x4c000000  /* USB host controller (OHCI) */
#define UHC_RH_PORT2        0x58        /* HcRhPortStatus, port 2 */
#define UHC_SET_PORT_RESET  (1 << 4)
#define PXA27X_PIC_USBC     11          /* its interrupt */

/* GPIOs (retropane-os docs/hw/touch.md, from AxDroid). */
#define X51V_GPIO_TS_CS     24  /* TSC2046 chip select (active low) */
#define X51V_GPIO_TS_PENIRQ 94  /* TSC2046 pen interrupt (low = touched) */
/* SD slot, as measured on the owner's X51v (RetroPane docs/hw/sd.md). */
#define X51V_GPIO_SD_DETECT 12  /* card detect (low = card in) */
#define X51V_GPIO_SD_WP     27  /* write protect (high = locked) */
#define X51V_GPIO_POWER     0   /* power button (low at rest) */
#define X51V_GPIO_CF_DETECT 13  /* CF card detect (low = card in) */
#define X51V_GPIO_CF_READY  84  /* CF slot powered (high) */
#define X51V_GPIO_HP_DETECT 83  /* headphone jack: high = in, with audio powered */
#define X51V_GPIO_AUDIO_ON  96  /* reads 1 whenever audio is powered */
#define X51V_GPIO_I2C_GATE  82  /* the codec's I2C gate (high for each transfer: Dell's wavedev.dll) */
#define X51V_GPIO_2700G_IRQ 85  /* 2700G interrupt (low = asserted, likely) */
#define X51V_WM8750_ADDR    0x1a /* codec on the I2C bus (Dell's wavedev.dll writes 0x1a) */
#define X51V_I2C_NOBODY     0x7f /* where the codec "sits" while its gate is closed */

/* SDRAM on SDCS2 (docs/hw in retropane-os). */
#define X51V_SDRAM_BASE     0xa8000000
#define X51V_RAM_SIZE       (64 * MiB)
#define X51V_BOOT_ROM_SIZE  (1 * MiB)
#define X51V_CS_SIZE        (64 * MiB)

/* Linux machine number; unused by RetroPane, which reads no ATAGs. */
#define X51V_ARM_ID         0

/*
 * Button matrix, 7 rows x 3 columns (retropane-os docs/hw/keypad.md), indexed
 * by PC scancode (extended keys as code | 0x80):
 *   arrows/Enter = D-pad, F1 Calendar, F2 Contacts, F3 Mail, F4 Home,
 *   F5 Record, F6 Wi-Fi.
 */
static const struct keymap x51v_keymap[0xE0] = {
    [0 ... 0xDF] = { -1, -1 },
    [0xC8] = { 2, 0 },  /* up    -> D-pad up    (row 0, col 2) */
    [0xD0] = { 2, 1 },  /* down  -> D-pad down  (row 1, col 2) */
    [0xCD] = { 2, 3 },  /* right -> D-pad right (row 3, col 2) */
    [0xCB] = { 2, 4 },  /* left  -> D-pad left  (row 4, col 2) */
    [0x1C] = { 2, 5 },  /* enter -> D-pad enter (row 5, col 2) */
    [0x3B] = { 0, 3 },  /* F1    -> Calendar    (row 3, col 0) */
    [0x3C] = { 0, 4 },  /* F2    -> Contacts    (row 4, col 0) */
    [0x3D] = { 0, 5 },  /* F3    -> Mail        (row 5, col 0) */
    [0x3E] = { 0, 6 },  /* F4    -> Home        (row 6, col 0) */
    [0x3F] = { 0, 1 },  /* F5    -> Record      (row 1, col 0) */
    [0x40] = { 0, 0 },  /* F6    -> Wi-Fi       (row 0, col 0) */
};

static struct arm_boot_info x51v_binfo = {
    .loader_start = X51V_SDRAM_BASE,
    .ram_size = X51V_RAM_SIZE,
};

/*
 * The CF slot's power and RESET, both CPLD outputs, and READY on GPIO84
 * (retropane-os docs/hw/cf.md "Slot power and reset", from Dell's socket
 * driver and device runs). The card answers (and GPIO84 reads high) only
 * while both power bits are on; the slot starts powered, as Windows Mobile
 * hands over (its CPLD table, measured: the x51v-cpld latches), so a driver
 * that cuts a power bit it doesn't own reads zeros, as on the device.
 * Power-up and the release of RESET put the card in its power-on state
 * (memory mode, idle); while RESET is held the card doesn't answer, but
 * GPIO84 stays high (it shows slot power, measured).
 * QEMU's card answers at once: busy times are not modelled.
 *
 * Card detect (GPIO13, low = card in) sticks at "no card" once the card
 * has been pulled out, until a machine reset: on the device a card pushed
 * back in was never seen there, even with the slot powered again for 1 s,
 * while CPLD 0x04 bit 1 followed it both ways (retropane-os rp-cfdetect,
 * 2026-10-16; a soft reset into Windows Mobile brought it back). The
 * simplest model that fits; whether a card pushed into a powered slot
 * clears it is unmeasured.
 *
 * Tests pull the card out and push it back in through the "cfslot" switch:
 * `qom-set /machine/peripheral/cfslot pressed true` ejects it (GPIO13 high,
 * CPLD 0x04 bit 1 set, GPIO84 low), `false` re-inserts it (CPLD 0x04
 * bit 1 clear; GPIO13 stays high, above); a card going
 * into a powered slot starts from its power-on state.
 */
typedef struct {
    void *socket;
    PCMCIACardState *card;     /* the card image, in the slot or not */
    qemu_irq ready;
    qemu_irq present;          /* to the CPLD's status register */
    qemu_irq cd;               /* GPIO13, inverted: raised = card in */
    bool in;
    bool cd_stuck;             /* GPIO13 "no card" since a removal */
    bool powered;
    bool held;
} X51vCfSlot;

static void x51v_cf_update(X51vCfSlot *c)
{
    pxa2xx_pcmcia_set_power(c->socket, c->powered && !c->held);
    qemu_set_irq(c->ready, c->in && c->powered);
    qemu_set_irq(c->present, c->in);
    qemu_set_irq(c->cd, c->in && !c->cd_stuck);
}

static void x51v_cf_slot_reset(void *opaque)
{
    X51vCfSlot *c = opaque;

    c->cd_stuck = false;
    x51v_cf_update(c);
}

static void x51v_cf_power(void *opaque, int n, int level)
{
    X51vCfSlot *c = opaque;

    if (level && !c->powered && c->in) {
        device_cold_reset(DEVICE(c->card));
    }
    c->powered = level;
    x51v_cf_update(c);
}

static void x51v_cf_reset(void *opaque, int n, int level)
{
    X51vCfSlot *c = opaque;

    if (!level && c->held && c->powered && c->in) {
        device_cold_reset(DEVICE(c->card));
    }
    c->held = level;
    x51v_cf_update(c);
}

/*
 * The codec, its I2C gate, its power and the headphone jack, as measured
 * under Windows Mobile (retropane-os rp-wmaudio, 2026-10-18): the codec
 * answers at its address only while powered (CPLD 0x1C bit 1) and while
 * GPIO82 (its I2C gate) is high; the jack reads on GPIO83, high = plugged
 * in, only while audio is powered (0 otherwise, plugged or not); GPIO96
 * reads 1 whenever audio is powered.
 */
typedef struct X51vAudio {
    I2CSlave *codec;
    qemu_irq jack_line, on_line;
    bool powered, gate, jack_in;
} X51vAudio;

static void x51v_audio_update(X51vAudio *a)
{
    i2c_slave_set_address(a->codec, a->powered && a->gate ? X51V_WM8750_ADDR
                                                          : X51V_I2C_NOBODY);
    qemu_set_irq(a->jack_line, a->powered && a->jack_in);
    qemu_set_irq(a->on_line, a->powered);
}

static void x51v_codec_gate(void *opaque, int n, int level)
{
    X51vAudio *a = opaque;

    a->gate = level;
    x51v_audio_update(a);
}

static void x51v_codec_power(void *opaque, int n, int level)
{
    X51vAudio *a = opaque;

    a->powered = level;
    x51v_audio_update(a);
}

static void x51v_jack(void *opaque, int n, int level)
{
    X51vAudio *a = opaque;

    a->jack_in = level;
    x51v_audio_update(a);
}

static void x51v_cf_eject(void *opaque, int n, int level)
{
    X51vCfSlot *c = opaque;

    if (!c->card || c->in == !level) {
        return;
    }
    if (level) {
        pxa2xx_pcmcia_detach(c->socket);
        c->in = false;
        c->cd_stuck = true;
    } else {
        device_cold_reset(DEVICE(c->card));
        pxa2xx_pcmcia_attach(c->socket, c->card);
        c->in = true;
    }
    x51v_cf_update(c);
}

/*
 * Port 2 of the PXA27x's USB host controller shares the USB connector's
 * transceiver with the device controller (UP2OCR.HXS). A root-hub reset of
 * port 2 drives SE0 on the connector, which the PC takes as an unplug
 * (retropane-os rp-unplug run 3, 2026-10-16). QEMU's OHCI model has no
 * connector, so its port 2 status register is watched here: every access
 * goes on to the OHCI model, and a "set port reset" also pulses the device
 * controller's "port2-reset" line (it acts only while HXS gives port 2 to
 * the host controller).
 */
typedef struct X51vUhcTap {
    MemoryRegion mr;
    MemoryRegion *ohci;
    qemu_irq port2_reset;
} X51vUhcTap;

static uint64_t uhc_tap_read(void *opaque, hwaddr addr, unsigned size)
{
    X51vUhcTap *t = opaque;
    uint64_t v = 0;

    memory_region_dispatch_read(t->ohci, UHC_RH_PORT2 + addr, &v,
                                size_memop(size) | MO_TE,
                                MEMTXATTRS_UNSPECIFIED);
    return v;
}

static void uhc_tap_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    X51vUhcTap *t = opaque;

    memory_region_dispatch_write(t->ohci, UHC_RH_PORT2 + addr, val,
                                 size_memop(size) | MO_TE,
                                 MEMTXATTRS_UNSPECIFIED);
    if (addr == 0 && (val & UHC_SET_PORT_RESET)) {
        qemu_irq_pulse(t->port2_reset);
    }
}

static const MemoryRegionOps uhc_tap_ops = {
    .read = uhc_tap_read,
    .write = uhc_tap_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void x51v_tap_uhc_port2(DeviceState *udc)
{
    X51vUhcTap *t = g_new0(X51vUhcTap, 1);
    MemoryRegionSection sec = memory_region_find(get_system_memory(),
                                                 PXA27X_UHC_BASE, 4);

    if (!sec.mr) {
        return;
    }
    t->ohci = sec.mr;   /* the reference is kept for the machine's life */
    t->port2_reset = qdev_get_gpio_in_named(udc, "port2-reset", 0);
    memory_region_init_io(&t->mr, NULL, &uhc_tap_ops, t, "x51v-uhc-port2", 4);
    memory_region_add_subregion_overlap(get_system_memory(),
                                        PXA27X_UHC_BASE + UHC_RH_PORT2,
                                        &t->mr, 1);
}

static void x51v_init(MachineState *machine)
{
    PXA2xxState *mpu;
    MemoryRegion *rom = g_new(MemoryRegion, 1);

    /* PXA270 SoC with on-chip peripherals; FFUART is serial_hd(0). */
    mpu = pxa270_init(x51v_binfo.ram_size, machine->cpu_type);

    /*
     * The X51v's SDRAM is on SDCS2, not bank 0: Dell's OAL address table maps
     * physical 0xA8000000 (64 MiB), and HaRET boots Linux with ramaddr
     * 0xA8000000. Move the SoC's RAM there; nothing answers at 0xA0000000.
     */
    memory_region_del_subregion(get_system_memory(), &mpu->sdram);
    memory_region_add_subregion(get_system_memory(), X51V_SDRAM_BASE,
                                &mpu->sdram);

    /*
     * The real boot ROM (Dell bootloader + Windows Mobile) is never needed:
     * RetroPane is loaded straight into SDRAM, the way the SD launcher does it
     * on hardware. A blank ROM keeps the reset vector mapped.
     */
    memory_region_init_rom(rom, NULL, "x51v.bootrom", X51V_BOOT_ROM_SIZE,
                           &error_fatal);
    memory_region_add_subregion(get_system_memory(), X51V_CS0_BOOT, rom);

    /*
     * Intel 2700G on CS3: 16 MiB local memory at the window base, registers at
     * +0x3FE0000. The rest of the window is unmodelled and logged.
     */
    DeviceState *gpu = qdev_new(TYPE_INTEL_2700G);
    /* Named, so tools can pick its console: `screendump <file> mbx2700g`. */
    qdev_set_id(gpu, g_strdup("mbx2700g"), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(gpu), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(gpu), 0, X51V_CS3_2700G);
    sysbus_mmio_map(SYS_BUS_DEVICE(gpu), 1,
                    X51V_CS3_2700G + INTEL_2700G_REG_OFFSET);
    create_unimplemented_device("x51v.2700g-unmapped",
                                X51V_CS3_2700G + INTEL_2700G_VRAM_SIZE,
                                INTEL_2700G_REG_OFFSET - INTEL_2700G_VRAM_SIZE);

    /* Board CPLD on CS4 (power rails, resets, LCD type). Its backlight
     * enable (0x18 bit 2) lights the panel; its CF outputs are wired with
     * the CF slot below. */
    DriveInfo *cf = drive_get(IF_IDE, 0, 0);
    DeviceState *cpld = qdev_new("x51v-cpld");
    qdev_set_id(cpld, g_strdup("cpld"), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(cpld), &error_fatal);
    qdev_connect_gpio_out_named(cpld, "backlight", 0,
                                qdev_get_gpio_in_named(gpu, "backlight", 0));
    /* The 2700G's interrupt reaches the PXA on GPIO85 (Windows Mobile's
     * OAL map; measured 2026-10-14: an enabled display event edges it). */
    qdev_connect_gpio_out_named(gpu, "irq", 0,
                                qemu_irq_invert(qdev_get_gpio_in(mpu->gpio,
                                                X51V_GPIO_2700G_IRQ)));
    sysbus_mmio_map(SYS_BUS_DEVICE(cpld), 0, X51V_CS4_CPLD);

    /*
     * USB device controller with a built-in host (enumerates like Windows,
     * then bridges the CDC serial data to `-global pxa27x-udc.chardev=ID`).
     */
    DeviceState *udc = qdev_new("pxa27x-udc");
    qdev_set_id(udc, g_strdup("udc"), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(udc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(udc), 0, PXA27X_UDC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(udc), 0,
                       qdev_get_gpio_in(mpu->pic, PXA27X_PIC_USBC));
    /* Each endpoint slot's DMA request: slot x is the PXA27x's request
     * 24 + x (endpoint 0 is 24; measured on the X51v with slots B-E). */
    for (int x = 1; x < 24; x++) {
        qdev_connect_gpio_out_named(udc, "dreq", x,
                                    qdev_get_gpio_in(mpu->dma, 24 + x));
    }
    x51v_tap_uhc_port2(udc);

    pxa27x_register_keypad(mpu->kp, x51v_keymap, 0xE0);

    /* Touch screen: TSC2046 on SSP1, chip select and pen IRQ on GPIOs. */
    DeviceState *ts = ssi_create_peripheral(mpu->ssp[0], "tsc2046");
    qdev_connect_gpio_out(mpu->gpio, X51V_GPIO_TS_CS,
                          qdev_get_gpio_in_named(ts, SSI_GPIO_CS, 0));
    qdev_connect_gpio_out(ts, 0,
                          qdev_get_gpio_in(mpu->gpio, X51V_GPIO_TS_PENIRQ));

    /* PWM0-3 (PWM0: the backlight), registers readable by tests through
     * `qom-get /machine/peripheral/pwm dcr0`. */
    DeviceState *pwm = qdev_new("pxa27x-pwm");
    qdev_set_id(pwm, g_strdup("pwm"), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pwm), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(pwm), 0, 0x40B00000);
    sysbus_mmio_map(SYS_BUS_DEVICE(pwm), 1, 0x40C00000);

    /* Power button on GPIO0: `qom-set /machine/peripheral/power pressed
     * true` presses it. */
    DeviceState *power = qdev_new("x51v-button");
    qdev_set_id(power, g_strdup("power"), &error_fatal);
    qdev_realize_and_unref(power, NULL, &error_fatal);
    qdev_connect_gpio_out(power, 0, qdev_get_gpio_in(mpu->gpio, X51V_GPIO_POWER));

    /* SD slot switches: `-drive if=sd,readonly=on` reads as a locked card,
     * no card as detect high. */
    pxa2xx_mmci_handlers(mpu->mmc,
                         qdev_get_gpio_in(mpu->gpio, X51V_GPIO_SD_WP),
                         qemu_irq_invert(qdev_get_gpio_in(mpu->gpio,
                                                          X51V_GPIO_SD_DETECT)));

    /* CompactFlash slot, PC Card socket 1: `-drive if=ide,file=cf.img`
     * puts a CF card in it (QEMU's microdrive model: CIS, memory and I/O
     * modes, ATA). Detect on GPIO13, low with a card; an empty slot reads
     * high (and CPLD 0x04 bit 1 set). Power and RESET from the CPLD, READY
     * on GPIO84 (x51v_cf_power). The card's interrupt is not modelled
     * (memory mode, polled). */
    {
        qemu_irq cd = qemu_irq_invert(qdev_get_gpio_in(mpu->gpio,
                                                       X51V_GPIO_CF_DETECT));
        X51vCfSlot *slot = g_new0(X51vCfSlot, 1);

        slot->socket = mpu->pcmcia[1];
        slot->cd = cd;             /* the board's, not the socket's */
        qemu_irq_lower(cd);
        if (cf) {
            slot->card = dscm1xxxx_init(cf);
            pxa2xx_pcmcia_attach(mpu->pcmcia[1], slot->card);
            slot->in = true;
        }
        slot->ready = qdev_get_gpio_in(mpu->gpio, X51V_GPIO_CF_READY);
        slot->present = qdev_get_gpio_in_named(cpld, "cf-present", 0);
        qdev_connect_gpio_out_named(cpld, "cf-power", 0,
                                    qemu_allocate_irq(x51v_cf_power, slot, 0));
        qdev_connect_gpio_out_named(cpld, "cf-reset", 0,
                                    qemu_allocate_irq(x51v_cf_reset, slot, 0));
        x51v_cf_update(slot);
        qemu_register_reset(x51v_cf_slot_reset, slot);

        DeviceState *eject = qdev_new("x51v-button");
        qdev_set_id(eject, g_strdup("cfslot"), &error_fatal);
        qdev_realize_and_unref(eject, NULL, &error_fatal);
        qdev_connect_gpio_out(eject, 0,
                              qemu_allocate_irq(x51v_cf_eject, slot, 0));
    }

    /* The headphone jack, a switch like the power button: `qom-set
     * /machine/peripheral/jack pressed true` plugs headphones in (GPIO83
     * high while audio is powered: x51v_audio_update). */
    DeviceState *jack = qdev_new("x51v-button");
    qdev_set_id(jack, g_strdup("jack"), &error_fatal);
    qdev_realize_and_unref(jack, NULL, &error_fatal);

    /* Audio: a WM8750 codec on the PXA I2C bus, fed by the I2S controller
     * (docs/research/hardware-inventory.md). `-machine audiodev=ID` picks
     * the output (tests record a wav). The codec hears the bus only
     * while GPIO82 is high (Dell's driver raises it for each transfer and
     * lowers it after: a gate on the codec's I2C lines); closed, it moves
     * to an address nobody uses, so a write to 0x1a finds no device, as
     * on the device; likewise while its power (CPLD 0x1C bit 1) is off.
     * The speaker amplifier (CPLD 0x14 bit 1) is not modelled. */
    {
        I2CSlave *codec = i2c_slave_new(TYPE_WM8750, X51V_WM8750_ADDR);
        X51vAudio *audio = g_new0(X51vAudio, 1);

        if (machine->audiodev) {
            qdev_prop_set_string(DEVICE(codec), "audiodev", machine->audiodev);
        }
        i2c_slave_realize_and_unref(codec, pxa2xx_i2c_bus(mpu->i2c[0]),
                                    &error_fatal);
        audio->codec = codec;
        audio->jack_line = qdev_get_gpio_in(mpu->gpio, X51V_GPIO_HP_DETECT);
        audio->on_line = qdev_get_gpio_in(mpu->gpio, X51V_GPIO_AUDIO_ON);
        qdev_connect_gpio_out(mpu->gpio, X51V_GPIO_I2C_GATE,
                              qemu_allocate_irq(x51v_codec_gate, audio, 0));
        qdev_connect_gpio_out_named(cpld, "codec-power", 0,
                                    qemu_allocate_irq(x51v_codec_power,
                                                      audio, 0));
        qdev_connect_gpio_out(jack, 0, qemu_allocate_irq(x51v_jack, audio, 0));
        x51v_audio_update(audio);
        mpu->i2s->opaque = codec;
        mpu->i2s->codec_out = wm8750_dac_dat;
        mpu->i2s->codec_in = wm8750_adc_dat;
        wm8750_data_req_set(DEVICE(codec), mpu->i2s->data_req, mpu->i2s);
    }

    /*
     * The clocks and core voltage as Windows Mobile leaves them for HaRET
     * (retropane-os ADR-0031, docs/hw/probe-2026-10-11.md): 520 MHz turbo
     * (CCCR 0x02000290, CLKCFG 0xB) and the voltage code for it on the
     * three GPIOs that set the core regulator (GPIO3 low, GPIO4 and GPIO38
     * high: code 0b110), all outputs. RetroPane's clock code reads both
     * before it changes anything.
     */
    mpu->cm_regs[0] = 0x02000290;
    mpu->ccsr_lat = 0x02000290;
    mpu->clkcfg = 0xB;
    address_space_stl_le(&address_space_memory, 0x40E00018, 1u << 4,
                         MEMTXATTRS_UNSPECIFIED, NULL);   /* GPSR0: GPIO4 */
    address_space_stl_le(&address_space_memory, 0x40E0001C, 1u << 6,
                         MEMTXATTRS_UNSPECIFIED, NULL);   /* GPSR1: GPIO38 */
    address_space_stl_le(&address_space_memory, 0x40E0000C,
                         (1u << 3) | (1u << 4),
                         MEMTXATTRS_UNSPECIFIED, NULL);   /* GPDR0 */
    address_space_stl_le(&address_space_memory, 0x40E00010, 1u << 6,
                         MEMTXATTRS_UNSPECIFIED, NULL);   /* GPDR1 */

    x51v_binfo.board_id = X51V_ARM_ID;
    arm_load_kernel(mpu->cpu, machine, &x51v_binfo);
}

/* A read-only marker for tests: this model has the codec's GPIO82 I2C gate
 * and the GPIO96 headphone jack (low = plugged in), shown by
 * `-M x51v,help`. */
static bool x51v_get_codec_gate(Object *obj, Error **errp)
{
    return true;
}

static void x51v_machine_init(MachineClass *mc)
{
    object_class_property_add_bool(OBJECT_CLASS(mc), "codec-gate",
                                   x51v_get_codec_gate, NULL);
    object_class_property_set_description(OBJECT_CLASS(mc), "codec-gate",
        "WM8750 I2C gate on GPIO82, headphone jack on GPIO96 (read-only)");
    object_class_property_add_bool(OBJECT_CLASS(mc), "codec-power",
                                   x51v_get_codec_gate, NULL);
    object_class_property_set_description(OBJECT_CLASS(mc), "codec-power",
        "WM8750 powered by CPLD 0x1C bit 1, jack on GPIO83 (read-only)");
    mc->desc = "Dell Axim X51v (PXA270 + 2700G) for RetroPane OS";
    mc->init = x51v_init;
    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("pxa270-c5");
    machine_add_audiodev_property(mc);
}

DEFINE_MACHINE("x51v", x51v_machine_init)
