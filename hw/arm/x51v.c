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
#include "hw/sysbus.h"
#include "exec/address-spaces.h"

/* Static memory chip selects (PXA27x Developer's Manual, memory map). */
#define X51V_CS0_BOOT       0x00000000  /* boot ROM / DiskOnChip G3 */
#define X51V_CS3_2700G      0x0c000000  /* Intel 2700G: 16 MiB memory + regs */
#define X51V_CS4_CPLD       0x10000000  /* board CPLD: power rails, LCD ID */

/* GPIOs (retropane-os docs/hw/touch.md, from AxDroid). */
#define X51V_GPIO_TS_CS     24  /* TSC2046 chip select (active low) */
#define X51V_GPIO_TS_PENIRQ 94  /* TSC2046 pen interrupt (low = touched) */

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
    .loader_start = PXA2XX_SDRAM_BASE,
    .ram_size = X51V_RAM_SIZE,
};

static void x51v_init(MachineState *machine)
{
    PXA2xxState *mpu;
    MemoryRegion *rom = g_new(MemoryRegion, 1);

    /* PXA270 SoC with on-chip peripherals; FFUART is serial_hd(0). */
    mpu = pxa270_init(x51v_binfo.ram_size, machine->cpu_type);

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

    /* Board CPLD on CS4 (power rails, LCD type). */
    DeviceState *cpld = qdev_new("x51v-cpld");
    qdev_set_id(cpld, g_strdup("cpld"), &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(cpld), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(cpld), 0, X51V_CS4_CPLD);

    pxa27x_register_keypad(mpu->kp, x51v_keymap, 0xE0);

    /* Touch screen: TSC2046 on SSP1, chip select and pen IRQ on GPIOs. */
    DeviceState *ts = ssi_create_peripheral(mpu->ssp[0], "tsc2046");
    qdev_connect_gpio_out(mpu->gpio, X51V_GPIO_TS_CS,
                          qdev_get_gpio_in_named(ts, SSI_GPIO_CS, 0));
    qdev_connect_gpio_out(ts, 0,
                          qdev_get_gpio_in(mpu->gpio, X51V_GPIO_TS_PENIRQ));

    x51v_binfo.board_id = X51V_ARM_ID;
    arm_load_kernel(mpu->cpu, machine, &x51v_binfo);
}

static void x51v_machine_init(MachineClass *mc)
{
    mc->desc = "Dell Axim X51v (PXA270 + 2700G) for RetroPane OS";
    mc->init = x51v_init;
    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("pxa270-c5");
}

DEFINE_MACHINE("x51v", x51v_machine_init)
