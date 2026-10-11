/*
 * Intel 2700G ("Marathon") multimedia accelerator.
 *
 * Copyright (c) 2026 The RetroPane OS contributors
 *
 * This code is licensed under the GNU GPL v2 or later.
 */

#ifndef HW_DISPLAY_INTEL_2700G_H
#define HW_DISPLAY_INTEL_2700G_H

#define TYPE_INTEL_2700G "intel-2700g"

/* Local memory on the 2700G7 (sysbus MMIO region 0). */
#define INTEL_2700G_VRAM_SIZE   (16 * 1024 * 1024)
/* Register block offset within the chip-select window (MMIO region 1). */
#define INTEL_2700G_REG_OFFSET  0x03FE0000

#endif
