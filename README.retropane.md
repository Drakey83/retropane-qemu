# RetroPane emulator (QEMU 9.1 fork)

This fork of QEMU 9.1.3 adds an **`x51v` machine** (Dell Axim X51v: Intel
PXA270 + Intel 2700G) for developing [RetroPane OS](https://github.com/Drakey83/retropane-os).
QEMU removed PXA2xx support in 9.2, so this fork keeps it alive
(see RetroPane ADR-0002).

- RetroPane work lives on the **`retropane`** branch. `master` mirrors upstream.
- CI builds `qemu-system-arm` for Linux and Windows. Tags `rp-v*` publish
  releases that `cargo xtask emu` in retropane-os downloads automatically, so
  contributors never need to build QEMU.
- The machine is defined in [`hw/arm/x51v.c`](hw/arm/x51v.c). Blocks not
  modelled yet (2700G, CPLD) are logged with `-d unimp`.

```bash
qemu-system-arm -M x51v -kernel image.elf -nographic -serial stdio
```

License: GPL-2.0-or-later, as QEMU.
