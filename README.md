# hs611-min-ab

A small, self-contained firmware for **A/B testing coil acquisition on a
pen tablet**, using the vendor-class USB transport so a host tool can stream
live coil amplitudes and change every acquisition setting on the fly.

It is the isolated, cleaned-up, documented descendant of the `DEBUG_MIN` build
from the [hs611-fw](../hs611-fw) project. Only the A/B firmware is here: no
release/HID code, no heat-map/DEBUG_DUMP code, one build target, one protocol.

The companion host tool is [tablet-ab](../tablet-ab). Its
[`docs/protocol.md`](../tablet-ab/docs/protocol.md) is the canonical wire
reference.

## What it does

- Enumerates as a vendor-class USB device (`256c:6111`), bulk IN for frames and
  bulk OUT for commands.
- Drives the tablet's receive-coil mux and reads the coupled amplitude of each
  loop, streaming a fixed 168-byte frame per acquisition: the whole 41-coil X
  profile, the whole 27-coil Y profile, the device-computed sub-pixel position,
  and a config/telemetry echo.
- Lets the host change, at run time, the timing backend, carrier frequency,
  burst length, ADC settings, window size, estimator, re-centre strategy, scan
  order, ramp-mitigation mode, re-acquire policy and pacing, then watch the
  profile and position respond. That is the whole point: find a stable coil
  signal by A/B-ing strategies on real hardware.

## Hardware

- **Tablet:** Huion HS611 (the coil tables and pin map are specific to it).
- **MCU:** GigaDevice GD32F350R8T6 (Cortex-M4, 16 KB SRAM, 64 KB flash) on a
  12 MHz crystal, core at 72 MHz.
- **App layout:** linked at `0x08004000`, behind the protected vendor DFU
  bootloader at `0x08000000..0x08003FFF` (48 KB app, 16 KB RAM).
- See [`docs/porting.md`](docs/porting.md) to fork it for another tablet.

## Build and flash

Requires `arm-none-eabi-gcc` and (for flashing) `uv` + a USB DFU connection.

```sh
make                # build/app.elf, build/app.bin, build/app.hex
make size           # footprint
make flash          # DFU-write build/app.bin at 0x08004000, verify
```

Hold the tablet button (PA1) while plugging USB in to enter the vendor DFU
bootloader (`28e9:0189`); the tablet enumerates as `256c:6111` once the app
runs. `dfu-util` does **not** work with this bootloader; `tools/flash.py` talks
to the DFU class interface directly.

Then connect with [tablet-ab](../tablet-ab) (see its `docs/usb-setup.md` for
Linux udev / Windows WinUSB setup) or any host that speaks the protocol in
[`docs/protocol.md`](../tablet-ab/docs/protocol.md).

## Source layout

| File | Responsibility |
|---|---|
| `src/main.c` | boot, USB bring-up, acquisition loop |
| `src/board.h` | pin/port/timer map, coil geometry, safe bands |
| `src/coil_tables.c/.h` | receive-coil mux tables (the tablet-specific data) |
| `src/afe.c/.h` | analog front end: RCU/GPIO/ADC, mux, software-timed measurement |
| `src/hw_timer.c/.h` | hardware-timed backend: TIMER1 carrier + DMA burst, ADC+DMA+TIMER2 |
| `src/estimator.c/.h` | sub-pixel estimators (N-centroid, vendor, Gaomon, log-Gaussian) |
| `src/scan.c/.h` | scan engine: windows, re-centre, re-acquire, ramp modes, frame build |
| `src/acq.c/.h` | runtime acquisition configuration and backend dispatch |
| `src/usb_transport.c/.h` | vendor-class descriptors, streaming, command dispatch |
| `src/usb_hw.c`, `src/usb_it.c` | USB clocks/interrupts and handlers |
| `src/protocol.h` | the wire protocol (version 6) |
| `src/dwt.h` | DWT cycle-counter delays and timestamps |

`docs/architecture.md` explains how the pieces fit and how a measurement and a
scan are timed.

## Forking for another tablet

All tablet-specific data is in `src/board.h` and `src/coil_tables.c`: the GPIO
used for the drive carrier and mux, the receive-loop count and mux tables, and
the carrier-period table. Replace those, adjust the linker script/bootloader
offset, and the rest (USB, scan engine, estimators, protocol) is reusable.
Details and a checklist: [`docs/porting.md`](docs/porting.md).

## Provenance

Extracted and rewritten from `hs611-fw`'s `src/min/` (`usb_min.c`,
`acq_timed.c`, `debug_proto.h`) plus the shared AFE (`src/acq.c`,
`src/tables.c`) and USB hardware layer (`src/usb_hw.c`, `src/usb_it.c`). The
acquisition and estimator maths are preserved; the code has been split into
named modules, the build-variant `#ifdef`s removed, and the protocol bumped to
version 6 (see `docs/protocol.md`).
