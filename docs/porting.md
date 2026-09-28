# Porting to another tablet

This firmware was written for the Huion HS611, but the tablet-specific parts are
deliberately isolated. To retarget it you need to know, for your tablet:

1. Which GPIO drives the LC tank carrier and its gate.
2. How the receive coils are selected (the mux), and how many there are per axis.
3. The carrier frequency band that resonates the pen.
4. Where the application is linked (bootloader offset) and the flash/RAM sizes.

Everything else — USB, the scan engine, estimators, protocol — is reusable.

## 1. Pin map and geometry — `src/board.h`

- `GPIOA_BASE`…: adjust if your MCU is not a GD32F3x0.
- `PIN_PA4` (sample/hold pulse), `PIN_PA5` (gate), `PIN_PB10` (carrier): map to
  the pins your timer channels and AFE use. If the carrier is not TIMER1, edit
  `src/hw_timer.c` accordingly (channel/AF/CCER bit numbers are GD32-specific).
- `MUX_B_IDLE_MASK` / `MUX_C_KEEP_MASK`: the GPIO output pattern applied between
  mux selections. For a non-GD32 mux, replace `afe_mux()` and the mux writes in
  `hw_timer.c`.
- `AXIS_A_N` / `AXIS_B_N`: the number of receive loops per axis. These drive the
  profile buffer sizes and the protocol frame length, so if they change the host
  protocol must change too (see note below).

## 2. Coil tables — `src/coil_tables.c`

Each entry is 6 bytes: `[index u16][GPIOB ODR mask u16][GPIOC ODR value u16]`.
`afe_mux(mask, val)` selects a loop with

```c
GPIOB->ODR &= mask_b;
GPIOC->ODR = (GPIOC->ODR & MUX_C_KEEP_MASK) | val_c;
```

Replace both tables with your tablet's loops. If your mux spans different ports
or needs a different write sequence, change `afe_mux()` and keep the 6-byte
entry shape (or change the entry helpers in `coil_tables.h`).

## 3. Carrier frequency — `src/hw_timer.c`

`freq_arr_table[13]` maps the protocol frequency index 1..12 to a TIMER1 ARR
value (period in 72 MHz counts). Derive the values for your pen's resonant band,
or sweep on hardware with `SET_FREQ_ARR` (raw ARR, clamped to the safe band) and
then bake the winners into the table. Widths/duty and the safe clamp
(`CARRIER_ARR_MIN/MAX`) live in `src/board.h`.

The software backend (`afe.c`) has its own NOP-sled burst (`DRIVE_BURST`) with a
per-frequency `(n1, n2)` table; update those to match the same band if you want
the software baseline to be meaningful.

## 4. Linker script and bootloader

`ld/gd32f350r8_app.ld` currently places the app at `0x08004000` (48 KB) behind
the vendor DFU bootloader. For another board, set the FLASH origin to just after
your bootloader (or `0x08000000` if there is none) and the correct FLASH/RAM
lengths. The `Makefile` selects the script; the flash offset is in the
`flash`/`flash-fast` targets.

## 5. USB identity

The descriptors in `src/usb_transport.c` use `256c:6111`. If you keep a distinct
VID/PID so the host tool does not confuse your tablet with the HS611, update:

- `DBG_VID` / `DBG_PID` and the string descriptors.
- The host tool's device finder (in `tablet-ab`: `VendorUsbLocator` /
  `VendorUsbOptions`) and its `docs/usb-setup.md` / `docs/protocol.md`.

## 6. If the coil counts change

`PROTO_NX` / `PROTO_NY` in `src/protocol.h` set the frame's profile sizes and
therefore `PROTO_FRAME_LEN`. Changing them is a **protocol break**: the host's
`ProtocolConstants` (frame length, header offset of the Y profile) must be
updated in lock-step. Prefer keeping the HS611 counts if you can, or bump the
protocol version and document the new layout in tablet-ab's
[`docs/protocol.md`](https://github.com/ig1711/tablet-ab/blob/main/docs/protocol.md).

## 7. Check it

```sh
make            # must be warning-free
make size       # confirm it fits your flash/RAM
make flash      # then connect with tablet-ab and stream frames
```

A useful first hardware check is the **repeat-coil** probe (`SET_BACKEND` /
`REPEAT_COIL`): it measures one fixed coil repeatedly and streams the history as
a time series, so you can see the carrier and front end working before worrying
about the mux tables.
