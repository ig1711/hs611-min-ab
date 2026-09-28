# Architecture

How the A/B firmware works: the acquisition path, the scan state machine, and
how frames reach the host.

## Boot

`main()` (`src/main.c`):

1. `dwt_init()` — enable the cycle counter (all delays and timestamps use it).
2. `usb_rcu_config()`, `usb_timer_init()`, `usbd_init(&g_usb_dev, &min_desc,
   &usbd_min_cb)`, `usb_intr_config()` — vendor-class USB device, `256c:6111`.
3. `afe_init()` — RCU/clock and AFE GPIO bring-up (`src/afe.c`).
4. `estimator_init()` — calibration defaults.
5. `acq_init()` — start the hardware-timed backend and apply defaults.
6. `scan_init()` — pacing defaults.

Then the main loop:

```c
for (;;) {
    if (USBD_CONFIGURED == g_usb_dev.dev.cur_status) {
        if (scan_wants_step(usb_stream_sof_take())) {
            scan_step(usb_stream_buffer());
            usb_stream_commit();
        }
        usb_stream_send();
    }
}
```

Acquisition and transfer are **decoupled**: `scan_step()` always writes into the
frame buffer that is not in flight, `usb_stream_commit()` publishes it, and
`usb_stream_send()` sends the latest ready frame when the endpoint is free. The
front end therefore keeps running across the USB transfer and the SOF gap.

## Measurement paths

One "measurement" of one receive loop is a mux select, a drive burst, a settle
and an ADC read. Two backends produce the same number so they can be A/B'd:

### Software backend (`afe.c`)

A faithful port of the vendor bit-banged path:

1. `afe_mux()` selects the loop.
2. `excite(freq)` emits a drive burst of `SW_BURST_PERIODS` carrier periods
   (compile-time, default 29; `make SW_BURST=<6..32>`) with inline-asm
   `DRIVE_BURST` NOP sleds (PA5 gate, PB10 carrier), wrapped in an interrupt
   lock. The protocol's `SET_BURST` affects only the hardware backend.
3. Three software-triggered ADC conversions on channel 6 are median-filtered
   (`adc_median6`) after the settle times A/B/C/D.

Slower and jittery, but the known-good reference.

### Hardware-timed backend (`hw_timer.c`)

The point of the exercise — an exact, interrupt-free measurement:

- **Carrier = TIMER1.** PB10 (CH2) is a 50 % PWM at the drive period; PA5 (CH0)
  is a gate held active for the burst. A finite burst of N periods is ended by
  **TIMER1 update DMA (channel 1)** writing the channel-enable register (CCER)
  on the Nth update, so the burst length is exact and needs no critical section.
- **ADC = ADC0 + DMA channel 0 + TIMER2.** ADC0 is configured once (regular
  sequence ch6 × N, scan mode, external trigger `TIMER2_TRGO`); TIMER2 runs in
  single-pulse mode purely as the one-shot trigger; DMA captures N samples,
  which are median-filtered in software.
- The read is still wrapped in an interrupt lock (short, ~50 µs) so USB ISRs do
  not couple into the sample window.

Safety: ARR is clamped to a safe band, duty is fixed at 50 %, the burst is
capped, the drive pins return to a safe idle state after every measurement, and
the timer is force-disabled after a bounded wait even if the DMA did not
complete.

The carrier period comes from a per-frequency-index table (vendored from the
software NOP sleds) or a raw override (`SET_FREQ_ARR`).

## Scan engine (`scan.c`)

A frame is built from two profiles: `g_prof_b[1..41]` (X, axis B) and
`g_prof_a[1..27]` (Y, axis A). The engine picks one of four acquisition shapes:

- **Full grid** — every coil on both axes; used to acquire a pen anywhere.
- **Window** — only `w` coils around the last peak per axis, the rest zeroed;
  used while tracking.
- **Coarse + fine** — a sparse grid with a rotating phase on a re-acquire frame,
  then a fine window around the best coarse sample (same frame).
- **Repeat coil** — measure one fixed axis-B coil repeatedly and stream the
  history as a time series (a direct "is the noise analog or timing?" probe).

Peak detection is a 3-largest search (`peak_find`). The sub-pixel position is
computed by the selected estimator (`estimator.c`) over the whole window:
N-centroid, vendor rational, Gaomon 5-point, or 3-point log-Gaussian.

### Re-centre strategies (`SET_RECENTER`)

The tracking window follows the pen; how it moves is a runtime choice:

- **0 edge-only** — slide only when the peak reaches the window edge.
- **1 follow** — re-centre on the peak channel every frame; a flat-top hold
  (`SET_FLAT_TOL`) prevents noise from jogging the window.
- **2 sticky** — a Schmitt trigger on the *sub-pixel* position: shift only after
  it stays outside a ±hysteresis band for `persist` frames, then at most `step`
  coils. This breaks the peak → window → scan-order ramp feedback loop.
- **3 deadband** — the legacy distance-deadband shift.

### Re-acquire (`SET_REACQ`)

If the last window peak falls below the threshold, the next frame re-acquires
(full grid, or coarse+fine when a stride ≥ 2 is set). While not tracking, a
minimum re-acquire period (`SET_REACQ` period, ms) gates attempts; between
attempts the scanner is idle and the last profile is re-sent.

### Ramp mitigation (`SET_RAMP`)

A measurement-order amplitude ramp appears when the shared pen tank retains
state between reads. Six+ mitigation modes are selectable one at a time:

| Mode | Name | Mechanism |
|---|---|---|
| 0 | none | baseline |
| 1 | carry-over | free-run acquisition (no SOF gate), tank stays charged |
| 2 | prime per window | long explicit burst(s) on a fixed reference before each axis |
| 3 | per-read steady state | every read's burst is long enough to reach steady state |
| 4 | bidirectional average | measure each window coil ascending then descending, average |
| 5 | slope correction | multiply each sample by a gain linear in scan order |
| 6 | targeted reverse | forward-scan the window, reverse-scan only the estimator coils |
| 7 | frame-alternated average | alternate scan order each frame, average the position |

### Scan order (`SET_SCAN_ORDER`)

0 ascending, 1 descending, and two interleaved orders for windowed scans:
2 outside-in (`X1 Y1 X5 Y5 X2 Y2 X4 Y4 X3 Y3`) and 3 center-out
(`X3 Y3 X2 Y2 X4 Y4 X1 Y1 X5 Y5`). The same coils read in a different order must
not change the profile — a probe for the measurement-order ramp.

## Estimators (`estimator.c`)

All return the position in 1/256-channel units. `estimator_position()` dispatches
on the selected mode:

- **N-centroid** — amplitude-weighted centroid over all window coils after
  subtracting an additive noise base (`SET_NCENTROID`); no calibration.
- **Vendor** — rational interpolator with a per-channel calibration table plus
  amplitude-band corrections and a saturated-peak fallback.
- **Gaomon** — 5-point side-dependent parabolic averaged with a 3-point linear
  offset, with the shared noise base subtracted as a floor.
- **Log-Gaussian** — 3-point `ln(A)` parabola, amplitude- and width-independent,
  with per-axis cubic correction coefficients (`SET_LOGAUSS`).

## Timing

- `dwt.h` provides `dwt_delay_us()`, `dwt_delay_cycles()` and `dwt_now_us()`.
  All software delays and the frame's device timestamp use the DWT cycle counter
  (72 cycles/µs).
- Settle sites A/B/C/D are stored as **DWT cycles** (`acq_set_settle_cycles`),
  so the host can tune them sub-microsecond. `SET_SETTLE` (0x06) writes whole
  microseconds (×72) as a convenience; `SET_SETTLE_CYC` (0x15) writes raw
  cycles. This matters because settle C samples the tank ring-down (~1.9 µs
  period at frequency index 6), so whole-µs steps alias it.
- The frame's `scan_us` is measured around the pure scan, so the host can
  separate device acquisition time from host-side frame period.
- Pacing: by default one acquisition per USB start-of-frame (1 ms). Continuous
  (`SET_PACING` 1) or the carry-over ramp mode free-run the scanner back to
  back; a keep-alive carrier runs through the frame build so the tank does not
  decay between scans.

## Wire protocol

See [`docs/protocol.md`](https://github.com/ig1711/tablet-ab/blob/main/docs/protocol.md)
(canonical) and `src/protocol.h`. Version 7: a 172-byte frame (a self-describing
nx/ny geometry block at offsets 32..35) and 64-byte commands.
