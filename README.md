# car-stereo-controls

> ⚠️ Work in progress

Wireless steering-wheel audio controls for a car head unit.

The steering-wheel buttons can't be wired through to the dash, and the head unit
only accepts commands over its wired SWC (steering-wheel-control) resistor input.
So the link is split in two over BLE:

```
[steering wheel]                         [behind dash, car-powered]
 buttons → XIAO nRF52840  ──BLE──▶  ESP32-C3  ──resistor codes──▶  head unit SWC input
 (coin-cell powered)                (always on, scans)
```

## Subprojects

| Dir | Board | Role |
|-----|-------|------|
| [`transmitter/`](transmitter/) | Seeed XIAO nRF52840 | On the wheel, battery powered. Reads the button cluster and broadcasts presses over connectionless BLE advertising. |
| [`receiver/`](receiver/) | ESP32-C3 | Behind the dash, always powered. Scans for the broadcasts and drives the head unit's SWC input. |

Each subproject is a self-contained PlatformIO project — open the subdirectory
(not this root) in your IDE, or build from inside it:

```bash
cd transmitter && pio run    # or: pio run -t upload
cd receiver     && pio run
```

## How it works

Signal path: button resistance → ADC voltage → BLE packet → mux channel
selection → resistance on the SWC wire → voltage read by the head unit.

### Transmitter (wheel side)

The wheel's 9 buttons form a resistor ladder on a single analog wire: each
button connects a different cumulative resistance (360 Ω–16.46 kΩ) to ground.
The XIAO idles in sleep (~20 µA) and wakes on a button press via a GPIO PORT
event. It then drives the ladder through a 5.1 k series resistor, samples the
node with the ADC (median of 3), and classifies the button by comparing the
resistance against a threshold table.

Button events are broadcast as connectionless BLE advertisements (no pairing,
no connection) carrying 7 bytes of manufacturer data:

```
[FF FF] [42]  [01]    [button] [event]              [seq]
company magic version 0-8      PRESS/HOLD/RELEASE   +1 per event
```

Repeatable buttons (VOL±/SRC±) re-send HOLD every 200 ms while held. Every
frame is re-broadcast unchanged until the next event, which serves as a
heartbeat. The chip returns to sleep 1 s after release.

### Receiver (dash side)

An ESP32-C3 scans passively at 100% duty cycle. The BLE callback filters on
company + magic + version, deduplicates by sequence number (each frame arrives
many times), and writes the newest event into a single-slot mailbox
(overwrite, not a FIFO: queued stale PRESS/RELEASE events would no longer
describe the current button state). A local rotary encoder (volume knob) is
decoded by a quadrature state machine; each detent enqueues one tap into a
ring buffer.

Both inputs are consumed by a single output state machine — the only code
that drives the output pins. A held wheel button is mirrored: its code is
asserted while the button is held, re-asserted only when the button changes,
and released on RELEASE or after 500 ms without a heartbeat (failsafe against
a dead transmitter). Encoder detents are counted and emitted as 50 ms press
pulses with 50 ms spacing between pulses.

### Output stage → head unit

Asserting a button: two 74HC4051 analog multiplexers (shared S0–S2 select
lines; per-mux active-low enables with 10 k pull-ups so both muxes are off
until the firmware initializes) connect one of nine fixed resistors between a
head-unit SWC wire and ground. Electrically this is identical to a wired OEM
button press.

The head unit holds each SWC input at 3.335 V through an internal ~650 Ω
pull-up and reads the divider voltage produced by the pressed button's
resistance (V = 3.335 · (R + R_on) / (R + R_on + 650), with mux
R_on ≈ 130 Ω). In learning mode it stores one voltage per button and later
matches live readings within roughly ±50 mV.

Installed codes:

| Ref | Button    | Mux/channel | Resistor | Wire voltage |
|-----|-----------|-------------|----------|--------------|
| R1  | VOL_UP    | M1 / Y0     | 100 Ω    | 0.87 V |
| R3  | SRC_PUSH  | M1 / Y2     | 220 Ω    | 1.16 V |
| R4  | SRC_UP    | M1 / Y3     | 330 Ω    | 1.38 V |
| R5  | SRC_DOWN  | M1 / Y4     | 470 Ω    | 1.60 V |
| R9  | CALL_END  | M2 / Y2     | 680 Ω    | 1.85 V |
| R8  | ANSWER    | M2 / Y1     | 1 k      | 2.11 V |
| R2  | VOL_DOWN  | M1 / Y1     | 2 k      | 2.55 V |
| R7  | VOICE     | M2 / Y0     | 3.3 k    | 2.80 V |
| R6  | MUTE      | M1 / Y5     | 6.8 k    | 3.05 V |
| —   | idle      | —           | open     | 3.335 V |

(M1 common → SWC wire 1, M2 common → SWC wire 2, shared ground. Support
parts: R10/R11 = 10 k pull-ups on the mux enables, C1/C2 = 100 nF decoupling
at each mux VCC.)

The nine values are chosen for voltage spacing on the head unit's divider —
minimum ~215 mV between any two codes on the combined axis (placed on the
rarely-used SRC pair), ≥ 245 mV around the volume/mute/call buttons, and the
set is collision-free even if the head unit compares both wires against one
learned list. They are unrelated to the wheel's own ladder values: a
resistance code is only meaningful relative to the pull-up that reads it, and
the two sides have different pull-ups (5.1 k vs ~650 Ω) — the wheel's
16.46 kΩ top value would compress to unusably small voltage steps on a 650 Ω
divider.

## Third-party code & assets

Everything not listed below is original work under [MIT](LICENCE).

### Code

- `receiver/src/rotary_encoder.{h,cpp}` — quadrature state tables derived from
  [Ben Buxton's rotary library](https://github.com/buxtronix/arduino/tree/master/libraries/Rotary),
  Copyright 2011 Ben Buxton, **GPL v3**. These two files remain under GPL v3 (it
  cannot be relicensed to MIT); fine for private/personal use, but they carry
  GPL terms if this repo is ever distributed.

### Hardware / CAD

- `circuit/custom/Seeed_Studio_XIAO_Series.kicad_sym` and
  `circuit/custom/XIAO-nRF52840-DIP.kicad_mod` — KiCad schematic symbol and
  footprint from Seeed Studio's official
  [XIAO KiCad libraries](https://wiki.seeedstudio.com/XIAO_BLE/#seeed-studio-xiao-nrf52840)
  ([symbols](https://files.seeedstudio.com/wiki/XIAO-KiCad-Library/XIAO_Series_SCH_Symbols.zip),
  [footprints](https://files.seeedstudio.com/wiki/XIAO-KiCad-Library/New_XIAO_Series_Footprints.zip)),
  © Seeed Studio, [CC BY-SA 4.0](https://wiki.seeedstudio.com/License/)
  (Seeed wiki resources license). The footprint has local modifications;
  as CC BY-SA derivatives, these two files remain CC BY-SA 4.0.

- `circuit/custom/ESP32-C3_SUPERMINI_TH.kicad_sym`,
  `circuit/custom/MODULE_ESP32-C3_SUPERMINI_TH.kicad_mod`, and
  `circuit/custom/ESP32-C3_SUPERMINI_TH.step` — symbol, footprint, and 3D model
  from [SnapMagic Search (SnapEDA)](https://www.snapeda.com/parts/ESP32-C3%20SuperMini_TH/Espressif%20Systems/view-part/),
  [CC BY-SA 4.0 with Design Exception 1.0](https://support.snapeda.com/en/articles/2957814-what-is-the-license-for-symbols-and-footprints)
  (the exception lets designs and boards built with these files be licensed
  freely; the files themselves remain CC BY-SA with attribution).

- `circuit/custom/XIAO-nRF52840 v15.step` — 3D board model from the GrabCAD
  Community Library:
  [Seeed Studio XIAO nRF52840 (Sense)](https://grabcad.com/library/seeed-studio-xiao-nrf52840-sense-1).
  ⚠️ GrabCAD community models are licensed for **private use**; public
  redistribution requires the uploader's permission.

- `3d models/battery_case.FCStd` and `3d models/Battery Enclosure - cr2450.stl`
  — CR2450 battery holder: FreeCAD project created to modify, and the modified
  STL exported from, the
  [CRxxxx battery holder generator](https://cults3d.com/en/3d-model/tool/crxxxx-battery-holder-generator)
  output by **shusy** on Cults3D, **CC BY-NC** — both files remain CC BY-NC.
  `3d models/transmitter_case.FCStd` is original work but embeds the modified
  holder, so it is not usable commercially as a whole.

### Tools

Designed with [KiCad](https://www.kicad.org/) (PCB) and
[FreeCAD](https://www.freecad.org/) (enclosure); the PCB was brought into the
FreeCAD enclosure project with
[kicadStepUp](https://github.com/easyw/kicadStepUpMod/) (easyw, AGPL-3.0).
Firmware built with [PlatformIO](https://platformio.org/). Tool licenses do
not apply to this repo's content.
