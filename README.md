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
