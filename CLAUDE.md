# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ESP32-S3 firmware (PlatformIO, Arduino framework) that turns a small e-paper/LCD HMI display into a
generic, remote-controlled display device — driven over WiFi/TCP, Bluetooth Low Energy, or Serial by
a PC-side client (the sibling `java-bshmidriver` library, or any other client speaking the same wire
protocol). Meant to be generic across similar ESP32-S3 + e-paper/LCD boards, not tied to CrowPanel
specifically, even though the CrowPanel 4.2" e-paper display is the first (and so far only) board
this has actually run on real hardware.

The wire protocol itself (frame envelope, all command payloads, TLV capability negotiation) is
specified in `doc/PROTOCOL.md` in **this** repository — the canonical, single source of truth both
this firmware and the sibling `java-bshmidriver` PC-side library (an independent implementation of
the same spec) must stay byte-for-byte consistent with. This repository is the main published
asset the spec lives alongside; `java-bshmidriver` references it from here rather than duplicating
it.

## Repository layout

- `doc/PROTOCOL.md` — the canonical wire protocol specification. Read this before changing any
  command payload, status code, or TLV field — `java-bshmidriver` must stay byte-for-byte
  consistent with whatever changes here. `doc/` also holds hardware reference material for the
  CrowPanel 4.2" board (SSD1683 datasheet, product manual, 3D case files).
- `lib/Protocol/` — transport-independent protocol logic (frame envelope, CRC16, RLE codec,
  command IDs, status codes), a PlatformIO private library so it links into both the real firmware
  build and the native unit tests without an ESP32 toolchain.
- `lib/Transport/` — the Stream-based frame transport (drives Serial, TCP, *and* BLE unchanged).
  BLE uses `h2zero/NimBLE-Arduino` (pinned in `platformio.ini`'s `lib_deps`) via
  `NimBLEStreamServer`, which wraps the GATT characteristic as a plain `Stream` — no BLE-specific
  code needed in the transport layer itself.
- `lib/Dispatcher/` — the `COMMAND_ID -> handler` table and `CommandContext` (request frame,
  `ACTIVE_TRANSPORT`, `ack()`/`nack()`/`reply()` helpers).
- `lib/Display/`, `lib/Storage/`, `lib/Gpio/`, `lib/Buttons/`, `lib/Macro/` — the draw engine
  (working buffer + `Adafruit_GFX` adapter), storage manager (SD/INTERNAL/PSRAM volumes), GPIO
  controller, button debouncing/event detection, and macro recording/playback, respectively.
- `include/boards/` — **all hardware-specific facts (display size/depth, product naming, pin
  assignments) are externalized here**, one header per board (`board_<name>.h`), selected at build
  time via `BoardConfig.h` + a `-DBOARD_*` flag in `platformio.ini`'s `build_flags`. Application
  code must never hardcode a resolution or pin number — read `board::kWhatever` instead. Adding
  support for a different board means copying `boards/board_template.h` (documents every required
  field) to a new `board_<name>.h`, adding it to `BoardConfig.h`'s `#if` chain, and adding a new
  `platformio.ini` env — not forking the firmware.
- `boards/crowpanel_4_2.json` — a real, custom PlatformIO board profile (MCU/flash/PSRAM/upload
  metadata only, no GPIO wiring — that's `include/boards/board_crowpanel_4_2.h`, a different thing
  with a similar name).
- `src/main.cpp` — brings up WiFi (STA mode; real credentials go in a gitignored
  `include/secrets.h`, copy `secrets.h.example`), a TCP server, and a BLE peripheral, and wires all
  three to the same dispatcher.

### Build & test

```bash
# Full firmware build (ESP32-S3 target, PlatformIO's bundled Espressif toolchain - no hardware
# needed just to build). PlatformIO is installed via the VS Code extension; its CLI lives at
# %USERPROFILE%\.platformio\penv\Scripts\pio.exe if it's not on PATH.
pio run -e esp32-s3-crowpanel

# Protocol-logic unit tests, host-native (no ESP32 involved) - requires a system C++ compiler
# (MSVC/LLVM/MinGW).
pio test -e native

# Flash + watch boot output on real hardware (CH340 USB-serial adapter on the CrowPanel board;
# adjust the port). upload_speed is pinned to 115200 in platformio.ini - CH340 adapters are prone
# to dropouts at esptool's default higher speed.
pio run -e esp32-s3-crowpanel -t upload --upload-port COM5
pio device monitor -p COM5 -b 115200
```

## Status

All three transports (Serial, WiFi/TCP, BLE) live-verified end-to-end and simultaneously functional
on one firmware image against a real CrowPanel 4.2" board; the full drawing/storage/GPIO/OTA/
power-management/macro command surface implemented and verified on real hardware. See
`doc/PROTOCOL.md` §22 "Implementation status" for the exhaustive, per-feature verification history
this firmware's development produced.
