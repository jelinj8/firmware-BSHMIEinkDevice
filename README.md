# BSHMIEinkDevice

ESP32-S3 firmware (PlatformIO, Arduino framework) turning a small e-paper/LCD HMI display into a
generic, remote-controlled display device — driven over WiFi/TCP, Bluetooth Low Energy, or Serial.

## Project goal

Most small e-paper/LCD "smart display" firmwares are written for one specific board and one
specific application. This firmware instead implements a fully specified, versionable wire protocol
(frame envelope, TLV capability negotiation, image transfer, local drawing primitives, storage,
GPIO, OTA, power management — see `doc/PROTOCOL.md`, right here in this repository) and keeps every
hardware-specific fact — display size/depth, pin assignments, product naming — externalized to a
per-board header, so the same firmware core can run on any similar ESP32-S3 + e-paper/LCD board,
not just one.

- Images are composed on the **PC side**; the device is primarily a "dumb" display target, but the
  protocol also supports button-press event listeners (device → PC), local drawing commands (draw
  primitives on-device, not just raster blits), configuration commands, and sensor extensions
  (device → PC).
- The PC side (a generic Java client, `java-bshmidriver`) and this firmware are two independent
  implementations of the exact same wire protocol spec — kept in lockstep by convention, not a
  build dependency.

## Supported boards

- **CrowPanel 4.2" E-paper E-ink HMI Display** — the first, and so far only, board this has
  actually run on real hardware. ESP32-S3-WROOM-1-N8R8, 400×300 1bpp e-paper (SSD1683 driver chip),
  partial refresh, UART/TF-card/GPIO expansion header.

Adding a new board means copying `include/boards/board_template.h` (documents every required
field) to a new `board_<name>.h`, adding it to `include/boards/BoardConfig.h`'s `#if` chain, and
adding a new `platformio.ini` `[env:...]` with its own `-DBOARD_*` build flag — not forking the
firmware. See `CLAUDE.md` for the full repository layout and the board-abstraction convention in
detail.

## Building

```bash
pio run -e esp32-s3-crowpanel                                    # build
pio run -e esp32-s3-crowpanel -t upload --upload-port COM5       # flash real hardware
pio device monitor -p COM5 -b 115200                              # watch boot/serial output
pio test -e native                                                 # host-native protocol-logic unit tests
```

PlatformIO is installed via the VS Code extension; its CLI lives at
`%USERPROFILE%\.platformio\penv\Scripts\pio.exe` if it's not on `PATH`. Real WiFi credentials go in
a gitignored `include/secrets.h` — copy `include/secrets.h.example` and fill it in.

## Wire protocol

The full protocol specification — frame envelope, all command payloads, TLV capability negotiation
— lives in `doc/PROTOCOL.md`, right here in this repository (the main published asset it belongs
alongside). This firmware and the PC-side `java-bshmidriver` library are independent
implementations of that one spec; `java-bshmidriver` references this copy rather than duplicating
it.

## Status

All three transports (Serial, WiFi/TCP, BLE) live-verified end-to-end and simultaneously functional
on one firmware image against a real CrowPanel 4.2" board: full-frame and partial image transfer,
local drawing primitives (lines/rects/circles/text/images), storage (SD/internal flash/PSRAM),
GPIO configuration and button/GPIO change events, OTA updates with hash verification, power
management (light/deep sleep with per-peripheral rail gating), macro recording/playback including
event-triggered auto-play, and two-tier PIN access control. See `doc/PROTOCOL.md` §22
"Implementation status" for the complete, per-feature verification history.

## License

MIT, see `LICENSE`.
