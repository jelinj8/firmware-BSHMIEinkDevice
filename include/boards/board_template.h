// Template for a new board/hardware definition.
//
// This project's firmware is meant to be generic across ESP32-S3 (or similar) + e-paper/LCD
// boards, not tied to the CrowPanel 4.2" specifically (see CLAUDE.md's project goal). All
// board-specific facts - display size/depth, product naming, and pin assignments - live in one
// header like this one per board, selected at build time via a `BOARD_*` preprocessor define in
// platformio.ini (see BoardConfig.h). Application code (main.cpp, and later the draw
// engine/storage manager/GPIO controller) should never hardcode a resolution, pin number, or
// product name directly - it should read `board::kWhatever` from whichever header
// BoardConfig.h selected.
//
// To add a new board:
//   1. Copy this file to board_<yourboard>.h and fill in every constant below for real.
//   2. Add an `#elif defined(BOARD_<YOURBOARD>) #include "boards/board_<yourboard>.h"` branch to
//      BoardConfig.h.
//   3. Add a new `[env:...]` in platformio.ini with `-DBOARD_<YOURBOARD>` in build_flags, plus
//      whatever board/memory_type/partition/upload_speed settings that hardware actually needs
//      (see the `esp32-s3-crowpanel` env for what CrowPanel needed and why - some of those
//      settings, like upload_speed, were tuned for CrowPanel's specific USB-serial adapter and
//      may not apply to a different board at all).
//
// This file is never itself compiled in (no BOARD_TEMPLATE define exists) - it's documentation
// via example, not a fallback.
#pragma once

#include <cstdint>

namespace crowpanel {
namespace board {

// ---- Identity --------------------------------------------------------------------------------

// Fixed hardware/firmware-defined model string, doc/PROTOCOL.md §5.2 DEVICE_MODEL. NOT the
// user-configurable device name (§13.3) - that's derived from this board's kNamePrefix below plus
// the MAC address, at runtime, in main.cpp's deviceName().
constexpr const char* kDeviceModel = "YourBoard-Model-Name";

// Prefix for the default (unconfigured) device name, e.g. "YourBoard-" + last 3 MAC bytes in hex.
constexpr const char* kNamePrefix = "YourBoard-";

// ---- Display, doc/PROTOCOL.md §5.2/§6 ----------------------------------------------------------

constexpr uint16_t kDisplayWidthPx = 0;   // TODO: real panel width in pixels
constexpr uint16_t kDisplayHeightPx = 0;  // TODO: real panel height in pixels
constexpr uint8_t kColorDepth = 1;        // bits/pixel - 1 = 1bpp B/W (§5.2 COLOR_DEPTH)

// Physical pixel pitch in micrometers, doc/PROTOCOL.md §5.2 PIXEL_PITCH_X/Y_UM - usually on the
// panel's datasheet (e.g. "0.212 mm" = 212 here). Report the real value, not a derived/rounded
// DPI - clients compute DPI themselves as 25400 / pitch_um. Separate X/Y even if square, in case
// a future panel isn't.
constexpr uint16_t kPixelPitchXUm = 0;  // TODO
constexpr uint16_t kPixelPitchYUm = 0;  // TODO

// Partial-refresh RAM window alignment, doc/PROTOCOL.md §5.2 PARTIAL_REFRESH_GRANULARITY_X/Y -
// driven by the display driver chip, not the panel itself. 8/1 is typical for SSD168x-family
// controllers (X is byte-addressed, Y is pixel-addressed) - confirm against your actual driver's
// datasheet, don't assume.
constexpr uint8_t kPartialRefreshGranularityX = 8;
constexpr uint8_t kPartialRefreshGranularityY = 1;

// ---- E-paper/LCD driver SPI pins (not yet used by any code - draw engine not implemented) ------

constexpr int kPinDisplayCs = -1;    // TODO
constexpr int kPinDisplayDc = -1;    // TODO - data/command select
constexpr int kPinDisplayRst = -1;   // TODO - reset
constexpr int kPinDisplayBusy = -1;  // TODO - busy/ready input
constexpr int kPinDisplaySck = -1;   // TODO - SPI clock (or -1 to use the default VSPI/HSPI pins)
constexpr int kPinDisplayMosi = -1;  // TODO - SPI MOSI (or -1 to use the default)

// Some boards gate the display FPC's 3.3V/level-shifting through its own enable GPIO (distinct
// from the SPI lines above) rather than leaving it always powered - CrowPanel does. -1 if this
// board has no such control (display is simply always powered whenever the board is).
constexpr int kPinDisplayPowerCtl = -1;  // TODO

// ---- TF/SD card (not yet used by any code - storage manager not implemented) --------------------
//
// Modeled as its own independent 4-wire SPI bus (CrowPanel wires the SD card and the display to
// completely separate SCK/MOSI/MISO/CS sets, not a shared bus) - a board that DOES share one
// physical SPI bus between the display and the SD card should just set kPinSdSck/kPinSdMosi equal
// to kPinDisplaySck/kPinDisplayMosi above; kPinSdMiso still needs its own value even then, since
// e-paper panels are typically write-only and the display pins above have no MISO at all.

constexpr int kPinSdCs = -1;    // TODO; -1 if this board has no card slot at all
constexpr int kPinSdSck = -1;   // TODO
constexpr int kPinSdMosi = -1;  // TODO
constexpr int kPinSdMiso = -1;  // TODO

// Same idea as kPinDisplayPowerCtl above, but for the TF card's 3.3V/level-shifting. -1 if none.
constexpr int kPinSdPowerCtl = -1;  // TODO

// ---- Buttons, doc/PROTOCOL.md §11 BUTTON_ID (not yet used by any code - button events not
// implemented). Use -1 for any button this board doesn't have.
//
// A "dial switch" is modeled as three independent digital inputs (scroll up / scroll down /
// press-confirm), matching how CrowPanel's actually wires one (see design note 43) - a board with
// a true rotary-encoder or single combined dial signal can leave kPinButtonDialUp/Down at -1 and
// use only kPinButtonDialConfirm (BUTTON_ID DIAL_SWITCH).

constexpr int kPinButtonDialUp = -1;
constexpr int kPinButtonDialDown = -1;
constexpr int kPinButtonDialConfirm = -1;
constexpr int kPinButtonMenu = -1;
constexpr int kPinButtonBack = -1;
// Boot/Reset are typically hardware strap/EN pins, not application-readable GPIOs - see
// doc/PROTOCOL.md §11's note on why BUTTON_ID still reserves IDs for them anyway (other boards
// may wire them differently).

// ---- Onboard status LED (not yet used by any code - no GPIO controller/pattern player exists
// yet, doc/PROTOCOL.md §15.5). This is a fixed board-level indicator, distinct from any
// user-wired indicator on the general-purpose header below (which GPIO_PLAY_PATTERN targets by
// PIN_ID, not through this constant). -1 if this board has none.

constexpr int kPinStatusLed = -1;

// ---- General-purpose GPIO header, doc/PROTOCOL.md §5.2 AVAILABLE_GPIO_PINS / §15 (not yet wired
// into the handshake or any GPIO controller - see design note 43). Raw MCU GPIO numbers safe to
// expose to PC-driven GPIO_CONFIGURE/WRITE/READ, i.e. not already committed to the display SPI
// bus, SD card, buttons, or LED above. Empty array for a board with no such header.

constexpr int kAvailableGpioPins[] = {};
constexpr size_t kAvailableGpioPinsCount = sizeof(kAvailableGpioPins) / sizeof(kAvailableGpioPins[0]);

}  // namespace board
}  // namespace crowpanel
