// Board definition: CrowPanel 4.2" E-paper HMI Display (ESP32-S3-WROOM-1-N8R8, SSD1683).
// See board_template.h for what every field means and how to add a different board.
//
// Display specs are real (from CLAUDE.md's hardware reference / the Elecrow wiki) and confirmed on
// real hardware via the handshake TLV round trip (doc/PROTOCOL.md §21). All pin assignments below
// (display SPI, SD card SPI, buttons, LED, GPIO header) are now real, taken directly from the
// board's own schematic (next-steps.md #2) - none of them have been confirmed by probing the
// physical board itself yet, only by reading the schematic, so still worth a multimeter/continuity
// check before trusting them for anything irreversible (e.g. driving a pin the schematic actually
// has tied to something else).
#pragma once

#include <cstddef>
#include <cstdint>

namespace crowpanel {
namespace board {

// ---- Identity --------------------------------------------------------------------------------

constexpr const char* kDeviceModel = "CrowPanel-4.2-EPD";
constexpr const char* kNamePrefix = "CrowPanel-";

// ---- Display, doc/PROTOCOL.md §5.2/§6 ----------------------------------------------------------

constexpr uint16_t kDisplayWidthPx = 400;
constexpr uint16_t kDisplayHeightPx = 300;
constexpr uint8_t kColorDepth = 1;  // black & white AM EPD

// Physical pixel pitch, doc/PROTOCOL.md §5.2 PIXEL_PITCH_X/Y_UM - from the spec sheet's
// "Pixel pitch 0.212*0.212" (mm), i.e. 212 micrometers, square. A client derives DPI itself
// (25400 / pitch_um ≈ 120 DPI here) - see design note 42, this isn't reported pre-computed.
constexpr uint16_t kPixelPitchXUm = 212;
constexpr uint16_t kPixelPitchYUm = 212;

// SSD1683: X (byte-addressed RAM window) aligns to 8px, Y (pixel-addressed) aligns to 1px.
constexpr uint8_t kPartialRefreshGranularityX = 8;
constexpr uint8_t kPartialRefreshGranularityY = 1;

// ---- E-paper driver (SSD1683) SPI pins ----------------------------------------------------------
// From the "EPD interface" block of the board schematic (U4, the 0.5mm-pitch 24-pin FPC
// connector) - net names on the ESP32 side read straight off as IO45_CS, IO46_D/C, IO47_RES,
// IO48_BUSY, IO12_SPI_CLK, IO11_SPI_MOSI. This is its own dedicated SPI bus, separate from the SD
// card's below (not shared) - no MISO pin exists for the display (e-paper panels are write-only).

constexpr int kPinDisplayCs = 45;
constexpr int kPinDisplayDc = 46;
constexpr int kPinDisplayRst = 47;
constexpr int kPinDisplayBusy = 48;
constexpr int kPinDisplaySck = 12;
constexpr int kPinDisplayMosi = 11;

// GPIO7 ("IO7_LCD_3.3_CTL" on the schematic - "LCD" here just means the EPD FPC, reused symbol
// naming) gates a level-shifter/power stage feeding the display FPC.
constexpr int kPinDisplayPowerCtl = 7;

// ---- TF/SD card -----------------------------------------------------------------------------------
// From the "TF Card" block (J1, micro-SD socket) - its own independent SPI bus (IO10_SPI_CS,
// IO40_SPI_MOSI, IO39_SPI_CLK, IO13_SPI_MISO on the schematic), not shared with the display above.

constexpr int kPinSdCs = 10;
constexpr int kPinSdSck = 39;
constexpr int kPinSdMosi = 40;
constexpr int kPinSdMiso = 13;

// GPIO42 ("IO42_TF_3.3_CTL") gates the TF card's own level-shifter/power stage, mirroring
// kPinDisplayPowerCtl above.
constexpr int kPinSdPowerCtl = 42;

// ---- Buttons, doc/PROTOCOL.md §11 BUTTON_ID -----------------------------------------------------
// From the board schematic. "Dial switch" is wired as three plain GPIOs (scroll up/down + press),
// not a rotary-encoder or single-button signal - see design note 43. RESET is the chip's EN pin -
// pulling it resets the whole MCU, so firmware can never observe that press (there's no "after" to
// read it in) - genuinely not wired here. BOOT (GPIO0) is different: it's only sampled at reset to
// choose ROM-bootloader vs normal boot, so once running it's a perfectly ordinary, safely-readable
// GPIO - the standard "boot button doubles as a user button" pattern on ESP32 boards (see design
// note 69) - wired here like any other button.

constexpr int kPinButtonDialUp = 6;        // silkscreen "UP"
constexpr int kPinButtonDialDown = 4;      // silkscreen "DOWN"
constexpr int kPinButtonDialConfirm = 5;   // silkscreen "CONF"
constexpr int kPinButtonMenu = 2;          // schematic net "IO2_MENU"
constexpr int kPinButtonBack = 1;          // schematic net "IO1_EXIT"
constexpr int kPinButtonBoot = 0;          // GPIO0, schematic net "BOOT" - safe to read once running

// ---- Onboard status LED --------------------------------------------------------------------------

constexpr int kPinStatusLed = 41;

// ---- General-purpose GPIO header, doc/PROTOCOL.md §5.2 AVAILABLE_GPIO_PINS / §15 -----------------
// The 2x10, 2.54mm expansion header (U5/P20-P35 on the schematic), plus kPinStatusLed - design
// note 43 originally recorded that LED as PC-controllable via GPIO_PLAY_PATTERN once a GPIO
// controller existed; it now does, so it's included here rather than reserved for firmware's own
// use - gives a built-in GPIO_WRITE/GPIO_PLAY_PATTERN test target with no external LED needed.

constexpr int kAvailableGpioPins[] = {3, 8, 9, 14, 15, 16, 17, 18, 19, 20, 21, 38, kPinStatusLed};
constexpr size_t kAvailableGpioPinsCount = sizeof(kAvailableGpioPins) / sizeof(kAvailableGpioPins[0]);

}  // namespace board
}  // namespace crowpanel
