// CrowPanel firmware entry point.
//
// Bring-up stub, now with three live transports and a real command dispatch table (Dispatcher.h,
// doc/PROTOCOL.md §4): proves the Protocol/Transport libraries link and run under the real
// ESP32-S3 Arduino toolchain, that the board is provisioned correctly for OTA updates (§16) and
// internal storage (§14), and that a PC client can exchange frames with it over Serial (§3.3),
// WiFi/TCP (§3.2), and BLE (§3.1/§19). Commands handled so far: HANDSHAKE_REQUEST ->
// HANDSHAKE_RESPONSE, FULL_IMAGE_TRANSFER (§6), PARTIAL_IMAGE_TRANSFER (§7), READ_SCREEN (§8, both
// SOURCE=PANEL/WORKING_BUFFER), CLEAR_ARTIFACTS (§9, degauss flash cycle), REFRESH (§12.8), the
// local drawing primitives DRAW_LINE/DRAW_RECT/DRAW_CIRCLE/CLEAR_REGION/DRAW_TEXT (§12.2-§12.6),
// SHIFT_REGION (§12.9, in-place scrolling), SET_CLIP_REGION (§12.10, constrains every subsequent
// write to a sub-rectangle), COPY_REGION (§12.11), SET_ORIENTATION (§12.12, rotates/mirrors the
// logical canvas - e.g. for portrait mounting), and SET_DRAW_OFFSET (§12.13, pans the logical
// canvas, e.g. for scrolling content partly off-canvas) - all but the handshake go through
// WorkingBuffer.h's persistent MCU-side working buffer (§2.1), which tracks the union of regions
// deferred via FLAGS.REFRESH_NOW=0 for REFRESH to later flip and enforces the current offset/clip/
// orientation pipeline on every read/write; the drawing primitives (including DRAW_TEXT, via
// EmbeddedFont.h) additionally go through WorkingBufferGfx.h, an Adafruit_GFX adapter reusing its
// Bresenham/midpoint-circle rasterizers. Also handled: FILE_LIST_REQUEST/FILE_DOWNLOAD_REQUEST/
// FILE_UPLOAD/FILE_DELETE/STORAGE_INFO_REQUEST (§14, StorageManager.h - VOLUME=SD via its own
// independent SPI bus and treated as hot-pluggable, VOLUME=INTERNAL/LittleFS, and VOLUME=PSRAM, a
// flat session-only in-memory volume), and RECORD_MACRO/SAVE_MACRO/PLAY_MACRO/PAUSE (§0x0A00,
// MacroRecorder.h/MacroPlayer.h - captures/replays dispatched command frames as a scriptable
// sequence, non-blocking during playback like GPIO_PLAY_PATTERN's own planned design; a fixed-name
// boot macro on SD or INTERNAL, if present, plays automatically on cold start). All registered
// onto gDispatcher below; everything else gets Dispatcher's own fallback, NACK(UNSUPPORTED_COMMAND).
// No GPIO controller yet, and BLE pairing/security is not implemented (§19's "not yet wired up"
// note) - see doc/PROTOCOL.md §21 "Implementation status".
//
// IMPORTANT (temporary, see doc/PROTOCOL.md §3.3's "dedicated UART" note): this board exposes
// only one physical UART (bridged to USB via the CH340 also used for flashing), so it's shared
// for now between the one-shot plain-text boot log below (self-test + WiFi/BLE bring-up status)
// and the framed binary protocol - they are NOT interleaved (the boot log finishes and prints one
// final human-readable marker line before the transport starts using this same UART exclusively
// for binary frames). Revisit once a second UART or native USB-CDC logging channel is confirmed
// available on this board, per §3.3.
#include <Arduino.h>
#include <ESPmDNS.h>
#include <GxEPD2_BW.h>
#include <LittleFS.h>
#include <NimBLEDevice.h>
#include <NimBLEStream.h>
#include <Preferences.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <driver/uart.h>
#include <mbedtls/md5.h>
#include <mbedtls/sha256.h>
#include <gdey/GxEPD2_420_GDEY042T81.h>
#include <algorithm>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

#include "BoardConfig.h"
#include "Crc16.h"
#include "Dispatcher.h"
#include "Protocol.h"
#include "RlePackBits.h"
#include "StreamFrameTransport.h"
#include "EmbeddedFont.h"
#include "ButtonController.h"
#include "GpioController.h"
#include "MacroPlayer.h"
#include "MacroRecorder.h"
#include "StorageManager.h"
#include "WorkingBuffer.h"
#include "WorkingBufferGfx.h"

// WiFi credentials are kept out of the repo (see firmware/include/secrets.h.example /
// .gitignore). Falls back to empty credentials - WiFi.begin() will simply fail and TCP stays
// unavailable - rather than a build error, so this still compiles for anyone who hasn't set up
// secrets.h yet (e.g. to work Serial-only).
#if __has_include("secrets.h")
#include "secrets.h"
#else
constexpr char kWifiSsid[] = "";
constexpr char kWifiPassword[] = "";
#endif

using namespace crowpanel;

namespace {

// Was duplicated inline in two places (the handshake TLV builder, OTA_STATUS_RESPONSE) with a
// comment asking the reader to keep them in sync by hand - now also used by the cold-boot screen
// (design note 83), a third place, so worth the one shared constant instead. __DATE__ " " __TIME__
// (adjacent string literals, concatenated at compile time - no runtime cost) appends a real build
// timestamp, requested directly: "The build identity would be nice for versioning (build date
// info)" - useful since the human-maintained "0.1.0-dev" part hasn't actually been bumped all
// project, so it alone can't tell two different builds apart the way this now can.
constexpr char kFirmwareVersion[] = "0.1.0-dev (" __DATE__ " " __TIME__ ")";

constexpr uint16_t kTcpPort = 5577;  // doc/PROTOCOL.md §3.2 - no well-known port, this project's choice
constexpr unsigned long kWifiConnectTimeoutMs = 15000;
constexpr unsigned long kOtaConfirmTimeoutMs = 5 * 60 * 1000;	// §16.4 auto-rollback safety net
constexpr uint16_t kBlePreferredMtu = 247;	 // doc/PROTOCOL.md §3.1 - NimBLE-Arduino's usual practical ceiling
constexpr uint32_t kBleStreamBufSize = 1024;  // NimBLEStreamServer's own default; fine for now, see §3.1

StreamFrameTransport gSerialTransport(Serial);

std::unique_ptr<WiFiServer> gTcpServer;
WiFiClient gTcpClient;
std::unique_ptr<StreamFrameTransport> gTcpTransport;

NimBLEStreamServer gBleStream;
StreamFrameTransport gBleTransport(gBleStream);

// doc/PROTOCOL.md §4 command dispatch table - handlers registered once in setup(), see the bottom
// of this file. Shared by all three transports; each just passes itself + its own ACTIVE_TRANSPORT
// value (§5.2) through to dispatch().
Dispatcher gDispatcher;

// SSD1683 e-paper driver (next-steps.md #3). GxEPD2_420_GDEY042T81 (400x300, SSD1683) is
// hardcoded here rather than made per-board like everything else in BoardConfig.h/board_*.h - the
// board-abstraction layer doesn't yet have a slot for "which driver class", since CrowPanel is
// still the only board with any display code at all. Worth promoting to a per-board selection
// (e.g. a driver-class typedef in board_*.h) the day a second SSD16xx-family board shows up
// needing a different GxEPD2 class - no need to build that generality before then. Template height
// = full panel height: at 400x300/1bpp (15000 bytes) the whole frame comfortably fits in one
// buffer given 8 MB PSRAM, so no page-by-page buffering is needed.
GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT> gDisplay(GxEPD2_420_GDEY042T81(
		board::kPinDisplayCs, board::kPinDisplayDc, board::kPinDisplayRst, board::kPinDisplayBusy));

// doc/PROTOCOL.md §2.1 - see WorkingBuffer.h for why this exists as an explicit MCU-side buffer
// rather than "whatever's in the SSD1683's own RAM".
WorkingBuffer gWorkingBuffer(gDisplay);

// doc/PROTOCOL.md §12 local drawing primitives - see WorkingBufferGfx.h.
WorkingBufferGfx gWorkingBufferGfx(gWorkingBuffer);

// doc/PROTOCOL.md §14 - see StorageManager.h.
StorageManager gStorageManager;

// doc/PROTOCOL.md §0x0A00 - see MacroRecorder.h/MacroPlayer.h.
MacroRecorder gMacroRecorder;
MacroPlayer gMacroPlayer;

// doc/PROTOCOL.md §15 - see GpioController.h.
GpioController gGpioController(board::kAvailableGpioPins, board::kAvailableGpioPinsCount);

// doc/PROTOCOL.md §11 - see ButtonController.h. RESET is the chip's EN pin - pulling it resets the
// whole MCU, so firmware can never observe that press and it's genuinely not wired here. BOOT
// (GPIO0) is only sampled at reset to choose ROM-bootloader vs normal boot; once running it's a
// perfectly ordinary GPIO, so it's wired in like any other button - see design note 69.
ButtonController gButtonController({
		{board::kPinButtonMenu, buttonId::kMenu},
		{board::kPinButtonBack, buttonId::kBack},
		{board::kPinButtonDialUp, buttonId::kDialUp},
		{board::kPinButtonDialDown, buttonId::kDialDown},
		{board::kPinButtonDialConfirm, buttonId::kDialSwitch},
		{board::kPinButtonBoot, buttonId::kBoot},
});

// A Stream that discards every write and never has anything to read - used as the "transport" for
// dispatching a macro-replayed command (main.cpp's stepMacroPlayback()), which has no live caller
// anywhere waiting for its ACK/NACK/reply. Print's default multi-byte write() loops over the
// single-byte overload below, so overriding just that one is enough to make StreamFrameTransport's
// send() see a normal "fully written" result rather than retrying/stalling.
class NullStream : public Stream {
public:
	int available() override { return 0; }
	int read() override { return -1; }
	int peek() override { return -1; }
	size_t write(uint8_t) override { return 1; }
};
NullStream gNullStream;
StreamFrameTransport gMacroTransport(gNullStream);

// doc/PROTOCOL.md §13.2 SET_BLE_ENABLED: whether BLE is currently meant to be discoverable/
// connectable at all - distinct from NimBLEAdvertising::isAdvertising(), which legitimately reads
// false while a central is already connected (legacy, non-extended advertising stops during a
// connection) even though BLE is still "enabled" in the §13.2 sense. Read by onDisconnect below to
// decide whether restarting advertising is actually wanted.
bool gBleEnabled = true;

// doc/PROTOCOL.md §5.3: the access level each transport's current connection has been granted,
// reset to kNone whenever that transport's connection is (re-)established - see each reset site's
// own comment (pollTcp(), BleServerCallbacks below, setup()) for why each one is the right point.
// There is only ever one live client per transport kind in this codebase (pollTcp() rejects a
// second TCP client outright; BLE/Serial are singletons), so one global per transport kind is
// sufficient - declared here, ahead of BleServerCallbacks, since it needs gBleAuthLevel below.
uint8_t gTcpAuthLevel = authLevel::kNone;
uint8_t gBleAuthLevel = authLevel::kNone;
uint8_t gSerialAuthLevel = authLevel::kNone;

// Legacy (non-extended) advertising stops while a central is connected; restart it on disconnect
// so a second central (or the same one reconnecting) can still find the device - but only if BLE
// hasn't been explicitly disabled via SET_BLE_ENABLED(0) in the meantime. Pairing/security
// (doc/PROTOCOL.md §19) isn't wired up yet, so this callback otherwise has nothing else to do.
// onConnect (doc/PROTOCOL.md §5.3, new) relocks gBleAuthLevel for the fresh connection - this
// class previously had no onConnect override at all.
class BleServerCallbacks : public NimBLEServerCallbacks {
	void onConnect(NimBLEServer* /*server*/, NimBLEConnInfo& /*connInfo*/) override {
		gBleAuthLevel = authLevel::kNone;
	}

	void onDisconnect(NimBLEServer* /*server*/, NimBLEConnInfo& /*connInfo*/, int /*reason*/) override {
		gBleAuthLevel = authLevel::kNone;
		if (gBleEnabled) {
			NimBLEDevice::startAdvertising();
		}
	}
};
BleServerCallbacks gBleServerCallbacks;

void selfTest() {
	// Shared CRC16 reference vector (also asserted in pc-java-lib's Crc16Test and firmware's
	// native Unity test) - if this ever prints a mismatch, the two implementations have drifted.
	const uint8_t vector[] = "123456789";
	uint16_t crc = crc16(vector, 9);
	Serial.printf("CRC16(\"123456789\") = 0x%04X (expected 0x29B1) %s\n", crc,
			crc == 0x29B1 ? "OK" : "MISMATCH");

	Frame frame;
	frame.commandId = cmd::kHandshakeRequest;
	frame.seq = 0;
	std::vector<uint8_t> encoded = frame.encode();

	Frame decoded;
	FrameError err = Frame::decode(encoded.data(), encoded.size(), decoded);
	Serial.printf("Frame round-trip: %s\n", err == FrameError::kNone ? "OK" : "FAILED");

	Serial.printf("PSRAM: found=%s size=%u bytes free=%u bytes\n", psramFound() ? "yes" : "no",
			ESP.getPsramSize(), ESP.getFreePsram());

	// Without this, plain malloc()/new (and every std::vector<uint8_t> using the default
	// allocator - notably StreamFrameTransport's receive buffer and Frame::payload, which both
	// have to hold an entire Logical Frame at once, doc/PROTOCOL.md §2/§3.2/§3.3) stay confined
	// to internal SRAM even though PSRAM is present and initialized above: this board's qio_opi
	// PSRAM profile does not itself configure malloc()/new to draw from PSRAM (that's a separate
	// ESP-IDF option this project's board profile doesn't set). Internal SRAM is only ~512 KB
	// total on the ESP32-S3, so anything past what's actually free there - confirmed on real
	// hardware to be somewhere between 450 KB and 600 KB for this firmware's own SRAM footprint -
	// silently fails to allocate and the firmware just stops producing any output (no crash, no
	// reset, no watchdog trip - std::vector's reallocation has nothing sensible to do). This bit
	// everything using an unbounded-size default-allocator std::vector as soon as a real OTA
	// payload (~1 MB, well past that threshold) was tried, even though small commands worked
	// fine. Threshold chosen well below any of those buffers' typical sizes so they always land
	// in PSRAM, while small/short-lived allocations elsewhere keep using faster internal SRAM.
	heap_caps_malloc_extmem_enable(8 * 1024);

	// Proves the default_8MB.csv partition table (platformio.ini) is actually in effect: an OTA
	// update (doc/PROTOCOL.md §16) needs a real inactive partition to write into, not just "the
	// build fit in flash".
	const esp_partition_t* running = esp_ota_get_running_partition();
	const esp_partition_t* next = esp_ota_get_next_update_partition(nullptr);
	Serial.printf("OTA: running=%s (0x%06x, %u bytes) next-update-slot=%s (0x%06x, %u bytes) "
				  "max-new-image=%u bytes\n",
			running ? running->label : "?", running ? running->address : 0,
			running ? running->size : 0, next ? next->label : "?", next ? next->address : 0,
			next ? next->size : 0, (unsigned) ESP.getFreeSketchSpace());

	// Proves the LittleFS data partition (doc/PROTOCOL.md §14 VOLUME INTERNAL) is present and
	// mountable. `true` formats it on first mount if the partition has never been formatted.
	bool fsOk = LittleFS.begin(true);
	Serial.printf("LittleFS (INTERNAL storage): mounted=%s total=%u bytes used=%u bytes\n",
			fsOk ? "yes" : "no", fsOk ? (unsigned) LittleFS.totalBytes() : 0,
			fsOk ? (unsigned) LittleFS.usedBytes() : 0);
}

// deviceName() is defined further down (after Preferences/gRuntimeDeviceNameOverride are set up) -
// forward-declared so the cold-boot screen below, which runs earlier in setup(), can call it.
std::string deviceName();

// wifiStatusLabel()/bleStatusLabel() are defined further down (after Preferences/kPrefKey* are set
// up), forward-declared for the same reason as deviceName() above.
std::string wifiStatusLabel();
std::string bleStatusLabel();

// SSD1683 bring-up (next-steps.md #3, doc/PROTOCOL.md §6/§12): proves the display SPI wiring and
// driver init independently of the protocol layer, and shows a cold-boot info screen identifying
// the loaded firmware, device, and resolution - requested directly: "add a cold boot screen...
// use opaque drawing" (design note 83), then simplified further, design note 85: "Current boot
// procedure should just draw the version infos with opaque font, no need to redraw the full
// screen" - only the text's own bounding box is flushed now (partial, not full), not the whole
// panel; on a fresh flash, tryPlayInitMacro() (setup(), below) takes over right after with its own
// "sleep 1s, clear, degauss" sequence, which already forces a real full-panel refresh of its own,
// so this screen doesn't need to do that itself. No separate full-black-then-full-white flash
// first either ("Is the multistep redraw init needed?", asked directly, design note 83): GxEPD2's
// own _initial_write/_initial_refresh flags (gdey/GxEPD2_420_GDEY042T81.cpp) already force the
// very first write+refresh after init() through a full update automatically regardless of what
// this call itself requests, so a preliminary black/white pass was never actually needed for a
// clean result - confirmed clean on real hardware, not just assumed. Also shows WiFi/BLE status
// (wifiStatusLabel()/bleStatusLabel(), below) - requested directly: "default boot macro could also
// show wifi off/unconfigured/on, BLE off/on" - added to this native screen rather than a macro,
// since this is where firmware/device/resolution were already being shown. Both labels read only
// persisted configuration, not live connection state, since this runs before
// connectWifiAndStartTcpServer()/setupBle() (setup(), below) ever touch the radios.
void displaySelfTest() {
	if (board::kPinDisplayCs < 0) {
		Serial.println("Display: pins not configured for this board - skipping self-test");
		return;
	}

	if (board::kPinDisplayPowerCtl >= 0) {
		pinMode(board::kPinDisplayPowerCtl, OUTPUT);
		digitalWrite(board::kPinDisplayPowerCtl, HIGH);  // enable the FPC's level-shifter/power stage
		delay(10);	// SSD1683 datasheet §9.1 step 1: "Supply VCI, Wait 10ms"
	}

	// GxEPD2_420_GDEY042T81's SCK/MOSI (board::kPinDisplaySck/Mosi = 12/11) are this chip's default
	// hardware SPI pins (see pins_arduino.h's SCK/MOSI/MISO/SS constants) - no custom SPIClass or
	// selectSPI() needed, the default global `SPI` object (GxEPD2_EPD's implicit default) already
	// lands on the right pins. serial_diag_bitrate=0: Serial is already Serial.begin()'d above, no
	// need for GxEPD2 to redundantly re-init it.
	Serial.println("Display: initializing SSD1683 (GDEY042T81 panel)...");
	gDisplay.init(/*serial_diag_bitrate=*/0, /*initial=*/true, /*reset_duration=*/10, /*pulldown_rst_mode=*/false);

	unsigned long start = millis();
	char line[192];  // kFirmwareVersion carries a build timestamp; +WIFI/BLE status lines below
	int len = snprintf(line, sizeof(line), "FW  %s\nDEV %s\nRES %ux%u\nWIFI %s\nBLE  %s", kFirmwareVersion,
			deviceName().c_str(), board::kDisplayWidthPx, board::kDisplayHeightPx, wifiStatusLabel().c_str(),
			bleStatusLabel().c_str());
	// Draws straight through WorkingBuffer/WorkingBufferGfx (§12's own path, the same one every
	// DRAW_TEXT command uses) rather than gDisplay's higher-level Adafruit_GFX fillScreen()/print(),
	// so this reuses the exact embedded font DRAW_TEXT itself renders with - opaqueBackground=true
	// fills each glyph cell's non-ink pixels with white, so nothing needs clearing by hand first.
	gWorkingBufferGfx.setDrawMode(drawMode::kReplace);
	constexpr int16_t kTextX = 10;
	constexpr int16_t kTextY = 10;
	TextLayoutResult textResult = drawText(gWorkingBufferGfx, gStorageManager, /*fontId=*/0x00, kTextX, kTextY,
			board::kDisplayWidthPx - 2 * kTextX, TextAlign::kLeft, /*wrap=*/false,
			reinterpret_cast<const uint8_t*>(line), static_cast<size_t>(len), color::kBlack, /*opaqueBackground=*/true);
	// Partial, not full - only the text's own bounding box, not the whole panel (design note 85).
	gWorkingBuffer.flush(kTextX, kTextY, textResult.width, textResult.height, /*full=*/false);
	Serial.printf("Display: cold-boot info screen shown (%lu ms)\n", millis() - start);

	// Deep sleep mode 2 (no RAM retention) - avoids leaving panel driving voltages on indefinitely
	// (screen fading over time); a later init() call (e.g. the next real draw/refresh command, or
	// WorkingBuffer::ensureControllerReady() after a LOW_POWER wake, §17) wakes it via RST, per the
	// datasheet's §9.2 flow.
	gDisplay.hibernate();
	Serial.println("Display: cold-boot screen complete, controller hibernating (wakes on next init())");
}

// doc/PROTOCOL.md §13: NVS-backed device settings, distinct from the LittleFS-backed VOLUME=INTERNAL
// (§14, user files) - a single Preferences namespace covers every §13.2/§13.3 persisted setting as
// they're added. Opened once in setup(), before anything that might call deviceName().
Preferences gPrefs;
constexpr const char* kPrefsNamespace = "crowpanel";
constexpr const char* kPrefKeyDeviceName = "devname";
constexpr const char* kPrefKeyWifiSsid = "wifi_ssid";
constexpr const char* kPrefKeyWifiPass = "wifi_pass";
constexpr const char* kPrefKeyWifiEnabled = "wifi_en";
constexpr const char* kPrefKeyBleEnabled = "ble_en";
constexpr const char* kPrefKeyBleHasPin = "ble_haspin";
constexpr const char* kPrefKeyBlePin = "ble_pin";
constexpr const char* kPrefKeyUsageHasPin = "usage_haspin";
constexpr const char* kPrefKeyUsagePin = "usage_pin";
constexpr const char* kPrefKeyAdminHasPin = "admin_haspin";
constexpr const char* kPrefKeyAdminPin = "admin_pin";

// doc/PROTOCOL.md §5.3: live in-RAM mirrors of the two access-control PINs above, loaded once in
// setup() and updated immediately by handleSetUsagePin/handleSetAdminPin - unlike the BLE pairing
// PIN (only ever applied at the next boot, see setupBle()), these must take effect on the very
// next dispatched frame, so a Preferences read per frame isn't good enough.
bool gHasUsagePin = false;
std::string gUsagePin;
bool gHasAdminPin = false;
std::string gAdminPin;

// Session-only device name override (doc/PROTOCOL.md §13.3 SET_DEVICE_NAME with FLAGS.PERSIST=0) -
// takes effect immediately but isn't written to NVS, so a reboot reverts to whatever's persisted
// (or the MAC-derived default below, if nothing ever was).
std::string gRuntimeDeviceNameOverride;

// doc/PROTOCOL.md §13.3: resolves, in priority order, a live SET_DEVICE_NAME override for this
// session, then whatever's persisted in NVS, then the MAC-derived default ("CrowPanel-XXXXXX") so
// multiple deployed units stay distinguishable in a BLE scan or DHCP client list with zero
// configuration required first.
std::string deviceName() {
	if (!gRuntimeDeviceNameOverride.empty()) {
		return gRuntimeDeviceNameOverride;
	}
	String stored = gPrefs.getString(kPrefKeyDeviceName, "");
	if (stored.length() > 0) {
		return std::string(stored.c_str());
	}
	String mac = WiFi.macAddress();  // e.g. "30:ED:A0:38:51:DC" - available once WiFi.mode() has run
	mac.replace(":", "");
	String name = String(board::kNamePrefix) + mac.substring(mac.length() - 6);
	return std::string(name.c_str());
}

// doc/PROTOCOL.md §13.2 SET_WIFI_CONFIG: a persisted (NVS) credential pair takes priority over the
// compile-time fallback in secrets.h, so a device configured over the wire keeps using its own
// credentials across reboots without secrets.h ever needing to know about it. Falling back to
// secrets.h at all (rather than requiring SET_WIFI_CONFIG first) is purely a bring-up/dev
// convenience predating this command.
void resolveWifiCredentials(std::string& ssid, std::string& password) {
	String storedSsid = gPrefs.getString(kPrefKeyWifiSsid, "");
	if (storedSsid.length() > 0) {
		ssid = std::string(storedSsid.c_str());
		password = std::string(gPrefs.getString(kPrefKeyWifiPass, "").c_str());
		return;
	}
	ssid = kWifiSsid;
	password = kWifiPassword;
}

// Shared by every "make sure WiFi is trying to connect" call site (boot, SET_WIFI_CONFIG,
// SET_WIFI_ENABLED(1), and LOW_POWER-wake restoration) - the single place that decides whether the
// radio ends up in STA mode (attempting a connection) or fully off. Found live via a real
// current-draw measurement (~120mA, unexpectedly high while otherwise idle): every one of these
// call sites used to set WiFi.mode(WIFI_STA) unconditionally but only called WiFi.begin() when
// credentials actually resolved to something non-empty, leaving the radio stuck in STA mode -
// associated with nothing, but still fully powered - whenever they didn't (e.g. no credentials
// configured yet). Consolidating here means there's only one place left that can get this wrong.
void startWifiOrPowerOff(const std::string& ssid, const std::string& password) {
	if (ssid.empty()) {
		WiFi.mode(WIFI_OFF);
		return;
	}
	// setHostname() before mode(WIFI_STA), not after: arduino-esp32's WiFiGenericClass::mode()
	// only pushes the hostname onto the actual netif/DHCP client at the moment of the STA
	// transition (WiFiGeneric.cpp, inside mode()'s "m & WIFI_MODE_STA" branch), reading whatever
	// its own internal default-hostname buffer holds *right then* - setHostname() only writes
	// that buffer, it doesn't push to the netif itself. Calling mode(WIFI_STA) first meant the
	// very first STA transition after every boot captured arduino-esp32's own auto-generated
	// default ("esp32s3-XXYYZZ" from the last 3 MAC bytes, CONFIG_IDF_TARGET-prefixed) instead of
	// deviceName() - found live: DEVICE_NAME/mDNS correctly showed "CrowPanel-3851DC" everywhere
	// (both read the buffer fresh, after it was long since corrected), but the router's own DHCP
	// lease still showed "esp32s3-3851DC" negotiated at connect time, confirmed via a fresh (not
	// stale) lease. A later WiFi disable/enable within the same boot masked the bug by luck - by
	// then the buffer already held the right value from this same call, so the next STA
	// transition picked it up - but the very first connection after any reboot/OTA never did.
	WiFi.setHostname(deviceName().c_str());
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid.c_str(), password.c_str());
}

// doc/PROTOCOL.md §13.2 SET_WIFI_ENABLED FLAGS.PERSIST: defaults to true (radio on) so existing
// deployments that never touch this setting keep today's always-on behavior unchanged.
bool wifiEnabledSetting() {
	return gPrefs.getBool(kPrefKeyWifiEnabled, true);
}

// Cold-boot-screen (displaySelfTest(), above) WiFi status label - "off" (SET_WIFI_ENABLED
// disabled), "unconfigured" (enabled, but no SSID resolves - neither a persisted SET_WIFI_CONFIG
// nor a compile-time secrets.h fallback), or "on" (enabled and about to try connecting). Reuses
// wifiEnabledSetting()/resolveWifiCredentials() exactly as every other call site does; this is
// config state only, not live connection status, since displaySelfTest() runs before
// connectWifiAndStartTcpServer() ever calls WiFi.begin().
std::string wifiStatusLabel() {
	if (!wifiEnabledSetting()) {
		return "off";
	}
	std::string ssid, password;
	resolveWifiCredentials(ssid, password);
	return ssid.empty() ? "unconfigured" : "on";
}

// Cold-boot-screen BLE status label - "off"/"on" mirroring kPrefKeyBleEnabled. Reads gPrefs
// directly rather than the runtime gBleEnabled mirror, since that isn't loaded from NVS until
// setupBle() (setup(), below), which - like WiFi - runs after this screen is already drawn.
std::string bleStatusLabel() {
	return gPrefs.getBool(kPrefKeyBleEnabled, true) ? "on" : "off";
}

// Starts the TCP server and the mDNS responder the first time WiFi is actually associated - called
// both from the boot-time blocking connect below and every pollTcp() iteration (loop()-driven,
// non-blocking), so a live SET_WIFI_CONFIG/SET_WIFI_ENABLED reconnect picks both up automatically
// without needing its own blocking wait. mDNS advertises the device as "<deviceName()>.local" plus
// a "_crowpanel._tcp" service naming this same TCP port, so a PC client can find/resolve the device
// without needing to already know its IP - reflects whatever deviceName() is at the moment of this
// specific connection, same as WiFi.setHostname() (startWifiOrPowerOff()) already does; a later
// SET_DEVICE_NAME doesn't retroactively rename an already-running mDNS responder, only the next
// reconnect does, which is the existing, established precedent here, not a new limitation.
void startTcpServerIfNeeded() {
	if (gTcpServer || WiFi.status() != WL_CONNECTED) {
		return;
	}
	gTcpServer.reset(new WiFiServer(kTcpPort));
	gTcpServer->begin();
	Serial.printf("TCP: server listening on %s:%u\n", WiFi.localIP().toString().c_str(), kTcpPort);

	if (MDNS.begin(deviceName().c_str())) {
		MDNS.addService("crowpanel", "tcp", kTcpPort);
		Serial.printf("mDNS: responding as \"%s.local\"\n", deviceName().c_str());
	} else {
		Serial.println("mDNS: MDNS.begin() failed - device will only be reachable by IP");
	}
}

// Connects to WiFi (STA mode) and, on success, starts the TCP server. Prints progress/result to
// Serial - still within the plain-text boot window, before the framed-protocol cutover. Failure
// (no credentials, wrong password, AP out of range, timeout, or WiFi explicitly disabled via
// SET_WIFI_ENABLED) is not fatal: it just leaves TCP unavailable for this session, Serial still
// works regardless. Only used for the initial boot-time attempt - a live reconnect triggered by
// SET_WIFI_CONFIG/SET_WIFI_ENABLED must never block loop() like this does, see those handlers.
void connectWifiAndStartTcpServer() {
	// Unconditional: initializes the WiFi driver so the MAC (deviceName()) is available even if we
	// never actually associate (e.g. no credentials configured yet) - startWifiOrPowerOff() below
	// still correctly reverses this to WIFI_OFF in that case, it just doesn't skip this first call.
	WiFi.mode(WIFI_STA);

	if (!wifiEnabledSetting()) {
		Serial.println("WiFi: disabled by a persisted SET_WIFI_ENABLED(0) - TCP transport unavailable "
						"this session");
		WiFi.mode(WIFI_OFF);
		return;
	}

	std::string ssid, password;
	resolveWifiCredentials(ssid, password);
	if (ssid.empty()) {
		Serial.println("WiFi: no credentials configured (see firmware/include/secrets.h.example, or "
						"SET_WIFI_CONFIG) - TCP transport unavailable this session");
		startWifiOrPowerOff(ssid, password);  // ssid empty - powers the radio off, doesn't connect
		return;
	}

	Serial.printf("WiFi: connecting to \"%s\" as \"%s\"", ssid.c_str(), deviceName().c_str());
	startWifiOrPowerOff(ssid, password);

	unsigned long start = millis();
	while (WiFi.status() != WL_CONNECTED && millis() - start < kWifiConnectTimeoutMs) {
		delay(250);
		Serial.print(".");
	}

	if (WiFi.status() != WL_CONNECTED) {
		Serial.printf("\nWiFi: FAILED to connect within %lu ms (status=%d) - TCP transport "
					  "unavailable this session\n",
				kWifiConnectTimeoutMs, static_cast<int>(WiFi.status()));
		return;
	}

	Serial.printf("\nWiFi: connected, IP=%s\n", WiFi.localIP().toString().c_str());
	startTcpServerIfNeeded();
}

// Brings up the BLE peripheral: one service/characteristic (doc/PROTOCOL.md §19) wrapped as a
// plain Stream by NimBLEStreamServer (§3.1), advertised so a central can find it by service UUID.
// Unlike WiFi this has no real failure mode to report - NimBLEDevice::init() either works or the
// board has a hardware/build problem - so this always leaves BLE available, no fallback needed.
void setupBle() {
	bool inited = NimBLEDevice::init(deviceName());
	Serial.printf("BLE: NimBLEDevice::init() -> %s\n", inited ? "true" : "FALSE");
	NimBLEDevice::setMTU(kBlePreferredMtu);

	gBleEnabled = gPrefs.getBool(kPrefKeyBleEnabled, true);

	// doc/PROTOCOL.md §13.2 SET_BLE_PIN: a persisted PIN switches the characteristic to encrypted
	// (secure=true) with a fixed passkey, rather than open to any central that finds it. Applied
	// only at boot, not live - SET_BLE_PIN itself just persists to NVS (see its handler) rather
	// than attempting a live re-init of the service/characteristic, which NimBLE doesn't support
	// as a simple runtime toggle the way advertising start/stop is. NimBLEDevice::setSecurityAuth's
	// "sc" (Secure Connections) is left on, matching this stack's own modern default.
	bool hasPin = gPrefs.getBool(kPrefKeyBleHasPin, false);
	if (hasPin) {
		uint32_t pin = gPrefs.getUInt(kPrefKeyBlePin, 0);
		NimBLEDevice::setSecurityAuth(/*bonding=*/true, /*mitm=*/true, /*sc=*/true);
		NimBLEDevice::setSecurityIOCap(BLE_SM_IO_CAP_DISP_ONLY);
		NimBLEDevice::setSecurityPasskey(pin);
	}
	bool streamOk = gBleStream.begin(NimBLEUUID(ble::kServiceUuid), NimBLEUUID(ble::kCharacteristicUuid),
			kBleStreamBufSize, kBleStreamBufSize, /*secure=*/hasPin);
	Serial.printf("BLE: gBleStream.begin() -> %s\n", streamOk ? "true" : "FALSE");

	NimBLEServer* server = NimBLEDevice::getServer();
	server->setCallbacks(&gBleServerCallbacks, /*deleteCallbacks=*/false);  // we own gBleServerCallbacks
	server->start();

	NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
	advertising->addServiceUUID(ble::kServiceUuid);
	// Primary advertising packet is already tight (flags + this 128-bit service UUID is 21 of the
	// 31 legacy bytes), so the name goes in the scan response instead of the primary packet -
	// NimBLEAdvertising::setName() routes there automatically once scan response is enabled (see
	// NimBLEAdvertising.cpp). Without this, no name is broadcast anywhere, which is why scanners
	// were showing the device unnamed/hard to pick out.
	advertising->enableScanResponse(true);
	advertising->setName(deviceName());
	if (gBleEnabled) {
		bool advOk = advertising->start();
		Serial.printf("BLE: advertising->start() -> %s\n", advOk ? "true" : "FALSE");
		Serial.printf("BLE: advertising as \"%s\", MTU=%u, service=%s, pairing=%s\n", deviceName().c_str(),
				NimBLEDevice::getMTU(), ble::kServiceUuid, hasPin ? "PIN-secured" : "open");
	} else {
		Serial.println("BLE: disabled by a persisted SET_BLE_ENABLED(0) - not advertising this session");
	}
}

// ---- Minimal TLV payload builder (doc/PROTOCOL.md §5.1/§5.2) - just enough for HANDSHAKE_RESPONSE.
// A reusable/general TLV writer belongs to the future command-dispatch layer, not this bring-up
// stub - kept inline and deliberately small here.
void appendTlvHeader(std::vector<uint8_t>& out, uint8_t type, uint8_t len) {
	out.push_back(type);
	out.push_back(len);
}

void appendTlvU8(std::vector<uint8_t>& out, uint8_t type, uint8_t value) {
	appendTlvHeader(out, type, 1);
	out.push_back(value);
}

void appendTlvU16LE(std::vector<uint8_t>& out, uint8_t type, uint16_t value) {
	appendTlvHeader(out, type, 2);
	out.push_back(static_cast<uint8_t>(value & 0xFF));
	out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

void appendTlvU32LE(std::vector<uint8_t>& out, uint8_t type, uint32_t value) {
	appendTlvHeader(out, type, 4);
	out.push_back(static_cast<uint8_t>(value & 0xFF));
	out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
	out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
	out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

void appendTlvString(std::vector<uint8_t>& out, uint8_t type, const char* value) {
	size_t len = strlen(value);
	appendTlvHeader(out, type, static_cast<uint8_t>(len));
	out.insert(out.end(), value, value + len);
}

void appendTlvBytes(std::vector<uint8_t>& out, uint8_t type, const uint8_t* data, size_t len) {
	appendTlvHeader(out, type, static_cast<uint8_t>(len));
	out.insert(out.end(), data, data + len);
}

// TLV TYPE values, doc/PROTOCOL.md §5.2.
constexpr uint8_t kTlvProtocolVersion = 0x01;
constexpr uint8_t kTlvDisplayWidthPx = 0x02;
constexpr uint8_t kTlvDisplayHeightPx = 0x03;
constexpr uint8_t kTlvColorDepth = 0x04;
constexpr uint8_t kTlvMaxChunkSize = 0x05;
constexpr uint8_t kTlvFeatureBitmask = 0x06;
constexpr uint8_t kTlvDeviceModel = 0x07;
constexpr uint8_t kTlvFirmwareVersion = 0x08;
constexpr uint8_t kTlvActiveTransport = 0x0C;
constexpr uint8_t kTlvAvailableGpioPins = 0x0D;
constexpr uint8_t kTlvLastWakeReason = 0x0E;
constexpr uint8_t kTlvDeviceName = 0x0F;
constexpr uint8_t kTlvPixelPitchXUm = 0x10;
constexpr uint8_t kTlvPixelPitchYUm = 0x11;
constexpr uint8_t kTlvGrantedLevel = 0x12;		 // doc/PROTOCOL.md §5.3 - what THIS handshake achieved
constexpr uint8_t kTlvUsagePinRequired = 0x13;	 // is a usage PIN currently configured (informational)
constexpr uint8_t kTlvAdminPinRequired = 0x14;	 // is an admin PIN currently configured (informational)

// FEATURE_BITMASK bits, doc/PROTOCOL.md §5.2.
constexpr uint32_t kFeaturePartialRefresh = 1u << 0;
constexpr uint32_t kFeatureRle = 1u << 1;
constexpr uint32_t kFeatureButtonEvents = 1u << 2;
constexpr uint32_t kFeatureDrawingPrimitives = 1u << 3;
constexpr uint32_t kFeatureSdCardSlot = 1u << 5;	  // capability, not live presence - see STORAGE_INFO
constexpr uint32_t kFeatureInternalStorage = 1u << 6;
constexpr uint32_t kFeatureGpio = 1u << 7;
constexpr uint32_t kFeatureInlineImageDraw = 1u << 9;	 // DRAW_IMAGE_DATA (0x0310)

// doc/PROTOCOL.md §17.2: computed once in setup() (see computeBootWakeReason()) and updated again
// after every LOW_POWER light-sleep resume (execution continues in the same handleSetPowerMode()
// call, no reboot involved) - read by both POWER_STATUS_RESPONSE and the handshake's own
// LAST_WAKE_REASON TLV below.
uint8_t gLastWakeReason = wakeReason::kPowerOn;

std::vector<uint8_t> buildHandshakeResponsePayload(uint8_t activeTransportValue, uint8_t grantedLevel) {
	std::vector<uint8_t> tlv;
	appendTlvU8(tlv, kTlvProtocolVersion, kProtocolVersion);
	appendTlvU16LE(tlv, kTlvDisplayWidthPx, board::kDisplayWidthPx);
	appendTlvU16LE(tlv, kTlvDisplayHeightPx, board::kDisplayHeightPx);
	appendTlvU8(tlv, kTlvColorDepth, board::kColorDepth);
	// doc/PROTOCOL.md §3.1: negotiated ATT MTU - 3 (ATT opcode+handle overhead). Advisory/unused
	// on TCP/Serial, included regardless since it's cheap and harmless there.
	appendTlvU16LE(tlv, kTlvMaxChunkSize, static_cast<uint16_t>(NimBLEDevice::getMTU() - 3));
	// bit4 SENSORS - not implemented yet (plan.md Phase 3).
	uint32_t featureBitmask = kFeaturePartialRefresh | kFeatureRle | kFeatureButtonEvents | kFeatureDrawingPrimitives |
			kFeatureInternalStorage | kFeatureInlineImageDraw;
	if (board::kPinSdCs >= 0) {
		featureBitmask |= kFeatureSdCardSlot;
	}
	if (board::kAvailableGpioPinsCount > 0) {
		featureBitmask |= kFeatureGpio;
	}
	appendTlvU32LE(tlv, kTlvFeatureBitmask, featureBitmask);
	appendTlvString(tlv, kTlvDeviceModel, board::kDeviceModel);
	appendTlvString(tlv, kTlvFirmwareVersion, kFirmwareVersion);
	appendTlvU8(tlv, kTlvActiveTransport, activeTransportValue);
	appendTlvU8(tlv, kTlvLastWakeReason, gLastWakeReason);
	appendTlvString(tlv, kTlvDeviceName, deviceName().c_str());
	if (board::kAvailableGpioPinsCount > 0) {
		// board::kAvailableGpioPins is int[] (raw MCU GPIO numbers always fit in a byte) - narrow
		// to uint8_t for the wire TLV (doc/PROTOCOL.md §5.2 AVAILABLE_GPIO_PINS).
		uint8_t pins[board::kAvailableGpioPinsCount];
		for (size_t i = 0; i < board::kAvailableGpioPinsCount; ++i) {
			pins[i] = static_cast<uint8_t>(board::kAvailableGpioPins[i]);
		}
		appendTlvBytes(tlv, kTlvAvailableGpioPins, pins, board::kAvailableGpioPinsCount);
	}
	appendTlvU16LE(tlv, kTlvPixelPitchXUm, board::kPixelPitchXUm);
	appendTlvU16LE(tlv, kTlvPixelPitchYUm, board::kPixelPitchYUm);
	appendTlvU8(tlv, kTlvGrantedLevel, grantedLevel);
	appendTlvU8(tlv, kTlvUsagePinRequired, gHasUsagePin ? 1 : 0);
	appendTlvU8(tlv, kTlvAdminPinRequired, gHasAdminPin ? 1 : 0);
	return tlv;
}

// doc/PROTOCOL.md §5.3: which per-transport granted-level global a live connection's handshake
// result belongs to - nullptr for activeTransport::kMacro (a macro replay of HANDSHAKE_REQUEST is
// pointless - HANDSHAKE_REQUEST is never gated - but can't crash if it somehow happens; macro
// entries bypass the access-control gate entirely regardless, see stepMacroPlayback()).
uint8_t* authLevelSlotFor(uint8_t activeTransportValue) {
	switch (activeTransportValue) {
		case activeTransport::kTcp:
			return &gTcpAuthLevel;
		case activeTransport::kBle:
			return &gBleAuthLevel;
		case activeTransport::kSerial:
			return &gSerialAuthLevel;
		default:
			return nullptr;
	}
}

// Registered onto gDispatcher in setup(); everything unregistered correctly falls through to
// Dispatcher's own NACK(UNSUPPORTED_COMMAND), doc/PROTOCOL.md §4. Never itself gated (registered
// with the default authLevel::kNone) - it's how a PIN gets presented in the first place.
//
// doc/PROTOCOL.md §5.3: parses the optional PIN_TYPE/PIN_LEN/PIN payload, checks the MENU+BACK
// physical-presence override (Serial only - requested directly as a recovery path specifically
// for when the device might be physically restrained/inaccessible over TCP/BLE, not a general
// bypass for every transport), and compares against whichever PIN was offered. Never NACKs due to
// a bad/missing PIN - a wrong pin is indistinguishable on the wire from no pin offered at all, the
// connection simply stays at whatever level it can prove - always replies with a real
// HANDSHAKE_RESPONSE reporting exactly what was achieved via the new GRANTED_LEVEL TLV.
void handleHandshakeRequest(const CommandContext& ctx) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < 2 || payload.size() != 2u + payload[1]) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint8_t pinType = payload[0];
	uint8_t pinLen = payload[1];
	std::string offeredPin(reinterpret_cast<const char*>(payload.data() + 2), pinLen);

	uint8_t granted = authLevel::kNone;
	if (ctx.activeTransportValue == activeTransport::kSerial && gButtonController.isPressed(buttonId::kMenu) &&
			gButtonController.isPressed(buttonId::kBack)) {
		granted = authLevel::kAdmin;
	} else if (pinType == authLevel::kAdmin && gHasAdminPin && offeredPin == gAdminPin) {
		granted = authLevel::kAdmin;
	} else if (pinType == authLevel::kUsage && gHasUsagePin && offeredPin == gUsagePin) {
		granted = authLevel::kUsage;
	}

	uint8_t* slot = authLevelSlotFor(ctx.activeTransportValue);
	if (slot != nullptr) {
		*slot = granted;
	}

	ctx.reply(cmd::kHandshakeResponse, buildHandshakeResponsePayload(ctx.activeTransportValue, granted));
}

// doc/PROTOCOL.md §13.3 SET_DEVICE_NAME.
constexpr size_t kMaxDeviceNameLen = 32;

void handleSetDeviceName(const CommandContext& ctx) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.empty()) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint8_t nameLen = payload[0];
	if (nameLen > kMaxDeviceNameLen || payload.size() != 1u + nameLen + 1u) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint8_t flags = payload[1 + nameLen];
	bool persist = (flags & configFlags::kPersist) != 0;
	if (nameLen == 0) {
		// "0 = clear any configured name, revert to the MAC-derived default" (§13.3) - PERSIST
		// additionally erases the NVS entry so the default sticks across a future reboot too;
		// without it, this boot reverts but whatever's in NVS (if anything) still applies next time.
		gRuntimeDeviceNameOverride.clear();
		if (persist) {
			gPrefs.remove(kPrefKeyDeviceName);
		}
	} else {
		std::string name(reinterpret_cast<const char*>(&payload[1]), nameLen);
		gRuntimeDeviceNameOverride = name;
		if (persist) {
			gPrefs.putString(kPrefKeyDeviceName, name.c_str());
		}
	}
	ctx.ack();
}

// doc/PROTOCOL.md §13.1 CONFIG_BACKUP/RESTORE: an opaque, versioned TLV blob (§5.1 entry format,
// a device-defined TYPE namespace independent of the handshake's own §5.2 TYPE namespace) - the PC
// client never parses entries itself, just stores/replays the whole blob. Reuses appendTlvString()
// (already generic - defined above for the handshake builder) rather than a second TLV writer.
namespace configTlv {
constexpr uint8_t kDeviceName = 0x01;
}  // namespace configTlv

constexpr uint8_t kConfigVersion = 0x01;

std::vector<uint8_t> buildConfigBackupPayload() {
	std::vector<uint8_t> out;
	out.push_back(kConfigVersion);
	// Only a *persisted* name is backed up - the MAC-derived default needs no backup entry, and a
	// session-only (non-PERSIST) override was never meant to survive past this boot anyway.
	String storedName = gPrefs.getString(kPrefKeyDeviceName, "");
	if (storedName.length() > 0) {
		appendTlvString(out, configTlv::kDeviceName, storedName.c_str());
	}
	return out;
}

void handleConfigBackupRequest(const CommandContext& ctx) {
	ctx.reply(cmd::kConfigBackupData, buildConfigBackupPayload());
}

// Shared by CONFIG_RESTORE (§13.1, PC-driven, always persists to NVS) and loadSdConfigLayer()
// (below - boot-time, SD-driven, deliberately never persists, so pulling the card reverts to
// whatever's actually in NVS on the next boot rather than leaving a stale copy behind). Applies
// entries it recognizes and skips unknown TYPEs (§5.1/§13.1's tolerance rule) rather than failing
// the whole blob over one entry from a newer firmware version. Returns false only for a malformed
// CONFIG_VERSION/TLV structure, not for unknown-but-well-formed entries.
bool applyConfigTlvBlob(const uint8_t* data, size_t length, bool persistToNvs) {
	if (length < 1 || data[0] != kConfigVersion) {
		return false;
	}
	size_t offset = 1;
	while (offset + 2 <= length) {
		uint8_t type = data[offset];
		uint8_t len = data[offset + 1];
		offset += 2;
		if (offset + len > length) {
			break;	// malformed tail - stop parsing rather than read out of bounds
		}
		if (type == configTlv::kDeviceName) {
			std::string name(reinterpret_cast<const char*>(&data[offset]), len);
			gRuntimeDeviceNameOverride = name;
			if (persistToNvs) {
				gPrefs.putString(kPrefKeyDeviceName, name.c_str());
			}
		}
		offset += len;
	}
	return true;
}

void handleConfigRestore(const CommandContext& ctx) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (!applyConfigTlvBlob(payload.data(), payload.size(), /*persistToNvs=*/true)) {
		ctx.nack(status::kBadParameters);
		return;
	}
	ctx.ack();
}

// doc/PROTOCOL.md §13.1: layered config resolution, requested directly - "layered settings - the
// one stored on SD overriding the one internal flash, so we can repurpose device easily". The SD
// card's file (if present) is exactly a CONFIG_BACKUP_DATA blob - the same VERSION+TLV bytes
// CONFIG_BACKUP_DATA already produces and CONFIG_RESTORE already consumes over the wire, just
// persisted as a file instead of transmitted - so backing up one unit's config, saving it to this
// path on an SD card, and moving that card to a different unit re-identifies it as the first unit
// on next boot, with zero new file format to design or maintain.
//
// Deliberately never written to NVS (applyConfigTlvBlob's persistToNvs=false) - the SD layer only
// applies for as long as that card stays inserted; pull the card and the next boot falls back to
// whatever's actually persisted in NVS (or the MAC-derived default), never a stale leftover copy.
// Checked once at boot, after gStorageManager.begin() - not on every read, so swapping cards while
// already running needs a reboot to take effect (consistent with how repurposing a physical device
// via a physical card swap is normally a power-off operation anyway).
constexpr const char* kSdConfigPath = "/device.config";

void loadSdConfigLayer() {
	std::vector<uint8_t> fileData;
	if (gStorageManager.download(volume::kSd, kSdConfigPath, fileData) != StorageManager::Result::kOk) {
		return;	 // no SD card, or no config file on it - the common case, not an error
	}
	if (!applyConfigTlvBlob(fileData.data(), fileData.size(), /*persistToNvs=*/false)) {
		Serial.printf("Config: %s found but failed to parse as a CONFIG_BACKUP_DATA blob - ignoring\n",
				kSdConfigPath);
	}
}

// doc/PROTOCOL.md §13.2: if this arrived over the very transport it's about to disrupt (a TCP
// client reconfiguring/disabling the WiFi link it's currently talking over), give the ACK a moment
// to actually leave before tearing anything down - same ordering precedent as OTA_APPLY (§16.2).
// Harmless/instant no-op for every other transport.
void settleAckBeforeDisruptingTransport(uint8_t activeTransportValue) {
	if (activeTransportValue == activeTransport::kTcp || activeTransportValue == activeTransport::kBle) {
		delay(100);
	}
}

void stopTcpServerAndClient() {
	gTcpTransport.reset();
	if (gTcpClient) {
		gTcpClient.stop();
	}
	gTcpServer.reset();
	MDNS.end();	 // startTcpServerIfNeeded() re-begins it fresh (current deviceName(), current IP) on
				 // whatever reconnect follows - harmless to call even if it was never actually started
}

// doc/PROTOCOL.md §13.2 SET_WIFI_CONFIG.
constexpr size_t kMaxSsidLen = 32;
constexpr size_t kMaxWifiPasswordLen = 63;

namespace wifiConfigFlags {
constexpr uint8_t kConnectNow = 1u << 1;
}  // namespace wifiConfigFlags

void handleSetWifiConfig(const CommandContext& ctx) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.empty()) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint8_t ssidLen = payload[0];
	if (ssidLen == 0 || ssidLen > kMaxSsidLen || payload.size() < 1u + ssidLen + 1u) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint8_t passwordLen = payload[1 + ssidLen];
	if (passwordLen > kMaxWifiPasswordLen || payload.size() != 1u + ssidLen + 1u + passwordLen + 1u) {
		ctx.nack(status::kBadParameters);
		return;
	}
	std::string ssid(reinterpret_cast<const char*>(&payload[1]), ssidLen);
	std::string password(reinterpret_cast<const char*>(&payload[2 + ssidLen]), passwordLen);
	uint8_t flags = payload[2 + ssidLen + passwordLen];
	bool persist = (flags & configFlags::kPersist) != 0;
	bool connectNow = (flags & wifiConfigFlags::kConnectNow) != 0;

	if (persist) {
		gPrefs.putString(kPrefKeyWifiSsid, ssid.c_str());
		gPrefs.putString(kPrefKeyWifiPass, password.c_str());
	}
	// ACK first - association can take several seconds and might disrupt the very connection this
	// arrived over (§13.2) - poll WIFI_STATUS_REQUEST for the actual outcome, never block the ACK.
	ctx.ack();
	if (connectNow) {
		settleAckBeforeDisruptingTransport(ctx.activeTransportValue);
		// ssid is already validated non-empty above, so this always takes startWifiOrPowerOff()'s
		// connect branch - using it anyway for consistency with every other "start WiFi" call site
		// (not assuming WiFi.mode(WIFI_STA) is already in effect here, unlike before: this command is
		// exactly how a device without any credentials yet gets its first ones, live, over the air).
		startWifiOrPowerOff(ssid, password);  // non-blocking; startTcpServerIfNeeded() picks up success
	}
}

void handleWifiStatusRequest(const CommandContext& ctx) {
	String ssid = WiFi.SSID();
	uint8_t ssidLen = static_cast<uint8_t>(std::min<size_t>(ssid.length(), 255));
	std::vector<uint8_t> out;
	out.push_back(WiFi.getMode() != WIFI_OFF ? 1 : 0);				  // ENABLED
	out.push_back(WiFi.status() == WL_CONNECTED ? 1 : 0);				  // CONNECTED
	out.push_back(ssidLen);											  // SSID_LEN
	out.insert(out.end(), ssid.c_str(), ssid.c_str() + ssidLen);		  // SSID
	IPAddress ip = WiFi.localIP();										  // all-zero if not connected
	out.push_back(ip[0]);
	out.push_back(ip[1]);
	out.push_back(ip[2]);
	out.push_back(ip[3]);
	ctx.reply(cmd::kWifiStatusResponse, out);
}

void handleSetWifiEnabled(const CommandContext& ctx) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != 2) {
		ctx.nack(status::kBadParameters);
		return;
	}
	bool enable = payload[0] != 0;
	bool persist = (payload[1] & configFlags::kPersist) != 0;

	// ACK first (§13.2) - disabling tears down any TCP connection as a documented side effect,
	// possibly including the one this command arrived over.
	ctx.ack();
	settleAckBeforeDisruptingTransport(ctx.activeTransportValue);

	if (persist) {
		gPrefs.putBool(kPrefKeyWifiEnabled, enable);
	}
	if (enable) {
		std::string ssid, password;
		resolveWifiCredentials(ssid, password);
		startWifiOrPowerOff(ssid, password);  // powers back off instead if no credentials resolve
	} else {
		stopTcpServerAndClient();
		WiFi.disconnect(/*wifioff=*/true);
		WiFi.mode(WIFI_OFF);
	}
}

// doc/PROTOCOL.md §13.2 SET_BLE_ENABLED: stop/start advertising rather than a full
// NimBLEDevice::deinit()/init() cycle - simpler, and sufficient to make the device
// undiscoverable/unconnectable, which is what "enabled" means here (mirrors WiFi's own
// ENABLED - the radio/stack stays initialized either way, only its externally-visible
// discoverability toggles).
void handleSetBleEnabled(const CommandContext& ctx) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != 2) {
		ctx.nack(status::kBadParameters);
		return;
	}
	bool enable = payload[0] != 0;
	bool persist = (payload[1] & configFlags::kPersist) != 0;

	// ACK first (§13.2) - disabling drops any connected central as a side effect, possibly
	// including the one this command arrived over.
	ctx.ack();
	settleAckBeforeDisruptingTransport(ctx.activeTransportValue);

	if (persist) {
		gPrefs.putBool(kPrefKeyBleEnabled, enable);
	}
	gBleEnabled = enable;
	if (enable) {
		NimBLEDevice::startAdvertising();
	} else {
		NimBLEDevice::stopAdvertising();
		NimBLEServer* server = NimBLEDevice::getServer();
		for (uint16_t connHandle : server->getPeerDevices()) {
			server->disconnect(connHandle);
		}
	}
}

void handleBleStatusRequest(const CommandContext& ctx) {
	NimBLEServer* server = NimBLEDevice::getServer();
	NimBLEAddress address = NimBLEDevice::getAddress();
	std::vector<uint8_t> out;
	out.push_back(gBleEnabled ? 1 : 0);						 // ENABLED
	out.push_back(server->getConnectedCount() > 0 ? 1 : 0);	 // CONNECTED
	out.push_back(gPrefs.getBool(kPrefKeyBleHasPin, false) ? 1 : 0);	 // HAS_PIN - never the value itself
	out.insert(out.end(), address.getVal(), address.getVal() + 6);	 // BLE_ADDRESS
	ctx.reply(cmd::kBleStatusResponse, out);
}

// doc/PROTOCOL.md §13.2 SET_BLE_PIN. Always persists to NVS regardless of FLAGS.PERSIST - unlike
// every other §13 setting, a session-only pairing PIN would be meaningless: pairing security is
// only ever applied at the *next* boot (see setupBle()'s comment on why this isn't done live), so
// without persisting, this command would silently have no effect, ever. FLAGS is still validated
// for a well-formed payload shape but its PERSIST bit is intentionally not consulted.
constexpr uint32_t kMaxBlePin = 999999;

void handleSetBlePin(const CommandContext& ctx) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != 6) {
		ctx.nack(status::kBadParameters);
		return;
	}
	bool hasPin = payload[0] != 0;
	uint32_t pin = static_cast<uint32_t>(payload[1]) | (static_cast<uint32_t>(payload[2]) << 8) |
			(static_cast<uint32_t>(payload[3]) << 16) | (static_cast<uint32_t>(payload[4]) << 24);
	if (hasPin && pin > kMaxBlePin) {
		ctx.nack(status::kBadParameters);
		return;
	}
	gPrefs.putBool(kPrefKeyBleHasPin, hasPin);
	if (hasPin) {
		gPrefs.putUInt(kPrefKeyBlePin, pin);
	} else {
		gPrefs.remove(kPrefKeyBlePin);
	}
	ctx.ack();
}

// doc/PROTOCOL.md §5.3 SET_USAGE_PIN/SET_ADMIN_PIN - shared body for both (only the Preferences
// keys and live-mirror globals differ). Unlike SET_BLE_PIN's numeric-only pairing code, this PIN
// is an arbitrary UTF-8 string (a human-typed passphrase). Always persists regardless of FLAGS,
// the same documented exception handleSetBlePin above establishes - a session-only access-control
// PIN would be just as meaningless. Unlike the BLE PIN, ALSO updates the live in-RAM mirror
// immediately - this gate is checked on every live dispatch, not just applied once at boot.
constexpr size_t kMaxAuthPinLen = 64;

void applyAuthPinChange(const CommandContext& ctx, const char* hasPinKey, const char* pinKey, bool& liveHasPin,
		std::string& livePin) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < 2) {
		ctx.nack(status::kBadParameters);
		return;
	}
	bool hasPin = payload[0] != 0;
	uint8_t pinLen = payload[1];
	if (payload.size() != 2u + pinLen + 1u || (hasPin && pinLen > kMaxAuthPinLen) || (!hasPin && pinLen != 0)) {
		ctx.nack(status::kBadParameters);
		return;
	}
	std::string pin(reinterpret_cast<const char*>(payload.data() + 2), pinLen);

	gPrefs.putBool(hasPinKey, hasPin);
	if (hasPin) {
		gPrefs.putString(pinKey, pin.c_str());
	} else {
		gPrefs.remove(pinKey);
	}
	liveHasPin = hasPin;
	livePin = hasPin ? pin : std::string();
	ctx.ack();
}

void handleSetUsagePin(const CommandContext& ctx) {
	applyAuthPinChange(ctx, kPrefKeyUsageHasPin, kPrefKeyUsagePin, gHasUsagePin, gUsagePin);
}

void handleSetAdminPin(const CommandContext& ctx) {
	applyAuthPinChange(ctx, kPrefKeyAdminHasPin, kPrefKeyAdminPin, gHasAdminPin, gAdminPin);
}

// doc/PROTOCOL.md §17 Power management. E-ink retains its image with zero power once flipped
// (§2.1), so neither mode here needs to touch the panel - the whole point is the display keeps
// showing whatever it last showed while the MCU sleeps.

// doc/PROTOCOL.md §17.1 SET_POWER_MODE.WAKE_BUTTON: only the buttons this firmware actually knows a
// pin for (ButtonController's own table) can be configured as a HARD_SLEEP wake source. Every one
// of them happens to sit on GPIO0-21, which is entirely within the ESP32-S3's RTC IO domain (the
// only GPIOs esp_sleep_enable_ext1_wakeup() can use), so no board has ever needed to reject one for
// being outside that range - but the check still exists in case a future board's button pin isn't.
int resolveButtonWakePin(uint8_t buttonIdValue) {
	switch (buttonIdValue) {
		case buttonId::kMenu:
			return board::kPinButtonMenu;
		case buttonId::kBack:
			return board::kPinButtonBack;
		case buttonId::kBoot:
			return board::kPinButtonBoot;
		case buttonId::kDialUp:
			return board::kPinButtonDialUp;
		case buttonId::kDialDown:
			return board::kPinButtonDialDown;
		case buttonId::kDialSwitch:
			return board::kPinButtonDialConfirm;
		default:
			return -1;
	}
}

constexpr size_t kSetPowerModePayloadSize = 7;	 // MODE(1) + FLAGS(1) + WAKE_AFTER_MS(4) + WAKE_BUTTON(1)

void handleSetPowerMode(const CommandContext& ctx) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != kSetPowerModePayloadSize) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint8_t mode = payload[0];
	uint8_t flags = payload[1];
	uint32_t wakeAfterMs = static_cast<uint32_t>(payload[2]) | (static_cast<uint32_t>(payload[3]) << 8) |
			(static_cast<uint32_t>(payload[4]) << 16) | (static_cast<uint32_t>(payload[5]) << 24);
	uint8_t wakeButtonId = payload[6];

	if (mode == powerMode::kActive) {
		ctx.ack();	// already active - no-op, per §17.1
		return;
	}
	if (mode != powerMode::kLowPower && mode != powerMode::kHardSleep) {
		ctx.nack(status::kBadParameters);
		return;
	}

	int wakePin = -1;
	if (mode == powerMode::kHardSleep && wakeButtonId != buttonId::kUnknown) {
		wakePin = resolveButtonWakePin(wakeButtonId);
		if (wakePin < 0) {
			ctx.nack(status::kBadParameters);
			return;
		}
	}

	// doc/PROTOCOL.md §5.3: HARD_SLEEP kills whichever radio carries a TCP/BLE connection - a
	// non-admin severing their own session that way, with no recovery but a timer/button, is the
	// risk this extra check guards against. Serial isn't radio-dependent and isn't severed by a
	// sleep the way a live TCP/BLE session is, so it only needs the baseline USAGE gate already
	// enforced generically (SET_POWER_MODE itself is registered at USAGE level). Same
	// cascading-fallback rule as the generic gate, via the same shared helper, so this can't drift
	// from it.
	if (mode == powerMode::kHardSleep && ctx.activeTransportValue != activeTransport::kSerial) {
		uint8_t required = Dispatcher::effectiveRequiredLevel(authLevel::kAdmin, gHasUsagePin, gHasAdminPin);
		if (ctx.authLevel < required) {
			ctx.nack(status::kNotAuthorized);
			return;
		}
	}

	// ACK before actually sleeping (§17.1's own ordering note, same precedent as OTA_APPLY/§16.2
	// and the network-config commands/§13.2) - otherwise the caller never sees confirmation.
	ctx.ack();
	settleAckBeforeDisruptingTransport(ctx.activeTransportValue);

	// Wake sources configured via esp_sleep_enable_*_wakeup() are sticky across separate
	// esp_light_sleep_start()/esp_deep_sleep_start() calls - they're NOT implicitly cleared just
	// because this call didn't ask for one. Without this, a WAKE_AFTER_MS from an earlier
	// SET_POWER_MODE call would keep firing on every later LOW_POWER/HARD_SLEEP too, even one
	// that requested WAKE_AFTER_MS=0 (found on real hardware: a second LOW_POWER call with no
	// timer woke via the *previous* call's timer instead of actually waiting for UART activity).
	esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

	if (wakeAfterMs > 0) {
		esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(wakeAfterMs) * 1000ULL);
	}

	if (mode == powerMode::kHardSleep) {
		if (wakePin >= 0) {
			esp_sleep_enable_ext1_wakeup(1ULL << wakePin, ESP_EXT1_WAKEUP_ANY_LOW);
		}
		// Full powerdown (doc/PROTOCOL.md §17, requested directly) - no matching restore needed on
		// this path, unlike LOW_POWER below: a HARD_SLEEP wake is a full reboot, and setup() already
		// re-initializes both rails from scratch (displaySelfTest(), gStorageManager.begin()) exactly
		// as if this were a fresh power-on, so there's nothing here to leave "recoverable".
		gWorkingBuffer.powerDown();
		gStorageManager.powerDownSd();
		esp_deep_sleep_start();	 // never returns - RAM/PSRAM lost, this is indistinguishable from a
								 // power-on reset except via esp_sleep_get_wakeup_cause() at the next boot
	}

	// LOW_POWER (light sleep): CPU pauses, RAM/PSRAM retained, execution resumes right here once
	// esp_light_sleep_start() returns - no reboot, no dispatcher/loop() re-entry involved.
	bool keepBleConnectable = (flags & setPowerModeFlags::kKeepBleConnectable) != 0;
	stopTcpServerAndClient();
	WiFi.mode(WIFI_OFF);  // unconditionally suspended regardless of the persisted SET_WIFI_ENABLED
						   // state (§17) - restored below from that same persisted state on wake
	// uart_set_wakeup_threshold() is required, not optional (confirmed on real hardware:
	// esp_sleep_enable_uart_wakeup() alone never actually woke the chip) - light-sleep UART wake
	// works by counting RX-pin positive edges against this threshold, and the default threshold
	// isn't tuned for reliable wake. 3 is the minimum valid value and matches the driver's own
	// example (enough edges for a single 8n1 byte's start+stop bits) - this is also exactly why
	// SerialFrameTransport::sendWakePreamble() (design note 77) exists: the byte(s) that supply
	// those edges are consumed by the wake detector itself, never reaching the UART FIFO.
	uart_set_wakeup_threshold(UART_NUM_0, 3);
	esp_sleep_enable_uart_wakeup(UART_NUM_0);
	if (!keepBleConnectable) {
		NimBLEDevice::stopAdvertising();
	}
	// Recoverable powerdown (doc/PROTOCOL.md §17, requested directly: "for light sleep it should be
	// recoverable") - unlike HARD_SLEEP above, execution resumes right here in the same call after
	// esp_light_sleep_start() returns. Both rails are left powered down on wake, deliberately: display
	// recovery is WorkingBuffer::ensureControllerReady()'s job, lazily, the moment anything is next
	// actually drawn (mirrors GxEPD2's own internal lazy-reinit design, see its own doc); SD recovery
	// is gStorageManager.mountSd()'s job, lazily, the moment any VOLUME=SD command next needs it -
	// same mechanism updateIdlePowerDown() already relies on. No reason to eagerly spend either rail's
	// power back here if nothing actually uses it after waking.
	gWorkingBuffer.powerDown();
	gStorageManager.powerDownSd();

	esp_light_sleep_start();

	// Resumed. esp_sleep_get_wakeup_cause() now reflects whatever source pulled us out of light
	// sleep - map it the same way computeBootWakeReason() does for a HARD_SLEEP reboot, below.
	switch (esp_sleep_get_wakeup_cause()) {
		case ESP_SLEEP_WAKEUP_TIMER:
			gLastWakeReason = wakeReason::kLowPowerTimer;
			break;
		case ESP_SLEEP_WAKEUP_BT:
			gLastWakeReason = wakeReason::kLowPowerBleActivity;
			break;
		case ESP_SLEEP_WAKEUP_UART:
		default:
			gLastWakeReason = wakeReason::kLowPowerSerialActivity;
			break;
	}

	if (wifiEnabledSetting()) {
		std::string ssid, password;
		resolveWifiCredentials(ssid, password);
		startWifiOrPowerOff(ssid, password);  // see design note 74; powers back off if no credentials resolve
	}
	if (!keepBleConnectable && gBleEnabled) {
		NimBLEDevice::startAdvertising();
	}
}

void handlePowerStatusRequest(const CommandContext& ctx) {
	// CURRENT_MODE is always ACTIVE (§17.2): a device able to reply is by definition not asleep.
	ctx.reply(cmd::kPowerStatusResponse, { powerMode::kActive, gLastWakeReason });
}

// doc/PROTOCOL.md §16 OTA firmware update. The entire image arrives as one Logical Frame payload
// (already reassembled by the transport layer, fragmented-and-rejoined transparently on BLE per
// §3.1) - handlers here just work with the already-complete ctx.request.payload, no chunking logic
// of their own needed. Relies on the dual-OTA-partition scheme already confirmed present on real
// hardware (design note 44's boot-log line, "OTA: running=app0 ... next-update-slot=app1 ...").

// Set once an OTA_INSTALL finishes successfully (whether or not FLAGS.APPLY_NOW is also set) -
// tracks the one image this firmware can have "staged" at a time, per §16.1's own NACK(BUSY) rule.
bool gOtaStaged = false;
const esp_partition_t* gOtaStagedPartition = nullptr;

// doc/PROTOCOL.md §16.3 RUNNING_SLOT: OTA partition subtypes are sequential from
// ESP_PARTITION_SUBTYPE_APP_OTA_MIN, so subtracting it directly gives the 0/1 slot index - no
// string-comparing partition labels needed.
uint8_t otaSlotIndex(const esp_partition_t* partition) {
	if (!partition) {
		return 0;
	}
	return static_cast<uint8_t>(partition->subtype - ESP_PARTITION_SUBTYPE_APP_OTA_MIN);
}

// Arduino's own initArduino() runs before setup() and immediately auto-confirms any pending OTA image
// unless this weak hook says otherwise (esp32-hal-misc.c in the Arduino-ESP32 core:
// `if (!verifyRollbackLater()) { ...if PENDING_VERIFY: verifyOta() ? mark_valid : mark_invalid... }`)
// - without this override, OTA_STATUS_RESPONSE.PENDING_VERIFICATION and OTA_CONFIRM never see a real
// pending state at all (confirmed on real hardware: PENDING_VERIFICATION read 0 immediately after a
// fresh OTA boot, before OTA_CONFIRM was ever sent). extern "C" is required: the weak symbol lives in
// a plain .c file with C linkage, so a plain C++ definition here would mangle to a different symbol
// and silently fail to override it (same extern "C" pattern used by this framework's own
// libraries/RainMaker/src/RMaker.cpp).
extern "C" bool verifyRollbackLater() {
	return true;
}

// doc/PROTOCOL.md §16.4's proactive half of the auto-rollback safety net: if OTA_CONFIRM never arrives
// (a bad image that boots and runs but is otherwise unreachable - not a crash, which the bootloader's
// own PENDING_VERIFY-not-cleared check already catches on the next boot), force a rollback rather than
// wait forever for a PC that may never reconnect. Called from loop() (see below).
void checkOtaAutoRollback() {
	const esp_partition_t* running = esp_ota_get_running_partition();
	esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
	esp_ota_get_state_partition(running, &state);
	if (state == ESP_OTA_IMG_PENDING_VERIFY && millis() > kOtaConfirmTimeoutMs) {
		esp_ota_mark_app_invalid_rollback_and_reboot();  // same API initArduino() itself would have
														  // used on a failed verifyOta()
	}
}

void handleOtaInstall(const CommandContext& ctx) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < 6) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint32_t totalLen = static_cast<uint32_t>(payload[0]) | (static_cast<uint32_t>(payload[1]) << 8) |
			(static_cast<uint32_t>(payload[2]) << 16) | (static_cast<uint32_t>(payload[3]) << 24);
	uint8_t hashAlgo = payload[4];
	uint8_t hashLen = payload[5];
	uint8_t expectedHashLen;
	switch (hashAlgo) {
		case otaHashAlgo::kNone:
			expectedHashLen = 0;
			break;
		case otaHashAlgo::kSha256:
			expectedHashLen = 32;
			break;
		case otaHashAlgo::kMd5:
			expectedHashLen = 16;
			break;
		default:
			ctx.nack(status::kBadParameters);
			return;
	}
	if (hashLen != expectedHashLen) {
		ctx.nack(status::kBadParameters);
		return;
	}
	size_t headerSize = 6u + hashLen + 1u;	 // + FLAGS byte
	if (payload.size() != headerSize + totalLen) {
		ctx.nack(status::kBadParameters);
		return;
	}
	if (gOtaStaged) {
		ctx.nack(status::kBusy);
		return;
	}

	const uint8_t* hash = payload.data() + 6;
	uint8_t flags = payload[6 + hashLen];
	const uint8_t* imageData = payload.data() + headerSize;

	const esp_partition_t* targetPartition = esp_ota_get_next_update_partition(nullptr);
	if (!targetPartition || totalLen > targetPartition->size) {
		ctx.nack(status::kInsufficientStorage);
		return;
	}

	// Verify the hash *before* writing anything - a bad/truncated transfer should never touch the
	// partition at all, per §16.1's own "wire-polarity-style convention" note on why this matters.
	if (hashAlgo == otaHashAlgo::kSha256) {
		uint8_t computed[32];
		mbedtls_sha256_ret(imageData, totalLen, computed, /*is224=*/0);
		if (memcmp(computed, hash, 32) != 0) {
			ctx.nack(status::kOtaHashMismatch);
			return;
		}
	} else if (hashAlgo == otaHashAlgo::kMd5) {
		uint8_t computed[16];
		mbedtls_md5_ret(imageData, totalLen, computed);
		if (memcmp(computed, hash, 16) != 0) {
			ctx.nack(status::kOtaHashMismatch);
			return;
		}
	}

	esp_ota_handle_t handle;
	if (esp_ota_begin(targetPartition, totalLen, &handle) != ESP_OK) {
		ctx.nack(status::kUnknownError);
		return;
	}
	if (esp_ota_write(handle, imageData, totalLen) != ESP_OK) {
		esp_ota_abort(handle);
		ctx.nack(status::kUnknownError);
		return;
	}
	// esp_ota_end() also validates the image header/checksum internally - a corrupt-but-hash-matching
	// (or hash-skipped, HASH_ALGO=NONE) image is still caught here.
	if (esp_ota_end(handle) != ESP_OK) {
		ctx.nack(status::kUnknownError);
		return;
	}

	gOtaStaged = true;
	gOtaStagedPartition = targetPartition;

	bool applyNow = (flags & otaInstallFlags::kApplyNow) != 0;
	if (applyNow) {
		esp_ota_set_boot_partition(targetPartition);
		ctx.ack();
		// §16.2's firmware note applies here too: let the ACK actually flush before rebooting.
		settleAckBeforeDisruptingTransport(ctx.activeTransportValue);
		delay(100);
		ESP.restart();
	} else {
		ctx.ack();
	}
}

void handleOtaApply(const CommandContext& ctx) {
	if (!gOtaStaged) {
		ctx.nack(status::kOtaNotStaged);
		return;
	}
	esp_ota_set_boot_partition(gOtaStagedPartition);
	ctx.ack();
	settleAckBeforeDisruptingTransport(ctx.activeTransportValue);
	delay(100);
	ESP.restart();
}

void handleOtaStatusRequest(const CommandContext& ctx) {
	const esp_partition_t* running = esp_ota_get_running_partition();
	esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
	esp_ota_get_state_partition(running, &state);
	bool pendingVerification = (state == ESP_OTA_IMG_PENDING_VERIFY);

	uint8_t versionLen = static_cast<uint8_t>(strlen(kFirmwareVersion));

	std::vector<uint8_t> out;
	out.push_back(otaSlotIndex(running));
	out.push_back(pendingVerification ? 1 : 0);
	out.push_back(versionLen);
	out.insert(out.end(), kFirmwareVersion, kFirmwareVersion + versionLen);
	ctx.reply(cmd::kOtaStatusResponse, out);
}

void handleOtaConfirm(const CommandContext& ctx) {
	esp_ota_mark_app_valid_cancel_rollback();
	ctx.ack();
}

void handleOtaRollback(const CommandContext& ctx) {
	const esp_partition_t* running = esp_ota_get_running_partition();
	const esp_partition_t* previous = esp_ota_get_next_update_partition(nullptr);
	if (!previous || previous == running) {
		ctx.nack(status::kOtaNotStaged);	// nothing else to roll back to
		return;
	}
	esp_ota_set_boot_partition(previous);
	ctx.ack();
	settleAckBeforeDisruptingTransport(ctx.activeTransportValue);
	delay(100);
	ESP.restart();
}

constexpr size_t kFullImageHeaderSize = 10;  // ENCODING(1) + FLAGS(1) + DECODED_LEN(4) + ENCODED_LEN(4)
constexpr size_t kPartialImageHeaderSize = 18;	 // ENCODING/FLAGS/X/Y/WIDTH/HEIGHT/DECODED_LEN/ENCODED_LEN
constexpr size_t kFullImageDecodedLen =
		(GxEPD2_420_GDEY042T81::WIDTH / 8) * GxEPD2_420_GDEY042T81::HEIGHT;  // 400/8 * 300 = 15000 bytes

uint16_t readU16LE(const uint8_t* p) {
	return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t readU32LE(const uint8_t* p) {
	return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
			(static_cast<uint32_t>(p[3]) << 24);
}

void writeU16LE(uint8_t* p, uint16_t value) {
	p[0] = static_cast<uint8_t>(value & 0xFF);
	p[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void writeU32LE(uint8_t* p, uint32_t value) {
	p[0] = static_cast<uint8_t>(value & 0xFF);
	p[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
	p[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
	p[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

// Shared by FULL_IMAGE_TRANSFER (§6) and PARTIAL_IMAGE_TRANSFER (§7) - both use the identical
// ENCODING/RAW/RLE_PACKBITS scheme. Returns true and fills `out` on success; false means the
// caller should NACK with `failStatus`.
bool decodeImageData(uint8_t encoding, const uint8_t* encodedData, size_t encodedLen, size_t decodedLen,
		std::vector<uint8_t>& out, uint8_t& failStatus) {
	if (encoding == 0x00) {  // RAW
		if (encodedLen != decodedLen) {
			failStatus = status::kBadParameters;
			return false;
		}
		out.assign(encodedData, encodedData + encodedLen);
		return true;
	}
	if (encoding == 0x01) {  // RLE_PACKBITS
		try {
			out = rleDecode(encodedData, encodedLen, decodedLen);
			return true;
		} catch (const std::invalid_argument&) {
			failStatus = status::kDecodeFail;
			return false;
		}
	}
	failStatus = status::kBadParameters;
	return false;
}

// Shared tail of every write-capable handler (§2.1) once its region of the working buffer is
// ready: per FLAGS.REFRESH_NOW/REFRESH_FULL, either flushes [x,y,w,h) to the panel immediately or
// leaves it for a later REFRESH (§12.8) to pick up via WorkingBuffer's dirty-region tracking.
void finishWrite(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t flags) {
	if (flags & writeFlags::kRefreshNow) {
		gWorkingBuffer.flush(x, y, w, h, /*full=*/flags & writeFlags::kRefreshFull);
	} else {
		gWorkingBuffer.markDirty(x, y, w, h);
	}
}

void applyImageWrite(const std::vector<uint8_t>& decoded, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
		uint8_t flags) {
	gWorkingBuffer.write(decoded.data(), x, y, w, h);
	finishWrite(x, y, w, h, flags);
}

// Maps a §12 drawing primitive's own logical geometry (the raw command coordinates, pre-offset) to
// the physical region actually touched - via WorkingBuffer::computeAffectedPhysicalRegion(), which
// applies the exact same SET_DRAW_OFFSET/SET_CLIP_REGION/SET_ORIENTATION pipeline every individual
// pixel write already went through - and forwards to finishWrite(). Shared by every §12 drawing
// primitive below to compute its affected region. A fully clipped/panned-away shape (nothing left)
// is a no-op: WorkingBufferGfx's drawPixel already silently dropped every one of its pixels (this
// same pipeline is enforced per-pixel in WorkingBuffer::getPixel/setPixel), so there is nothing new
// to flip regardless.
void finishDraw(int32_t x, int32_t y, int32_t w, int32_t h, uint8_t flags) {
	uint16_t px, py, pw, ph;
	if (!gWorkingBuffer.computeAffectedPhysicalRegion(x, y, w, h, px, py, pw, ph)) {
		return;
	}
	finishWrite(px, py, pw, ph, flags);
}

// Hand-rolled (not Adafruit_GFX's drawLine, which is 1px-only) Bresenham stepping + a square brush
// stamped at each point via fillRect - the simplest common technique for a LINE_WIDTH>1 line,
// reusing WorkingBufferGfx's already-correct, already-clipped, already-mode-aware fillRect for the
// actual pixel writes (only the stepping itself is hand-rolled).
void drawThickLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t lineWidth, uint8_t colorValue) {
	int16_t half = lineWidth / 2;
	int16_t dx = abs(x1 - x0);
	int16_t dy = -abs(y1 - y0);
	int16_t sx = x0 < x1 ? 1 : -1;
	int16_t sy = y0 < y1 ? 1 : -1;
	int16_t err = dx + dy;
	while (true) {
		gWorkingBufferGfx.fillRect(x0 - half, y0 - half, lineWidth, lineWidth, colorValue);
		if (x0 == x1 && y0 == y1) {
			break;
		}
		int16_t e2 = 2 * err;
		if (e2 >= dy) {
			err += dy;
			x0 += sx;
		}
		if (e2 <= dx) {
			err += dx;
			y0 += sy;
		}
	}
}

// doc/PROTOCOL.md §6, next-steps.md #4: always targets the full panel (no region fields on the
// wire) - see doc/SSD1683_Datasheet.PDF §7's Write RAM (Black White) command description for why
// the working buffer keeps wire polarity (bit=1=BLACK) while the controller itself is the opposite
// (WorkingBuffer::flush() handles that translation, not this handler).
void handleFullImageTransfer(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);  // this board has no display wired up at all
		return;
	}

	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < kFullImageHeaderSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	uint8_t encoding = payload[0];
	uint8_t flags = payload[1];
	uint32_t decodedLen = readU32LE(&payload[2]);
	uint32_t encodedLen = readU32LE(&payload[6]);
	if (decodedLen != kFullImageDecodedLen || encodedLen != payload.size() - kFullImageHeaderSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	std::vector<uint8_t> decoded;
	uint8_t failStatus;
	if (!decodeImageData(encoding, payload.data() + kFullImageHeaderSize, encodedLen, decodedLen, decoded,
				failStatus)) {
		ctx.nack(failStatus);
		return;
	}

	applyImageWrite(decoded, 0, 0, GxEPD2_420_GDEY042T81::WIDTH, GxEPD2_420_GDEY042T81::HEIGHT, flags);
	ctx.ack();
}

// doc/PROTOCOL.md §7, next-steps.md #6: like FULL_IMAGE_TRANSFER but scoped to a region, which -
// unlike §12's drawing primitives - must be alignment-checked against
// PARTIAL_REFRESH_GRANULARITY_X/Y (board::kPartialRefreshGranularityX/Y) and stay in-bounds; §7 is
// explicit that violations NACK(BAD_PARAMETERS) rather than being silently rounded/clamped.
void handlePartialImageTransfer(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}

	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < kPartialImageHeaderSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	uint8_t encoding = payload[0];
	uint8_t flags = payload[1];
	uint16_t x = readU16LE(&payload[2]);
	uint16_t y = readU16LE(&payload[4]);
	uint16_t w = readU16LE(&payload[6]);
	uint16_t h = readU16LE(&payload[8]);
	uint32_t decodedLen = readU32LE(&payload[10]);
	uint32_t encodedLen = readU32LE(&payload[14]);

	bool aligned = (x % board::kPartialRefreshGranularityX == 0) && (w % board::kPartialRefreshGranularityX == 0) &&
			(y % board::kPartialRefreshGranularityY == 0) && (h % board::kPartialRefreshGranularityY == 0);
	bool inBounds = (x + w <= GxEPD2_420_GDEY042T81::WIDTH) && (y + h <= GxEPD2_420_GDEY042T81::HEIGHT);
	uint32_t expectedDecodedLen = static_cast<uint32_t>(w / 8) * h;
	if (!aligned || !inBounds || decodedLen != expectedDecodedLen ||
			encodedLen != payload.size() - kPartialImageHeaderSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	std::vector<uint8_t> decoded;
	uint8_t failStatus;
	if (!decodeImageData(encoding, payload.data() + kPartialImageHeaderSize, encodedLen, decodedLen, decoded,
				failStatus)) {
		ctx.nack(failStatus);
		return;
	}

	applyImageWrite(decoded, x, y, w, h, flags);
	ctx.ack();
}

constexpr size_t kReadScreenFullPayloadSize = 2;    // SOURCE + MODE
constexpr size_t kReadScreenRegionPayloadSize = 10;  // + X/Y/WIDTH/HEIGHT
constexpr size_t kScreenDataHeaderSize = 17;  // ENCODING+X+Y+WIDTH+HEIGHT+DECODED_LEN+ENCODED_LEN

// doc/PROTOCOL.md §8: replies with SCREEN_DATA, reusing the exact §6 RAW/RLE scheme (always
// RLE_PACKBITS-encodes the response here - "any conformant encoder is valid" per §6, and RLE is a
// safe default for typical e-ink content). Reads WorkingBuffer's raw physical buffer bytes directly
// (see WorkingBuffer::readWorkingBufferRegion()/readPanelRegion() - deliberately NOT the logical
// SET_DRAW_OFFSET/SET_ORIENTATION/clip pipeline the §12 drawing primitives go through), so X/Y/
// WIDTH/HEIGHT here are physical panel coordinates, matching what this response itself echoes back.
void handleReadScreen(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < kReadScreenFullPayloadSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	uint8_t source = payload[0];
	uint8_t mode = payload[1];
	uint16_t x = 0;
	uint16_t y = 0;
	uint16_t w = GxEPD2_420_GDEY042T81::WIDTH;
	uint16_t h = GxEPD2_420_GDEY042T81::HEIGHT;

	if (mode == readScreenMode::kRegion) {
		if (payload.size() != kReadScreenRegionPayloadSize) {
			ctx.nack(status::kBadParameters);
			return;
		}
		x = readU16LE(&payload[2]);
		y = readU16LE(&payload[4]);
		w = readU16LE(&payload[6]);
		h = readU16LE(&payload[8]);
	} else if (mode != readScreenMode::kFull || payload.size() != kReadScreenFullPayloadSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	if (source > readScreenSource::kWorkingBuffer || static_cast<uint32_t>(x) + w > GxEPD2_420_GDEY042T81::WIDTH ||
			static_cast<uint32_t>(y) + h > GxEPD2_420_GDEY042T81::HEIGHT) {
		ctx.nack(status::kBadParameters);
		return;
	}

	std::vector<uint8_t> raw = source == readScreenSource::kPanel ? gWorkingBuffer.readPanelRegion(x, y, w, h)
																   : gWorkingBuffer.readWorkingBufferRegion(x, y, w, h);
	std::vector<uint8_t> encoded = rleEncode(raw.data(), raw.size());

	std::vector<uint8_t> response(kScreenDataHeaderSize + encoded.size());
	response[0] = 0x01;  // ENCODING: RLE_PACKBITS
	writeU16LE(&response[1], x);
	writeU16LE(&response[3], y);
	writeU16LE(&response[5], w);
	writeU16LE(&response[7], h);
	writeU32LE(&response[9], static_cast<uint32_t>(raw.size()));
	writeU32LE(&response[13], static_cast<uint32_t>(encoded.size()));
	std::memcpy(response.data() + kScreenDataHeaderSize, encoded.data(), encoded.size());

	ctx.reply(cmd::kScreenData, response);
}

constexpr size_t kClearArtifactsPayloadSize = 2;
constexpr uint8_t kDefaultClearArtifactsCycles = 3;

// doc/PROTOCOL.md §9: runs CYCLES full black/white flash cycles directly against the display driver
// (same technique as displaySelfTest()'s bring-up flash, not through the logical working buffer) to
// physically discharge residual e-ink particle bias. CYCLES=0 uses a firmware default. FLAGS bit0
// RESTORE_CONTENT=0 leaves the panel blank and marks the "last physically presented" snapshot
// (§8 SOURCE=PANEL) blank to match, since the working buffer's own logical content is untouched;
// =1 re-flips the current working buffer back via the normal flush() path instead, which keeps that
// snapshot in sync on its own.
void handleClearArtifacts(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != kClearArtifactsPayloadSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	uint8_t cycles = payload[0] == 0 ? kDefaultClearArtifactsCycles : payload[0];
	uint8_t flags = payload[1];

	gDisplay.setFullWindow();
	for (uint8_t i = 0; i < cycles; i++) {
		gDisplay.fillScreen(GxEPD_BLACK);
		gDisplay.display(/*partial_update_mode=*/false);
		gDisplay.fillScreen(GxEPD_WHITE);
		gDisplay.display(/*partial_update_mode=*/false);
	}

	if (flags & clearArtifactsFlags::kRestoreContent) {
		gWorkingBuffer.flush(0, 0, GxEPD2_420_GDEY042T81::WIDTH, GxEPD2_420_GDEY042T81::HEIGHT, /*full=*/true);
	} else {
		gWorkingBuffer.markPanelBlank();
	}
	ctx.ack();
}

// doc/PROTOCOL.md §12.8, next-steps.md #6: has no pixel payload of its own - MODE=0x01 forces a
// full-panel refresh (whatever's currently in the working buffer, regardless of what's tracked as
// dirty); MODE=0x00 flushes just the union of regions written with FLAGS.REFRESH_NOW=0 since the
// last refresh, or is a harmless no-op ACK if nothing is currently pending.
void handleRefresh(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}

	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.empty()) {
		ctx.nack(status::kBadParameters);
		return;
	}

	uint8_t mode = payload[0];
	if (mode == 0x01) {
		gWorkingBuffer.flush(0, 0, GxEPD2_420_GDEY042T81::WIDTH, GxEPD2_420_GDEY042T81::HEIGHT, /*full=*/true);
		gWorkingBuffer.clearDirty();
	} else if (mode == 0x00) {
		if (gWorkingBuffer.hasDirtyRegion()) {
			uint16_t x, y, w, h;
			gWorkingBuffer.getDirtyRegion(x, y, w, h);
			gWorkingBuffer.flush(x, y, w, h, /*full=*/false);
			gWorkingBuffer.clearDirty();
		}
	} else {
		ctx.nack(status::kBadParameters);
		return;
	}
	ctx.ack();
}

constexpr size_t kDrawLinePayloadSize = 12;
constexpr size_t kDrawRectPayloadSize = 13;
constexpr size_t kDrawCirclePayloadSize = 11;
constexpr size_t kClearRegionPayloadSize = 10;

// doc/PROTOCOL.md §12.2. Reuses WorkingBufferGfx's Adafruit_GFX-derived drawLine() for LINE_WIDTH
// 1 (the fast, common case); LINE_WIDTH>1 hand-rolls the Bresenham stepping (see drawThickLine())
// since Adafruit_GFX has no built-in thick-line primitive.
void handleDrawLine(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != kDrawLinePayloadSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	int16_t x0 = static_cast<int16_t>(readU16LE(&payload[0]));
	int16_t y0 = static_cast<int16_t>(readU16LE(&payload[2]));
	int16_t x1 = static_cast<int16_t>(readU16LE(&payload[4]));
	int16_t y1 = static_cast<int16_t>(readU16LE(&payload[6]));
	uint8_t colorValue = payload[8];
	uint8_t mode = payload[9];
	uint8_t lineWidth = payload[10] == 0 ? 1 : payload[10];  // §12.2: "px, min 1"
	uint8_t flags = payload[11];
	if (colorValue > color::kBlack || mode > drawMode::kAnd) {
		ctx.nack(status::kBadParameters);
		return;
	}

	gWorkingBufferGfx.setDrawMode(mode);
	if (lineWidth <= 1) {
		gWorkingBufferGfx.drawLine(x0, y0, x1, y1, colorValue);
	} else {
		drawThickLine(x0, y0, x1, y1, lineWidth, colorValue);
	}

	int32_t half = lineWidth / 2 + 1;  // +1: safety margin, not a precise brush-radius computation
	int32_t bx = std::min(x0, x1) - half;
	int32_t by = std::min(y0, y1) - half;
	finishDraw(bx, by, std::max(x0, x1) - bx + half, std::max(y0, y1) - by + half, flags);
	ctx.ack();
}

// doc/PROTOCOL.md §12.3. FILLED=1 draws once via fillRect(); an outline (FILLED=0) with
// LINE_WIDTH>1 draws LINE_WIDTH nested drawRect() outlines shrinking inward by 1px each time -
// always contained within the original [x,y,w,h), so no bounding-box padding is needed here
// (unlike DRAW_LINE's brush, which extends past its own endpoints).
void handleDrawRect(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != kDrawRectPayloadSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	int16_t x = static_cast<int16_t>(readU16LE(&payload[0]));
	int16_t y = static_cast<int16_t>(readU16LE(&payload[2]));
	int16_t w = static_cast<int16_t>(readU16LE(&payload[4]));
	int16_t h = static_cast<int16_t>(readU16LE(&payload[6]));
	uint8_t colorValue = payload[8];
	uint8_t mode = payload[9];
	bool filled = payload[10] != 0;
	uint8_t lineWidth = payload[11] == 0 ? 1 : payload[11];
	uint8_t flags = payload[12];
	if (colorValue > color::kBlack || mode > drawMode::kAnd) {
		ctx.nack(status::kBadParameters);
		return;
	}

	gWorkingBufferGfx.setDrawMode(mode);
	if (filled) {
		gWorkingBufferGfx.fillRect(x, y, w, h, colorValue);
	} else {
		uint8_t maxInset = static_cast<uint8_t>(std::min<int16_t>(lineWidth, std::min(w, h) / 2 + 1));
		for (uint8_t inset = 0; inset < maxInset; inset++) {
			gWorkingBufferGfx.drawRect(x + inset, y + inset, w - 2 * inset, h - 2 * inset, colorValue);
		}
	}

	finishDraw(x, y, w, h, flags);
	ctx.ack();
}

// doc/PROTOCOL.md §12.4. Same FILLED/LINE_WIDTH approach as DRAW_RECT, via
// fillCircle()/drawCircle().
void handleDrawCircle(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != kDrawCirclePayloadSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	int16_t cx = static_cast<int16_t>(readU16LE(&payload[0]));
	int16_t cy = static_cast<int16_t>(readU16LE(&payload[2]));
	int16_t radius = static_cast<int16_t>(readU16LE(&payload[4]));
	uint8_t colorValue = payload[6];
	uint8_t mode = payload[7];
	bool filled = payload[8] != 0;
	uint8_t lineWidth = payload[9] == 0 ? 1 : payload[9];
	uint8_t flags = payload[10];
	if (colorValue > color::kBlack || mode > drawMode::kAnd) {
		ctx.nack(status::kBadParameters);
		return;
	}

	gWorkingBufferGfx.setDrawMode(mode);
	if (filled) {
		gWorkingBufferGfx.fillCircle(cx, cy, radius, colorValue);
	} else {
		uint8_t maxInset = static_cast<uint8_t>(std::min<int16_t>(lineWidth, radius + 1));
		for (uint8_t inset = 0; inset < maxInset; inset++) {
			gWorkingBufferGfx.drawCircle(cx, cy, radius - inset, colorValue);
		}
	}

	finishDraw(cx - radius, cy - radius, 2 * radius + 1, 2 * radius + 1, flags);
	ctx.ack();
}

// doc/PROTOCOL.md §12.5: fills a rectangle with a solid color - no DRAW_MODE field on the wire, so
// this always behaves as REPLACE regardless of whatever mode a previous drawing command left set.
void handleClearRegion(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != kClearRegionPayloadSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	int16_t x = static_cast<int16_t>(readU16LE(&payload[0]));
	int16_t y = static_cast<int16_t>(readU16LE(&payload[2]));
	int16_t w = static_cast<int16_t>(readU16LE(&payload[4]));
	int16_t h = static_cast<int16_t>(readU16LE(&payload[6]));
	uint8_t colorValue = payload[8];
	uint8_t flags = payload[9];
	if (colorValue > color::kBlack) {
		ctx.nack(status::kBadParameters);
		return;
	}

	gWorkingBufferGfx.setDrawMode(drawMode::kReplace);
	gWorkingBufferGfx.fillRect(x, y, w, h, colorValue);

	finishDraw(x, y, w, h, flags);
	ctx.ack();
}

constexpr size_t kFastClearPayloadSize = 2;  // COLOR(1) + FLAGS(1)

// doc/PROTOCOL.md §12.16 FAST_CLEAR, requested directly: "fast buffer filling with 1 or 0... the
// opposite to brush color that we already have (do we?), skipping all clipping and mapping
// guards" - there's no persistent "current brush color" anywhere in this protocol (every §12
// command already takes its own explicit COLOR byte, same as every other primitive), and this one
// deliberately bypasses SET_CLIP_REGION/SET_DRAW_OFFSET/SET_ORIENTATION entirely via
// WorkingBuffer::fastClear() + finishWrite() directly (not finishDraw(), which is what applies
// those three) - always the whole physical panel, no X/Y/WIDTH/HEIGHT field on the wire at all.
void handleFastClear(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != kFastClearPayloadSize) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint8_t colorValue = payload[0];
	uint8_t flags = payload[1];
	if (colorValue > color::kBlack) {
		ctx.nack(status::kBadParameters);
		return;
	}

	gWorkingBuffer.fastClear(colorValue);
	finishWrite(0, 0, board::kDisplayWidthPx, board::kDisplayHeightPx, flags);
	ctx.ack();
}

// doc/PROTOCOL.md's SET_CUSTOM_FONT_FOLDER section: usage-level, session-only (no FLAGS/PERSIST
// byte at all - the same "pure in-RAM state" family as SET_DRAW_OFFSET/SET_CLIP_REGION/
// SET_ORIENTATION above) command that points DRAW_TEXT's FONT_ID=0xFF custom font (CustomFont.h)
// at a folder of glyph files. PATH must carry a mandatory 2-byte volume prefix - "S:"/"F:" select
// SD/INTERNAL directly; "R:" instead reads a PSRAM file and uses *its* content (which must itself
// be an "S:"/"F:"-prefixed path) as the real folder, resolved exactly once, right here - unlike
// DRAW_TEXT's own FLAGS.TEXT_IS_PATH, which re-reads on every single draw. This is the one
// deliberate difference from DRAW_TEXT's own R:/S:/F: convention: no default-to-PSRAM for a
// missing/unrecognized prefix, since PSRAM has no subdirectories and can never be a valid folder
// target. The folder itself (not the individual glyph files, which load lazily - CustomFont.h) is
// checked to exist here, so a typo'd path fails fast at SET time rather than at the next draw.
void handleSetCustomFontFolder(const CommandContext& ctx) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.empty()) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint8_t pathLen = payload[0];
	if (payload.size() != 1u + pathLen || pathLen < 2) {
		ctx.nack(status::kBadParameters);
		return;
	}
	std::string path(reinterpret_cast<const char*>(&payload[1]), pathLen);
	if (path[1] != ':' || (path[0] != 'R' && path[0] != 'S' && path[0] != 'F')) {
		ctx.nack(status::kBadParameters);
		return;
	}

	uint8_t resolvedVolume;
	std::string resolvedPath;
	if (path[0] == 'R') {
		std::vector<uint8_t> pointerData;
		StorageManager::Result pointerResult = gStorageManager.download(volume::kPsram, path.substr(2), pointerData);
		if (pointerResult != StorageManager::Result::kOk) {
			ctx.nack(status::kFileNotFound);
			return;
		}
		std::string realPath(reinterpret_cast<const char*>(pointerData.data()), pointerData.size());
		if (realPath.size() < 2 || realPath[1] != ':' || (realPath[0] != 'S' && realPath[0] != 'F')) {
			ctx.nack(status::kBadParameters);
			return;
		}
		resolvedVolume = realPath[0] == 'S' ? volume::kSd : volume::kInternal;
		resolvedPath = realPath.substr(2);
	} else {
		resolvedVolume = path[0] == 'S' ? volume::kSd : volume::kInternal;
		resolvedPath = path.substr(2);
	}

	std::vector<StorageManager::Entry> entries;
	StorageManager::Result listResult = gStorageManager.list(resolvedVolume, resolvedPath, entries);
	if (listResult == StorageManager::Result::kVolumeNotPresent) {
		ctx.nack(status::kVolumeNotPresent);
		return;
	}
	if (listResult != StorageManager::Result::kOk) {
		ctx.nack(status::kFileNotFound);
		return;
	}

	customFont::setFolder(resolvedVolume, resolvedPath);
	ctx.ack();
}

// X/Y/WIDTH/FONT_ID/COLOR/BACKGROUND/DRAW_MODE/ALIGN/WRAP/FLAGS/TEXT_LEN
constexpr size_t kDrawTextHeaderSize = 15;

// doc/PROTOCOL.md §12.6: FONT_ID 0x00 (classic, small) and 0x01 (u8g2_font_unifont_t_extended,
// larger, real precomposed Czech glyphs - added on request, "do we have space for a larger font?")
// are the two embedded fonts (EmbeddedFont.h); 0xFF is the folder-driven custom proportional font
// (CustomFont.h, set up via SET_CUSTOM_FONT_FOLDER above) - no other value is implemented, so
// anything else NACKs BAD_PARAMETERS rather than silently falling back. For FONT_ID=0xFF, TEXT is
// a raw single-byte codepage rather than UTF-8 (see EmbeddedFont.h's drawText() doc), and a
// pre-flight customFont::ensureReady() check NACKs FILE_NOT_FOUND if no folder was ever configured
// this session or its XX.gly fallback glyph itself is missing/corrupt - any other individual
// glyph's own file being missing/corrupt is not an error, it silently falls back to XX at draw
// time. WIDTH=0 means a single unbounded line (ALIGN/WRAP are then ignored, not validated either
// way). BACKGROUND=OPAQUE fills each glyph's non-ink pixels with the opposite of COLOR, routed
// through the same drawPixel()/compositePixel() path as the ink pixels so DRAW_MODE applies
// uniformly to both - "inverted" text is simply COLOR=WHITE with an OPAQUE background
// (EmbeddedFont.h's drawCodepointLine doc has the detail). FLAGS.TEXT_IS_PATH (drawTextFlags,
// Protocol.h) redirects TEXT to mean a path instead of literal text - the file's content, read
// fresh every time, becomes what's actually drawn, letting a recorded macro (§18) stay
// parametrized by writing new content to that file before each replay rather than needing a
// different macro per value. No VOLUME field exists on this command's own payload, so an optional
// "R:"/"S:"/"F:" prefix on TEXT picks PSRAM/SD/INTERNAL (stripped before the rest is used as the
// path); no prefix defaults to PSRAM. FLAGS.MISSING_FILE_TOLERANT (only meaningful alongside
// TEXT_IS_PATH) turns a missing referenced file from NACK(FILE_NOT_FOUND) into a no-op ACK -
// nothing painted, working buffer left untouched - for callers (e.g. a boot/status macro) that
// would rather silently skip a line than fail the whole command when its backing file hasn't been
// written yet.
void handleDrawText(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < kDrawTextHeaderSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	int16_t x = static_cast<int16_t>(readU16LE(&payload[0]));
	int16_t y = static_cast<int16_t>(readU16LE(&payload[2]));
	int16_t width = static_cast<int16_t>(readU16LE(&payload[4]));
	uint8_t fontId = payload[6];
	uint8_t colorValue = payload[7];
	uint8_t backgroundValue = payload[8];
	uint8_t mode = payload[9];
	uint8_t alignValue = payload[10];
	uint8_t wrapValue = payload[11];
	uint8_t flags = payload[12];
	uint16_t textLen = readU16LE(&payload[13]);
	if ((fontId > 0x01 && fontId != 0xFF) || colorValue > color::kBlack ||
			backgroundValue > textBackground::kOpaque || mode > drawMode::kAnd ||
			alignValue > static_cast<uint8_t>(TextAlign::kRight) || payload.size() != kDrawTextHeaderSize + textLen) {
		ctx.nack(status::kBadParameters);
		return;
	}
	if (fontId == 0xFF) {
		uint8_t failStatus;
		if (!customFont::ensureReady(gStorageManager, failStatus)) {
			ctx.nack(failStatus);
			return;
		}
	}

	const uint8_t* textData = payload.data() + kDrawTextHeaderSize;
	size_t textDataLen = textLen;
	std::vector<uint8_t> fileContent;
	if (flags & drawTextFlags::kTextIsPath) {
		std::string path(reinterpret_cast<const char*>(textData), textDataLen);
		// Optional 2-byte volume prefix (no VOLUME field exists on this command's own wire payload,
		// unlike DRAW_IMAGE) - "R:"/"S:"/"F:" select PSRAM/SD/INTERNAL and are stripped before the
		// rest is used as the path; anything else (no prefix, or an unrecognized one) defaults to
		// PSRAM with the path used exactly as given, unstripped.
		uint8_t sourceVolume = volume::kPsram;
		if (path.size() >= 2 && path[1] == ':' &&
				(path[0] == 'R' || path[0] == 'S' || path[0] == 'F')) {
			sourceVolume = path[0] == 'S' ? volume::kSd : path[0] == 'F' ? volume::kInternal : volume::kPsram;
			path.erase(0, 2);
		}
		StorageManager::Result storageResult = gStorageManager.download(sourceVolume, path, fileContent);
		if (storageResult == StorageManager::Result::kVolumeNotPresent) {
			ctx.nack(status::kVolumeNotPresent);
			return;
		}
		if (storageResult != StorageManager::Result::kOk) {
			if (flags & drawTextFlags::kMissingFileTolerant) {
				// No usable text to draw and the caller opted in to tolerating that - skip the
				// draw entirely (working buffer left untouched) rather than failing the command.
				ctx.ack();
				return;
			}
			ctx.nack(status::kFileNotFound);
			return;
		}
		textData = fileContent.data();
		textDataLen = fileContent.size();
	}

	gWorkingBufferGfx.setDrawMode(mode);
	TextLayoutResult result = drawText(gWorkingBufferGfx, gStorageManager, fontId, x, y, width,
			static_cast<TextAlign>(alignValue), wrapValue != 0, textData, textDataLen, colorValue,
			backgroundValue == textBackground::kOpaque);

	finishDraw(x, y, result.width, result.height, flags);
	ctx.ack();
}

constexpr size_t kDrawImageHeaderSize = 8;  // X/Y/DRAW_MODE/FLAGS/VOLUME/PATH_LEN
constexpr size_t kDrawImageDataHeaderSize = 10;  // X/Y/DRAW_MODE/FLAGS/DATA_LEN
constexpr size_t kEpiHeaderSize = 19;	 // MAGIC+FORMAT_VERSION+WIDTH+HEIGHT+ENCODING+FLAGS+COLOR_DECODED_LEN+COLOR_ENCODED_LEN
constexpr uint8_t kEpiFormatVersion = 0x01;
// Sanity cap against a corrupt/malicious .epi file claiming an enormous WIDTH*HEIGHT (e.g. read
// from a removable SD card) - comfortably larger than any real use case on a 400x300 panel, small
// enough to bound the temporary decode buffer this handler allocates.
constexpr uint32_t kMaxEpiPixels = 4'000'000;

// Reads one bit from a packed 1bpp row-major MSB-first buffer (doc/PROTOCOL.md §6's layout) sized
// for `width` - the same layout WorkingBuffer uses internally, but this is a temporary decode
// buffer with its own width, not the panel's, so it needs its own (not WorkingBuffer's private)
// bit-addressing helper.
bool getPackedBit(const std::vector<uint8_t>& data, uint16_t width, uint16_t x, uint16_t y) {
	size_t bytesPerRow = (width + 7) / 8;
	return (data[y * bytesPerRow + x / 8] & (0x80 >> (x % 8))) != 0;
}

// A fully downloaded-and-decoded .epi file, ready to blit - shared by DRAW_IMAGE (§12.7) and
// DRAW_IMAGE_ROW (§12.x), which both need every referenced image's own dimensions before any pixel
// can be positioned (DRAW_IMAGE_ROW's alignment depends on every image's width up front), so
// decoding is split from drawing rather than combined into one step like earlier single-image-only
// code did.
struct DecodedEpiImage {
	uint16_t width = 0;
	uint16_t height = 0;
	bool hasMask = false;
	std::vector<uint8_t> colorData;
	std::vector<uint8_t> maskData;
};

// Parses and decodes one already-in-memory .epi file - does not draw anything (see
// blitDecodedEpiImage() for that) and does not care where the bytes came from (device storage via
// decodeEpiImage() below, or embedded directly in a command payload, e.g. DRAW_IMAGE_DATA). Reuses
// decodeImageData() (the exact §6 RAW/RLE scheme) since the .epi format's COLOR_DATA/MASK_DATA
// streams are byte-for-byte that same encoding.
bool decodeEpiFromBytes(const std::vector<uint8_t>& fileData, DecodedEpiImage& outImage, uint8_t& failStatus) {
	if (fileData.size() < kEpiHeaderSize || std::memcmp(fileData.data(), "EPI1", 4) != 0 ||
			fileData[4] != kEpiFormatVersion) {
		failStatus = status::kDecodeFail;
		return false;
	}
	uint16_t width = readU16LE(&fileData[5]);
	uint16_t height = readU16LE(&fileData[7]);
	uint8_t encoding = fileData[9];
	uint8_t epiFlagsValue = fileData[10];
	uint32_t colorDecodedLen = readU32LE(&fileData[11]);
	uint32_t colorEncodedLen = readU32LE(&fileData[15]);
	if (width == 0 || height == 0 || static_cast<uint32_t>(width) * height > kMaxEpiPixels) {
		failStatus = status::kBadParameters;
		return false;
	}
	uint32_t expectedColorLen = static_cast<uint32_t>((width + 7) / 8) * height;
	size_t pos = kEpiHeaderSize;
	if (colorDecodedLen != expectedColorLen || pos + colorEncodedLen > fileData.size()) {
		failStatus = status::kBadParameters;
		return false;
	}

	std::vector<uint8_t> colorData;
	if (!decodeImageData(encoding, fileData.data() + pos, colorEncodedLen, colorDecodedLen, colorData, failStatus)) {
		return false;
	}
	pos += colorEncodedLen;

	bool hasMask = (epiFlagsValue & epiFlags::kHasMask) != 0;
	std::vector<uint8_t> maskData;
	if (hasMask) {
		if (pos + 8 > fileData.size()) {
			failStatus = status::kBadParameters;
			return false;
		}
		uint32_t maskDecodedLen = readU32LE(&fileData[pos]);
		uint32_t maskEncodedLen = readU32LE(&fileData[pos + 4]);
		pos += 8;
		if (maskDecodedLen != colorDecodedLen || pos + maskEncodedLen > fileData.size()) {
			failStatus = status::kBadParameters;
			return false;
		}
		if (!decodeImageData(encoding, fileData.data() + pos, maskEncodedLen, maskDecodedLen, maskData, failStatus)) {
			return false;
		}
	}

	outImage.width = width;
	outImage.height = height;
	outImage.hasMask = hasMask;
	outImage.colorData = std::move(colorData);
	outImage.maskData = std::move(maskData);
	return true;
}

// Downloads one .epi file from device storage (§14 VOLUME) and decodes it via decodeEpiFromBytes()
// - used by DRAW_IMAGE/DRAW_IMAGE_ROW, which reference a stored file rather than embedding one
// inline (contrast DRAW_IMAGE_DATA, which calls decodeEpiFromBytes() directly on its own payload).
bool decodeEpiImage(uint8_t volumeValue, const std::string& path, DecodedEpiImage& outImage, uint8_t& failStatus) {
	std::vector<uint8_t> fileData;
	StorageManager::Result storageResult = gStorageManager.download(volumeValue, path, fileData);
	if (storageResult == StorageManager::Result::kVolumeNotPresent) {
		failStatus = status::kVolumeNotPresent;
		return false;
	}
	if (storageResult != StorageManager::Result::kOk) {
		failStatus = status::kFileNotFound;
		return false;
	}
	return decodeEpiFromBytes(fileData, outImage, failStatus);
}

// Blits an already-decoded image at (x,y) - non-ink/transparent (masked) pixels are skipped
// entirely, never drawn, so DRAW_MODE never applies to them, matching §12.1's convention for
// text/masked images - unless `ignoreMask` (FLAGS.IGNORE_MASK, drawImageFlags) is set, which draws
// every pixel opaque regardless of the .epi data's own HAS_MASK/MASK_DATA. The caller must already
// have set gWorkingBufferGfx's draw mode.
void blitDecodedEpiImage(const DecodedEpiImage& image, int16_t x, int16_t y, bool ignoreMask = false) {
	for (uint16_t row = 0; row < image.height; row++) {
		for (uint16_t col = 0; col < image.width; col++) {
			if (!ignoreMask && image.hasMask && !getPackedBit(image.maskData, image.width, col, row)) {
				continue;
			}
			bool black = getPackedBit(image.colorData, image.width, col, row);
			gWorkingBufferGfx.drawPixel(static_cast<int16_t>(x + col), static_cast<int16_t>(y + row),
					black ? color::kBlack : color::kWhite);
		}
	}
}

// doc/PROTOCOL.md §12.7: draws a `.epi` image file (§14 VOLUME) at (X,Y) - a §12 drawing primitive
// like any other, so it goes through the same DRAW_MODE/clip/offset/orientation pipeline as
// DRAW_LINE/DRAW_RECT/etc (via gWorkingBufferGfx.drawPixel()), unlike FULL_IMAGE_TRANSFER/
// PARTIAL_IMAGE_TRANSFER which write the panel-sized buffer directly.
void handleDrawImage(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < kDrawImageHeaderSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	int16_t x = static_cast<int16_t>(readU16LE(&payload[0]));
	int16_t y = static_cast<int16_t>(readU16LE(&payload[2]));
	uint8_t mode = payload[4];
	uint8_t flags = payload[5];
	uint8_t volumeValue = payload[6];
	uint8_t pathLen = payload[7];
	if (mode > drawMode::kAnd || volumeValue > volume::kPsram ||
			payload.size() != kDrawImageHeaderSize + pathLen) {
		ctx.nack(status::kBadParameters);
		return;
	}
	std::string path(reinterpret_cast<const char*>(payload.data() + kDrawImageHeaderSize), pathLen);

	DecodedEpiImage image;
	uint8_t failStatus;
	if (!decodeEpiImage(volumeValue, path, image, failStatus)) {
		ctx.nack(failStatus);
		return;
	}

	gWorkingBufferGfx.setDrawMode(mode);
	blitDecodedEpiImage(image, x, y, (flags & drawImageFlags::kIgnoreMask) != 0);

	finishDraw(x, y, image.width, image.height, flags);
	ctx.ack();
}

// doc/PROTOCOL.md §12.x DRAW_IMAGE_DATA: like DRAW_IMAGE but the `.epi` image is embedded in DATA
// rather than referenced by a stored path - no prior FILE_UPLOAD needed. DATA is parsed with the
// exact same rules as the .epi file format (decodeEpiFromBytes(), shared verbatim with DRAW_IMAGE),
// including the IGNORE_MASK flag handling.
void handleDrawImageData(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < kDrawImageDataHeaderSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	int16_t x = static_cast<int16_t>(readU16LE(&payload[0]));
	int16_t y = static_cast<int16_t>(readU16LE(&payload[2]));
	uint8_t mode = payload[4];
	uint8_t flags = payload[5];
	uint32_t dataLen = readU32LE(&payload[6]);
	if (mode > drawMode::kAnd || payload.size() != kDrawImageDataHeaderSize + dataLen) {
		ctx.nack(status::kBadParameters);
		return;
	}
	std::vector<uint8_t> epiBytes(payload.begin() + kDrawImageDataHeaderSize, payload.end());

	DecodedEpiImage image;
	uint8_t failStatus;
	if (!decodeEpiFromBytes(epiBytes, image, failStatus)) {
		ctx.nack(failStatus);
		return;
	}

	gWorkingBufferGfx.setDrawMode(mode);
	blitDecodedEpiImage(image, x, y, (flags & drawImageFlags::kIgnoreMask) != 0);

	finishDraw(x, y, image.width, image.height, flags);
	ctx.ack();
}

constexpr size_t kDrawImageRowHeaderSize = 13;	 // X/Y/WIDTH/ALIGN/SPACING/DRAW_MODE/FLAGS/VOLUME/COUNT

// doc/PROTOCOL.md §12.x DRAW_IMAGE_ROW: draws COUNT .epi images (all from the same VOLUME) in a
// horizontal row, each keeping its own natural width/height from its own file header (never
// scaled) - e.g. a strip of status icons, or a "big number" composed from per-digit glyph images.
// ALIGN/SPACING work like DRAW_TEXT's ALIGN/WRAP-adjacent layout concept: WIDTH<=0 means no
// reference box at all (images are simply packed left-to-right starting at X, ALIGN ignored,
// exactly like DRAW_TEXT's WIDTH=0 case); WIDTH>0 is the alignment reference box. BLOCK distributes
// the leftover space (WIDTH minus the sum of every image's own width) evenly across the gaps
// between images, ignoring SPACING entirely (ties to zero, and to plain left-packing, when
// COUNT==1, since there is no gap to distribute across); every other ALIGN value uses SPACING as a
// fixed gap between images instead. All images share one Y (top-aligned) and one DRAW_MODE.
void handleDrawImageRow(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < kDrawImageRowHeaderSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	int16_t x = static_cast<int16_t>(readU16LE(&payload[0]));
	int16_t y = static_cast<int16_t>(readU16LE(&payload[2]));
	int16_t width = static_cast<int16_t>(readU16LE(&payload[4]));
	uint8_t alignValue = payload[6];
	uint16_t spacing = readU16LE(&payload[7]);
	uint8_t mode = payload[9];
	uint8_t flags = payload[10];
	uint8_t volumeValue = payload[11];
	uint8_t count = payload[12];
	if (mode > drawMode::kAnd || volumeValue > volume::kPsram || alignValue > imageRowAlign::kBlock ||
			count == 0) {
		ctx.nack(status::kBadParameters);
		return;
	}

	// Every path is needed before any drawing (positions depend on every image's own width), so
	// parse them all up front rather than interleaving parsing with decode-and-draw.
	std::vector<std::string> paths;
	size_t pos = kDrawImageRowHeaderSize;
	for (uint8_t i = 0; i < count; i++) {
		if (pos + 1 > payload.size()) {
			ctx.nack(status::kBadParameters);
			return;
		}
		uint8_t pathLen = payload[pos];
		pos += 1;
		if (pos + pathLen > payload.size()) {
			ctx.nack(status::kBadParameters);
			return;
		}
		paths.emplace_back(reinterpret_cast<const char*>(payload.data() + pos), pathLen);
		pos += pathLen;
	}
	if (pos != payload.size()) {
		ctx.nack(status::kBadParameters);
		return;
	}

	std::vector<DecodedEpiImage> images(count);
	uint8_t failStatus;
	for (uint8_t i = 0; i < count; i++) {
		if (!decodeEpiImage(volumeValue, paths[i], images[i], failStatus)) {
			ctx.nack(failStatus);
			return;
		}
	}

	uint32_t sumWidths = 0;
	uint16_t maxHeight = 0;
	for (const DecodedEpiImage& image : images) {
		sumWidths += image.width;
		maxHeight = std::max(maxHeight, image.height);
	}

	uint8_t effectiveAlign = width > 0 ? alignValue : imageRowAlign::kLeft;
	uint16_t gap = spacing;
	int32_t startX = x;
	int32_t naturalWidth = static_cast<int32_t>(sumWidths) + static_cast<int32_t>(count - 1) * spacing;
	if (effectiveAlign == imageRowAlign::kCenter) {
		startX = x + (width - naturalWidth) / 2;
	} else if (effectiveAlign == imageRowAlign::kRight) {
		startX = x + width - naturalWidth;
	} else if (effectiveAlign == imageRowAlign::kBlock && count > 1) {
		int32_t extra = static_cast<int32_t>(width) - static_cast<int32_t>(sumWidths);
		gap = static_cast<uint16_t>(std::max<int32_t>(0, extra / (count - 1)));
	}

	gWorkingBufferGfx.setDrawMode(mode);
	int32_t cursorX = startX;
	for (const DecodedEpiImage& image : images) {
		blitDecodedEpiImage(image, static_cast<int16_t>(cursorX), y);
		cursorX += static_cast<int32_t>(image.width) + gap;
	}

	finishDraw(x, y, width > 0 ? width : naturalWidth, maxHeight, flags);
	ctx.ack();
}

constexpr size_t kFillImageHeaderSize = 13;  // X/Y/WIDTH/HEIGHT/TILE_MODE/DRAW_MODE/FLAGS/VOLUME/PATH_LEN

// doc/PROTOCOL.md §12.15 FILL_IMAGE: tiles a .epi image repeatedly to cover a target rectangle -
// e.g. a background pattern, or a decorative border strip. TILE_MODE picks which axis/axes
// actually repeat; the other keeps the image's own natural size (a single row or column). A
// partial tile at the target rectangle's far edge is never specially cropped - achieved by
// temporarily narrowing the clip region (§12.10) to the intersection of FILL_IMAGE's own target
// rect and whatever clip was already active, restored afterward, so every blitted tile's existing
// per-pixel clipping crops it for free, and FILL_IMAGE can never escape a caller's own already-
// active clip either.
void handleFillImage(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < kFillImageHeaderSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	int16_t x = static_cast<int16_t>(readU16LE(&payload[0]));
	int16_t y = static_cast<int16_t>(readU16LE(&payload[2]));
	int16_t width = static_cast<int16_t>(readU16LE(&payload[4]));
	int16_t height = static_cast<int16_t>(readU16LE(&payload[6]));
	uint8_t tileMode = payload[8];
	uint8_t mode = payload[9];
	uint8_t flags = payload[10];
	uint8_t volumeValue = payload[11];
	uint8_t pathLen = payload[12];
	if (mode > drawMode::kAnd || volumeValue > volume::kPsram || tileMode > fillTileMode::kBoth ||
			payload.size() != kFillImageHeaderSize + pathLen) {
		ctx.nack(status::kBadParameters);
		return;
	}
	std::string path(reinterpret_cast<const char*>(payload.data() + kFillImageHeaderSize), pathLen);

	DecodedEpiImage image;
	uint8_t failStatus;
	if (!decodeEpiImage(volumeValue, path, image, failStatus)) {
		ctx.nack(failStatus);
		return;
	}

	int16_t fillWidth = tileMode == fillTileMode::kVertical ? static_cast<int16_t>(image.width) : width;
	int16_t fillHeight = tileMode == fillTileMode::kHorizontal ? static_cast<int16_t>(image.height) : height;

	uint16_t savedClipX, savedClipY, savedClipW, savedClipH;
	gWorkingBuffer.getClipRegion(savedClipX, savedClipY, savedClipW, savedClipH);
	int32_t clipX0 = std::max<int32_t>(x, savedClipX);
	int32_t clipY0 = std::max<int32_t>(y, savedClipY);
	int32_t clipX1 =
			std::min<int32_t>(static_cast<int32_t>(x) + fillWidth, static_cast<int32_t>(savedClipX) + savedClipW);
	int32_t clipY1 =
			std::min<int32_t>(static_cast<int32_t>(y) + fillHeight, static_cast<int32_t>(savedClipY) + savedClipH);

	if (clipX1 > clipX0 && clipY1 > clipY0) {
		gWorkingBuffer.setClipRegion(static_cast<uint16_t>(clipX0), static_cast<uint16_t>(clipY0),
				static_cast<uint16_t>(clipX1 - clipX0), static_cast<uint16_t>(clipY1 - clipY0));
		gWorkingBufferGfx.setDrawMode(mode);

		if (tileMode == fillTileMode::kHorizontal) {
			for (int32_t tx = x; tx < x + fillWidth; tx += image.width) {
				blitDecodedEpiImage(image, static_cast<int16_t>(tx), y);
			}
		} else if (tileMode == fillTileMode::kVertical) {
			for (int32_t ty = y; ty < y + fillHeight; ty += image.height) {
				blitDecodedEpiImage(image, x, static_cast<int16_t>(ty));
			}
		} else {  // kBoth
			for (int32_t ty = y; ty < y + fillHeight; ty += image.height) {
				for (int32_t tx = x; tx < x + fillWidth; tx += image.width) {
					blitDecodedEpiImage(image, static_cast<int16_t>(tx), static_cast<int16_t>(ty));
				}
			}
		}

		gWorkingBuffer.setClipRegion(savedClipX, savedClipY, savedClipW, savedClipH);
	}

	finishDraw(x, y, fillWidth, fillHeight, flags);
	ctx.ack();
}

constexpr size_t kShiftRegionPayloadSize = 13;

// doc/PROTOCOL.md §12.9: scrolls a region's content in place (e.g. for a ticker or a log panel)
// without resending it each step - content shifted past the region's own edge is discarded, and
// the vacated strip is filled with FILL_COLOR. No DRAW_MODE field, like CLEAR_REGION - always a
// plain overwrite, not composited against what's underneath.
void handleShiftRegion(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != kShiftRegionPayloadSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	uint16_t x = readU16LE(&payload[0]);
	uint16_t y = readU16LE(&payload[2]);
	uint16_t w = readU16LE(&payload[4]);
	uint16_t h = readU16LE(&payload[6]);
	uint8_t direction = payload[8];
	uint16_t step = readU16LE(&payload[9]);
	uint8_t fillColor = payload[11];
	uint8_t flags = payload[12];
	if (direction > shiftDirection::kDown || fillColor > color::kBlack) {
		ctx.nack(status::kBadParameters);
		return;
	}

	gWorkingBuffer.shift(x, y, w, h, direction, step, fillColor != 0);

	finishDraw(x, y, w, h, flags);
	ctx.ack();
}

constexpr size_t kSetClipRegionPayloadSize = 8;

// doc/PROTOCOL.md §12.10: constrains every subsequent §12 drawing primitive, SHIFT_REGION, and
// COPY_REGION's destination side to this rectangle - pixels outside it are silently left
// untouched, the same way pixels outside the panel already are (enforced once, in
// WorkingBuffer::getPixel/setPixel, so every caller gets it automatically). WIDTH=0 or HEIGHT=0
// resets the clip to the full panel. Persists across commands until changed again or the device
// reboots - this is device-global state, not per-command like FLAGS/DRAW_MODE.
void handleSetClipRegion(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != kSetClipRegionPayloadSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	uint16_t x = readU16LE(&payload[0]);
	uint16_t y = readU16LE(&payload[2]);
	uint16_t w = readU16LE(&payload[4]);
	uint16_t h = readU16LE(&payload[6]);
	gWorkingBuffer.setClipRegion(x, y, w, h);
	ctx.ack();
}

constexpr size_t kCopyRegionPayloadSize = 13;

// doc/PROTOCOL.md §12.11: copies a rectangle of the working buffer to another location - a plain
// overwrite (no DRAW_MODE), like SHIFT_REGION/CLEAR_REGION. The source is read regardless of the
// current clip region (WorkingBuffer::copyRegion uses getPixelUnclipped for it - see its own
// doc); only the destination is subject to clipping, same as every other write.
void handleCopyRegion(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != kCopyRegionPayloadSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	uint16_t srcX = readU16LE(&payload[0]);
	uint16_t srcY = readU16LE(&payload[2]);
	uint16_t dstX = readU16LE(&payload[4]);
	uint16_t dstY = readU16LE(&payload[6]);
	uint16_t w = readU16LE(&payload[8]);
	uint16_t h = readU16LE(&payload[10]);
	uint8_t flags = payload[12];

	gWorkingBuffer.copyRegion(srcX, srcY, dstX, dstY, w, h);

	finishDraw(dstX, dstY, w, h, flags);
	ctx.ack();
}

constexpr size_t kSetDrawOffsetPayloadSize = 4;

// doc/PROTOCOL.md §12.13: a persistent (dx,dy) pan applied to every §12 read/write - enforced once,
// in WorkingBuffer::getPixel/setPixel (and computeAffectedPhysicalRegion for the flush/dirty
// bounding box), so every caller gets it automatically, same as SET_CLIP_REGION. (0,0) is the
// identity value - no separate reset needed, unlike SET_CLIP_REGION's WIDTH=0/HEIGHT=0 sentinel.
void handleSetDrawOffset(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != kSetDrawOffsetPayloadSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	int16_t dx = static_cast<int16_t>(readU16LE(&payload[0]));
	int16_t dy = static_cast<int16_t>(readU16LE(&payload[2]));
	gWorkingBuffer.setDrawOffset(dx, dy);
	ctx.ack();
}

constexpr size_t kSetOrientationPayloadSize = 2;

// doc/PROTOCOL.md §12.12: rotation (0/90/180/270 CW) + H/V mirroring of the logical canvas that
// every §12 coordinate (including SET_CLIP_REGION and SET_DRAW_OFFSET) is expressed in - enforced
// once, in WorkingBuffer::toPhysical() (called from getPixel/setPixel/computeAffectedPhysicalRegion),
// mapping logical positions to physical buffer positions right before the final bounds check.
void handleSetOrientation(const CommandContext& ctx) {
	if (board::kPinDisplayCs < 0) {
		ctx.nack(status::kUnsupportedCommand);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != kSetOrientationPayloadSize) {
		ctx.nack(status::kBadParameters);
		return;
	}

	uint8_t rotation = payload[0];
	uint8_t flags = payload[1];
	if (rotation > orientation::kRotate270) {
		ctx.nack(status::kBadParameters);
		return;
	}

	gWorkingBuffer.setOrientation(rotation, (flags & orientationFlags::kMirrorH) != 0,
			(flags & orientationFlags::kMirrorV) != 0);
	ctx.ack();
}

// Reads VOLUME(1)/PATH_LEN(1)/PATH(PATH_LEN) - the common request shape shared by FILE_LIST_REQUEST
// (§14.1), FILE_DOWNLOAD_REQUEST (§14.2), and FILE_DELETE (§14.4). Returns false (having already
// NACKed) if the payload doesn't match that shape or VOLUME is out of range.
bool parseVolumeAndPath(const CommandContext& ctx, uint8_t& volumeValue, std::string& path) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < 2) {
		ctx.nack(status::kBadParameters);
		return false;
	}
	volumeValue = payload[0];
	uint8_t pathLen = payload[1];
	if (volumeValue > volume::kPsram || payload.size() != 2 + pathLen) {
		ctx.nack(status::kBadParameters);
		return false;
	}
	path.assign(reinterpret_cast<const char*>(payload.data() + 2), pathLen);
	return true;
}

// doc/PROTOCOL.md §14.1: lists PATH's directory entries on VOLUME. A PATH that doesn't exist, or
// exists but isn't a directory, both NACK(FILE_NOT_FOUND) - StorageManager collapses that
// distinction (see its own doc), there's no dedicated status code for "not a directory".
void handleFileListRequest(const CommandContext& ctx) {
	uint8_t volumeValue;
	std::string path;
	if (!parseVolumeAndPath(ctx, volumeValue, path)) {
		return;
	}

	std::vector<StorageManager::Entry> entries;
	StorageManager::Result result = gStorageManager.list(volumeValue, path, entries);
	if (result == StorageManager::Result::kVolumeNotPresent) {
		ctx.nack(status::kVolumeNotPresent);
		return;
	}
	if (result != StorageManager::Result::kOk) {
		ctx.nack(status::kFileNotFound);
		return;
	}

	std::vector<uint8_t> response(2);
	writeU16LE(response.data(), static_cast<uint16_t>(entries.size()));
	for (const StorageManager::Entry& entry : entries) {
		response.push_back(static_cast<uint8_t>(entry.name.size()));
		response.insert(response.end(), entry.name.begin(), entry.name.end());
		response.push_back(entry.isDirectory ? entryType::kDirectory : entryType::kFile);
		uint8_t sizeBuf[4];
		writeU32LE(sizeBuf, entry.size);
		response.insert(response.end(), sizeBuf, sizeBuf + 4);
	}
	ctx.reply(cmd::kFileListResponse, response);
}

// doc/PROTOCOL.md §14.2: reads PATH's whole content back in one frame (no resumable/segmented
// transfer, per §14's own note - a large file's `PAYLOAD_LEN` u32 already has plenty of headroom).
void handleFileDownloadRequest(const CommandContext& ctx) {
	uint8_t volumeValue;
	std::string path;
	if (!parseVolumeAndPath(ctx, volumeValue, path)) {
		return;
	}

	std::vector<uint8_t> fileData;
	StorageManager::Result result = gStorageManager.download(volumeValue, path, fileData);
	if (result == StorageManager::Result::kVolumeNotPresent) {
		ctx.nack(status::kVolumeNotPresent);
		return;
	}
	if (result != StorageManager::Result::kOk) {
		ctx.nack(status::kFileNotFound);
		return;
	}

	std::vector<uint8_t> response(4 + fileData.size());
	writeU32LE(response.data(), static_cast<uint32_t>(fileData.size()));
	if (!fileData.empty()) {
		std::memcpy(response.data() + 4, fileData.data(), fileData.size());
	}
	ctx.reply(cmd::kFileData, response);
}

// doc/PROTOCOL.md §14.3: overwrites any existing file at PATH.
void handleFileUpload(const CommandContext& ctx) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < 2) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint8_t volumeValue = payload[0];
	uint8_t pathLen = payload[1];
	size_t headerSize = 2 + pathLen + 4;
	if (volumeValue > volume::kPsram || payload.size() < headerSize) {
		ctx.nack(status::kBadParameters);
		return;
	}
	std::string path(reinterpret_cast<const char*>(payload.data() + 2), pathLen);
	uint32_t fileLen = readU32LE(&payload[2 + pathLen]);
	if (payload.size() != headerSize + fileLen) {
		ctx.nack(status::kBadParameters);
		return;
	}

	StorageManager::Result result =
			gStorageManager.upload(volumeValue, path, payload.data() + headerSize, fileLen);
	switch (result) {
		case StorageManager::Result::kOk:
			ctx.ack();
			break;
		case StorageManager::Result::kVolumeNotPresent:
			ctx.nack(status::kVolumeNotPresent);
			break;
		case StorageManager::Result::kInsufficientStorage:
			ctx.nack(status::kInsufficientStorage);
			break;
		default:
			ctx.nack(status::kUnknownError);
			break;
	}
}

// doc/PROTOCOL.md §14.4.
void handleFileDelete(const CommandContext& ctx) {
	uint8_t volumeValue;
	std::string path;
	if (!parseVolumeAndPath(ctx, volumeValue, path)) {
		return;
	}

	StorageManager::Result result = gStorageManager.remove(volumeValue, path);
	switch (result) {
		case StorageManager::Result::kOk:
			ctx.ack();
			break;
		case StorageManager::Result::kVolumeNotPresent:
			ctx.nack(status::kVolumeNotPresent);
			break;
		case StorageManager::Result::kFileNotFound:
			ctx.nack(status::kFileNotFound);
			break;
		default:
			ctx.nack(status::kUnknownError);
			break;
	}
}

// doc/PROTOCOL.md §14.5: PRESENT reflects *live* SD-card presence (hot-pluggable, unlike the
// handshake's SD_CARD_SLOT feature bit, a fixed capability) - StorageManager::info() always
// succeeds for a valid VOLUME, reporting PRESENT=false rather than NACKing when no card is inserted.
void handleStorageInfoRequest(const CommandContext& ctx) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != 1 || payload[0] > volume::kPsram) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint8_t volumeValue = payload[0];

	StorageManager::Info info;
	gStorageManager.info(volumeValue, info);

	std::vector<uint8_t> response(10);
	response[0] = volumeValue;
	response[1] = info.present ? 0x01 : 0x00;
	writeU32LE(&response[2], info.totalBytes);
	writeU32LE(&response[6], info.freeBytes);
	ctx.reply(cmd::kStorageInfoResponse, response);
}

// Reads VOLUME(1)/PATH_LEN(1)/PATH(PATH_LEN) starting at `offset` within the payload -
// generalizes parseVolumeAndPath() above (which always starts at offset 0) so FILE_COPY (§14.6)
// can parse two such fields, one after another, without duplicating the same lines twice. Unlike
// parseVolumeAndPath(), does not require the field to reach the end of the payload - callers check
// that themselves once every field has been consumed. On success, advances `offset` past the field
// just read. Returns false (having already NACKed) if the payload is too short or VOLUME is out of
// range.
bool parseVolumeAndPathAt(const CommandContext& ctx, size_t& offset, uint8_t& volumeValue, std::string& path) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < offset + 2) {
		ctx.nack(status::kBadParameters);
		return false;
	}
	volumeValue = payload[offset];
	uint8_t pathLen = payload[offset + 1];
	if (volumeValue > volume::kPsram || payload.size() < offset + 2 + pathLen) {
		ctx.nack(status::kBadParameters);
		return false;
	}
	path.assign(reinterpret_cast<const char*>(payload.data() + offset + 2), pathLen);
	offset += 2 + pathLen;
	return true;
}

// Reads PATH_LEN(1)/PATH(PATH_LEN) (no VOLUME byte) starting at `offset` - FILE_RENAME (§14.7)'s
// DST_PATH shares its single VOLUME field with SRC_PATH, so only SRC_PATH goes through
// parseVolumeAndPathAt() above. Same offset-advancing/NACK-on-failure contract as that function.
bool parsePathAt(const CommandContext& ctx, size_t& offset, std::string& path) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < offset + 1) {
		ctx.nack(status::kBadParameters);
		return false;
	}
	uint8_t pathLen = payload[offset];
	if (payload.size() < offset + 1 + pathLen) {
		ctx.nack(status::kBadParameters);
		return false;
	}
	path.assign(reinterpret_cast<const char*>(payload.data() + offset + 1), pathLen);
	offset += 1 + pathLen;
	return true;
}

// doc/PROTOCOL.md §14.6: copies a file, optionally across volumes (e.g. SD -> PSRAM, to stage a
// macro for fast, session-only playback) - StorageManager::copyFile() has no single-call
// cross-volume filesystem primitive to reuse, so it's implemented as download()+upload() instead.
// Overwrites any existing file at the destination, matching FILE_UPLOAD's own semantics (§14.3).
void handleFileCopy(const CommandContext& ctx) {
	size_t offset = 0;
	uint8_t srcVolume, dstVolume;
	std::string srcPath, dstPath;
	if (!parseVolumeAndPathAt(ctx, offset, srcVolume, srcPath)) {
		return;
	}
	if (!parseVolumeAndPathAt(ctx, offset, dstVolume, dstPath)) {
		return;
	}
	if (offset != ctx.request.payload.size()) {
		ctx.nack(status::kBadParameters);
		return;
	}

	StorageManager::Result result = gStorageManager.copyFile(srcVolume, srcPath, dstVolume, dstPath);
	switch (result) {
		case StorageManager::Result::kOk:
			ctx.ack();
			break;
		case StorageManager::Result::kVolumeNotPresent:
			ctx.nack(status::kVolumeNotPresent);
			break;
		case StorageManager::Result::kFileNotFound:
			ctx.nack(status::kFileNotFound);
			break;
		case StorageManager::Result::kInsufficientStorage:
			ctx.nack(status::kInsufficientStorage);
			break;
		default:
			ctx.nack(status::kUnknownError);
			break;
	}
}

// doc/PROTOCOL.md §14.7: renames/moves PATH within a single VOLUME (unlike FILE_COPY, no
// cross-volume form - matches POSIX rename() semantics, and both StorageManager backends
// implement it as a cheap in-place move with no data copy). Overwrites any existing file at the
// destination, matching FILE_UPLOAD's own semantics (§14.3) - the intended use is swapping in a
// new version of a staged file (e.g. a macro) without a delete-then-reupload gap.
void handleFileRename(const CommandContext& ctx) {
	size_t offset = 0;
	uint8_t volumeValue;
	std::string srcPath, dstPath;
	if (!parseVolumeAndPathAt(ctx, offset, volumeValue, srcPath)) {
		return;
	}
	if (!parsePathAt(ctx, offset, dstPath)) {
		return;
	}
	if (offset != ctx.request.payload.size()) {
		ctx.nack(status::kBadParameters);
		return;
	}

	StorageManager::Result result = gStorageManager.renameFile(volumeValue, srcPath, dstPath);
	switch (result) {
		case StorageManager::Result::kOk:
			ctx.ack();
			break;
		case StorageManager::Result::kVolumeNotPresent:
			ctx.nack(status::kVolumeNotPresent);
			break;
		case StorageManager::Result::kFileNotFound:
			ctx.nack(status::kFileNotFound);
			break;
		default:
			ctx.nack(status::kUnknownError);
			break;
	}
}

// doc/PROTOCOL.md §0x0A00 RECORD_MACRO: empty payload. NACK(BUSY) if already recording, rather than
// silently discarding whatever was captured so far - restarting a recording is a deliberate act
// (SAVE_MACRO or a fresh RECORD_MACRO after that), never implicit.
void handleRecordMacro(const CommandContext& ctx) {
	if (gMacroRecorder.isRecording()) {
		ctx.nack(status::kBusy);
		return;
	}
	gMacroRecorder.start();
	ctx.ack();
}

constexpr size_t kSaveMacroHeaderSize = 2;	 // VOLUME + PATH_LEN

// doc/PROTOCOL.md §0x0A00 SAVE_MACRO: VOLUME(1)+PATH_LEN(1)+PATH - stops recording and persists the
// captured .macro file via StorageManager::upload(), reusing the exact same call every other
// FILE_UPLOAD-shaped command already goes through - VOLUME=PSRAM works here exactly like SD/
// INTERNAL, with no special-casing needed, since it's now a real StorageManager volume in its own
// right (§14). NACK(BAD_PARAMETERS) if not currently recording.
void handleSaveMacro(const CommandContext& ctx) {
	if (!gMacroRecorder.isRecording()) {
		ctx.nack(status::kBadParameters);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < kSaveMacroHeaderSize) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint8_t volumeValue = payload[0];
	uint8_t pathLen = payload[1];
	if (volumeValue > volume::kPsram || payload.size() != kSaveMacroHeaderSize + pathLen) {
		ctx.nack(status::kBadParameters);
		return;
	}
	std::string path(reinterpret_cast<const char*>(payload.data() + kSaveMacroHeaderSize), pathLen);

	std::vector<uint8_t> macroFile = gMacroRecorder.stop();
	StorageManager::Result result = gStorageManager.upload(volumeValue, path, macroFile.data(), macroFile.size());
	switch (result) {
		case StorageManager::Result::kOk:
			ctx.ack();
			break;
		case StorageManager::Result::kVolumeNotPresent:
			ctx.nack(status::kVolumeNotPresent);
			break;
		case StorageManager::Result::kInsufficientStorage:
			ctx.nack(status::kInsufficientStorage);
			break;
		default:
			ctx.nack(status::kUnknownError);
			break;
	}
}

// doc/PROTOCOL.md §0x0A00 PLAY_MACRO: VOLUME(1)+PATH_LEN(1)+PATH - downloads and parses the .macro
// file, then hands it to gMacroPlayer and returns immediately. The ACK here means "playback
// started", NOT "playback finished" - actually stepping through the macro's entries happens
// non-blocking, from loop() (stepMacroPlayback()), the same "timed/sequenced operation is a
// loop()-driven state machine, never a blocking call inside a command handler" rule already used
// elsewhere (plan.md Phase 3's GPIO_PLAY_PATTERN note) - a long-running macro (PAUSE entries can
// make one run for as long as it likes) must never stall the other transports from being polled.
void handlePlayMacro(const CommandContext& ctx) {
	if (gMacroPlayer.isPlaying()) {
		ctx.nack(status::kBusy);
		return;
	}
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < 2) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint8_t volumeValue = payload[0];
	uint8_t pathLen = payload[1];
	if (volumeValue > volume::kPsram || payload.size() != 2 + pathLen) {
		ctx.nack(status::kBadParameters);
		return;
	}
	std::string path(reinterpret_cast<const char*>(payload.data() + 2), pathLen);

	std::vector<uint8_t> fileData;
	StorageManager::Result result = gStorageManager.download(volumeValue, path, fileData);
	if (result == StorageManager::Result::kVolumeNotPresent) {
		ctx.nack(status::kVolumeNotPresent);
		return;
	}
	if (result != StorageManager::Result::kOk) {
		ctx.nack(status::kFileNotFound);
		return;
	}

	std::vector<MacroEntry> entries;
	if (!MacroPlayer::parse(fileData, entries)) {
		ctx.nack(status::kDecodeFail);
		return;
	}

	gMacroPlayer.start(std::move(entries));
	ctx.ack();
}

constexpr size_t kPausePayloadSize = 4;  // DURATION_MS u32 LE

// doc/PROTOCOL.md §0x0A00 PAUSE: always ACKs immediately, live or replayed - there's no "next
// command" to delay for an already PC-paced live transport, so a live PAUSE is accepted but has no
// observable effect beyond the ACK. Only when dispatched *during macro playback*
// (ACTIVE_TRANSPORT=MACRO, §5.2) does it actually delay anything: it tells gMacroPlayer to hold off
// on its next entry for DURATION_MS, which is what "wait before interpreting the next command"
// (the original request) means in a non-blocking player - PAUSE itself never blocks.
void handlePause(const CommandContext& ctx) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != kPausePayloadSize) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint32_t durationMs = readU32LE(payload.data());
	if (ctx.activeTransportValue == activeTransport::kMacro) {
		gMacroPlayer.pauseFor(durationMs);
	}
	ctx.ack();
}

// doc/PROTOCOL.md §15: GPIO_CONFIGURE/WRITE/READ/PLAY_PATTERN handlers, plus the GPIO_EVENT push
// path. All per-pin state and pattern playback lives in GpioController - these handlers just
// validate payload shape + PIN_ID availability (doc/PROTOCOL.md's "unexposed pin ->
// NACK(PIN_UNAVAILABLE)" rule, checked once here rather than duplicated inside GpioController) and
// translate the wire payload to/from its API.

void handleGpioConfigure(const CommandContext& ctx) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != 3) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint8_t pin = payload[0];
	uint8_t mode = payload[1];
	uint8_t flags = payload[2];
	if (!gGpioController.isPinAvailable(pin)) {
		ctx.nack(status::kPinUnavailable);
		return;
	}
	if (!gGpioController.configure(pin, mode, flags)) {
		ctx.nack(status::kBadParameters);
		return;
	}
	ctx.ack();
}

void handleGpioWrite(const CommandContext& ctx) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != 2) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint8_t pin = payload[0];
	uint8_t value = payload[1];
	if (!gGpioController.isPinAvailable(pin)) {
		ctx.nack(status::kPinUnavailable);
		return;
	}
	if (!gGpioController.write(pin, value)) {
		ctx.nack(status::kBadParameters);  // not configured OUTPUT
		return;
	}
	ctx.ack();
}

void handleGpioReadRequest(const CommandContext& ctx) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() != 1) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint8_t pin = payload[0];
	if (!gGpioController.isPinAvailable(pin)) {
		ctx.nack(status::kPinUnavailable);
		return;
	}
	uint8_t value = 0;
	uint8_t mode = 0;
	if (!gGpioController.read(pin, value, mode)) {
		ctx.nack(status::kBadParameters);  // never configured
		return;
	}
	ctx.reply(cmd::kGpioReadResponse, {pin, value, mode});
}

void handleGpioPlayPattern(const CommandContext& ctx) {
	const std::vector<uint8_t>& payload = ctx.request.payload;
	if (payload.size() < 4) {
		ctx.nack(status::kBadParameters);
		return;
	}
	uint8_t pin = payload[0];
	uint8_t flags = payload[1];
	uint8_t stepCount = payload[2];
	uint8_t repeatCount = payload[3];
	if (stepCount == 0 || payload.size() != 4u + static_cast<size_t>(stepCount) * 2u) {
		ctx.nack(status::kBadParameters);
		return;
	}
	if (!gGpioController.isPinAvailable(pin)) {
		ctx.nack(status::kPinUnavailable);
		return;
	}
	std::vector<uint16_t> stepDurationsMs(stepCount);
	for (size_t i = 0; i < stepCount; ++i) {
		stepDurationsMs[i] = readU16LE(payload.data() + 4 + i * 2);
	}
	if (!gGpioController.playPattern(pin, flags, std::move(stepDurationsMs), repeatCount)) {
		ctx.nack(status::kBadParameters);  // not configured OUTPUT
		return;
	}
	ctx.ack();
}

// GPIO_EVENT (doc/PROTOCOL.md §15.4) is the first device-initiated push this firmware sends
// unprompted by any request (BUTTON_EVENT, below, is the second) - there's no "reply to" transport
// for it, so it's broadcast on every live transport rather than routed to just one.
//
// gBleStream.ready() (design note 70) gates the BLE send specifically: StreamFrameTransport::send()
// treats every zero-byte stream_.write() as "buffer momentarily full, keep retrying" and only gives
// up after its full 2-second stall watchdog - correct for a real but congested link, but
// NimBLEStreamServer::write() returns 0 *immediately, every single call* whenever no BLE central is
// subscribed (confirmed by reading its source), so without this guard every single push - on every
// button press, GPIO change, anything - blocked the entire single-threaded loop() for a full ~2s
// whenever BLE simply had nobody connected, not just this feature's own timing. Serial/TCP don't
// need the same guard: TCP is already skipped via the `if (gTcpTransport)` null-check below when no
// client is connected, and HardwareSerial::write() buffers into its TX FIFO regardless of whether a
// host is actually reading, so it won't zero-return the same way.
uint8_t gGpioEventSeq = 0;

// doc/PROTOCOL.md §18.5 "event-triggered macros": on any BUTTON_EVENT or GPIO_EVENT, auto-plays a
// same-named macro from VOLUME=PSRAM if one exists - purely opt-in, a missing file is the expected
// common case, never logged/reported. Reuses handlePlayMacro's own download+parse sequence
// (below), ending in gMacroPlayer.enqueue() rather than start() so a trigger arriving while a macro
// is already playing queues behind it (appended to the tail of the same entries_ vector) instead of
// being dropped or clobbering the in-progress one - deliberately NOT the same contract as live
// PLAY_MACRO, which still NACK(BUSY)s a concurrent request (see handlePlayMacro) since a live PC
// caller depends on a definite answer, unlike this fire-and-forget internal path.
//
// If /on_any_event.macro exists on VOLUME=PSRAM, /trigger_id.txt is overwritten with the plain-text
// path of the specific macro that would apply, whether or not that specific file actually exists -
// a "universal variable" another macro can read via DRAW_TEXT's existing FLAGS.TEXT_IS_PATH
// (§12.6) to show/log which event fired. The specific macro plays if present; otherwise
// /on_any_event.macro plays as a fallback, if that exists; otherwise nothing happens.
void tryAutoPlayEventMacro(const std::string& specificPath) {
	static const char* kAnyEventPath = "/on_any_event.macro";
	static const char* kTriggerIdPath = "/trigger_id.txt";

	bool anyEventExists = gStorageManager.exists(volume::kPsram, kAnyEventPath);
	if (anyEventExists) {
		gStorageManager.upload(volume::kPsram, kTriggerIdPath,
				reinterpret_cast<const uint8_t*>(specificPath.data()), specificPath.size());
	}

	std::string pathToPlay;
	if (gStorageManager.exists(volume::kPsram, specificPath)) {
		pathToPlay = specificPath;
	} else if (anyEventExists) {
		pathToPlay = kAnyEventPath;
	} else {
		return;
	}

	std::vector<uint8_t> fileData;
	if (gStorageManager.download(volume::kPsram, pathToPlay, fileData) != StorageManager::Result::kOk) {
		return;
	}
	std::vector<MacroEntry> entries;
	if (!MacroPlayer::parse(fileData, entries)) {
		return;
	}
	gMacroPlayer.enqueue(std::move(entries));
}

std::string buttonEventMacroPath(uint8_t buttonId, uint8_t eventType) {
	const char* suffix = eventType == buttonEventType::kLongPress	 ? "_longpress"
			: eventType == buttonEventType::kShortPress			 ? "_shortpress"
			: eventType == buttonEventType::kRelease				 ? "_release"
																	 : "_press";
	return "/on_button_" + std::to_string(buttonId) + suffix + ".macro";
}

std::string gpioEventMacroPath(uint8_t pin, uint8_t value) {
	return "/on_gpio_" + std::to_string(pin) + (value ? "_rising" : "_falling") + ".macro";
}

void pushGpioEvent(uint8_t pin, uint8_t value) {
	Frame frame;
	frame.commandId = cmd::kGpioEvent;
	frame.seq = gGpioEventSeq++;
	uint32_t timestampMs = millis();
	frame.payload = {pin, value, static_cast<uint8_t>(timestampMs & 0xFF), static_cast<uint8_t>((timestampMs >> 8) & 0xFF),
			static_cast<uint8_t>((timestampMs >> 16) & 0xFF), static_cast<uint8_t>((timestampMs >> 24) & 0xFF)};
	gSerialTransport.send(frame);
	if (gBleStream.ready()) {
		gBleTransport.send(frame);
	}
	if (gTcpTransport) {
		gTcpTransport->send(frame);
	}
	tryAutoPlayEventMacro(gpioEventMacroPath(pin, value));
}

// doc/PROTOCOL.md §11 BUTTON_EVENT: same broadcast-to-every-live-transport approach as
// pushGpioEvent() above, including the gBleStream.ready() guard (see that function's comment for
// why it's needed) - reuses the pattern GPIO_EVENT established rather than inventing a second one.
uint8_t gButtonEventSeq = 0;

void pushButtonEvent(uint8_t buttonId, uint8_t eventType) {
	Frame frame;
	frame.commandId = cmd::kButtonEvent;
	frame.seq = gButtonEventSeq++;
	uint32_t timestampMs = millis();
	frame.payload = {buttonId, eventType, static_cast<uint8_t>(timestampMs & 0xFF),
			static_cast<uint8_t>((timestampMs >> 8) & 0xFF), static_cast<uint8_t>((timestampMs >> 16) & 0xFF),
			static_cast<uint8_t>((timestampMs >> 24) & 0xFF)};
	gSerialTransport.send(frame);
	if (gBleStream.ready()) {
		gBleTransport.send(frame);
	}
	if (gTcpTransport) {
		gTcpTransport->send(frame);
	}
	tryAutoPlayEventMacro(buttonEventMacroPath(buttonId, eventType));
}

// doc/PROTOCOL.md §0x0005 LOG_MESSAGE - bidirectional debugging/synchronization utility, requested
// directly: "we could make the log frame bidirectional, so it could be put inside a macro by the
// PC and when executed just repeated back - the PC than could wait for receiving that frame (e.g.
// for timing or for waiting for macro completion before sending more commands)". Echoes whatever
// payload it was sent right back out - same broadcast-to-every-live-transport approach as
// pushGpioEvent()/pushButtonEvent() above, not ctx.reply() (which would route to gMacroTransport's
// null sink for a macro-replayed entry, §18.3's own "no live caller listening" design - broadcast
// is what makes the echo actually reach the PC either way, live or replayed). Ordinary recordable
// content, exactly like PAUSE - isMacroRecordable() below has no special case excluding it - so a
// PC embeds one mid-recording as a marker, and replaying that macro later re-triggers this same
// echo at that exact point in playback.
uint8_t gLogMessageSeq = 0;

void pushLogMessage(const std::vector<uint8_t>& payload) {
	Frame frame;
	frame.commandId = cmd::kLogMessage;
	frame.seq = gLogMessageSeq++;
	frame.payload = payload;
	gSerialTransport.send(frame);
	if (gBleStream.ready()) {
		gBleTransport.send(frame);
	}
	if (gTcpTransport) {
		gTcpTransport->send(frame);
	}
}

void handleLogMessage(const CommandContext& ctx) {
	pushLogMessage(ctx.request.payload);
	ctx.ack();
}

// Commands excluded from recording (doc/PROTOCOL.md §0x0A00): the macro meta-commands themselves -
// recording RECORD_MACRO/SAVE_MACRO/PLAY_MACRO into a macro would be meaningless (nested/self-
// referential) at best. PAUSE is deliberately NOT excluded - it's ordinary recordable content,
// exactly like any drawing/image command, since it's what lets a recorded macro reproduce
// deliberate timing on playback.
bool isMacroRecordable(uint16_t commandId) {
	return commandId != cmd::kRecordMacro && commandId != cmd::kSaveMacro && commandId != cmd::kPlayMacro;
}

// Every live transport's frame handler calls this instead of gDispatcher.dispatch() directly, so a
// command gets captured (doc/PROTOCOL.md §0x0A00 RECORD_MACRO) before whatever its own handler does
// - recording happens regardless of whether the command goes on to ACK or NACK, matching "record
// what was received", not "record what succeeded". Never called for macro-replayed commands
// themselves (see stepMacroPlayback()) - those dispatch straight to gDispatcher, so a recording
// active while a macro plays can never end up recording that same macro into itself.
void dispatchAndMaybeRecord(StreamFrameTransport& transport, uint8_t activeTransportValue, const Frame& frame,
		uint8_t grantedLevel) {
	if (gMacroRecorder.isRecording() && isMacroRecordable(frame.commandId)) {
		gMacroRecorder.record(frame.commandId, frame.payload);
	}
	gDispatcher.dispatch(transport, activeTransportValue, frame, grantedLevel, gHasUsagePin, gHasAdminPin);
}

// Called every loop() iteration (doc/PROTOCOL.md §0x0A00): advances macro playback by at most one
// entry per call, so a long macro never blocks the transports from being polled in between. Each
// entry is dispatched through the exact same gDispatcher every live command uses, just with
// ACTIVE_TRANSPORT=MACRO and a null "transport" (gMacroTransport) standing in for a real caller, so
// its own drawing/etc. handlers behave identically whether the command arrived live or from a
// macro.
void stepMacroPlayback() {
	const MacroEntry* entry = gMacroPlayer.next();
	if (entry == nullptr) {
		return;
	}
	Frame frame;
	frame.commandId = entry->commandId;
	frame.payload = entry->payload;
	// doc/PROTOCOL.md §5.3: bypasses the access-control gate entirely - by the time a macro is
	// playing, starting it (PLAY_MACRO) and recording it (RECORD_MACRO/SAVE_MACRO) already
	// required admin once; replaying what an admin already approved doesn't need re-checking per
	// entry. Passing kAdmin with both tiers reported "not configured" makes the gate a guaranteed
	// no-op regardless of live configuration, without Dispatcher needing a separate bypass path.
	gDispatcher.dispatch(gMacroTransport, activeTransport::kMacro, frame, authLevel::kAdmin, /*usagePinConfigured=*/false,
			/*adminPinConfigured=*/false);
}

// doc/PROTOCOL.md §0x0A00: on cold start, checks for a fixed-name boot macro - SD first, then
// INTERNAL - and starts playing it automatically if one exists, exactly as if PLAY_MACRO had just
// been received. Neither volume having one is the common case and not an error - this only NACKs
// nothing, since there's no live caller to NACK to at boot.
constexpr const char* kBootMacroPath = "/boot.macro";

// Downloads and parses kBootMacroPath (SD then INTERNAL), appending its entries onto whatever's
// already in `outEntries` - called by tryPlayInitMacro() (below) to chain the regular boot macro
// right after its own entries, design note 85: "The sequence should be system init - init macro -
// boot macro". No standalone "just play the boot macro, nothing else" entry point remains -
// tryPlayInitMacro() runs unconditionally every cold boot now ("Not flash dependent, just three
// step boot init on cold boot"), so it's always this function that ends up reaching the boot macro.
void appendBootMacroEntries(std::vector<MacroEntry>& outEntries) {
	for (uint8_t volumeValue : {volume::kSd, volume::kInternal}) {
		std::vector<uint8_t> fileData;
		if (gStorageManager.download(volumeValue, kBootMacroPath, fileData) != StorageManager::Result::kOk) {
			continue;
		}
		std::vector<MacroEntry> bootEntries;
		if (MacroPlayer::parse(fileData, bootEntries)) {
			outEntries.insert(outEntries.end(), std::make_move_iterator(bootEntries.begin()),
					std::make_move_iterator(bootEntries.end()));
		}
		return;
	}
}

// doc/PROTOCOL.md §0x0A00, design note 85 - requested directly: "add additional 'init' optional
// macro, run first... Set it to sleep 1s, clear and degauss. That would be 'fixed' boot init
// procedure" - then corrected twice: not gated on detecting a fresh flash ("Not flash dependent,
// just three step boot init on cold boot"), and chained before the regular boot macro rather than
// replacing it ("The sequence should be system init - init macro - boot macro"). Runs every cold
// boot, unconditionally, right alongside (and always before) the regular boot macro. A real
// .macro file (same format, same SD-then-INTERNAL check as kBootMacroPath above), not a hardcoded
// routine, specifically so it stays user-editable/replaceable later - just auto-seeded with this
// fixed default content the first time neither volume already has one. Reuses MacroRecorder (a
// local instance, not gMacroRecorder - this has nothing to do with any live PC-initiated
// recording) rather than hand-rolling the entry byte layout a second time.
constexpr const char* kInitMacroPath = "/init.macro";

std::vector<uint8_t> buildDefaultInitMacro() {
	MacroRecorder recorder;
	recorder.start();
	recorder.record(cmd::kPause, { 0xE8, 0x03, 0x00, 0x00 });		// DURATION_MS=1000 (u32 LE)
	recorder.record(cmd::kFastClear, { color::kWhite, 0x00 });	// COLOR=WHITE, FLAGS=0 (deferred -
																	// the degauss cycle right after
																	// overrides the panel anyway)
	// CYCLES=1, not 0/default (kDefaultClearArtifactsCycles=3, i.e. 6 full updates - measured at
	// ~4s each in practice, design note 83's own boot-log timing - noticeably slower than the
	// 2-full-update self-test this whole sequence replaced). A fresh boot has no real accumulated
	// ghosting to clear yet, unlike CLEAR_ARTIFACTS's general-purpose default aimed at a
	// long-running device's own history, so one cycle (2 full updates) is plenty here - asked
	// directly: "the original firmware was us[]ing sort of fast flashing instead of slow
	// redraws... Now it redraws filled screen several times and the init is even slower than
	// before." GxEPD2_420_GDEY042T81's own useFastFullUpdate=true is already the default (checked
	// directly in its header, not assumed) - every full update here already uses the fastest
	// waveform this panel supports, so cycle count was the only real lever left.
	recorder.record(cmd::kClearArtifacts, { 0x01, 0x00 });  // CYCLES=1, FLAGS=0 (RESTORE_CONTENT=0, leave blank)
	return recorder.stop();
}

// LOG_MESSAGE (doc/PROTOCOL.md §0x0005) markers appended directly as macro entries (not sent
// live) - requested directly: "add init_done and boot_done messages after running the startup
// macros" - so a PC watching via CommandClient#waitForLogMessage knows exactly when each stage of
// the cold-boot sequence has actually finished, rather than only when PLAY_MACRO's own ACK fired
// ("started", not "finished" - see waitForLogMessage's own javadoc). Built directly as MacroEntry
// values (not via MacroRecorder) since there's no live command being recorded here.
MacroEntry makeLogMessageEntry(const std::string& marker) {
	MacroEntry entry;
	entry.commandId = cmd::kLogMessage;
	entry.payload.assign(marker.begin(), marker.end());
	return entry;
}

// "system init - init macro - boot macro": runs every cold boot (setup(), below), seeding
// kInitMacroPath with the fixed default above the first time neither volume has one, then chains
// the regular boot macro (appendBootMacroEntries()) right after its own entries into the same
// gMacroPlayer.start() call - MacroPlayer only ever holds one playback at a time, so this is a
// single combined sequence, not two separate ones racing to call start() twice. "init_done" is
// appended right after the init macro's own entries (before the boot macro is chained on), and
// "boot_done" right at the very end - both fire even if no boot macro exists on either volume,
// since the three-step sequence always runs.
void tryPlayInitMacro() {
	std::vector<uint8_t> fileData;
	bool found = false;
	for (uint8_t volumeValue : {volume::kSd, volume::kInternal}) {
		if (gStorageManager.download(volumeValue, kInitMacroPath, fileData) == StorageManager::Result::kOk) {
			found = true;
			break;
		}
	}
	if (!found) {
		fileData = buildDefaultInitMacro();
		gStorageManager.upload(volume::kInternal, kInitMacroPath, fileData.data(), fileData.size());
	}
	std::vector<MacroEntry> entries;
	if (!MacroPlayer::parse(fileData, entries)) {
		return;
	}
	entries.push_back(makeLogMessageEntry("init_done"));
	appendBootMacroEntries(entries);
	entries.push_back(makeLogMessageEntry("boot_done"));
	gMacroPlayer.start(std::move(entries));
}

// Accepts at most one active TCP client at a time, matching the protocol's single-logical-
// connection model (doc/PROTOCOL.md §3.2) - an additional connection attempt while one is active
// is rejected outright rather than queued.
void pollTcp() {
	startTcpServerIfNeeded();  // picks up a live SET_WIFI_CONFIG/SET_WIFI_ENABLED reconnect
	if (!gTcpServer) {
		return;
	}

	WiFiClient incoming = gTcpServer->available();
	if (incoming) {
		if (gTcpClient && gTcpClient.connected()) {
			incoming.stop();
		} else {
			gTcpClient = incoming;
			gTcpAuthLevel = authLevel::kNone;	// doc/PROTOCOL.md §5.3 - a new connection always relocks
			gTcpTransport.reset(new StreamFrameTransport(gTcpClient));
			gTcpTransport->setHandler([](const Frame& f) {
				dispatchAndMaybeRecord(*gTcpTransport, activeTransport::kTcp, f, gTcpAuthLevel);
			});
		}
	}

	if (gTcpTransport) {
		if (gTcpClient.connected()) {
			gTcpTransport->poll();
		} else {
			gTcpTransport.reset();
			gTcpAuthLevel = authLevel::kNone;
		}
	}
}

// doc/PROTOCOL.md §17.2 LAST_WAKE_REASON: called once, early in setup(), to classify how this boot
// happened. esp_sleep_get_wakeup_cause() alone only distinguishes *why* a genuine deep-sleep exit
// occurred (timer vs EXT1 button) - it doesn't tell you whether this was a deep-sleep exit at all,
// since it can return a stale/meaningless value on an unrelated reset. esp_reset_reason() answers
// that first: ESP_RST_DEEPSLEEP means "this boot really is a HARD_SLEEP wake", ESP_RST_POWERON
// means a genuine cold power-on ("never slept", per §17.2's own wording) - anything else (a manual
// RESET/EN-pin reset, a software restart, a watchdog, brownout, ...) is treated as
// HARD_SLEEP_EXTERNAL_RESET, the closest fit among the enum's three HARD_SLEEP reasons for "this
// wasn't a clean power-on and wasn't one of our own recognized deep-sleep wake sources either".
uint8_t computeBootWakeReason() {
	esp_reset_reason_t resetReason = esp_reset_reason();
	if (resetReason == ESP_RST_DEEPSLEEP) {
		switch (esp_sleep_get_wakeup_cause()) {
			case ESP_SLEEP_WAKEUP_TIMER:
				return wakeReason::kHardSleepTimer;
			case ESP_SLEEP_WAKEUP_EXT1:
				return wakeReason::kHardSleepButton;
			default:
				return wakeReason::kHardSleepExternalReset;
		}
	}
	if (resetReason == ESP_RST_POWERON) {
		return wakeReason::kPowerOn;
	}
	return wakeReason::kHardSleepExternalReset;
}

}  // namespace

void setup() {
	// Default HardwareSerial RX ring buffer (256 bytes) fills in ~22ms at 115200 baud - too small
	// for a large single-frame payload (OTA_INSTALL, doc/PROTOCOL.md §16, can be ~1MB): if loop()
	// is ever kept away from StreamFrameTransport::poll() for longer than that (WiFi/BLE
	// housekeeping, macro playback, etc.), the UART driver silently drops the overflow bytes -
	// found on real hardware as a payload that reliably stalls partway through a multi-second
	// transfer with zero response (the frame parser waits forever for bytes that already got
	// dropped; no crash, no NACK, nothing to see on the wire). Must be called before begin().
	Serial.setRxBufferSize(16384);
	Serial.begin(115200);
	delay(500);
	gLastWakeReason = computeBootWakeReason();
	Serial.println("CrowPanel firmware bring-up stub");
	// Adafruit_GFX::_cp437 defaults to false, which makes the classic drawChar() path apply a
	// legacy off-by-one shift to any character code >= 176 (an old Arduino IDE charset-encoding
	// quirk, kept for backward compat with sketches written against it) - without disabling that,
	// every CP437 box-drawing/block glyph (EmbeddedFont.h's kBoxDrawingGlyphs, codes 176-223) would
	// render as its neighbor instead of itself.
	gWorkingBufferGfx.cp437(true);
	gPrefs.begin(kPrefsNamespace, false);  // before anything that might call deviceName()
	gHasUsagePin = gPrefs.getBool(kPrefKeyUsageHasPin, false);
	gUsagePin = gPrefs.getString(kPrefKeyUsagePin, "").c_str();
	gHasAdminPin = gPrefs.getBool(kPrefKeyAdminHasPin, false);
	gAdminPin = gPrefs.getString(kPrefKeyAdminPin, "").c_str();
	gSerialAuthLevel = authLevel::kNone;  // doc/PROTOCOL.md §5.3 - every boot starts fully relocked
	gStorageManager.begin();
	loadSdConfigLayer();  // before anything that might call deviceName() and reads the SD card
	selfTest();
	displaySelfTest();
	connectWifiAndStartTcpServer();
	setupBle();
	Serial.println("--- entering framed-protocol mode on this UART: no further plain-text output ---");

	// doc/PROTOCOL.md §5.3: every handler below now takes a required access level - kNone (the
	// default, omitted) for HANDSHAKE_REQUEST only, kUsage for normal display-driving/read
	// operations, kAdmin for configuration/security/OTA/macro-recording/storage-writes. See §5.3's
	// own table for the reasoning behind each one; SET_POWER_MODE is kUsage at the dispatch level
	// but layers its own extra HARD_SLEEP/transport check inside the handler itself.
	gDispatcher.registerHandler(cmd::kHandshakeRequest, handleHandshakeRequest);
	gDispatcher.registerHandler(cmd::kLogMessage, handleLogMessage, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kSetDeviceName, handleSetDeviceName, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kSetWifiConfig, handleSetWifiConfig, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kWifiStatusRequest, handleWifiStatusRequest, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kSetWifiEnabled, handleSetWifiEnabled, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kSetBleEnabled, handleSetBleEnabled, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kBleStatusRequest, handleBleStatusRequest, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kSetBlePin, handleSetBlePin, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kSetUsagePin, handleSetUsagePin, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kSetAdminPin, handleSetAdminPin, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kSetPowerMode, handleSetPowerMode, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kOtaInstall, handleOtaInstall, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kOtaApply, handleOtaApply, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kOtaStatusRequest, handleOtaStatusRequest, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kOtaConfirm, handleOtaConfirm, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kOtaRollback, handleOtaRollback, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kPowerStatusRequest, handlePowerStatusRequest, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kConfigBackupRequest, handleConfigBackupRequest, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kConfigRestore, handleConfigRestore, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kFullImageTransfer, handleFullImageTransfer, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kPartialImageTransfer, handlePartialImageTransfer, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kReadScreen, handleReadScreen, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kClearArtifacts, handleClearArtifacts, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kRefresh, handleRefresh, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kDrawLine, handleDrawLine, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kDrawRect, handleDrawRect, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kDrawCircle, handleDrawCircle, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kClearRegion, handleClearRegion, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kDrawText, handleDrawText, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kDrawImage, handleDrawImage, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kDrawImageData, handleDrawImageData, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kDrawImageRow, handleDrawImageRow, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kFillImage, handleFillImage, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kFastClear, handleFastClear, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kSetCustomFontFolder, handleSetCustomFontFolder, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kShiftRegion, handleShiftRegion, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kSetClipRegion, handleSetClipRegion, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kCopyRegion, handleCopyRegion, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kSetDrawOffset, handleSetDrawOffset, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kSetOrientation, handleSetOrientation, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kFileListRequest, handleFileListRequest, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kFileDownloadRequest, handleFileDownloadRequest, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kFileUpload, handleFileUpload, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kFileDelete, handleFileDelete, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kStorageInfoRequest, handleStorageInfoRequest, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kFileCopy, handleFileCopy, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kFileRename, handleFileRename, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kRecordMacro, handleRecordMacro, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kSaveMacro, handleSaveMacro, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kPlayMacro, handlePlayMacro, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kPause, handlePause, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kGpioConfigure, handleGpioConfigure, authLevel::kAdmin);
	gDispatcher.registerHandler(cmd::kGpioWrite, handleGpioWrite, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kGpioReadRequest, handleGpioReadRequest, authLevel::kUsage);
	gDispatcher.registerHandler(cmd::kGpioPlayPattern, handleGpioPlayPattern, authLevel::kUsage);
	gGpioController.setChangeEventCallback(pushGpioEvent);
	gButtonController.begin();
	gButtonController.setEventCallback(pushButtonEvent);

	gSerialTransport.setHandler([](const Frame& f) {
		dispatchAndMaybeRecord(gSerialTransport, activeTransport::kSerial, f, gSerialAuthLevel);
	});
	gBleTransport.setHandler(
			[](const Frame& f) { dispatchAndMaybeRecord(gBleTransport, activeTransport::kBle, f, gBleAuthLevel); });

	tryPlayInitMacro();  // chains the regular boot macro right after its own entries - see its own doc
}

void loop() {
	gSerialTransport.poll();
	pollTcp();
	gBleTransport.poll();
	stepMacroPlayback();
	gGpioController.update();
	gButtonController.update();
	gStorageManager.updateIdlePowerDown();
	checkOtaAutoRollback();
}
