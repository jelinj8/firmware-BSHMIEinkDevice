// CrowPanel wire protocol - shared constants and the Logical Frame envelope.
// See doc/PROTOCOL.md at the repository root for the full specification.
//
// This header is deliberately Arduino-framework-free (plain C++17) so it can be unit tested on
// the "native" PlatformIO platform without an ESP32 toolchain.
#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

namespace crowpanel {

// ---- Common Logical Frame envelope (doc/PROTOCOL.md §2) --------------------------------------

constexpr uint8_t kFrameMagic = 0xA5;
constexpr uint8_t kProtocolVersion = 0x01;
constexpr size_t kFrameHeaderSize = 9;	 // MAGIC..PAYLOAD_LEN
constexpr size_t kFrameCrcSize = 2;

enum class FrameError {
	kNone,
	kTooShort,
	kBadMagic,
	kLengthMismatch,
	kCrcMismatch,
};

struct Frame {
	uint8_t version = kProtocolVersion;
	uint16_t commandId = 0;
	uint8_t seq = 0;
	std::vector<uint8_t> payload;

	// Serializes this frame to wire bytes, appending a freshly computed CRC16 trailer.
	std::vector<uint8_t> encode() const;

	// Parses exactly one frame out of `bytes` (no trailing data allowed). On success returns
	// FrameError::kNone and fills `out`; on failure returns the specific error and leaves `out`
	// untouched.
	static FrameError decode(const uint8_t* bytes, size_t length, Frame& out);
};

// ---- Command IDs (doc/PROTOCOL.md §4) ---------------------------------------------------------

namespace cmd {
constexpr uint16_t kHandshakeRequest = 0x0001;
constexpr uint16_t kHandshakeResponse = 0x0002;
constexpr uint16_t kAck = 0x0003;
constexpr uint16_t kNack = 0x0004;
constexpr uint16_t kLogMessage = 0x0005;

constexpr uint16_t kFullImageTransfer = 0x0100;
constexpr uint16_t kPartialImageTransfer = 0x0101;
constexpr uint16_t kReadScreen = 0x0102;
constexpr uint16_t kScreenData = 0x0103;
constexpr uint16_t kClearArtifacts = 0x0104;

constexpr uint16_t kButtonEvent = 0x0200;

constexpr uint16_t kDrawLine = 0x0300;
constexpr uint16_t kDrawRect = 0x0301;
constexpr uint16_t kDrawCircle = 0x0302;
constexpr uint16_t kClearRegion = 0x0303;
constexpr uint16_t kDrawText = 0x0304;
constexpr uint16_t kDrawImage = 0x0305;
constexpr uint16_t kRefresh = 0x0306;
constexpr uint16_t kShiftRegion = 0x0307;
constexpr uint16_t kSetClipRegion = 0x0308;
constexpr uint16_t kCopyRegion = 0x0309;
constexpr uint16_t kSetDrawOffset = 0x030A;
constexpr uint16_t kSetOrientation = 0x030B;
constexpr uint16_t kDrawImageRow = 0x030C;
constexpr uint16_t kFillImage = 0x030D;
constexpr uint16_t kFastClear = 0x030E;
constexpr uint16_t kSetCustomFontFolder = 0x030F;
constexpr uint16_t kDrawImageData = 0x0310;

constexpr uint16_t kConfigBackupRequest = 0x0400;
constexpr uint16_t kConfigBackupData = 0x0401;
constexpr uint16_t kConfigRestore = 0x0402;
constexpr uint16_t kSetWifiConfig = 0x0403;
constexpr uint16_t kWifiStatusRequest = 0x0404;
constexpr uint16_t kWifiStatusResponse = 0x0405;
constexpr uint16_t kSetWifiEnabled = 0x0406;
constexpr uint16_t kSetBleEnabled = 0x0407;
constexpr uint16_t kSetBlePin = 0x0408;
constexpr uint16_t kBleStatusRequest = 0x0409;
constexpr uint16_t kBleStatusResponse = 0x040A;
constexpr uint16_t kSetDeviceName = 0x040B;
constexpr uint16_t kSetUsagePin = 0x040C;
constexpr uint16_t kSetAdminPin = 0x040D;

constexpr uint16_t kFileListRequest = 0x0600;
constexpr uint16_t kFileListResponse = 0x0601;
constexpr uint16_t kFileDownloadRequest = 0x0602;
constexpr uint16_t kFileData = 0x0603;
constexpr uint16_t kFileUpload = 0x0604;
constexpr uint16_t kFileDelete = 0x0605;
constexpr uint16_t kStorageInfoRequest = 0x0606;
constexpr uint16_t kStorageInfoResponse = 0x0607;
constexpr uint16_t kFileCopy = 0x0608;
constexpr uint16_t kFileRename = 0x0609;

constexpr uint16_t kGpioConfigure = 0x0700;
constexpr uint16_t kGpioWrite = 0x0701;
constexpr uint16_t kGpioReadRequest = 0x0702;
constexpr uint16_t kGpioReadResponse = 0x0703;
constexpr uint16_t kGpioEvent = 0x0704;
constexpr uint16_t kGpioPlayPattern = 0x0705;

constexpr uint16_t kOtaInstall = 0x0800;
constexpr uint16_t kOtaApply = 0x0801;
constexpr uint16_t kOtaStatusRequest = 0x0802;
constexpr uint16_t kOtaStatusResponse = 0x0803;
constexpr uint16_t kOtaConfirm = 0x0804;
constexpr uint16_t kOtaRollback = 0x0805;

constexpr uint16_t kSetPowerMode = 0x0900;
constexpr uint16_t kPowerStatusRequest = 0x0901;
constexpr uint16_t kPowerStatusResponse = 0x0902;

constexpr uint16_t kRecordMacro = 0x0A00;
constexpr uint16_t kSaveMacro = 0x0A01;
constexpr uint16_t kPlayMacro = 0x0A02;
constexpr uint16_t kPause = 0x0A03;
}  // namespace cmd

// ---- ACK/NACK status codes (doc/PROTOCOL.md §10) ----------------------------------------------

namespace status {
constexpr uint8_t kOk = 0x00;
constexpr uint8_t kCrcFail = 0x01;
constexpr uint8_t kBusy = 0x02;
constexpr uint8_t kUnsupportedCommand = 0x03;
constexpr uint8_t kBadParameters = 0x04;
constexpr uint8_t kDecodeFail = 0x05;
constexpr uint8_t kVersionMismatch = 0x06;
constexpr uint8_t kChunkSequenceError = 0x07;
constexpr uint8_t kFileNotFound = 0x08;
constexpr uint8_t kInsufficientStorage = 0x09;
constexpr uint8_t kVolumeNotPresent = 0x0A;
constexpr uint8_t kPinUnavailable = 0x0B;
constexpr uint8_t kOtaHashMismatch = 0x0C;
constexpr uint8_t kOtaNotStaged = 0x0D;
constexpr uint8_t kNotAuthorized = 0x0E;
constexpr uint8_t kUnknownError = 0xFF;
}  // namespace status

// ---- Access control levels (doc/PROTOCOL.md §5.3) ---------------------------------------------
//
// Reused directly as HANDSHAKE_REQUEST's own PIN_TYPE field values - "which tier does this PIN
// authenticate for" and "what tier has this connection been granted" are the same concept.

namespace authLevel {
constexpr uint8_t kNone = 0x00;
constexpr uint8_t kUsage = 0x01;
constexpr uint8_t kAdmin = 0x02;
}  // namespace authLevel

// ---- Shared write-command FLAGS byte (doc/PROTOCOL.md §2.1) -------------------------------

namespace writeFlags {
constexpr uint8_t kRefreshNow = 1u << 0;
constexpr uint8_t kRefreshFull = 1u << 1;
}  // namespace writeFlags

// ---- Local drawing primitives shared enums (doc/PROTOCOL.md §12.1) -----------------------------

namespace drawMode {
constexpr uint8_t kReplace = 0x00;
constexpr uint8_t kOr = 0x01;
constexpr uint8_t kXor = 0x02;
constexpr uint8_t kAnd = 0x03;
}  // namespace drawMode

namespace color {
constexpr uint8_t kWhite = 0x00;
constexpr uint8_t kBlack = 0x01;
}  // namespace color

// ---- READ_SCREEN SOURCE/MODE bytes (doc/PROTOCOL.md §8) ---------------------------------------

namespace readScreenSource {
constexpr uint8_t kPanel = 0x00;
constexpr uint8_t kWorkingBuffer = 0x01;
}  // namespace readScreenSource

namespace readScreenMode {
constexpr uint8_t kFull = 0x00;
constexpr uint8_t kRegion = 0x01;
}  // namespace readScreenMode

// ---- CLEAR_ARTIFACTS FLAGS byte (doc/PROTOCOL.md §9) -------------------------------------------

namespace clearArtifactsFlags {
constexpr uint8_t kRestoreContent = 1u << 0;
}  // namespace clearArtifactsFlags

// ---- Storage VOLUME byte (doc/PROTOCOL.md §14, also used by DRAW_IMAGE §12.7) ------------------

namespace volume {
constexpr uint8_t kSd = 0x00;
constexpr uint8_t kInternal = 0x01;
constexpr uint8_t kPsram = 0x02;  // session-only, lost on reboot - doc/PROTOCOL.md §14's own note
}  // namespace volume

// ---- FILE_LIST_RESPONSE ENTRY_TYPE byte (doc/PROTOCOL.md §14.1) -------------------------------

namespace entryType {
constexpr uint8_t kFile = 0x00;
constexpr uint8_t kDirectory = 0x01;
}  // namespace entryType

// ---- .epi image file format FLAGS byte (doc/PROTOCOL.md §12.7) --------------------------------

namespace epiFlags {
constexpr uint8_t kHasMask = 1u << 0;
}  // namespace epiFlags

// ---- DRAW_IMAGE_ROW ALIGN byte (doc/PROTOCOL.md §12.x) -----------------------------------------
// BLOCK auto-distributes leftover space (WIDTH minus the sum of every image's own width) evenly
// across the gaps between images, ignoring SPACING; every other value uses SPACING as a fixed gap.

namespace imageRowAlign {
constexpr uint8_t kLeft = 0x00;
constexpr uint8_t kCenter = 0x01;
constexpr uint8_t kRight = 0x02;
constexpr uint8_t kBlock = 0x03;
}  // namespace imageRowAlign

// ---- FILL_IMAGE TILE_MODE byte (doc/PROTOCOL.md §12.x) -----------------------------------------
// The axis NOT named keeps the image's own natural size (a single row/column) rather than tiling.

namespace fillTileMode {
constexpr uint8_t kHorizontal = 0x00;
constexpr uint8_t kVertical = 0x01;
constexpr uint8_t kBoth = 0x02;
}  // namespace fillTileMode

// ---- DRAW_TEXT BACKGROUND byte (doc/PROTOCOL.md §12.6) -----------------------------------------
// OPAQUE fills every non-ink glyph pixel with the opposite of COLOR - "inverted" text is just
// COLOR=WHITE with an OPAQUE background, no separate concept needed.

namespace textBackground {
constexpr uint8_t kTransparent = 0x00;
constexpr uint8_t kOpaque = 0x01;
}  // namespace textBackground

// ---- DRAW_TEXT-specific FLAGS bits (doc/PROTOCOL.md §12.6) - bits0-1 remain the shared §2.1
// REFRESH_NOW/REFRESH_FULL meaning; DRAW_TEXT extends the reserved bits2-7 with its own meaning.
// TEXT_IS_PATH: TEXT is a UTF-8 path rather than literal text - the referenced file's own content,
// read fresh every time this command runs, becomes the text to draw. Lets a macro (§18) stay
// parametrized: write the desired text to a file, then a recorded/replayed DRAW_TEXT with this bit
// set picks up whatever that file currently holds. No VOLUME field exists on DRAW_TEXT's own wire
// payload (unlike DRAW_IMAGE/§12.7), so the volume is instead selected by an optional 2-byte prefix
// on TEXT itself - "R:" PSRAM, "S:" SD, "F:" INTERNAL (flash) - stripped before the rest is used as
// the actual path; no prefix (or any other/unrecognized one) defaults to PSRAM with TEXT used
// as-is, unstripped.

namespace drawTextFlags {
constexpr uint8_t kTextIsPath = 1u << 2;
}  // namespace drawTextFlags

// ---- DRAW_IMAGE/DRAW_IMAGE_DATA-specific FLAGS bit (doc/PROTOCOL.md §12.7) - bits0-1 remain the
// shared §2.1 REFRESH_NOW/REFRESH_FULL meaning. IGNORE_MASK: draw every pixel opaque, ignoring the
// .epi data's own HAS_MASK/MASK_DATA if present - the caller's override of the default
// mask-respecting behavior, not a property of the .epi file itself.

namespace drawImageFlags {
constexpr uint8_t kIgnoreMask = 1u << 2;
}  // namespace drawImageFlags

// ---- SHIFT_REGION direction byte (doc/PROTOCOL.md §12.9) ---------------------------------------

namespace shiftDirection {
constexpr uint8_t kLeft = 0x00;
constexpr uint8_t kRight = 0x01;
constexpr uint8_t kUp = 0x02;
constexpr uint8_t kDown = 0x03;
}  // namespace shiftDirection

// ---- SET_ORIENTATION ROTATION/FLAGS bytes (doc/PROTOCOL.md §12.12) ----------------------------
// ROTATION describes how the logical canvas is rotated (clockwise) relative to the physical
// panel; 90/270 swap the logical canvas's width/height (portrait <-> landscape). MIRROR_H/V are
// applied in logical space, before rotation.

namespace orientation {
constexpr uint8_t kRotate0 = 0x00;
constexpr uint8_t kRotate90 = 0x01;
constexpr uint8_t kRotate180 = 0x02;
constexpr uint8_t kRotate270 = 0x03;
}  // namespace orientation

namespace orientationFlags {
constexpr uint8_t kMirrorH = 1u << 0;
constexpr uint8_t kMirrorV = 1u << 1;
}  // namespace orientationFlags

// ---- Shared FLAGS.PERSIST bit (doc/PROTOCOL.md §13.2/§13.3) - every SET_* config command in §13
// uses the same "bit0 PERSIST: save to NVS, becomes the power-on default; 0 = this boot only".

namespace configFlags {
constexpr uint8_t kPersist = 1u << 0;
}  // namespace configFlags

// ---- BUTTON_EVENT BUTTON_ID (doc/PROTOCOL.md §11) ------------------------------------------------

namespace buttonId {
constexpr uint8_t kUnknown = 0x00;
constexpr uint8_t kDialSwitch = 0x01;	// press/confirm
constexpr uint8_t kMenu = 0x02;
constexpr uint8_t kBack = 0x03;
constexpr uint8_t kBoot = 0x04;	 // GPIO0 - safe to read once running, see design note 69
constexpr uint8_t kReset = 0x05;	 // not emitted by this firmware - EN pin, resets the MCU, see §11
constexpr uint8_t kDialUp = 0x06;
constexpr uint8_t kDialDown = 0x07;
}  // namespace buttonId

// ---- BUTTON_EVENT EVENT_TYPE (doc/PROTOCOL.md §11) -----------------------------------------------

namespace buttonEventType {
constexpr uint8_t kPress = 0x00;
constexpr uint8_t kRelease = 0x01;
constexpr uint8_t kLongPress = 0x02;
// Fires immediately after kRelease whenever that press-release cycle never crossed the
// kLongPress threshold - the common "quick tap" case, doc/PROTOCOL.md §11.
constexpr uint8_t kShortPress = 0x03;
}  // namespace buttonEventType

// ---- OTA_INSTALL HASH_ALGO / FLAGS (doc/PROTOCOL.md §16.1) ---------------------------------------

namespace otaHashAlgo {
constexpr uint8_t kNone = 0x00;
constexpr uint8_t kSha256 = 0x01;
constexpr uint8_t kMd5 = 0x02;
}  // namespace otaHashAlgo

namespace otaInstallFlags {
constexpr uint8_t kApplyNow = 1u << 0;
}  // namespace otaInstallFlags

// ---- GPIO_CONFIGURE MODE (doc/PROTOCOL.md §15.1) ------------------------------------------------

namespace gpioMode {
constexpr uint8_t kInput = 0x00;
constexpr uint8_t kInputPullup = 0x01;
constexpr uint8_t kInputPulldown = 0x02;
constexpr uint8_t kOutput = 0x03;
constexpr uint8_t kPwmOutput = 0x04;	  // reserved, not implemented
constexpr uint8_t kAnalogInput = 0x05;	  // reserved, not implemented
}  // namespace gpioMode

// ---- GPIO_CONFIGURE FLAGS byte (doc/PROTOCOL.md §15.1) ------------------------------------------

namespace gpioConfigureFlags {
constexpr uint8_t kEnableChangeEvents = 1u << 0;
}  // namespace gpioConfigureFlags

// ---- GPIO_PLAY_PATTERN FLAGS byte (doc/PROTOCOL.md §15.5) --------------------------------------

namespace gpioPatternFlags {
constexpr uint8_t kInitialLevelHigh = 1u << 0;
constexpr uint8_t kRepeatForever = 1u << 1;
}  // namespace gpioPatternFlags

// ---- BLE chunk sub-header (doc/PROTOCOL.md §3.1) -----------------------------------------------

constexpr size_t kChunkHeaderSize = 2;	 // CHUNK_INDEX + CHUNK_FLAGS
namespace chunkFlags {
constexpr uint8_t kMoreFollows = 1u << 0;
}  // namespace chunkFlags

// ---- Power management (doc/PROTOCOL.md §17) ----------------------------------------------------

namespace powerMode {
constexpr uint8_t kActive = 0x00;
constexpr uint8_t kLowPower = 0x01;
constexpr uint8_t kHardSleep = 0x02;
}  // namespace powerMode

namespace setPowerModeFlags {
constexpr uint8_t kKeepBleConnectable = 1u << 0;  // LOW_POWER only
}  // namespace setPowerModeFlags

// Shared by POWER_STATUS_RESPONSE.LAST_WAKE_REASON and the handshake's LAST_WAKE_REASON TLV.
namespace wakeReason {
constexpr uint8_t kPowerOn = 0x00;
constexpr uint8_t kHardSleepTimer = 0x01;
constexpr uint8_t kHardSleepButton = 0x02;
constexpr uint8_t kHardSleepExternalReset = 0x03;
constexpr uint8_t kLowPowerTimer = 0x04;
constexpr uint8_t kLowPowerSerialActivity = 0x05;
constexpr uint8_t kLowPowerBleActivity = 0x06;
}  // namespace wakeReason

// ---- BLE GATT layout (doc/PROTOCOL.md §19) - plain strings, no NimBLE dependency here so this
// stays usable from the Arduino-framework-free native build too. ---------------------------------

namespace ble {
constexpr char kServiceUuid[] = "748ac078-f79c-4b13-9ead-1a05597acb3f";
constexpr char kCharacteristicUuid[] = "01940d49-522c-4b4e-b0c9-3be78c56c960";
}  // namespace ble

// ACTIVE_TRANSPORT values, doc/PROTOCOL.md §5.2 (also used by the handshake TLV builder).
namespace activeTransport {
constexpr uint8_t kTcp = 0x00;
constexpr uint8_t kBle = 0x01;
constexpr uint8_t kSerial = 0x02;
constexpr uint8_t kMacro = 0x03;  // dispatched from a recorded macro (§0A00), not a live transport
}  // namespace activeTransport

}  // namespace crowpanel
