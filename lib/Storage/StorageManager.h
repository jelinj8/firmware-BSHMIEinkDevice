// doc/PROTOCOL.md §14: generalizes file access across SD (fs::SDFS), INTERNAL (fs::LittleFSFS), and
// PSRAM (a flat in-memory volume this class owns directly, no real filesystem underneath). SD and
// INTERNAL both already implement Arduino's common fs::FS interface, so their operations are
// written once, parameterized by volume, rather than duplicated per filesystem; PSRAM instead
// branches to its own std::map-backed implementation at the top of each method, since there's no
// fs::FS-conformant in-memory filesystem readily available and implementing Arduino's whole FS/File
// virtual interface just for a session-only store isn't worth it. INTERNAL is treated as supporting
// real subdirectories (LittleFS actually does, unlike the doc's "may be a flat namespace" allowance
// for a simpler firmware) - kept uniform with SD rather than special-cased for no real benefit;
// PSRAM, like the doc's flat-namespace allowance, has no subdirectories at all.
//
// SD is treated as hot-pluggable, since the ESP32 SD library has no built-in card-detect callback:
// mountSd() lazily remounts (SD.end() then SD.begin()) only once every kRemountDebounceMs (5s) of
// continuous access, or immediately after the card was last powered down (powerDownSd(), below) -
// rather than on every single operation, which measured as real, blocking-enough latency once a
// PC-side recursive folder-sync feature started driving bursts of ~200 individual
// FILE_UPLOAD/FILE_LIST commands back-to-back (design note 95), nor indefinitely once-per-session,
// which would leave a card hot-swapped mid-burst undetected for arbitrarily long (design note 97).
// The tradeoff: a card hot-swapped between two accesses less than kRemountDebounceMs apart goes
// undetected for up to that long, rather than on the very next command.
#pragma once

#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <SD.h>
#include <SPI.h>
#include <esp32-hal-psram.h>

#include "BoardConfig.h"
#include "Protocol.h"

#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace crowpanel {

// Owns one PSRAM-allocated (ps_malloc(), not the default allocator - doc/PROTOCOL.md §14 VOLUME=
// PSRAM is explicitly meant to live in PSRAM specifically, not just "whatever heap the default
// allocator happens to pick") byte buffer. Move-only, since it owns a raw allocation.
class PsramBuffer {
public:
	PsramBuffer() = default;

	PsramBuffer(const uint8_t* data, size_t size) : size_(size) {
		if (size_ > 0) {
			data_ = static_cast<uint8_t*>(ps_malloc(size_));
			std::memcpy(data_, data, size_);
		}
	}

	PsramBuffer(PsramBuffer&& other) noexcept : data_(other.data_), size_(other.size_) {
		other.data_ = nullptr;
		other.size_ = 0;
	}

	PsramBuffer& operator=(PsramBuffer&& other) noexcept {
		if (this != &other) {
			free(data_);
			data_ = other.data_;
			size_ = other.size_;
			other.data_ = nullptr;
			other.size_ = 0;
		}
		return *this;
	}

	PsramBuffer(const PsramBuffer&) = delete;
	PsramBuffer& operator=(const PsramBuffer&) = delete;

	~PsramBuffer() { free(data_); }

	const uint8_t* data() const { return data_; }
	size_t size() const { return size_; }

private:
	uint8_t* data_ = nullptr;
	size_t size_ = 0;
};

class StorageManager {
public:
	enum class Result {
		kOk,
		kVolumeNotPresent,
		kFileNotFound,
		kInsufficientStorage,
		kIoError,
	};

	struct Entry {
		std::string name;	// basename only, no path separators (doc/PROTOCOL.md §14.1)
		bool isDirectory = false;
		uint32_t size = 0;
	};

	struct Info {
		bool present = false;
		uint32_t totalBytes = 0;
		uint32_t freeBytes = 0;
	};

	// Configures the SD card's own independent SPI bus (board::kPinSdSck/Mosi/Miso, separate from
	// the display's) and powers its level-shifter (board::kPinSdPowerCtl) - safe to call even if
	// kPinSdCs<0 (no card slot on this board), in which case every VOLUME=SD operation below just
	// reports kVolumeNotPresent. Does NOT mount the card itself - see mountSd()'s own doc for why.
	void begin() {
		if (board::kPinSdCs < 0) {
			return;
		}
		if (board::kPinSdPowerCtl >= 0) {
			pinMode(board::kPinSdPowerCtl, OUTPUT);
			digitalWrite(board::kPinSdPowerCtl, HIGH);
			sdPowered_ = true;
		}
		sdSpi_.begin(board::kPinSdSck, board::kPinSdMiso, board::kPinSdMosi, board::kPinSdCs);
		lastAccessMs_ = millis();
	}

	// Cuts the SD level-shifter's power (board::kPinSdPowerCtl) after a clean SD.end() - used both
	// before entering LOW_POWER/HARD_SLEEP (doc/PROTOCOL.md §17, requested directly: "for full
	// powerdown we can turn them off completely, for light sleep it should be recoverable") and by
	// updateIdlePowerDown() below (also requested directly: "inactivity powering down the SD would
	// be nice"). Safe to call repeatedly - a no-op once already powered down. SD.end() first matters
	// because mountSd() otherwise leaves the SD library's internal state assuming the card stays
	// continuously mounted between commands (class doc above) - cutting power out from under that
	// without an orderly end() first is exactly the "breaking the filesystem" risk to avoid; end()
	// itself is a fast local deinit, no card I/O, so this never blocks noticeably.
	void powerDownSd() {
		if (!sdPowered_ || board::kPinSdPowerCtl < 0) {
			return;
		}
		SD.end();
		digitalWrite(board::kPinSdPowerCtl, LOW);
		sdPowered_ = false;
		sdMounted_ = false;	 // forces mountSd()'s next call to do a real remount (design note 95)
	}

	// Call once per loop() iteration - cuts SD power after kIdlePowerDownMs with no SD-volume
	// activity (mountSd() below refreshes lastAccessMs_ on every real access). A no-op whenever the
	// card is already powered down (e.g. an explicit powerDownSd() from SET_POWER_MODE already ran,
	// or this board has no SD slot at all).
	void updateIdlePowerDown() {
		if (sdPowered_ && board::kPinSdPowerCtl >= 0 && millis() - lastAccessMs_ >= kIdlePowerDownMs) {
			powerDownSd();
		}
	}

	Result list(uint8_t volumeValue, const std::string& path, std::vector<Entry>& outEntries) {
		if (volumeValue == volume::kPsram) {
			outEntries.clear();
			for (const auto& kv : psramFiles_) {
				Entry e;
				e.name = baseName(kv.first.c_str());
				e.isDirectory = false;
				e.size = static_cast<uint32_t>(kv.second.size());
				outEntries.push_back(std::move(e));
			}
			return Result::kOk;
		}
		fs::FS* volumeFs = resolveVolume(volumeValue);
		if (volumeFs == nullptr) {
			return Result::kVolumeNotPresent;
		}
		File dir = volumeFs->open(normalizePath(path).c_str());
		if (!dir || !dir.isDirectory()) {
			return Result::kFileNotFound;
		}
		outEntries.clear();
		for (File entry = dir.openNextFile(); entry; entry = dir.openNextFile()) {
			Entry e;
			e.name = baseName(entry.name());
			e.isDirectory = entry.isDirectory();
			e.size = entry.isDirectory() ? 0 : static_cast<uint32_t>(entry.size());
			outEntries.push_back(e);
			entry.close();
		}
		dir.close();
		return Result::kOk;
	}

	Result download(uint8_t volumeValue, const std::string& path, std::vector<uint8_t>& outData) {
		if (volumeValue == volume::kPsram) {
			auto it = psramFiles_.find(normalizePath(path));
			if (it == psramFiles_.end()) {
				return Result::kFileNotFound;
			}
			outData.assign(it->second.data(), it->second.data() + it->second.size());
			return Result::kOk;
		}
		fs::FS* volumeFs = resolveVolume(volumeValue);
		if (volumeFs == nullptr) {
			return Result::kVolumeNotPresent;
		}
		File f = volumeFs->open(normalizePath(path).c_str(), FILE_READ);
		if (!f || f.isDirectory()) {
			return Result::kFileNotFound;
		}
		outData.resize(f.size());
		if (!outData.empty()) {
			f.read(outData.data(), outData.size());
		}
		f.close();
		return Result::kOk;
	}

	// Overwrites any existing file at `path` (doc/PROTOCOL.md §14.3). Missing parent directories are
	// created automatically (the `create=true` 3rd argument below - confirmed by reading the
	// Arduino-ESP32 core's `VFSImpl::open()`: with the 2-arg overload's `create=false` default, a
	// nested path whose parent doesn't yet exist opens "successfully" but the directory is never
	// actually made, so the write silently fails - needed for a client-side recursive folder-sync
	// feature to be able to upload into not-yet-existing subdirectories at all).
	Result upload(uint8_t volumeValue, const std::string& path, const uint8_t* data, size_t length) {
		if (volumeValue == volume::kPsram) {
			std::string normalized = normalizePath(path);
			size_t usedByOthers = psramUsedBytes();
			auto existing = psramFiles_.find(normalized);
			if (existing != psramFiles_.end()) {
				usedByOthers -= existing->second.size();
			}
			if (usedByOthers + length > kPsramVolumeBudgetBytes) {
				return Result::kInsufficientStorage;
			}
			psramFiles_[normalized] = PsramBuffer(data, length);
			return Result::kOk;
		}
		fs::FS* volumeFs = resolveVolume(volumeValue);
		if (volumeFs == nullptr) {
			return Result::kVolumeNotPresent;
		}
		File f = volumeFs->open(normalizePath(path).c_str(), FILE_WRITE, /*create=*/true);
		if (!f) {
			return Result::kIoError;
		}
		size_t written = length == 0 ? 0 : f.write(data, length);
		f.close();
		return written == length ? Result::kOk : Result::kInsufficientStorage;
	}

	// Lightweight presence check - doesn't read any content, unlike download(). Used by the
	// event-triggered macro auto-play path (main.cpp) to decide which of a specific/fallback macro
	// to play without paying for a download() copy of whichever one won't actually be used.
	bool exists(uint8_t volumeValue, const std::string& path) {
		if (volumeValue == volume::kPsram) {
			return psramFiles_.find(normalizePath(path)) != psramFiles_.end();
		}
		fs::FS* volumeFs = resolveVolume(volumeValue);
		return volumeFs != nullptr && volumeFs->exists(normalizePath(path).c_str());
	}

	Result remove(uint8_t volumeValue, const std::string& path) {
		if (volumeValue == volume::kPsram) {
			return psramFiles_.erase(normalizePath(path)) > 0 ? Result::kOk : Result::kFileNotFound;
		}
		fs::FS* volumeFs = resolveVolume(volumeValue);
		if (volumeFs == nullptr) {
			return Result::kVolumeNotPresent;
		}
		std::string normalized = normalizePath(path);
		if (!volumeFs->exists(normalized.c_str())) {
			return Result::kFileNotFound;
		}
		return volumeFs->remove(normalized.c_str()) ? Result::kOk : Result::kIoError;
	}

	// doc/PROTOCOL.md §14.6: copies via download()+upload() rather than a native per-filesystem
	// copy, reusing the two already-tested operations - this is also what makes a cross-volume copy
	// (e.g. SD -> PSRAM, to stage a macro for fast, session-only playback) come for free, since
	// download()/upload() already handle every VOLUME independently. Overwrites any existing file
	// at the destination, matching upload()'s own semantics.
	Result copyFile(uint8_t srcVolumeValue, const std::string& srcPath, uint8_t dstVolumeValue,
			const std::string& dstPath) {
		std::vector<uint8_t> data;
		Result result = download(srcVolumeValue, srcPath, data);
		if (result != Result::kOk) {
			return result;
		}
		return upload(dstVolumeValue, dstPath, data.data(), data.size());
	}

	// doc/PROTOCOL.md §14.7: same-volume rename/move, overwriting any existing file at `dstPath`
	// (matches upload()'s own overwrite semantics) - the intended use is swapping in a new version
	// of a staged file (e.g. a macro) without a delete-then-reupload gap. PSRAM is a plain
	// std::map key move (no data copy); SD/INTERNAL use the underlying fs::FS::rename(), which -
	// unlike upload()'s FILE_WRITE open - does NOT itself overwrite an existing destination, so any
	// existing `dstPath` is removed first.
	Result renameFile(uint8_t volumeValue, const std::string& srcPath, const std::string& dstPath) {
		std::string normSrc = normalizePath(srcPath);
		std::string normDst = normalizePath(dstPath);
		if (volumeValue == volume::kPsram) {
			auto it = psramFiles_.find(normSrc);
			if (it == psramFiles_.end()) {
				return Result::kFileNotFound;
			}
			if (normDst != normSrc) {
				psramFiles_[normDst] = std::move(it->second);
				psramFiles_.erase(it);
			}
			return Result::kOk;
		}
		fs::FS* volumeFs = resolveVolume(volumeValue);
		if (volumeFs == nullptr) {
			return Result::kVolumeNotPresent;
		}
		if (!volumeFs->exists(normSrc.c_str())) {
			return Result::kFileNotFound;
		}
		if (normDst != normSrc && volumeFs->exists(normDst.c_str())) {
			volumeFs->remove(normDst.c_str());
		}
		return volumeFs->rename(normSrc.c_str(), normDst.c_str()) ? Result::kOk : Result::kIoError;
	}

	// Unlike the other operations, a not-present SD card is reported via `outInfo.present=false`
	// (Result::kOk), not Result::kVolumeNotPresent - doc/PROTOCOL.md §14.5 documents PRESENT as
	// exactly this "is a card physically inserted right now" query, not an error condition.
	Result info(uint8_t volumeValue, Info& outInfo) {
		if (volumeValue == volume::kSd) {
			outInfo.present = mountSd();
			if (outInfo.present) {
				setTotalAndFreeBytes(outInfo, SD.totalBytes(), SD.usedBytes());
			} else {
				outInfo.totalBytes = 0;
				outInfo.freeBytes = 0;
			}
			return Result::kOk;
		}
		if (volumeValue == volume::kInternal) {
			outInfo.present = true;  // mounted once at boot, main.cpp's selfTest()
			setTotalAndFreeBytes(outInfo, LittleFS.totalBytes(), LittleFS.usedBytes());
			return Result::kOk;
		}
		if (volumeValue == volume::kPsram) {
			outInfo.present = psramFound();
			setTotalAndFreeBytes(outInfo, kPsramVolumeBudgetBytes, psramUsedBytes());
			return Result::kOk;
		}
		return Result::kVolumeNotPresent;  // unknown VOLUME value - caller should already validate
	}

private:
	// A dedicated budget for this volume, deliberately NOT the whole chip's PSRAM
	// (ESP.getPsramSize()) - that's shared with other subsystems (BLE/WiFi buffers, the working
	// buffer, etc.), so reporting it as "available to this volume" would overpromise. 2 MiB is
	// comfortably enough for macros (§0x0A00) and small session assets while leaving the rest of
	// this board's 8 MB PSRAM for everything else.
	static constexpr size_t kPsramVolumeBudgetBytes = 2u * 1024 * 1024;

	size_t psramUsedBytes() const {
		size_t total = 0;
		for (const auto& kv : psramFiles_) {
			total += kv.second.size();
		}
		return total;
	}

	// doc/PROTOCOL.md §14.5's TOTAL_BYTES/FREE_BYTES are u32 (max ~4.29 GB) - real hardware testing
	// with a 32 GB SD card showed this is a genuine, not-hypothetical limit for any card sold today:
	// SD.totalBytes()/usedBytes() return correct 64-bit byte counts, but a plain narrowing cast
	// silently truncates each of them independently, losing different high bits from each and
	// producing numbers that no longer relate sensibly to each other (FREE_BYTES could even come
	// out *larger* than TOTAL_BYTES). Saturating to UINT32_MAX instead keeps what's reported honest
	// ("at least ~4 GB", never a wrong smaller number) without changing the already-documented
	// 10-byte wire layout - see design note 54 for why this wasn't instead widened to u64 now.
	static void setTotalAndFreeBytes(Info& outInfo, uint64_t total, uint64_t used) {
		uint64_t free = used > total ? 0 : total - used;
		outInfo.totalBytes = total > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(total);
		outInfo.freeBytes = free > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(free);
	}

	fs::FS* resolveVolume(uint8_t volumeValue) {
		if (volumeValue == volume::kSd) {
			return mountSd() ? static_cast<fs::FS*>(&SD) : nullptr;
		}
		if (volumeValue == volume::kInternal) {
			return static_cast<fs::FS*>(&LittleFS);
		}
		return nullptr;
	}

	// Lazy, debounced: the actual SD.end()+SD.begin() remount cycle only runs if more than
	// kRemountDebounceMs has passed since the last VOLUME=SD access (or there's been none yet this
	// "session") - not on every single operation, but not indefinitely skipped either. Originally
	// this remounted unconditionally every time specifically to notice a hot-swapped card without
	// any hardware card-detect interrupt (design note 47's "hot-pluggable" rationale) - measured
	// directly as a real cost once a PC-side recursive folder-sync feature started driving a burst
	// of ~200 individual FILE_UPLOAD/FILE_LIST commands back-to-back (each paying a full remount
	// before this change): several seconds of added latency, real enough to blow past a client
	// timeout. An earlier version of this fix remounted only once per "session" (first access after
	// boot, or after the card was last powered down) - simple, but accepted an unbounded staleness
	// window (a card swapped while the device stayed continuously active, with commands always less
	// than kIdlePowerDownMs apart, would never be re-detected). `kRemountDebounceMs` bounds that
	// window instead - directly requested: "The SD remounting should be still done, just with some
	// timeout to prevent too frequent remounts... don't remount if it was remounted/accessed in the
	// last 5 seconds or similar" - design note 97. `sdMounted_` is also unconditionally reset to
	// false by powerDownSd() (called on HARD_SLEEP/LOW_POWER and the idle-power-down timeout below),
	// so every one of those remains an immediate real remount point regardless of the debounce
	// window's own timing.
	static constexpr unsigned long kRemountDebounceMs = 5000;

	bool mountSd() {
		if (board::kPinSdCs < 0) {
			return false;
		}
		unsigned long now = millis();
		bool skipRemount = sdMounted_ && (now - lastAccessMs_ < kRemountDebounceMs);
		lastAccessMs_ = now;
		if (!sdPowered_ && board::kPinSdPowerCtl >= 0) {
			digitalWrite(board::kPinSdPowerCtl, HIGH);
			delay(10);	// not datasheet-verified - a conservative guess mirroring the display's own
						// VCI settle delay (displaySelfTest()'s comment), since no SD power-up timing
						// spec was available when this was written
			sdPowered_ = true;
		}
		if (skipRemount) {
			return true;
		}
		SD.end();
		sdMounted_ = SD.begin(board::kPinSdCs, sdSpi_);
		return sdMounted_;
	}

	static std::string normalizePath(const std::string& path) {
		if (path.empty()) {
			return "/";
		}
		return path.front() == '/' ? path : "/" + path;
	}

	static std::string baseName(const char* fullPath) {
		const char* slash = strrchr(fullPath, '/');
		return slash != nullptr ? std::string(slash + 1) : std::string(fullPath);
	}

	SPIClass sdSpi_{HSPI};
	std::map<std::string, PsramBuffer> psramFiles_;	// VOLUME=PSRAM - flat, session-only

	// Idle timeout before updateIdlePowerDown() cuts SD power on its own, independent of any
	// SET_POWER_MODE sleep - not yet exposed over the wire (no command asked for it), just a
	// firmware constant for now, easy to retune.
	static constexpr unsigned long kIdlePowerDownMs = 30000;

	bool sdPowered_ = false;
	bool sdMounted_ = false;  // set once mountSd() actually remounts; cleared by powerDownSd()
	unsigned long lastAccessMs_ = 0;
};

}  // namespace crowpanel
