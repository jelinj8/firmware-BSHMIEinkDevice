// Custom, folder-driven, proportional-width font for DRAW_TEXT FONT_ID 0xFF (doc/PROTOCOL.md
// §12.6.x). Unlike the two embedded fonts in EmbeddedFont.h, this font's glyphs live as
// individual binary files (".gly", see doc/PROTOCOL.md's glyph file format section) on SD or
// INTERNAL storage, in a folder selected at runtime by SET_CUSTOM_FONT_FOLDER
// (main.cpp::handleSetCustomFontFolder, the only writer of setFolder() below). Each of the 256
// codepage bytes 0x00-0xFF maps to its own "<NN>.gly" file (NN = 2 uppercase hex digits); a
// mandatory "XX.gly" file supplies both the fallback glyph for any missing/corrupt codepoint and
// this font's line-height reference (see xxGlyph()'s own doc) - "XX" can never collide with a
// real hex byte value, so it's an unambiguous reserved name.
//
// Glyphs load lazily (on first use by a DRAW_TEXT call) and are cached in PSRAM
// (StorageManager::PsramBuffer) for the lifetime of the current folder - setFolder() discards the
// whole cache, since every cached glyph belonged to whichever folder was active before.
#pragma once

#include "Protocol.h"
#include "StorageManager.h"
#include "WorkingBufferGfx.h"

#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace crowpanel {
namespace customFont {

struct GlyphBitmap {
	uint16_t width = 0;
	uint16_t height = 0;
	PsramBuffer bitmap;  // packed 1bpp, row-major, MSB-first - bit=1=ink, bit=0=transparent
	bool ok = false;	  // false = failed to load (missing/corrupt); glyphFor() never returns one
						  // of these directly, it substitutes xxGlyph() instead (see glyphFor())
};

struct FolderConfig {
	bool configured = false;
	uint8_t volume = volume::kSd;
	std::string path;
};

// Function-local static singletons - the same pattern EmbeddedFont.h's largeFont::renderer()/
// metrics() already use for this firmware's single-instance-of-everything model.
inline FolderConfig& folderConfig() {
	static FolderConfig cfg;
	return cfg;
}

inline std::map<uint8_t, GlyphBitmap>& glyphCache() {
	static std::map<uint8_t, GlyphBitmap> cache;
	return cache;
}

inline GlyphBitmap& xxGlyphSlot() {
	static GlyphBitmap g;
	return g;
}

inline bool& xxLoadAttempted() {
	static bool v = false;
	return v;
}

constexpr char kGlyFileMagic[4] = {'G', 'L', 'Y', '1'};
constexpr uint8_t kGlyFormatVersion = 0x01;
constexpr size_t kGlyHeaderSize = 9;  // MAGIC(4) + FORMAT_VERSION(1) + WIDTH(2) + HEIGHT(2)

// Parses one already-downloaded .gly file's bytes (doc/PROTOCOL.md's glyph file format section)
// into `out`. Returns false (leaving `out.ok=false`) on any structural problem - wrong magic/
// version, or a size that doesn't match WIDTH/HEIGHT's own implied packed-bitmap size - callers
// treat that identically to "file not found" (fall back to XX).
inline bool parseGlyFile(const std::vector<uint8_t>& data, GlyphBitmap& out) {
	if (data.size() < kGlyHeaderSize || std::memcmp(data.data(), kGlyFileMagic, 4) != 0 ||
			data[4] != kGlyFormatVersion) {
		return false;
	}
	uint16_t width = static_cast<uint16_t>(data[5] | (data[6] << 8));
	uint16_t height = static_cast<uint16_t>(data[7] | (data[8] << 8));
	if (width == 0 || height == 0) {
		return false;
	}
	size_t bytesPerRow = (static_cast<size_t>(width) + 7) / 8;
	if (data.size() != kGlyHeaderSize + bytesPerRow * height) {
		return false;
	}
	out.width = width;
	out.height = height;
	out.bitmap = PsramBuffer(data.data() + kGlyHeaderSize, data.size() - kGlyHeaderSize);
	out.ok = true;
	return true;
}

inline std::string glyphFileName(const std::string& folder, const std::string& baseName) {
	return folder + "/" + baseName + ".gly";
}

inline std::string hexBaseName(uint8_t code) {
	static const char kHexDigits[] = "0123456789ABCDEF";
	char buf[3] = {kHexDigits[(code >> 4) & 0xF], kHexDigits[code & 0xF], '\0'};
	return std::string(buf);
}

// Called by handleSetCustomFontFolder() once a folder is resolved (doc/PROTOCOL.md's
// SET_CUSTOM_FONT_FOLDER section) - replaces the active folder and discards every cached glyph,
// XX included, since they all belonged to whatever folder was active before.
inline void setFolder(uint8_t volumeValue, const std::string& path) {
	FolderConfig& cfg = folderConfig();
	cfg.configured = true;
	cfg.volume = volumeValue;
	cfg.path = path;
	glyphCache().clear();
	xxGlyphSlot() = GlyphBitmap();
	xxLoadAttempted() = false;
}

// Loads (or returns the already-cached result of loading) the mandatory XX.gly fallback glyph,
// which also supplies this font's line-height reference (doc/PROTOCOL.md: glyphs are "bottom
// aligned, using XX as line height reference"). handleDrawText() calls this as a pre-flight check
// before any FONT_ID=0xFF draw and NACKs FILE_NOT_FOUND if it returns false - either no folder was
// ever configured this session, or the configured folder's own XX.gly is missing/corrupt.
inline bool ensureReady(StorageManager& storage, uint8_t& failStatus) {
	const FolderConfig& cfg = folderConfig();
	if (!cfg.configured) {
		failStatus = status::kFileNotFound;
		return false;
	}
	if (xxLoadAttempted()) {
		if (!xxGlyphSlot().ok) {
			failStatus = status::kFileNotFound;
			return false;
		}
		return true;
	}
	xxLoadAttempted() = true;
	std::vector<uint8_t> data;
	StorageManager::Result result = storage.download(cfg.volume, glyphFileName(cfg.path, "XX"), data);
	if (result == StorageManager::Result::kOk && parseGlyFile(data, xxGlyphSlot())) {
		return true;
	}
	failStatus = status::kFileNotFound;
	return false;
}

// Valid only after a successful ensureReady() this session.
inline const GlyphBitmap& xxGlyph() { return xxGlyphSlot(); }

// Lazily loads (or returns the already-cached) glyph for one raw codepage byte, silently
// substituting - and caching that substitution, so a missing individual glyph is never
// re-attempted from disk on every subsequent draw - xxGlyph() if its own file is missing/corrupt.
// Callers must have already called ensureReady() successfully (so xxGlyph() is valid) before
// calling this, since the substitution relies on it.
inline const GlyphBitmap& glyphFor(StorageManager& storage, uint8_t code) {
	std::map<uint8_t, GlyphBitmap>& cache = glyphCache();
	auto it = cache.find(code);
	if (it != cache.end()) {
		return it->second.ok ? it->second : xxGlyph();
	}
	GlyphBitmap glyph;
	std::vector<uint8_t> data;
	const FolderConfig& cfg = folderConfig();
	StorageManager::Result result = storage.download(cfg.volume, glyphFileName(cfg.path, hexBaseName(code)), data);
	if (result == StorageManager::Result::kOk) {
		parseGlyFile(data, glyph);
	}
	GlyphBitmap& stored = cache[code] = std::move(glyph);
	return stored.ok ? stored : xxGlyph();
}

// bit=1=ink(COLOR), bit=0=transparent - the same packed-1bpp row-major MSB-first addressing
// convention as main.cpp's own getPackedBit() for .epi images (doc/PROTOCOL.md §12.7),
// reimplemented locally rather than shared, since lib/Display must not depend back on src/main.cpp.
inline bool glyphBit(const GlyphBitmap& glyph, uint16_t x, uint16_t y) {
	size_t bytesPerRow = (static_cast<size_t>(glyph.width) + 7) / 8;
	uint8_t byteValue = glyph.bitmap.data()[y * bytesPerRow + x / 8];
	return (byteValue & (0x80 >> (x % 8))) != 0;
}

// Draws one already-resolved line's worth of raw codepage bytes with its top-left at (x,y),
// bottom-aligned within `cellHeight` (doc/PROTOCOL.md: glyphs are "bottom aligned, use XX as line
// height reference") - each glyph's own height may be <= cellHeight, so its top is placed at
// y + (cellHeight - glyph.height), pinning its *bottom* edge to the line's own bottom rather than
// its top. No extra spacing between glyphs (doc/PROTOCOL.md: "no extra character spacing") -
// advance is exactly each glyph's own stored width, letting adjacent glyphs be "welded" together.
inline int16_t drawByteLine(WorkingBufferGfx& gfx, StorageManager& storage, int16_t x, int16_t y,
		const uint8_t* bytes, size_t count, uint8_t colorValue, bool opaqueBackground, int16_t cellHeight) {
	uint8_t backgroundValue = colorValue == color::kBlack ? color::kWhite : color::kBlack;
	int16_t cursorX = x;
	for (size_t i = 0; i < count; i++) {
		const GlyphBitmap& glyph = glyphFor(storage, bytes[i]);
		int16_t glyphTopY = static_cast<int16_t>(y + (cellHeight - glyph.height));
		if (opaqueBackground) {
			gfx.fillRect(cursorX, y, glyph.width, cellHeight, backgroundValue);
		}
		for (uint16_t row = 0; row < glyph.height; row++) {
			for (uint16_t col = 0; col < glyph.width; col++) {
				if (glyphBit(glyph, col, row)) {
					gfx.drawPixel(static_cast<int16_t>(cursorX + col), static_cast<int16_t>(glyphTopY + row),
							colorValue);
				}
			}
		}
		cursorX = static_cast<int16_t>(cursorX + glyph.width);
	}
	return static_cast<int16_t>(cursorX - x);
}

}  // namespace customFont
}  // namespace crowpanel
