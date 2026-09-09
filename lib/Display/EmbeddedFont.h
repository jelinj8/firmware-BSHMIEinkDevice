// The embedded fonts for DRAW_TEXT (doc/PROTOCOL.md §12.6). FONT_ID 0x00 reuses Adafruit_GFX's
// already-verified classic 5-column/8-row built-in font (glcdfont.c, bundled with the
// GxEPD2/Adafruit_GFX dependency already in this project) for the base ASCII glyphs (0x20-0x7E)
// rather than re-deriving glyph bitmaps by hand - only a small composition layer is hand-authored
// here, each accented codepoint drawn as its unaccented base letter (from the classic font,
// unmodified) plus a simple 2-row, 5-column diacritic mark drawn above it. Originally scoped to
// just the ~30 codepoints Czech text needs (grew from there on request: "extend the accents
// workaround for the builtin fonts to whole europe unicode characters (latin), not limiting to
// Czech set") - kAccentedGlyphs now covers essentially all of Latin-1 Supplement (every
// precomposed accented letter U+00C0-U+00FF that's actually composable this way) plus Latin
// Extended-A's own acute/caron letters (Polish/Slovak), across 7 diacritic shapes: acute, grave,
// circumflex, diaeresis (umlaut), tilde, caron, ring. That reaches French, German, Spanish,
// Portuguese, Italian, Dutch, and most Nordic text, on top of Czech/Slovak/Polish - not
// exhaustively every European orthography, though: three categories are explicitly out of scope,
// still falling back to '?' same as any other unmapped codepoint:
//   - Diacritics that sit *below* the baseline (cedilla Ç/ç, ogonek Ą/ą Ę/ę) - this cell layout
//     only has room for a mark above the glyph body (see "Cell layout" below), not below.
//   - Diacritic shapes not yet designed (breve Ă/ă, macron Ā/ā, dot-above Ż/ż, double-acute Ő/ő) -
//     omitted by choice, not oversight: each affects only one or two languages (Romanian/Latvian/
//     Polish/Hungarian respectively) versus the shapes actually implemented, which cover several
//     languages apiece, and cramming yet more distinct 2-row/5-column shapes into this already-tight
//     space risks the whole set becoming harder to tell apart, not just adding one more.
//   - True ligatures/stroke-through letters (Æ/æ Ø/ø Ł/ł Đ/đ Þ/þ Ð/ð ß) - these aren't a base
//     letter plus an overlay at all, they'd need dedicated hand-drawn glyphs.
//
// This is a deliberately low-resolution approximation, not typographically precise - e.g. an
// accented 'í' shows the classic font's own built-in tittle *and* the added diacritic mark
// stacked, rather than replacing one with the other, and all seven diacritic shapes are only as
// distinguishable as 2 rows x 5 columns allows (grave/acute and caron/circumflex are deliberately
// each other's mirror image, matching their real typographic relationship, which is what keeps
// each pair readable as *related-but-different* rather than just noise). Acceptable for a small
// HMI display's status text; not a design goal to improve further without a concrete reason to.
//
// Also exposes glcdfont.c's own CP437 box-drawing/block characters (codes 176-223 - corners,
// tees, crosses, both single- and double-line variants, plus shade/block fills) via
// kBoxDrawingGlyphs, mapped from their real Unicode "Box Drawings"/"Block Elements" codepoints
// (e.g. U+2500 ─, U+250C ┌) - no new bitmap data needed, these glyphs already exist in the
// classic font, just previously unmapped. Adafruit_GFX::cp437(true) must be called once on the
// GFX instance (done in main.cpp's setup()) for these codes to map correctly - it defaults to
// false, which applies a legacy off-by-one shift to codes >=176 for backward compat with old
// sketches that relied on it.
//
// Cell layout, top to bottom: rows 0-1 = diacritic (blank for plain ASCII), rows 2-9 = the classic
// font's own 8-row glyph body. 10 rows total, 6px advance per character (the classic font's own
// 5px glyph width + 1px spacing, unchanged).
//
// FONT_ID 0x01 is a larger font for the same use case, added on request ("do we have space for a
// larger font?"). Two designs were tried here in sequence:
//
// The first used `Fonts/FreeMono12pt7b.h`, one of Adafruit_GFX's bundled ready-to-use GFXfonts -
// genuinely monospace, no FreeType/fontconvert needed to generate it - reusing FONT_ID 0x00's own
// diacritic-*codepoint*-mapping (textGlyph::resolveGlyph()) but with its own purpose-sized 8x6
// diacritic bitmaps hand-drawn to look like real accent marks, after an even earlier attempt
// (crudely 2x2px-scaling FONT_ID 0x00's own tiny 5x2 dot patterns) was reported directly as
// looking like "disconnected blobs" ("The diacritics is ugly").
//
// That was replaced with the current design after a further, more fundamental request: real
// precomposed accented glyphs instead of any diacritic-overlay composition at all ("I'd prefer
// getting a proper font including the accented characters... avoid the manual workaround"). No
// bundled Adafruit_GFX GFXfont has Latin Extended-A coverage (checked directly - every one of
// them, `FreeMono`/`FreeSans`/etc., only covers 0x20-0x7E), so this font now comes from a
// different library entirely: `U8g2_for_Adafruit_GFX` (a real, well-established bridge that
// renders u8g2's own font ROM data onto any Adafruit_GFX-derived object, confirmed by reading its
// source to draw via Adafruit_GFX::drawFastHLine()/drawFastVLine() - which, like every other
// Adafruit_GFX primitive this project already relies on, fall through to WorkingBufferGfx's own
// overridden drawPixel() with no hardware-accelerated override in between, so DRAW_MODE
// compositing, §12.1, still applies uniformly). Its bundled `u8g2_font_unifont_t_extended` (GNU
// Unifont, sliced to codepoints U+0020-U+02BD) genuinely includes the *entire* Latin-1 Supplement
// and Latin Extended-A blocks - every accented European Latin letter kAccentedGlyphs below
// approximates via composition, and then some (cedilla, ogonek, ligatures/stroke letters
// included) - as real precomposed glyphs, no composition, no diacritic bitmaps, no FreeType
// needed (u8g2 ships this font pre-built); FONT_ID 0x01 was never affected by
// kAccentedGlyphs' own Czech-only-then-broader-Europe scoping, only FONT_ID 0x00 was. See the
// largeFont namespace below for the remaining details -
// Unifont's own half-width/full-width dichotomy is what lets this still slot into this file's
// existing fixed-advance layout code, and why the background fill is still hand-rolled rather
// than using the library's own (confirmed, by reading its decoder, to only cover each glyph's
// tight ink bounding box, not the full advance cell - the same reason FreeMono12pt7b's own opaque
// fill had to be hand-rolled before it). Still no box-drawing glyph coverage (this font's sliced
// range doesn't reach the Box Drawings/Block Elements blocks) - those codepoints fall back to '?',
// same honest unmapped-codepoint behavior FONT_ID 0x00 already has.
//
// Also implements §12.6's optional word-wrap and horizontal alignment (splitTextLines()/
// TextAlign) - both are reference-box concepts (WIDTH in the wire payload), independent of
// WorkingBuffer's separately-settable clip region (§12.10), which is what actually constrains
// where pixels land; the two compose naturally (e.g. wrapped/aligned text drawn after a clip
// region was narrowed to a scrolled sub-area).
#pragma once

#include <Adafruit_GFX.h>
#include <U8g2_for_Adafruit_GFX.h>

#include "CustomFont.h"
#include "Protocol.h"
#include "StorageManager.h"
#include "WorkingBufferGfx.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace crowpanel {

namespace textGlyph {

enum class Diacritic : uint8_t { kNone, kAcute, kCaron, kRing, kGrave, kCircumflex, kDiaeresis, kTilde };

struct AccentedGlyph {
	uint32_t codepoint;
	char base;
	Diacritic diacritic;
};

// Accented Latin letters beyond plain ASCII - Czech's own set (acute/caron/ring), plus the rest of
// Latin-1 Supplement (grave/circumflex/diaeresis/tilde, and Å/å) and Latin Extended-A's own
// remaining acute/caron letters (Polish/Slovak) - see this file's own header comment for exactly
// what's covered and what's deliberately not (cedilla/ogonek, breve/macron/dot-above/double-acute,
// ligatures/stroke letters).
constexpr AccentedGlyph kAccentedGlyphs[] = {
		// Czech/Slovak (acute, caron, ring)
		{0x00C1, 'A', Diacritic::kAcute}, {0x00E1, 'a', Diacritic::kAcute},
		{0x010C, 'C', Diacritic::kCaron}, {0x010D, 'c', Diacritic::kCaron},
		{0x010E, 'D', Diacritic::kCaron}, {0x010F, 'd', Diacritic::kCaron},
		{0x00C9, 'E', Diacritic::kAcute}, {0x00E9, 'e', Diacritic::kAcute},
		{0x011A, 'E', Diacritic::kCaron}, {0x011B, 'e', Diacritic::kCaron},
		{0x00CD, 'I', Diacritic::kAcute}, {0x00ED, 'i', Diacritic::kAcute},
		{0x013D, 'L', Diacritic::kCaron}, {0x013E, 'l', Diacritic::kCaron},	// Slovak Ľ/ľ
		{0x0147, 'N', Diacritic::kCaron}, {0x0148, 'n', Diacritic::kCaron},
		{0x00D3, 'O', Diacritic::kAcute}, {0x00F3, 'o', Diacritic::kAcute},
		{0x0154, 'R', Diacritic::kAcute}, {0x0155, 'r', Diacritic::kAcute},	// Slovak Ŕ/ŕ
		{0x0158, 'R', Diacritic::kCaron}, {0x0159, 'r', Diacritic::kCaron},
		{0x0160, 'S', Diacritic::kCaron}, {0x0161, 's', Diacritic::kCaron},
		{0x0164, 'T', Diacritic::kCaron}, {0x0165, 't', Diacritic::kCaron},
		{0x00DA, 'U', Diacritic::kAcute}, {0x00FA, 'u', Diacritic::kAcute},
		{0x016E, 'U', Diacritic::kRing},  {0x016F, 'u', Diacritic::kRing},
		{0x00DD, 'Y', Diacritic::kAcute}, {0x00FD, 'y', Diacritic::kAcute},
		{0x017D, 'Z', Diacritic::kCaron}, {0x017E, 'z', Diacritic::kCaron},

		// Polish/Croatian/Serbian-Latin acute additions (Czech's own A/E/I/O/U/Y acutes above
		// already cover the letters Polish shares with Czech)
		{0x0106, 'C', Diacritic::kAcute}, {0x0107, 'c', Diacritic::kAcute},	// Ć/ć
		{0x0139, 'L', Diacritic::kAcute}, {0x013A, 'l', Diacritic::kAcute},	// Ĺ/ĺ (Slovak)
		{0x0143, 'N', Diacritic::kAcute}, {0x0144, 'n', Diacritic::kAcute},	// Ń/ń
		{0x015A, 'S', Diacritic::kAcute}, {0x015B, 's', Diacritic::kAcute},	// Ś/ś
		{0x0179, 'Z', Diacritic::kAcute}, {0x017A, 'z', Diacritic::kAcute},	// Ź/ź

		// Latin-1 Supplement: grave (French/Italian/Portuguese)
		{0x00C0, 'A', Diacritic::kGrave}, {0x00E0, 'a', Diacritic::kGrave},
		{0x00C8, 'E', Diacritic::kGrave}, {0x00E8, 'e', Diacritic::kGrave},
		{0x00CC, 'I', Diacritic::kGrave}, {0x00EC, 'i', Diacritic::kGrave},
		{0x00D2, 'O', Diacritic::kGrave}, {0x00F2, 'o', Diacritic::kGrave},
		{0x00D9, 'U', Diacritic::kGrave}, {0x00F9, 'u', Diacritic::kGrave},

		// Latin-1 Supplement: circumflex (French/Portuguese/Romanian)
		{0x00C2, 'A', Diacritic::kCircumflex}, {0x00E2, 'a', Diacritic::kCircumflex},
		{0x00CA, 'E', Diacritic::kCircumflex}, {0x00EA, 'e', Diacritic::kCircumflex},
		{0x00CE, 'I', Diacritic::kCircumflex}, {0x00EE, 'i', Diacritic::kCircumflex},
		{0x00D4, 'O', Diacritic::kCircumflex}, {0x00F4, 'o', Diacritic::kCircumflex},
		{0x00DB, 'U', Diacritic::kCircumflex}, {0x00FB, 'u', Diacritic::kCircumflex},

		// Latin-1 Supplement (+ one Extended-A: Ÿ): diaeresis/umlaut (German/Dutch/French/Nordic)
		{0x00C4, 'A', Diacritic::kDiaeresis}, {0x00E4, 'a', Diacritic::kDiaeresis},
		{0x00CB, 'E', Diacritic::kDiaeresis}, {0x00EB, 'e', Diacritic::kDiaeresis},
		{0x00CF, 'I', Diacritic::kDiaeresis}, {0x00EF, 'i', Diacritic::kDiaeresis},
		{0x00D6, 'O', Diacritic::kDiaeresis}, {0x00F6, 'o', Diacritic::kDiaeresis},
		{0x00DC, 'U', Diacritic::kDiaeresis}, {0x00FC, 'u', Diacritic::kDiaeresis},
		{0x0178, 'Y', Diacritic::kDiaeresis}, {0x00FF, 'y', Diacritic::kDiaeresis},

		// Latin-1 Supplement: tilde (Portuguese/Spanish)
		{0x00C3, 'A', Diacritic::kTilde}, {0x00E3, 'a', Diacritic::kTilde},
		{0x00D1, 'N', Diacritic::kTilde}, {0x00F1, 'n', Diacritic::kTilde},
		{0x00D5, 'O', Diacritic::kTilde}, {0x00F5, 'o', Diacritic::kTilde},

		// Latin-1 Supplement: ring (Nordic - Å/å; Ů/ů above is Extended-A's own ring letter)
		{0x00C5, 'A', Diacritic::kRing}, {0x00E5, 'a', Diacritic::kRing},
};
constexpr size_t kAccentedGlyphCount = sizeof(kAccentedGlyphs) / sizeof(kAccentedGlyphs[0]);

constexpr int16_t kDiacriticHeight = 2;
constexpr int16_t kBodyYOffset = kDiacriticHeight;	 // classic glyph body starts this far down
constexpr int16_t kCellHeight = kBodyYOffset + 8;	 // classic font body is 8 rows tall
constexpr int16_t kAdvanceWidth = 6;				 // classic font: 5px glyph + 1px spacing

// Diacritic bit patterns: 2 rows, 5 columns. bit4=leftmost column .. bit0=rightmost column (only
// the low 5 bits of each byte are meaningful) - simplified single-height approximations, see file
// header.
inline void diacriticRows(Diacritic d, uint8_t& row0, uint8_t& row1) {
	switch (d) {
		case Diacritic::kAcute:
			row0 = 0x02;  // 0b00010 - top: one dot, right-of-center
			row1 = 0x04;  // 0b00100 - bottom: one dot, center (rising left-to-right, like ´)
			break;
		case Diacritic::kGrave:
			row0 = 0x08;  // 0b01000 - top: one dot, left-of-center
			row1 = 0x04;  // 0b00100 - bottom: one dot, center (falling left-to-right, like ` -
						  // acute's mirror image, matching their real typographic relationship)
			break;
		case Diacritic::kCaron:
			row0 = 0x0A;  // 0b01010 - top: two dots, either side of center
			row1 = 0x04;  // 0b00100 - bottom: one dot, center (a v-shape converging downward, ˇ)
			break;
		case Diacritic::kCircumflex:
			row0 = 0x04;  // 0b00100 - top: one dot, center
			row1 = 0x0A;  // 0b01010 - bottom: two dots, either side of center (a ^ peak, caron's
						  // own mirror image top-to-bottom, matching their real relationship)
			break;
		case Diacritic::kRing:
			row0 = 0x0A;  // 0b01010 - top: two dots, either side of center
			row1 = 0x0A;  // 0b01010 - bottom: same (parallel, not converging - distinct from caron)
			break;
		case Diacritic::kDiaeresis:
			row0 = 0x0A;  // 0b01010 - top: two dots, either side of center
			row1 = 0x00;  // bottom row blank - two plain dots and nothing else, like ¨ (distinct
						  // from kRing, which fills both rows)
			break;
		case Diacritic::kTilde:
			row0 = 0x15;  // 0b10101 - top: three dots spread across the full width, suggesting a
						  // wave, like ~
			row1 = 0x00;  // bottom row blank
			break;
		case Diacritic::kNone:
		default:
			row0 = 0;
			row1 = 0;
			break;
	}
}

struct RawGlyph {
	uint32_t codepoint;
	uint8_t classicFontIndex;  // raw index into Adafruit_GFX's bundled 256-glyph classic font
};

// CP437 box-drawing and shade/block characters, Unicode "Box Drawings"/"Block Elements" blocks -
// these bitmaps already exist in Adafruit_GFX's classic font (glcdfont.c, codes 176-223, the
// original IBM PC character set it's descended from) and need no new pixel art, just this
// codepoint mapping. Covers the light (single-line) set - │┤┐└┴┬├─┼, the common "ASCII table"
// corner/tee/cross characters - as well as the double-line and mixed single/double variants, and
// the shade/block fill characters.
constexpr RawGlyph kBoxDrawingGlyphs[] = {
		{0x2591, 176}, {0x2592, 177}, {0x2593, 178},  // light/medium/dark shade
		{0x2502, 179}, {0x2524, 180}, {0x2561, 181}, {0x2562, 182}, {0x2556, 183}, {0x2555, 184},
		{0x2563, 185}, {0x2551, 186}, {0x2557, 187}, {0x255D, 188}, {0x255C, 189}, {0x255B, 190},
		{0x2510, 191},	// ┐ top-right corner
		{0x2514, 192},	// └ bottom-left corner
		{0x2534, 193},	// ┴ bottom tee
		{0x252C, 194},	// ┬ top tee
		{0x251C, 195},	// ├ left tee
		{0x2500, 196},	// ─ horizontal
		{0x253C, 197},	// ┼ cross
		{0x255E, 198}, {0x255F, 199}, {0x255A, 200}, {0x2554, 201}, {0x2569, 202}, {0x2566, 203},
		{0x2560, 204}, {0x2550, 205}, {0x256C, 206}, {0x2567, 207}, {0x2568, 208}, {0x2564, 209},
		{0x2565, 210}, {0x2559, 211}, {0x2558, 212}, {0x2552, 213}, {0x2553, 214}, {0x256B, 215},
		{0x256A, 216},
		{0x2518, 217},	// ┘ bottom-right corner
		{0x250C, 218},	// ┌ top-left corner
		{0x2588, 219}, {0x2584, 220}, {0x258C, 221}, {0x2590, 222}, {0x2580, 223},	 // blocks
};
constexpr size_t kBoxDrawingGlyphCount = sizeof(kBoxDrawingGlyphs) / sizeof(kBoxDrawingGlyphs[0]);

inline void resolveGlyph(uint32_t codepoint, char& base, Diacritic& diacritic) {
	if (codepoint >= 0x20 && codepoint <= 0x7E) {
		base = static_cast<char>(codepoint);
		diacritic = Diacritic::kNone;
		return;
	}
	for (size_t i = 0; i < kAccentedGlyphCount; i++) {
		if (kAccentedGlyphs[i].codepoint == codepoint) {
			base = kAccentedGlyphs[i].base;
			diacritic = kAccentedGlyphs[i].diacritic;
			return;
		}
	}
	for (size_t i = 0; i < kBoxDrawingGlyphCount; i++) {
		if (kBoxDrawingGlyphs[i].codepoint == codepoint) {
			base = static_cast<char>(kBoxDrawingGlyphs[i].classicFontIndex);
			diacritic = Diacritic::kNone;
			return;
		}
	}
	base = '?';  // unmapped codepoint - not an error, just an honest "can't render this" fallback
	diacritic = Diacritic::kNone;
}

}  // namespace textGlyph

// Decodes one UTF-8 codepoint starting at data[pos] (pos < len); advances pos past the consumed
// byte(s). 1-byte (ASCII), 2-byte (up to U+07FF - Latin-1 Supplement, Latin Extended-A), and
// 3-byte (up to U+FFFF - covers the rest of the Basic Multilingual Plane, including the Box
// Drawings/Block Elements codepoints textGlyph::kBoxDrawingGlyphs maps) sequences are recognized;
// anything else (a 4-byte lead byte - astral codepoints/emoji are out of scope for this font
// anyway, or a truncated/invalid continuation) decodes as '?' and advances by exactly 1 byte, so a
// malformed tail can never get the decoder stuck.
inline uint32_t decodeUtf8(const uint8_t* data, size_t len, size_t& pos) {
	uint8_t b0 = data[pos];
	if (b0 < 0x80) {
		pos += 1;
		return b0;
	}
	if ((b0 & 0xE0) == 0xC0 && pos + 1 < len && (data[pos + 1] & 0xC0) == 0x80) {
		uint32_t cp = (static_cast<uint32_t>(b0 & 0x1F) << 6) | (data[pos + 1] & 0x3F);
		pos += 2;
		return cp;
	}
	if ((b0 & 0xF0) == 0xE0 && pos + 2 < len && (data[pos + 1] & 0xC0) == 0x80 && (data[pos + 2] & 0xC0) == 0x80) {
		uint32_t cp = (static_cast<uint32_t>(b0 & 0x0F) << 12) | (static_cast<uint32_t>(data[pos + 1] & 0x3F) << 6) |
				(data[pos + 2] & 0x3F);
		pos += 3;
		return cp;
	}
	pos += 1;
	return '?';
}

inline std::vector<uint32_t> decodeUtf8String(const uint8_t* text, size_t len) {
	std::vector<uint32_t> codepoints;
	size_t pos = 0;
	while (pos < len) {
		codepoints.push_back(decodeUtf8(text, len, pos));
	}
	return codepoints;
}

// Draws one already-decoded line's worth of codepoints with its top-left at (x,y). The caller
// must already have set `gfx`'s draw mode (doc/PROTOCOL.md §12.1). `opaqueBackground=false`
// (transparent, the original v1 behavior) relies on Adafruit_GFX::drawChar()'s bg==color
// convention to skip non-ink pixels entirely, matching §12.1's "non-ink pixels are skipped
// regardless of DRAW_MODE" the same way it already does for the other primitives.
// `opaqueBackground=true` instead passes the *opposite* of `colorValue` as `bg` - `drawChar()`
// then paints every non-ink pixel with it through the exact same drawPixel()/compositePixel()
// path as the ink pixels, so DRAW_MODE still applies uniformly to the whole glyph cell, background
// included. `colorValue=WHITE` with an opaque background is how "inverted" text (light ink on a
// dark fill) is expressed - there's no separate "inverted" concept, it falls straight out of this
// one flag plus which of the two 1bpp values COLOR already is. Returns the line's pixel width
// (codepoints.size() * kAdvanceWidth - this is a fixed-width font, no per-glyph measurement
// needed).
inline int16_t drawCodepointLine(WorkingBufferGfx& gfx, int16_t x, int16_t y, const uint32_t* codepoints,
		size_t count, uint8_t colorValue, bool opaqueBackground) {
	// bg==color is what makes drawChar() skip background pixels (see its own doc); the opposite
	// 1bpp value is the only other choice there is when a background is actually wanted.
	uint8_t backgroundValue =
			opaqueBackground ? (colorValue == color::kBlack ? color::kWhite : color::kBlack) : colorValue;

	int16_t cursorX = x;
	for (size_t i = 0; i < count; i++) {
		char base;
		textGlyph::Diacritic diacritic;
		textGlyph::resolveGlyph(codepoints[i], base, diacritic);

		gfx.drawChar(cursorX, y + textGlyph::kBodyYOffset, base, colorValue, backgroundValue, 1);

		// The diacritic rows sit above drawChar()'s own cell, so they need their own background
		// fill when opaque - always walked (not just when diacritic != kNone) so a plain ASCII
		// character's diacritic rows also get filled for a consistent opaque block per character.
		uint8_t row0, row1;
		textGlyph::diacriticRows(diacritic, row0, row1);
		for (int col = 0; col < 5; col++) {
			uint8_t bit = static_cast<uint8_t>(0x10 >> col);
			if (row0 & bit) {
				gfx.drawPixel(static_cast<int16_t>(cursorX + col), y, colorValue);
			} else if (opaqueBackground) {
				gfx.drawPixel(static_cast<int16_t>(cursorX + col), y, backgroundValue);
			}
			if (row1 & bit) {
				gfx.drawPixel(static_cast<int16_t>(cursorX + col), static_cast<int16_t>(y + 1), colorValue);
			} else if (opaqueBackground) {
				gfx.drawPixel(static_cast<int16_t>(cursorX + col), static_cast<int16_t>(y + 1), backgroundValue);
			}
		}
		// drawChar() itself fills this same 6th "spacer" column (cursorX+5) across its own 8-row
		// body when bg != color, so opaque backgrounds tile seamlessly between characters there -
		// but that spacer column never carries diacritic ink, so it must be filled here too,
		// otherwise a 1px-wide, 2px-tall gap remains at the top of every character cell.
		if (opaqueBackground) {
			gfx.drawPixel(static_cast<int16_t>(cursorX + 5), y, backgroundValue);
			gfx.drawPixel(static_cast<int16_t>(cursorX + 5), static_cast<int16_t>(y + 1), backgroundValue);
		}

		cursorX = static_cast<int16_t>(cursorX + textGlyph::kAdvanceWidth);
	}
	return static_cast<int16_t>(cursorX - x);
}

namespace largeFont {

// Lazily bound to whichever WorkingBufferGfx first draws with this font - safe since this firmware
// only ever has one instance (gWorkingBufferGfx, main.cpp). See file header for why routing
// through U8G2_FOR_ADAFRUIT_GFX still respects DRAW_MODE compositing.
inline U8G2_FOR_ADAFRUIT_GFX& renderer(WorkingBufferGfx& gfx) {
	static U8G2_FOR_ADAFRUIT_GFX instance;
	static bool initialized = false;
	if (!initialized) {
		instance.begin(gfx);
		instance.setFont(u8g2_font_unifont_t_extended);
		initialized = true;
	}
	return instance;
}

// GNU Unifont's own defining convention (not something specific to this one range/slice): every
// glyph is either exactly half-width or full-width, no other advance exists. Every codepoint
// u8g2_font_unifont_t_extended actually covers (U+0020-U+02BD - Basic Latin through Latin
// Extended-B) is half-width, which is what makes treating it as a fixed-advance font for this
// file's existing layout code (splitTextLines()/drawText() below) correct, not an approximation.
constexpr int16_t kAdvanceWidth = 8;

// Ascent/descent are queried from the font itself rather than hand-computed from bitmap data the
// way FreeMono12pt7b's kAscent/kDescent were - this library already exposes font-wide metrics
// directly. Cached after the first call; they never change once the font is set (renderer() only
// ever sets u8g2_font_unifont_t_extended, never swaps fonts).
struct Metrics {
	int16_t ascent;
	int16_t descent;
};

inline Metrics metrics(WorkingBufferGfx& gfx) {
	static Metrics m;
	static bool computed = false;
	if (!computed) {
		U8G2_FOR_ADAFRUIT_GFX& r = renderer(gfx);
		// NOT getFontAscent() (u8g2_font_info_t::ascent_A) - that's anchored to the ascent of a
		// plain, unaccented capital 'A' specifically, which is shorter than this font's own
		// accented capitals (Č Ž Ř Š Ď Ě Ň Ý Á É Í Ó Ú Ů - Czech, per this file's own coverage),
		// whose diacritic marks sit above a plain cap's own height. Reported directly on real
		// hardware: "no space for accents above the capitals" - the diacritic got clipped at the
		// top of the cell. max_char_height+y_offset is u8g2_font_info_t's own documented formula
		// (see its struct comment in U8g2_for_Adafruit_GFX.h) for the font-wide true worst-case
		// ascent across every glyph the font actually contains, not just 'A' - accessed directly
		// via the public u8g2_font_t member since U8G2_FOR_ADAFRUIT_GFX has no wrapper getter for
		// it. Gives every accented capital enough headroom above the baseline.
		m.ascent = static_cast<int16_t>(r.u8g2.font_info.max_char_height + r.u8g2.font_info.y_offset);
		// descent_g is "usually a negative value" per U8g2_for_Adafruit_GFX.h's own struct comment
		// (offset below the baseline) - negated once here so the rest of this file can treat
		// "descent" as a plain positive pixel count, matching kDescent's old meaning for
		// FreeMono12pt7b.
		m.descent = static_cast<int16_t>(-r.getFontDescent());
		computed = true;
	}
	return m;
}

// Draws one already-decoded line's worth of codepoints with its top-left at (x,y), FONT_ID 0x01's
// counterpart to drawCodepointLine() above. Unlike FreeMono12pt7b, accented Czech glyphs need no
// composition step at all - u8g2_font_unifont_t_extended has them as real precomposed glyphs, so
// textGlyph::resolveGlyph()/diacriticRows() (FONT_ID 0x00's own machinery) are never called here.
// A codepoint outside this font's coverage (box-drawing is still unsupported - textGlyph::
// kBoxDrawingGlyphs' codepoints are all well past U+02BD) falls back to '?', checked via
// u8g2_IsGlyph() before drawing - the same honest-fallback philosophy textGlyph::resolveGlyph()
// itself uses for FONT_ID 0x00. Background handling is still a manual full-cell fill, not this
// library's own setFontMode(0)/background-color feature - confirmed by reading its decoder
// (u8g2_font_decode_glyph() in U8g2_for_Adafruit_GFX.cpp) that a background fill there only covers
// each glyph's own tight ink bounding box, not the full fixed advance cell, which would leave
// visible gaps around narrow glyphs (e.g. '.', 'i') - see file header for why FreeMono12pt7b's own
// opaque fill was hand-rolled for the same underlying reason.
inline int16_t drawCodepointLine(WorkingBufferGfx& gfx, int16_t x, int16_t y, const uint32_t* codepoints,
		size_t count, uint8_t colorValue, bool opaqueBackground) {
	U8G2_FOR_ADAFRUIT_GFX& r = renderer(gfx);
	Metrics m = metrics(gfx);
	int16_t cellHeightNow = static_cast<int16_t>(m.ascent + m.descent);
	uint8_t backgroundValue = colorValue == color::kBlack ? color::kWhite : color::kBlack;

	r.setFontMode(1);  // always transparent - background, when wanted, is filled by hand below
	r.setForegroundColor(colorValue);

	int16_t cursorX = x;
	int16_t baselineY = static_cast<int16_t>(y + m.ascent);
	for (size_t i = 0; i < count; i++) {
		if (opaqueBackground) {
			gfx.fillRect(cursorX, y, kAdvanceWidth, cellHeightNow, backgroundValue);
		}

		uint16_t encoding = codepoints[i] <= 0xFFFF ? static_cast<uint16_t>(codepoints[i]) : '?';
		if (!u8g2_IsGlyph(&r.u8g2, encoding)) {
			encoding = '?';
		}
		r.drawGlyph(cursorX, baselineY, encoding);

		cursorX = static_cast<int16_t>(cursorX + kAdvanceWidth);
	}
	return static_cast<int16_t>(cursorX - x);
}

}  // namespace largeFont

// Splits `codepoints` into lines: always breaks on '\n' (U+000A, consumed - never drawn as a
// glyph) regardless of `wrap`. If `wrap` is false (or `maxWidthPx<=0`), every other character is
// preserved *exactly* as given, runs of spaces included - deliberately not tokenized into "words"
// at all, so e.g. a box-drawing/ASCII-art table's fixed internal spacing survives untouched. Only
// when `wrap` is true *and* `maxWidthPx>0` does it additionally break between words (splitting on
// plain space U+0020, collapsing a run of spaces to the single separator space re-inserted between
// words) so no line exceeds `maxWidthPx` - a single word longer than `maxWidthPx` on its own is
// never split mid-word (no hyphenation), it simply overflows that one line. Always returns at
// least one (possibly empty) line. `widthOf` returns one codepoint's pixel advance - a constant
// lambda for the two fixed-advance embedded fonts (textGlyph::kAdvanceWidth /
// largeFont::kAdvanceWidth), or a per-glyph lookup (customFont::glyphFor()'s own width) for the
// proportional custom font, FONT_ID 0xFF - this is the one piece of layout math both share.
inline std::vector<std::vector<uint32_t>> splitTextLines(const std::vector<uint32_t>& codepoints, int16_t maxWidthPx,
		bool wrap, const std::function<int16_t(uint32_t)>& widthOf) {
	std::vector<std::vector<uint32_t>> lines;

	if (!wrap || maxWidthPx <= 0) {
		std::vector<uint32_t> currentLine;
		for (uint32_t cp : codepoints) {
			if (cp == '\n') {
				lines.push_back(currentLine);
				currentLine.clear();
			} else {
				currentLine.push_back(cp);
			}
		}
		lines.push_back(currentLine);
		return lines;
	}

	std::vector<uint32_t> currentLine;
	std::vector<uint32_t> word;
	int32_t currentLineWidth = 0;
	int32_t wordWidth = 0;
	auto appendWord = [&]() {
		if (word.empty()) {
			return;
		}
		if (!currentLine.empty()) {
			int32_t prospectiveWidth = currentLineWidth + widthOf(' ') + wordWidth;
			if (prospectiveWidth > maxWidthPx) {
				lines.push_back(currentLine);
				currentLine.clear();
				currentLineWidth = 0;
			}
		}
		if (!currentLine.empty()) {
			currentLine.push_back(' ');
			currentLineWidth += widthOf(' ');
		}
		currentLine.insert(currentLine.end(), word.begin(), word.end());
		currentLineWidth += wordWidth;
		word.clear();
		wordWidth = 0;
	};

	for (uint32_t cp : codepoints) {
		if (cp == '\n') {
			appendWord();
			lines.push_back(currentLine);
			currentLine.clear();
			currentLineWidth = 0;
		} else if (cp == ' ') {
			appendWord();
		} else {
			word.push_back(cp);
			wordWidth += widthOf(cp);
		}
	}
	appendWord();
	lines.push_back(currentLine);
	return lines;
}

enum class TextAlign : uint8_t { kLeft = 0, kCenter = 1, kRight = 2 };

struct TextLayoutResult {
	int16_t width;
	int16_t height;
};

// doc/PROTOCOL.md §12.6: draws `text` with its top-left at (x,y), using `fontId` (0x00 the classic
// small font, 0x01 the larger u8g2_font_unifont_t_extended font, 0xFF the folder-driven custom
// proportional font, CustomFont.h - handleDrawText, main.cpp, validates this before calling in,
// and for 0xFF must already have called customFont::ensureReady() successfully). `text` is UTF-8
// for fonts 0x00/0x01; for font 0xFF it is instead a raw single-byte codepage (no UTF-8 decoding -
// each byte 0x00-0xFF is directly a glyph index, see CustomFont.h). An embedded '\n' always starts
// a new line, whether or not WIDTH/WRAP are in use (e.g. for a pre-formatted multi-line block - an
// ASCII-art table drawn with box-drawing characters, textGlyph::kBoxDrawingGlyphs, being the
// motivating case: its fixed internal spacing must survive exactly, which is why word-wrap-style
// whitespace collapsing only ever applies when WRAP is genuinely requested - see splitTextLines()).
// `widthPx<=0` means no alignment/wrap reference box at all - every line is left-aligned at `x`,
// `align`/`wrap` are ignored entirely; `widthPx>0` is the alignment/wrap reference box.
// `opaqueBackground` fills each glyph cell's non-ink pixels with the opposite of `colorValue` (see
// drawCodepointLine/largeFont::drawCodepointLine/customFont::drawByteLine) - it does NOT extend to
// the alignment padding left/right of a CENTER/RIGHT-aligned short line within `widthPx`, only to
// the glyph cells actually drawn. `storage` is only used by the font-0xFF path (glyph loading);
// fonts 0x00/0x01 ignore it. The caller must already have set `gfx`'s draw mode. Returns the
// bounding box actually used: width is `widthPx` if given, else the widest line's natural width;
// height is always `lineCount * cellHeight`, where cellHeight is each font's own fixed cell height
// (textGlyph::kCellHeight, the large font's ascent+descent, or font 0xFF's XX glyph height).
inline TextLayoutResult drawText(WorkingBufferGfx& gfx, StorageManager& storage, uint8_t fontId, int16_t x, int16_t y,
		int16_t widthPx, TextAlign align, bool wrap, const uint8_t* text, size_t len, uint8_t colorValue,
		bool opaqueBackground) {
	bool large = fontId == 0x01;
	bool custom = fontId == 0xFF;

	int16_t cellHeight = textGlyph::kCellHeight;
	if (large) {
		largeFont::Metrics m = largeFont::metrics(gfx);
		cellHeight = static_cast<int16_t>(m.ascent + m.descent);
	} else if (custom) {
		cellHeight = static_cast<int16_t>(customFont::xxGlyph().height);
	}

	std::function<int16_t(uint32_t)> widthOf;
	if (large) {
		widthOf = [](uint32_t) { return largeFont::kAdvanceWidth; };
	} else if (custom) {
		widthOf = [&storage](uint32_t cp) {
			return static_cast<int16_t>(customFont::glyphFor(storage, static_cast<uint8_t>(cp)).width);
		};
	} else {
		widthOf = [](uint32_t) { return textGlyph::kAdvanceWidth; };
	}

	std::vector<uint32_t> codepoints;
	if (custom) {
		// Raw single-byte codepage, not UTF-8 (see this function's own doc) - each byte is directly
		// a glyph index. Word/line splitting below only ever compares against '\n' (0x0A) and ' '
		// (0x20), identical values either way, so no font-0xFF-specific branch is needed there.
		codepoints.reserve(len);
		for (size_t i = 0; i < len; i++) {
			codepoints.push_back(text[i]);
		}
	} else {
		codepoints = decodeUtf8String(text, len);
	}

	std::vector<std::vector<uint32_t>> lines = splitTextLines(codepoints, widthPx, wrap, widthOf);

	int16_t maxLineWidth = 0;
	int16_t lineY = y;
	for (const std::vector<uint32_t>& line : lines) {
		int16_t lineWidth = 0;
		for (uint32_t cp : line) {
			lineWidth = static_cast<int16_t>(lineWidth + widthOf(cp));
		}
		int16_t lineX = x;
		if (widthPx > 0) {
			if (align == TextAlign::kCenter) {
				lineX = static_cast<int16_t>(x + (widthPx - lineWidth) / 2);
			} else if (align == TextAlign::kRight) {
				lineX = static_cast<int16_t>(x + widthPx - lineWidth);
			}
		}
		if (custom) {
			std::vector<uint8_t> bytesLine(line.begin(), line.end());
			customFont::drawByteLine(gfx, storage, lineX, lineY, bytesLine.data(), bytesLine.size(), colorValue,
					opaqueBackground, cellHeight);
		} else if (large) {
			largeFont::drawCodepointLine(gfx, lineX, lineY, line.data(), line.size(), colorValue, opaqueBackground);
		} else {
			drawCodepointLine(gfx, lineX, lineY, line.data(), line.size(), colorValue, opaqueBackground);
		}
		if (lineWidth > maxLineWidth) {
			maxLineWidth = lineWidth;
		}
		lineY = static_cast<int16_t>(lineY + cellHeight);
	}
	return {widthPx > 0 ? widthPx : maxLineWidth, static_cast<int16_t>(lines.size() * cellHeight)};
}

}  // namespace crowpanel
