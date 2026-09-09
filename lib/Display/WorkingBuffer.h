// The protocol's "working buffer" (doc/PROTOCOL.md §2.1): an MCU-side, persistent, full-panel
// 1bpp bitmap (wire polarity: bit=1=BLACK, matching §6) that FULL_IMAGE_TRANSFER (§6) and
// PARTIAL_IMAGE_TRANSFER (§7) write into, and that REFRESH (§12.8) flips to the panel from - a
// command with no pixel payload of its own, so something has to remember what's pending.
//
// Kept as an explicit MCU-side copy rather than "whatever's currently in the SSD1683's own RAM" -
// the earlier FULL_IMAGE_TRANSFER-only implementation used the latter, but PARTIAL_IMAGE_TRANSFER
// + REFRESH's "union of bounding boxes deferred since the last refresh" semantics inherently need
// to track region-level state ACROSS multiple deferred writes before any of them touch the
// controller at all, which the controller's own current/previous RAM banks (with their existing
// full-vs-partial-write-count-as-different-things behavior) aren't a natural fit for.
//
// Like Dispatcher.h, this is hardcoded to GxEPD2_420_GDEY042T81 (next-steps.md #3's finding) rather
// than made generic per-board - same reasoning: no second SSD16xx-family board exists yet to
// justify the abstraction.
#pragma once

#include <Arduino.h>
#include <GxEPD2_BW.h>
#include <gdey/GxEPD2_420_GDEY042T81.h>

#include "BoardConfig.h"
#include "Protocol.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace crowpanel {

class WorkingBuffer {
public:
	using Display = GxEPD2_BW<GxEPD2_420_GDEY042T81, GxEPD2_420_GDEY042T81::HEIGHT>;

	static constexpr uint16_t kWidth = GxEPD2_420_GDEY042T81::WIDTH;
	static constexpr uint16_t kHeight = GxEPD2_420_GDEY042T81::HEIGHT;
	static constexpr size_t kSizeBytes = (kWidth / 8) * kHeight;

	explicit WorkingBuffer(Display& display)
			: display_(display), buffer_(kSizeBytes, 0x00), panelBuffer_(kSizeBytes, 0x00),
			  clip_{0, 0, kWidth, kHeight} {}

	// doc/PROTOCOL.md §17 SET_POWER_MODE, requested directly ("for full powerdown we can turn them
	// off completely, for light sleep it should be recoverable"). Safe to call for either HARD_SLEEP
	// (no matching restore needed - a HARD_SLEEP wake is a full reboot, and this class's own
	// constructor already starts with needsControllerResync_=true, so a fresh boot's first real draw
	// goes through exactly the same lazy resync as a LOW_POWER wake would - see flush()) or LOW_POWER
	// (this class owns bringing it back, lazily, in ensureControllerReady() below - never called
	// eagerly on wake, only when something is actually about to be drawn).
	//
	// hibernate() alone - regardless of whether board::kPinDisplayPowerCtl even exists on this board
	// - already leaves GxEPD2 in a state (its own internal _hibernating/_init_display_done flags,
	// gdey/GxEPD2_420_GDEY042T81.cpp) where the *next* write of any kind transparently triggers a
	// full hardware reset + SWRESET before doing anything else - confirmed by reading that driver's
	// source rather than assumed, since it's the exact mechanism ensureControllerReady() below relies
	// on instead of duplicating an explicit init() call itself. That SWRESET clears the controller's
	// own internal "current"/"previous" RAM banks (used for partial-update diffing) even though the
	// *physical* pixels are completely unaffected (e-ink retains its image with zero power, §2.1) -
	// needsControllerResync_ tracks that the next flush() must reseed those banks from panelBuffer_
	// (this class's own always-correct shadow of "what's really on screen") before trusting any
	// partial-update diff again, regardless of whether VCI itself was ever physically cut.
	void powerDown() {
		display_.hibernate();
		needsControllerResync_ = true;
		if (board::kPinDisplayPowerCtl >= 0) {
			digitalWrite(board::kPinDisplayPowerCtl, LOW);
			vccCut_ = true;
		}
	}

	// Deliberately no updateIdlePowerDown() here, unlike StorageManager - considered and measured
	// against the SSD1683 datasheet (electrical characteristics, p.43) rather than added by default:
	// hibernate() above already runs unconditionally before *any* powerDown(), which alone already
	// drops the controller to its Deep Sleep Mode 1 current (3uA typ/5uA max from VCI) regardless of
	// whether board::kPinDisplayPowerCtl gets cut too - so an ACTIVE-mode idle timeout could only ever
	// additionally save that same few uA (bounded by the datasheet; the level-shifter's own leakage
	// when cut is unmeasured). Against the ESP32-S3's own tens-of-mA ACTIVE-mode draw, that's not a
	// meaningful saving - unlike LOW_POWER/HARD_SLEEP (SET_POWER_MODE, main.cpp), where the *rest* of
	// the system is also minimized and the same few uA becomes proportionally real. Requested, built,
	// measured against the datasheet, and explicitly dropped again for this reason - not an oversight.

	// doc/PROTOCOL.md §12.12 SET_ORIENTATION: rotation/mirroring of the *logical* canvas that every
	// §12 coordinate (drawing primitives, SET_CLIP_REGION, SET_DRAW_OFFSET) is expressed in, applied
	// once - here - at the same single chokepoint as everything else in this class, right before a
	// logical coordinate is turned into a physical buffer position (toPhysical()). 90°/270° swap the
	// logical canvas's width/height (logicalWidth()/logicalHeight()) relative to the physical panel
	// (kWidth/kHeight) - this is what makes a portrait orientation possible without the panel itself
	// changing. Mirroring is applied in logical space, before rotation.
	void setOrientation(uint8_t rotation, bool mirrorH, bool mirrorV) {
		orientation_.rotation = rotation;
		orientation_.mirrorH = mirrorH;
		orientation_.mirrorV = mirrorV;
	}

	void getOrientation(uint8_t& rotation, bool& mirrorH, bool& mirrorV) const {
		rotation = orientation_.rotation;
		mirrorH = orientation_.mirrorH;
		mirrorV = orientation_.mirrorV;
	}

	uint16_t logicalWidth() const {
		return (orientation_.rotation == orientation::kRotate90 || orientation_.rotation == orientation::kRotate270)
				? kHeight
				: kWidth;
	}

	uint16_t logicalHeight() const {
		return (orientation_.rotation == orientation::kRotate90 || orientation_.rotation == orientation::kRotate270)
				? kWidth
				: kHeight;
	}

	// doc/PROTOCOL.md §12.13 SET_DRAW_OFFSET: a persistent (dx,dy) pan applied - here, the same
	// single chokepoint - to every §12 write/read before anything else (clip check, then rotation/
	// mirror). Lets a caller address logical positions outside [0,logicalWidth)x[0,logicalHeight)
	// (e.g. an anchor that, once shifted, lands off-canvas) without every individual command's own
	// X/Y wire field needing to be signed - out-of-canvas positions are simply silently dropped by
	// the existing bounds check, the same way any other off-panel geometry already is. (0,0) is the
	// identity/reset value - no separate reset sentinel is needed, unlike SET_CLIP_REGION's
	// WIDTH=0/HEIGHT=0, since a plain (0,0) offset already means "no offset".
	void setDrawOffset(int16_t dx, int16_t dy) {
		offset_.dx = dx;
		offset_.dy = dy;
	}

	void getDrawOffset(int16_t& dx, int16_t& dy) const {
		dx = offset_.dx;
		dy = offset_.dy;
	}

	// doc/PROTOCOL.md §12.10 SET_CLIP_REGION: constrains every subsequent §12 drawing primitive
	// (and SHIFT_REGION) to this rectangle - enforced once, here, in getPixel/setPixel, so every
	// caller (WorkingBufferGfx::drawPixel, shift()) gets it automatically rather than each needing
	// its own check. `write()` (§6/§7 image transfer) deliberately does NOT go through
	// getPixel/setPixel and so is NOT affected by the clip region - image transfer already has its
	// own explicit bounds validation (NACK on violation, §7), a different and stricter contract
	// than "silently clipped" that this shouldn't quietly change. `width==0 || height==0` resets
	// the clip to the full logical canvas. Starts as the full panel on construction.
	//
	// X/Y/WIDTH/HEIGHT are in *logical* (post-orientation) coordinates, and are stored as given -
	// deliberately NOT pre-transformed against the current orientation, and NOT offset by
	// SET_DRAW_OFFSET. That means the clip window automatically "follows" a later SET_ORIENTATION
	// change (the same numbers get reinterpreted against the new logical width/height, so a
	// portrait-mode clip box doesn't need to be re-issued after rotating - see toPhysical(), which
	// re-derives logicalWidth()/Height() from the *current* orientation on every check) while
	// staying completely unaffected by SET_DRAW_OFFSET (an independent, orthogonal pan of drawing
	// content within this same logical frame - doc/PROTOCOL.md §12.13).
	void setClipRegion(uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
		if (width == 0 || height == 0) {
			resetClipRegion();
			return;
		}
		clip_.x0 = x;
		clip_.y0 = y;
		clip_.x1 = static_cast<uint16_t>(std::min<uint32_t>(static_cast<uint32_t>(x) + width, logicalWidth()));
		clip_.y1 = static_cast<uint16_t>(std::min<uint32_t>(static_cast<uint32_t>(y) + height, logicalHeight()));
	}

	void resetClipRegion() { clip_ = {0, 0, logicalWidth(), logicalHeight()}; }

	void getClipRegion(uint16_t& x, uint16_t& y, uint16_t& width, uint16_t& height) const {
		x = clip_.x0;
		y = clip_.y0;
		width = clip_.x1 - clip_.x0;
		height = clip_.y1 - clip_.y0;
	}

	// Copies `data` (row-major, MSB-first, bit=1=BLACK - same layout as the wire, §6/§7) into
	// [x,y,w,h) of the buffer. Does NOT touch the controller or dirty-region tracking - callers
	// decide separately whether to flush() immediately (FLAGS.REFRESH_NOW=1) or markDirty() for a
	// later REFRESH (FLAGS.REFRESH_NOW=0). Precondition: x and w are multiples of 8 (always true
	// for FULL_IMAGE_TRANSFER's whole-panel width, and enforced for PARTIAL_IMAGE_TRANSFER by the
	// caller's own §7 granularity validation before this is ever called).
	void write(const uint8_t* data, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
		size_t srcBytesPerRow = (w + 7) / 8;
		size_t dstBytesPerRow = kWidth / 8;
		size_t dstByteX = x / 8;
		for (uint16_t row = 0; row < h; row++) {
			std::memcpy(buffer_.data() + (y + row) * dstBytesPerRow + dstByteX, data + row * srcBytesPerRow,
					srcBytesPerRow);
		}
	}

	// Bit-level access (wire polarity: 1=BLACK), for §12's local drawing primitives - unlike write()
	// above, there's no byte-alignment precondition here, matching §12's "no alignment constraint
	// applies here (unlike §7)". Out-of-range reads return false (white); out-of-range writes are
	// silently clipped - ordinary framebuffer semantics, not a §7-style NACK situation, since §12
	// never validates its geometry's bounds against the panel. `x`/`y` are *logical* coordinates
	// (pre-offset, pre-orientation) - every caller (WorkingBufferGfx::drawPixel, shift(),
	// copyRegion()) always passes these, never physical buffer coordinates directly; signed (not
	// uint16_t) because SET_DRAW_OFFSET (§12.13) can legitimately shift a logical coordinate
	// negative before this rejects it as out of bounds.
	bool getPixel(int32_t x, int32_t y) const {
		x += offset_.dx;
		y += offset_.dy;
		if (!inClipRegion(x, y)) {
			return false;
		}
		uint16_t px, py;
		if (!toPhysical(x, y, px, py)) {
			return false;
		}
		return (buffer_[pixelByteIndex(px, py)] & pixelBitMask(px)) != 0;
	}

	void setPixel(int32_t x, int32_t y, bool value) {
		x += offset_.dx;
		y += offset_.dy;
		if (!inClipRegion(x, y)) {
			return;
		}
		uint16_t px, py;
		if (!toPhysical(x, y, px, py)) {
			return;
		}
		uint8_t& byte = buffer_[pixelByteIndex(px, py)];
		uint8_t mask = pixelBitMask(px);
		if (value) {
			byte |= mask;
		} else {
			byte &= static_cast<uint8_t>(~mask);
		}
	}

	// Like getPixel(), but ignores the clip region (only the logical canvas bounds still apply) -
	// the clip region constrains what gets *written*, not what existing content may be *read*
	// back, so COPY_REGION's source side (§12.11) uses this rather than getPixel() - reading a copy
	// source that happens to fall outside whatever the clip region is currently set to should still
	// see the real content there, not the "outside clip reads as white" behavior getPixel() gives
	// destinations. Still honors SET_DRAW_OFFSET/SET_ORIENTATION, same as getPixel() - those are a
	// coordinate-space mapping, not part of "clipping".
	bool getPixelUnclipped(int32_t x, int32_t y) const {
		x += offset_.dx;
		y += offset_.dy;
		uint16_t px, py;
		if (!toPhysical(x, y, px, py)) {
			return false;
		}
		return (buffer_[pixelByteIndex(px, py)] & pixelBitMask(px)) != 0;
	}

	// Applies doc/PROTOCOL.md §12.1's compositing rule for one pixel: new_dst = combine(dst, src,
	// mode). Callers mark the overall affected region dirty/flushed themselves once per command
	// (not per pixel) - see finishWrite() in main.cpp - since that's known upfront and marking per
	// pixel would be needless overhead for e.g. a filled circle.
	void compositePixel(int32_t x, int32_t y, bool src, uint8_t mode) {
		bool result;
		switch (mode) {
			case drawMode::kOr:
				result = getPixel(x, y) || src;
				break;
			case drawMode::kXor:
				result = getPixel(x, y) != src;
				break;
			case drawMode::kAnd:
				result = getPixel(x, y) && src;
				break;
			case drawMode::kReplace:
			default:
				result = src;
				break;
		}
		setPixel(x, y, result);
	}

	// doc/PROTOCOL.md §12.11 COPY_REGION: copies [srcX,srcY,w,h) to [dstX,dstY,w,h) - a plain
	// overwrite, like SHIFT_REGION, no DRAW_MODE. Reads the whole source into a temporary buffer
	// first so overlapping source/destination rects (in either direction) are always handled
	// correctly, same idea as memmove vs memcpy. Source reads bypass the clip region
	// (getPixelUnclipped - see its own doc) since the destination write (setPixel) is where
	// clipping should apply, not the source read; a source pixel outside the panel reads as white.
	void copyRegion(uint16_t srcX, uint16_t srcY, uint16_t dstX, uint16_t dstY, uint16_t w, uint16_t h) {
		std::vector<bool> temp(static_cast<size_t>(w) * h);
		for (uint16_t row = 0; row < h; row++) {
			for (uint16_t col = 0; col < w; col++) {
				temp[static_cast<size_t>(row) * w + col] = getPixelUnclipped(srcX + col, srcY + row);
			}
		}
		for (uint16_t row = 0; row < h; row++) {
			for (uint16_t col = 0; col < w; col++) {
				setPixel(dstX + col, dstY + row, temp[static_cast<size_t>(row) * w + col]);
			}
		}
	}

	// doc/PROTOCOL.md §12.9 SHIFT_REGION: shifts [x,y,w,h)'s content by `step` pixels in
	// `direction`; content moved past the region's own edge is discarded (never wraps), and the
	// vacated strip is filled with `fill`. Iterates in the direction that lets it work in place
	// (ascending when reading from a higher index, descending when reading from a lower one) so no
	// temporary copy of the region is needed. `step` >= the region's own extent in the shift axis
	// just fills the whole region - nothing survives a shift that large anyway.
	void shift(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t direction, uint16_t step, bool fill) {
		bool horizontal = direction == shiftDirection::kLeft || direction == shiftDirection::kRight;
		uint16_t extent = horizontal ? w : h;
		if (step >= extent) {
			for (uint16_t row = 0; row < h; row++) {
				for (uint16_t col = 0; col < w; col++) {
					setPixel(x + col, y + row, fill);
				}
			}
			return;
		}
		switch (direction) {
			case shiftDirection::kLeft:
				for (uint16_t row = 0; row < h; row++) {
					for (uint16_t col = 0; col < w - step; col++) {
						setPixel(x + col, y + row, getPixel(x + col + step, y + row));
					}
					for (uint16_t col = w - step; col < w; col++) {
						setPixel(x + col, y + row, fill);
					}
				}
				break;
			case shiftDirection::kRight:
				for (uint16_t row = 0; row < h; row++) {
					for (uint16_t col = w; col > step; col--) {
						setPixel(x + col - 1, y + row, getPixel(x + col - 1 - step, y + row));
					}
					for (uint16_t col = 0; col < step; col++) {
						setPixel(x + col, y + row, fill);
					}
				}
				break;
			case shiftDirection::kUp:
				for (uint16_t col = 0; col < w; col++) {
					for (uint16_t row = 0; row < h - step; row++) {
						setPixel(x + col, y + row, getPixel(x + col, y + row + step));
					}
					for (uint16_t row = h - step; row < h; row++) {
						setPixel(x + col, y + row, fill);
					}
				}
				break;
			case shiftDirection::kDown:
			default:
				for (uint16_t col = 0; col < w; col++) {
					for (uint16_t row = h; row > step; row--) {
						setPixel(x + col, y + row - 1, getPixel(x + col, y + row - 1 - step));
					}
					for (uint16_t row = 0; row < step; row++) {
						setPixel(x + col, y + row, fill);
					}
				}
				break;
		}
	}

	// Maps a §12 drawing primitive's own logical geometry (e.g. DRAW_RECT's raw X/Y/WIDTH/HEIGHT,
	// pre-offset) to the physical rectangle that must actually be flushed/marked dirty - used by
	// main.cpp's finishDraw() to compute REFRESH's affected region, since flush()/markDirty()
	// address the physical panel/buffer directly and must reflect the exact same offset+clip+
	// orientation pipeline every individual pixel write already goes through above (getPixel()/
	// setPixel()), just computed once for the whole bounding box instead of per pixel. Returns
	// false if nothing of the given rectangle survives (fully panned away by SET_DRAW_OFFSET, or
	// fully outside the clip/logical-canvas bounds) - the caller should treat that as a no-op,
	// exactly like WorkingBufferGfx::drawPixel silently dropping every one of a fully-clipped
	// shape's pixels already means there's nothing to flip regardless (design note 47).
	bool computeAffectedPhysicalRegion(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t& px, uint16_t& py,
			uint16_t& pw, uint16_t& ph) const {
		x += offset_.dx;
		y += offset_.dy;
		int32_t x1 = x + w;
		int32_t y1 = y + h;
		x = std::max<int32_t>(x, clip_.x0);
		y = std::max<int32_t>(y, clip_.y0);
		x1 = std::min<int32_t>(x1, std::min<int32_t>(clip_.x1, logicalWidth()));
		y1 = std::min<int32_t>(y1, std::min<int32_t>(clip_.y1, logicalHeight()));
		if (x1 <= x || y1 <= y) {
			return false;
		}
		// (x,y) and (x1-1,y1-1) are opposite corners of the clamped logical rect - every
		// orientation (§12.12) is one of a rectangle's 8 symmetries, which always maps opposite
		// corners to opposite corners, so transforming just these two and taking min/max is exact.
		uint16_t cx0, cy0, cx1, cy1;
		toPhysical(x, y, cx0, cy0);
		toPhysical(x1 - 1, y1 - 1, cx1, cy1);
		px = std::min(cx0, cx1);
		py = std::min(cy0, cy1);
		pw = static_cast<uint16_t>(std::max(cx0, cx1) - px + 1);
		ph = static_cast<uint16_t>(std::max(cy0, cy1) - py + 1);
		return true;
	}

	// Flips [x,y,w,h) of the buffer to the panel. `full`: slow whole-waveform LUT, always visibly
	// affects the entire panel regardless of the requested region (doc/PROTOCOL.md §2.1
	// FLAGS.REFRESH_FULL: "full-panel refresh cycle") and powers the panel down afterward; `!full`:
	// fast region-scoped diff LUT against the "previous" RAM bank, kept on afterward (cheaper for
	// back-to-back partial updates) and immediately resynced (writeImagePartAgain) so the *next*
	// partial diff is computed against what's actually now on screen, not stale data.
	//
	// The SSD1683's RAM window is byte-addressed in X (8px/byte, board::kPartialRefreshGranularityX)
	// - GxEPD2's own writeImagePart() rounds X down and W up to whole bytes internally, but derives
	// the rounded W from the *original* W, not one that already accounts for how far X just shifted
	// left, so a non-byte-aligned x/w (routine here - §12's drawing primitives have no alignment
	// constraint, unlike §7) can end up with its rightmost columns silently left out of the physical
	// write even though they're correct in buffer_. Caught via real-hardware testing: a DRAW_TEXT
	// ending at a non-byte-aligned column had its last character's rightmost column(s) missing from
	// the panel. Fixed by pre-aligning here, before calling into GxEPD2, so its own rounding becomes
	// a no-op and the full requested region is always actually written (a few extra already-correct
	// pixels on the left/right edges get harmlessly re-flushed too).
	void flush(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool full) {
		ensureControllerReady();
		uint16_t alignedX = static_cast<uint16_t>(x - (x % 8));
		uint16_t rightEdge = static_cast<uint16_t>(std::min<uint32_t>(static_cast<uint32_t>(x) + w, kWidth));
		uint16_t alignedRight = static_cast<uint16_t>(std::min<uint32_t>(((rightEdge + 7u) / 8u) * 8u, kWidth));
		uint16_t alignedW = alignedRight > alignedX ? static_cast<uint16_t>(alignedRight - alignedX) : uint16_t{0};

		if (full) {
			display_.epd2.writeImagePartAgain(buffer_.data(), alignedX, y, kWidth, kHeight, alignedX, y, alignedW, h,
					/*invert=*/true, false, false);
			display_.epd2.refresh(false);
			display_.epd2.powerOff();
		} else {
			display_.epd2.writeImagePart(buffer_.data(), alignedX, y, kWidth, kHeight, alignedX, y, alignedW, h,
					/*invert=*/true, false, false);
			display_.epd2.refresh(alignedX, y, alignedW, h);
			if (display_.epd2.hasFastPartialUpdate) {
				display_.epd2.writeImagePartAgain(buffer_.data(), alignedX, y, kWidth, kHeight, alignedX, y, alignedW,
						h, /*invert=*/true, false, false);
			}
		}
		syncPanelSnapshot(alignedX, y, alignedW, h);
	}

	// doc/PROTOCOL.md §8 READ_SCREEN: SOURCE=WORKING_BUFFER/PANEL read the current in-memory buffer
	// (including any writes still deferred via FLAGS.REFRESH_NOW=0) or the content last physically
	// presented, respectively - two genuinely different snapshots, so unlike every §12 accessor
	// above these read raw physical buffer bytes directly (bit=1=BLACK, row-major, MSB-first, same
	// layout as §6/§7) with NO SET_DRAW_OFFSET/SET_ORIENTATION/clip applied: X/Y/WIDTH/HEIGHT here
	// are physical panel coordinates, matching what SCREEN_DATA itself echoes back. No alignment
	// constraint (unlike §7) - packRegion() packs bit-by-bit, correct for any x/w.
	std::vector<uint8_t> readWorkingBufferRegion(uint16_t x, uint16_t y, uint16_t w, uint16_t h) const {
		return packRegion(buffer_, x, y, w, h);
	}

	std::vector<uint8_t> readPanelRegion(uint16_t x, uint16_t y, uint16_t w, uint16_t h) const {
		return packRegion(panelBuffer_, x, y, w, h);
	}

	// doc/PROTOCOL.md §9 CLEAR_ARTIFACTS with FLAGS.RESTORE_CONTENT=0: the degauss cycle leaves the
	// physical panel blank (white) directly against the display driver, bypassing this class
	// entirely (see main.cpp's handleClearArtifacts) - so the "last physically presented" snapshot
	// (panelBuffer_, read by readPanelRegion() above) needs updating to match by hand, since no
	// flush() call happens to do it automatically. (RESTORE_CONTENT=1 instead re-flips the current
	// working buffer via the normal flush() path, which already keeps panelBuffer_ in sync on its
	// own - no separate call needed for that case.)
	void markPanelBlank() { std::fill(panelBuffer_.begin(), panelBuffer_.end(), 0x00); }

	// doc/PROTOCOL.md §12.16 FAST_CLEAR, requested directly: "fast buffer filling with 1 or 0...
	// skipping all clipping and mapping guards". Unlike every other §12 primitive - which all go
	// through SET_CLIP_REGION/SET_DRAW_OFFSET/SET_ORIENTATION via getPixel()/setPixel()/
	// computeAffectedPhysicalRegion() - this writes buffer_'s raw physical bytes directly with a
	// single std::fill, no per-pixel clip/coordinate-transform checks at all. Correct specifically
	// *because* it always covers the entire physical panel: every pixel ends up the same value
	// regardless of what any of those three would otherwise have done, so skipping them changes
	// nothing about the result, only how fast it is to produce.
	void fastClear(uint8_t colorValue) {
		std::fill(buffer_.begin(), buffer_.end(), colorValue == color::kBlack ? uint8_t{0xFF} : uint8_t{0x00});
	}

	// doc/PROTOCOL.md §12.8: REFRESH(MODE=0x00) acts on "the union of bounding boxes deferred since
	// the last refresh" - i.e. only regions written with FLAGS.REFRESH_NOW=0 (a REFRESH_NOW=1 write
	// flushes itself immediately and was never "deferred" in the first place, so must NOT be
	// folded into this tracking).
	void markDirty(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
		uint16_t nx1 = x + w;
		uint16_t ny1 = y + h;
		if (!dirty_.valid) {
			dirty_ = {true, x, y, nx1, ny1};
		} else {
			dirty_.x0 = std::min(dirty_.x0, x);
			dirty_.y0 = std::min(dirty_.y0, y);
			dirty_.x1 = std::max(dirty_.x1, nx1);
			dirty_.y1 = std::max(dirty_.y1, ny1);
		}
	}

	bool hasDirtyRegion() const { return dirty_.valid; }

	void getDirtyRegion(uint16_t& x, uint16_t& y, uint16_t& w, uint16_t& h) const {
		x = dirty_.x0;
		y = dirty_.y0;
		w = dirty_.x1 - dirty_.x0;
		h = dirty_.y1 - dirty_.y0;
	}

	void clearDirty() { dirty_.valid = false; }

private:
	struct DirtyRegion {
		bool valid = false;
		uint16_t x0 = 0, y0 = 0, x1 = 0, y1 = 0;  // [x0,x1) x [y0,y1)
	};

	// [x0,x1) x [y0,y1), in *logical* coordinates (doc/PROTOCOL.md §12.10's own doc above explains
	// why this is never itself transformed when SET_ORIENTATION/SET_DRAW_OFFSET change).
	struct ClipRegion {
		uint16_t x0 = 0, y0 = 0, x1 = 0, y1 = 0;
	};

	struct DrawOffset {
		int16_t dx = 0, dy = 0;
	};

	struct Orientation {
		uint8_t rotation = orientation::kRotate0;
		bool mirrorH = false;
		bool mirrorV = false;
	};

	bool inClipRegion(int32_t x, int32_t y) const {
		return x >= clip_.x0 && x < clip_.x1 && y >= clip_.y0 && y < clip_.y1;
	}

	// Maps an already-offset logical coordinate to its physical buffer position, per the current
	// orientation (§12.12/setOrientation()). Returns false if outside the logical canvas -
	// equivalent to the old "outside panel bounds" check, just against the (possibly axis-swapped)
	// logical dimensions instead of the fixed physical ones; the caller (getPixel()/setPixel()/
	// getPixelUnclipped()/computeAffectedPhysicalRegion()) is expected to have already applied the
	// SET_DRAW_OFFSET pan and (for getPixel/setPixel) the clip-window check before calling this.
	bool toPhysical(int32_t lx, int32_t ly, uint16_t& px, uint16_t& py) const {
		uint16_t lw = logicalWidth();
		uint16_t lh = logicalHeight();
		if (lx < 0 || ly < 0 || lx >= lw || ly >= lh) {
			return false;
		}
		if (orientation_.mirrorH) {
			lx = lw - 1 - lx;
		}
		if (orientation_.mirrorV) {
			ly = lh - 1 - ly;
		}
		switch (orientation_.rotation) {
			case orientation::kRotate90:
				px = static_cast<uint16_t>(kWidth - 1 - ly);
				py = static_cast<uint16_t>(lx);
				break;
			case orientation::kRotate180:
				px = static_cast<uint16_t>(kWidth - 1 - lx);
				py = static_cast<uint16_t>(kHeight - 1 - ly);
				break;
			case orientation::kRotate270:
				px = static_cast<uint16_t>(ly);
				py = static_cast<uint16_t>(kHeight - 1 - lx);
				break;
			case orientation::kRotate0:
			default:
				px = static_cast<uint16_t>(lx);
				py = static_cast<uint16_t>(ly);
				break;
		}
		return true;
	}

	static size_t pixelByteIndex(uint16_t x, uint16_t y) { return y * (kWidth / 8) + x / 8; }
	static uint8_t pixelBitMask(uint16_t x) { return 0x80 >> (x % 8); }

	// Called after flush() actually writes [x,y,w,h) to the panel - keeps panelBuffer_ (the "last
	// physically presented" snapshot behind readPanelRegion()) in sync. Bit-level (not a byte-range
	// memcpy) since flush() itself has no alignment constraint - drawing primitives (unlike
	// PARTIAL_IMAGE_TRANSFER) can flush an arbitrary unaligned region.
	void syncPanelSnapshot(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
		for (uint16_t row = 0; row < h; row++) {
			for (uint16_t col = 0; col < w; col++) {
				uint16_t px = x + col;
				uint16_t py = y + row;
				if (px >= kWidth || py >= kHeight) {
					continue;
				}
				size_t idx = pixelByteIndex(px, py);
				uint8_t mask = pixelBitMask(px);
				if (buffer_[idx] & mask) {
					panelBuffer_[idx] |= mask;
				} else {
					panelBuffer_[idx] &= static_cast<uint8_t>(~mask);
				}
			}
		}
	}

	// Packs a rectangular region of `src` (buffer_ or panelBuffer_) into row-major, MSB-first,
	// bit=1=BLACK bytes (doc/PROTOCOL.md §6's format) for READ_SCREEN (§8) - which explicitly has no
	// alignment constraint (unlike §7), so this must work for arbitrary x/w, unlike write()'s
	// byte-range memcpy. Pixels outside the physical panel read as white (0), matching §8's own doc.
	static std::vector<uint8_t> packRegion(const std::vector<uint8_t>& src, uint16_t x, uint16_t y, uint16_t w,
			uint16_t h) {
		size_t bytesPerRow = (w + 7) / 8;
		std::vector<uint8_t> out(bytesPerRow * h, 0x00);
		for (uint16_t row = 0; row < h; row++) {
			for (uint16_t col = 0; col < w; col++) {
				uint16_t px = x + col;
				uint16_t py = y + row;
				if (px >= kWidth || py >= kHeight) {
					continue;
				}
				if (src[pixelByteIndex(px, py)] & pixelBitMask(px)) {
					out[row * bytesPerRow + col / 8] |= static_cast<uint8_t>(0x80 >> (col % 8));
				}
			}
		}
		return out;
	}

	// Lazily undoes powerDown() - called at the very top of flush(), the one true "about to actually
	// draw something" chokepoint, rather than eagerly when SET_POWER_MODE wakes (doc/PROTOCOL.md §17
	// design note 79/80: "could the lazy screen reinit be done in the update method"). Mirrors
	// GxEPD2's own internal design here - every one of its write functions already lazily calls
	// _InitDisplay() itself if _init_display_done is false, which is exactly what powerDown()'s
	// hibernate() call already arranges; this just extends that same laziness to the two things
	// GxEPD2 has no way to know about on its own: the external VCI rail (board::kPinDisplayPowerCtl,
	// entirely outside GxEPD2's CS/DC/RST/BUSY pins) and reseeding the controller's own "current"/
	// "previous" RAM banks - which its automatic reinit (a real SWRESET) unconditionally clears -
	// from panelBuffer_, this class's own accurate shadow of what's really still on the physical
	// panel. Without this, the first partial update after any reinit would have its diff computed
	// against the controller's just-cleared RAM instead of the panel's actual prior content, risking
	// a visibly wrong/ghosted result specifically in that update's own region (confirmed by reading
	// gdey/GxEPD2_420_GDEY042T81.cpp's _InitDisplay()/hibernate() - not assumed). The resync write
	// itself carries no refresh() call, so nothing is ever visibly affected by it.
	void ensureControllerReady() {
		if (!needsControllerResync_) {
			return;
		}
		if (vccCut_) {
			digitalWrite(board::kPinDisplayPowerCtl, HIGH);
			delay(10);	// SSD1683 datasheet §9.1 step 1: "Supply VCI, Wait 10ms", same as displaySelfTest()
			vccCut_ = false;
		}
		display_.epd2.writeImagePartAgain(panelBuffer_.data(), 0, 0, kWidth, kHeight, 0, 0, kWidth, kHeight,
				/*invert=*/true, false, false);
		needsControllerResync_ = false;
	}

	Display& display_;
	std::vector<uint8_t> buffer_;
	std::vector<uint8_t> panelBuffer_;	// doc/PROTOCOL.md §8 SOURCE=PANEL - see readPanelRegion()
	DirtyRegion dirty_;
	ClipRegion clip_;
	DrawOffset offset_;
	Orientation orientation_;

	// Starts true (not just after an explicit powerDown()) - main.cpp's own boot self-test already
	// leaves the controller hibernating before this class ever does its first real flush(), so the
	// very first post-boot draw exercises exactly the same lazy-resync path a LOW_POWER wake does,
	// rather than relying on a coincidence (both panelBuffer_ and the self-test's own final fill
	// happening to already agree on "blank/white") that would stop holding the moment either side
	// changes.
	bool needsControllerResync_ = true;
	bool vccCut_ = false;
};

}  // namespace crowpanel
