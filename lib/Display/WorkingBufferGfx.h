// Adapts WorkingBuffer to Adafruit_GFX so doc/PROTOCOL.md §12's local drawing primitives
// (DRAW_LINE/RECT/CIRCLE) can reuse Adafruit_GFX's well-tested Bresenham line and midpoint-circle
// rasterizers instead of hand-rolling them. Only drawPixel() needs to know about DRAW_MODE
// compositing (§12.1) - every other Adafruit_GFX primitive (drawLine, drawRect, fillRect,
// drawCircle, fillCircle, ...) funnels through drawPixel()/writePixel() already (confirmed by
// reading Adafruit_GFX.cpp: none of them are separately overridden here, so none bypass it).
#pragma once

#include <Adafruit_GFX.h>

#include "Protocol.h"
#include "WorkingBuffer.h"

namespace crowpanel {

class WorkingBufferGfx : public Adafruit_GFX {
public:
	explicit WorkingBufferGfx(WorkingBuffer& buffer)
			: Adafruit_GFX(WorkingBuffer::kWidth, WorkingBuffer::kHeight), buffer_(buffer) {}

	// Applies to every draw call until changed again - set once per command handler before issuing
	// its Adafruit_GFX call(s), since Adafruit_GFX's drawPixel(x,y,color) signature has no room for
	// an extra mode parameter.
	void setDrawMode(uint8_t mode) { drawMode_ = mode; }

	void drawPixel(int16_t x, int16_t y, uint16_t color) override {
		// WorkingBuffer::compositePixel() takes signed logical coordinates and already handles
		// negative/out-of-canvas values gracefully (SET_DRAW_OFFSET, §12.13, can legitimately push
		// a logical coordinate negative before it's rejected) - no separate guard needed here.
		// Callers always pass color::kWhite/kBlack (0/1); `!= 0` (rather than `== color::kBlack`)
		// sidesteps the parameter name shadowing the crowpanel::color namespace.
		buffer_.compositePixel(x, y, color != 0, drawMode_);
	}

private:
	WorkingBuffer& buffer_;
	uint8_t drawMode_ = drawMode::kReplace;
};

}  // namespace crowpanel
