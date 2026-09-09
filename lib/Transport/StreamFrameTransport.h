// Byte-stream frame transport (doc/PROTOCOL.md §3.2/§3.3): TCP and Serial framing are identical -
// no chunking, PAYLOAD_LEN in the Logical Frame header is already a length prefix. Works over
// anything that implements Arduino's Stream interface, so the exact same code drives both
// HardwareSerial (Serial transport) and WiFiClient (TCP transport) - mirrors pc-java-lib's
// FrameStreamReader/AbstractStreamFrameTransport split (same resync policy, see there for the
// rationale).
//
// Unlike the Java side (which has real blocking reader threads), this is written for Arduino's
// single-threaded loop() model: poll() is non-blocking and should be called frequently from
// loop() - it consumes whatever bytes are currently available and dispatches any complete,
// CRC-valid frames to the handler, then returns immediately.
#pragma once

#include <Arduino.h>
#include <functional>
#include <vector>

#include "Protocol.h"

namespace crowpanel {

class StreamFrameTransport {
public:
	using FrameHandler = std::function<void(const Frame&)>;

	explicit StreamFrameTransport(Stream& stream) : stream_(stream) {}

	void setHandler(FrameHandler handler) { handler_ = std::move(handler); }

	// Non-blocking: reads whatever bytes are currently available and dispatches every complete
	// frame found. Call this frequently from loop().
	void poll() {
		while (stream_.available() > 0) {
			buffer_.push_back(static_cast<uint8_t>(stream_.read()));
		}
		while (tryDispatchOne()) {
			// keep draining - more than one frame may already be fully buffered
		}
	}

	// Encodes and writes one frame immediately, looping on short writes rather than assuming one
	// write() call queues everything. HardwareSerial/WiFiClient's write() already loops/blocks
	// internally so this was previously harmless for them, but NimBLEStream::write() (BLE) is a
	// non-blocking ring-buffer write that returns the actual (possibly smaller) count queued -
	// silently dropping the untransmitted tail without this loop. Bounded by a progress watchdog
	// so a stuck/gone peer can't hang the whole firmware loop() forever.
	void send(const Frame& frame) {
		std::vector<uint8_t> wire = frame.encode();
		size_t offset = 0;
		unsigned long deadline = millis() + kSendStallTimeoutMs;
		while (offset < wire.size()) {
			size_t written = stream_.write(wire.data() + offset, wire.size() - offset);
			offset += written;
			if (offset >= wire.size()) {
				break;
			}
			if (written == 0) {
				stream_.flush();  // nudge a full TX buffer to drain before retrying
				if (millis() > deadline) {
					break;  // no progress for too long - peer likely gone; drop the rest
				}
			} else {
				deadline = millis() + kSendStallTimeoutMs;  // made progress, reset the watchdog
			}
		}
	}

private:
	static constexpr size_t kMaxReasonablePayload = 8 * 1024 * 1024;
	static constexpr unsigned long kSendStallTimeoutMs = 2000;

	Stream& stream_;
	FrameHandler handler_;
	std::vector<uint8_t> buffer_;

	// Attempts to parse+dispatch one frame from the front of buffer_. Returns true if it did
	// (caller should try again - more frames may be queued), false if buffer_ doesn't currently
	// hold a complete frame (caller should stop and wait for poll() to add more bytes).
	bool tryDispatchOne() {
		while (true) {
			size_t skip = 0;
			while (skip < buffer_.size() && buffer_[skip] != kFrameMagic) {
				skip++;
			}
			if (skip > 0) {
				buffer_.erase(buffer_.begin(), buffer_.begin() + skip);
			}
			if (buffer_.size() < kFrameHeaderSize) {
				return false;
			}

			uint32_t payloadLen = static_cast<uint32_t>(buffer_[5]) | (static_cast<uint32_t>(buffer_[6]) << 8) |
					(static_cast<uint32_t>(buffer_[7]) << 16) | (static_cast<uint32_t>(buffer_[8]) << 24);
			if (payloadLen > kMaxReasonablePayload) {
				buffer_.erase(buffer_.begin());  // drop just the MAGIC byte, keep resyncing
				continue;
			}

			size_t total = kFrameHeaderSize + payloadLen + kFrameCrcSize;
			if (buffer_.size() < total) {
				return false;
			}

			Frame frame;
			FrameError err = Frame::decode(buffer_.data(), total, frame);
			if (err == FrameError::kNone) {
				buffer_.erase(buffer_.begin(), buffer_.begin() + total);
				if (handler_) {
					handler_(frame);
				}
				return true;
			}
			// CRC (or other) failure on a candidate whose header looked plausible - the declared
			// length can't be trusted either, so only drop the MAGIC byte and rescan.
			buffer_.erase(buffer_.begin());
		}
	}
};

}  // namespace crowpanel
