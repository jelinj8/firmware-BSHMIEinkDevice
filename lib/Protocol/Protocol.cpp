#include "Protocol.h"

#include "Crc16.h"

namespace crowpanel {

namespace {
// Little-endian helpers - explicit rather than relying on struct-packing/host endianness, so
// this stays correct regardless of the target's native byte order.
void putU16LE(std::vector<uint8_t>& buf, uint16_t v) {
	buf.push_back(static_cast<uint8_t>(v & 0xFF));
	buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

void putU32LE(std::vector<uint8_t>& buf, uint32_t v) {
	buf.push_back(static_cast<uint8_t>(v & 0xFF));
	buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
	buf.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
	buf.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
}

uint16_t getU16LE(const uint8_t* p) {
	return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

uint32_t getU32LE(const uint8_t* p) {
	return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
			(static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
}  // namespace

std::vector<uint8_t> Frame::encode() const {
	std::vector<uint8_t> buf;
	buf.reserve(kFrameHeaderSize + payload.size() + kFrameCrcSize);
	buf.push_back(kFrameMagic);
	buf.push_back(version);
	putU16LE(buf, commandId);
	buf.push_back(seq);
	putU32LE(buf, static_cast<uint32_t>(payload.size()));
	buf.insert(buf.end(), payload.begin(), payload.end());

	// CRC covers VERSION..end of PAYLOAD, i.e. everything after MAGIC.
	uint16_t crc = crc16(buf.data() + 1, buf.size() - 1);
	putU16LE(buf, crc);
	return buf;
}

FrameError Frame::decode(const uint8_t* bytes, size_t length, Frame& out) {
	if (length < kFrameHeaderSize + kFrameCrcSize) {
		return FrameError::kTooShort;
	}
	if (bytes[0] != kFrameMagic) {
		return FrameError::kBadMagic;
	}
	uint8_t version = bytes[1];
	uint16_t commandId = getU16LE(bytes + 2);
	uint8_t seq = bytes[4];
	uint32_t payloadLen = getU32LE(bytes + 5);

	size_t expectedTotal = kFrameHeaderSize + static_cast<size_t>(payloadLen) + kFrameCrcSize;
	if (expectedTotal != length) {
		return FrameError::kLengthMismatch;
	}

	uint16_t crcComputed = crc16(bytes + 1, kFrameHeaderSize - 1 + payloadLen);
	uint16_t crcReceived = getU16LE(bytes + kFrameHeaderSize + payloadLen);
	if (crcComputed != crcReceived) {
		return FrameError::kCrcMismatch;
	}

	out.version = version;
	out.commandId = commandId;
	out.seq = seq;
	out.payload.assign(bytes + kFrameHeaderSize, bytes + kFrameHeaderSize + payloadLen);
	return FrameError::kNone;
}

}  // namespace crowpanel
