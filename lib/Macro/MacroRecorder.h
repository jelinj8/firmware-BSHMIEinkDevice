// doc/PROTOCOL.md §0x0A00 RECORD_MACRO/SAVE_MACRO: captures dispatched command frames into an
// in-memory buffer while a recording is active. RECORD_MACRO starts it; SAVE_MACRO stops it and
// persists the finished buffer as a real file via StorageManager (any VOLUME, including PSRAM
// itself - "persisting" there just turns it into a queryable/downloadable named file instead of
// this recorder's own private in-flight buffer). What actually gets captured (which commands are
// recordable at all - the macro meta-commands themselves are excluded) is main.cpp's policy, not
// this class's - this class just appends whatever it's told to.
#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

namespace crowpanel {

class MacroRecorder {
public:
	bool isRecording() const { return recording_; }

	void start() {
		recording_ = true;
		buffer_.clear();
	}

	// Appends one entry in the .macro file format (doc/PROTOCOL.md §0x0A00): COMMAND_ID(2 LE) +
	// PAYLOAD_LEN(4 LE) + PAYLOAD.
	void record(uint16_t commandId, const std::vector<uint8_t>& payload) {
		size_t offset = buffer_.size();
		buffer_.resize(offset + 6 + payload.size());
		buffer_[offset] = static_cast<uint8_t>(commandId & 0xFF);
		buffer_[offset + 1] = static_cast<uint8_t>((commandId >> 8) & 0xFF);
		uint32_t len = static_cast<uint32_t>(payload.size());
		buffer_[offset + 2] = static_cast<uint8_t>(len & 0xFF);
		buffer_[offset + 3] = static_cast<uint8_t>((len >> 8) & 0xFF);
		buffer_[offset + 4] = static_cast<uint8_t>((len >> 16) & 0xFF);
		buffer_[offset + 5] = static_cast<uint8_t>((len >> 24) & 0xFF);
		if (!payload.empty()) {
			std::memcpy(buffer_.data() + offset + 6, payload.data(), payload.size());
		}
	}

	// Stops recording and returns the finished file (MAGIC+FORMAT_VERSION+entries) for the caller
	// to persist via StorageManager::upload(). Always resets to idle, regardless of what the caller
	// does with the returned bytes.
	std::vector<uint8_t> stop() {
		recording_ = false;
		std::vector<uint8_t> result;
		result.reserve(5 + buffer_.size());
		result.push_back('M');
		result.push_back('A');
		result.push_back('C');
		result.push_back('1');
		result.push_back(0x01);  // FORMAT_VERSION
		result.insert(result.end(), buffer_.begin(), buffer_.end());
		buffer_.clear();
		return result;
	}

private:
	bool recording_ = false;
	std::vector<uint8_t> buffer_;
};

}  // namespace crowpanel
