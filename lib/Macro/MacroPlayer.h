// doc/PROTOCOL.md §0x0A00 PLAY_MACRO/PAUSE: non-blocking macro playback. `next()` must be called
// every loop() iteration (main.cpp's stepMacroPlayback()) and whatever it returns dispatched
// exactly like a live command - the same "timed/sequenced operation is a loop()-driven state
// machine, never a blocking delay() inside a command handler" rule already noted for
// GPIO_PLAY_PATTERN (plan.md Phase 3), since the other transports must keep being polled
// throughout a macro's run, however long its PAUSE entries make that take.
#pragma once

#include <Arduino.h>

#include <cstdint>
#include <iterator>
#include <vector>

namespace crowpanel {

struct MacroEntry {
	uint16_t commandId;
	std::vector<uint8_t> payload;
};

class MacroPlayer {
public:
	bool isPlaying() const { return playing_; }

	// Parses a raw .macro file's bytes (MAGIC+FORMAT_VERSION+entries, the same layout
	// MacroRecorder::stop() produces) into `outEntries`. Returns false (leaving `outEntries`
	// unspecified) on a malformed file.
	static bool parse(const std::vector<uint8_t>& fileData, std::vector<MacroEntry>& outEntries) {
		constexpr size_t kHeaderSize = 5;
		if (fileData.size() < kHeaderSize || fileData[0] != 'M' || fileData[1] != 'A' || fileData[2] != 'C' ||
				fileData[3] != '1' || fileData[4] != 0x01) {
			return false;
		}
		outEntries.clear();
		size_t pos = kHeaderSize;
		while (pos < fileData.size()) {
			if (pos + 6 > fileData.size()) {
				return false;
			}
			uint16_t commandId =
					static_cast<uint16_t>(fileData[pos]) | (static_cast<uint16_t>(fileData[pos + 1]) << 8);
			uint32_t payloadLen = static_cast<uint32_t>(fileData[pos + 2]) |
					(static_cast<uint32_t>(fileData[pos + 3]) << 8) |
					(static_cast<uint32_t>(fileData[pos + 4]) << 16) |
					(static_cast<uint32_t>(fileData[pos + 5]) << 24);
			pos += 6;
			if (pos + payloadLen > fileData.size()) {
				return false;
			}
			MacroEntry entry;
			entry.commandId = commandId;
			entry.payload.assign(fileData.begin() + static_cast<long>(pos),
					fileData.begin() + static_cast<long>(pos + payloadLen));
			outEntries.push_back(std::move(entry));
			pos += payloadLen;
		}
		return true;
	}

	void start(std::vector<MacroEntry> entries) {
		entries_ = std::move(entries);
		cursor_ = 0;
		playing_ = true;
		pauseUntilMs_ = 0;
	}

	// Starts playback if idle, exactly like start(). If a playback is already in progress, appends
	// `entries` to the tail of the currently-playing sequence instead of replacing it - the
	// event-triggered auto-play path (main.cpp's tryAutoPlayEventMacro) uses this so a second
	// trigger firing mid-playback plays after the first finishes, rather than being dropped or
	// clobbering it. Safe across the vector reallocation this append can cause: cursor_ is a plain
	// index, re-resolved fresh on every next() call, never a cached pointer/iterator into entries_.
	void enqueue(std::vector<MacroEntry> entries) {
		if (!playing_) {
			start(std::move(entries));
			return;
		}
		entries_.insert(entries_.end(), std::make_move_iterator(entries.begin()),
				std::make_move_iterator(entries.end()));
	}

	void stop() {
		playing_ = false;
		entries_.clear();
		cursor_ = 0;
	}

	// Called by handlePause() (main.cpp) when a PAUSE entry is dispatched during playback - delays
	// the next next() call from returning an entry until `durationMs` has elapsed.
	void pauseFor(uint32_t durationMs) { pauseUntilMs_ = millis() + durationMs; }

	// Returns the next entry to dispatch, or nullptr if playback shouldn't advance yet (still
	// paused, or finished/idle) - advances the internal cursor only when it does return an entry.
	const MacroEntry* next() {
		if (!playing_) {
			return nullptr;
		}
		if (pauseUntilMs_ != 0) {
			if (millis() < pauseUntilMs_) {
				return nullptr;
			}
			pauseUntilMs_ = 0;
		}
		if (cursor_ >= entries_.size()) {
			stop();
			return nullptr;
		}
		return &entries_[cursor_++];
	}

private:
	std::vector<MacroEntry> entries_;
	size_t cursor_ = 0;
	bool playing_ = false;
	uint32_t pauseUntilMs_ = 0;
};

}  // namespace crowpanel
