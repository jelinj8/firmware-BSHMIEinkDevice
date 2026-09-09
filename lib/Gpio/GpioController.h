// Generic digital GPIO configure/read/write/pattern-playback (doc/PROTOCOL.md §15). Mirrors the
// StorageManager/MacroPlayer precedent: a small dedicated class owning per-pin state, driven
// non-blockingly from loop() via update() rather than blocking anywhere - the same
// loop()-driven-state-machine rule already used for GPIO_PLAY_PATTERN's pulse-train playback and
// for MacroPlayer.
//
// Only pins listed in the board's kAvailableGpioPins[] (board_*.h) are ever touched - isPinAvailable()
// is the single gate every handler in main.cpp checks before calling anything else here.
#pragma once

#include <Arduino.h>
#include <functional>
#include <map>
#include <vector>

#include "Protocol.h"

namespace crowpanel {

class GpioController {
public:
	// Pushed once per debounced logical level change on a pin configured with
	// ENABLE_CHANGE_EVENTS (doc/PROTOCOL.md §15.4 GPIO_EVENT) - caller wires this to whatever
	// broadcasts a frame to the live transports.
	using ChangeEventCallback = std::function<void(uint8_t pinId, uint8_t value)>;

	GpioController(const int* availablePins, size_t availablePinsCount)
			: availablePins_(availablePins), availablePinsCount_(availablePinsCount) {}

	void setChangeEventCallback(ChangeEventCallback callback) { changeEventCallback_ = std::move(callback); }

	bool isPinAvailable(uint8_t pin) const {
		for (size_t i = 0; i < availablePinsCount_; ++i) {
			if (availablePins_[i] == pin) {
				return true;
			}
		}
		return false;
	}

	// Returns false (and leaves pinState untouched) for a MODE this firmware doesn't implement
	// (PWM_OUTPUT/ANALOG_INPUT/reserved) - caller replies NACK(BAD_PARAMETERS) in that case.
	// Reconfiguring a pin always cancels any GPIO_PLAY_PATTERN in progress on it (doc/PROTOCOL.md
	// §15.5's cancellation rule).
	bool configure(uint8_t pin, uint8_t mode, uint8_t flags) {
		if (mode > gpioMode::kOutput) {
			return false;  // PWM_OUTPUT/ANALOG_INPUT reserved, or a genuinely unknown value
		}
		PinState& state = pins_[pin];
		state.pattern.active = false;
		state.configured = true;
		state.mode = mode;
		state.changeEventsEnabled = (mode != gpioMode::kOutput) && (flags & gpioConfigureFlags::kEnableChangeEvents) != 0;
		switch (mode) {
			case gpioMode::kInput:
				pinMode(pin, INPUT);
				break;
			case gpioMode::kInputPullup:
				pinMode(pin, INPUT_PULLUP);
				break;
			case gpioMode::kInputPulldown:
				pinMode(pin, INPUT_PULLDOWN);
				break;
			case gpioMode::kOutput:
			default:
				pinMode(pin, OUTPUT);
				digitalWrite(pin, LOW);
				break;
		}
		state.lastValue = (mode == gpioMode::kOutput) ? 0 : readLevel(pin);
		state.pendingValue = state.lastValue;
		state.pendingSinceMs = millis();
		return true;
	}

	// Only valid for a pin currently configured OUTPUT. A manual write always wins over (cancels)
	// an in-progress GPIO_PLAY_PATTERN on the same pin.
	bool write(uint8_t pin, uint8_t value) {
		auto it = pins_.find(pin);
		if (it == pins_.end() || !it->second.configured || it->second.mode != gpioMode::kOutput) {
			return false;
		}
		it->second.pattern.active = false;
		digitalWrite(pin, value ? HIGH : LOW);
		it->second.lastValue = value ? 1 : 0;
		return true;
	}

	// Live input level for an INPUT* pin, or the last-driven level for an OUTPUT pin (doc/PROTOCOL.md
	// §15.3). Returns false if the pin was never configured.
	bool read(uint8_t pin, uint8_t& value, uint8_t& mode) const {
		auto it = pins_.find(pin);
		if (it == pins_.end() || !it->second.configured) {
			return false;
		}
		mode = it->second.mode;
		value = (it->second.mode == gpioMode::kOutput) ? it->second.lastValue : readLevel(pin);
		return true;
	}

	// Starts (or replaces) a non-blocking pulse train on an OUTPUT pin, doc/PROTOCOL.md §15.5.
	// stepDurationsMs must be non-empty; the pin always ends LOW once the sequence (and all
	// repeats) finish.
	bool playPattern(uint8_t pin, uint8_t flags, std::vector<uint16_t> stepDurationsMs, uint8_t repeatCount) {
		if (stepDurationsMs.empty()) {
			return false;
		}
		auto it = pins_.find(pin);
		if (it == pins_.end() || !it->second.configured || it->second.mode != gpioMode::kOutput) {
			return false;
		}
		Pattern& pattern = it->second.pattern;
		pattern.stepDurationsMs = std::move(stepDurationsMs);
		pattern.stepIndex = 0;
		pattern.currentLevel = (flags & gpioPatternFlags::kInitialLevelHigh) ? 1 : 0;
		pattern.repeatForever = (flags & gpioPatternFlags::kRepeatForever) != 0;
		pattern.repeatsRemaining = pattern.repeatForever ? 0 : repeatCount;
		pattern.active = true;
		digitalWrite(pin, pattern.currentLevel ? HIGH : LOW);
		it->second.lastValue = pattern.currentLevel;
		pattern.nextTransitionMs = millis() + pattern.stepDurationsMs[0];
		return true;
	}

	// Call frequently from loop(): advances any in-progress patterns and edge-detects/pushes
	// GPIO_EVENT for pins with ENABLE_CHANGE_EVENTS set.
	void update() {
		unsigned long now = millis();
		for (auto& entry : pins_) {
			uint8_t pin = entry.first;
			PinState& state = entry.second;
			if (!state.configured) {
				continue;
			}
			if (state.pattern.active) {
				stepPattern(pin, state, now);
			}
			if (state.mode != gpioMode::kOutput && state.changeEventsEnabled) {
				pollChangeEvent(pin, state, now);
			}
		}
	}

private:
	struct Pattern {
		bool active = false;
		std::vector<uint16_t> stepDurationsMs;
		size_t stepIndex = 0;
		uint8_t currentLevel = 0;
		bool repeatForever = false;
		uint8_t repeatsRemaining = 0;
		unsigned long nextTransitionMs = 0;
	};

	struct PinState {
		bool configured = false;
		uint8_t mode = gpioMode::kInput;
		bool changeEventsEnabled = false;
		uint8_t lastValue = 0;
		// Simple debounce: a candidate level must hold for kDebounceMs before it's accepted as the
		// new lastValue and reported via GPIO_EVENT - avoids a mechanical switch/button firing a
		// burst of spurious events on contact bounce.
		uint8_t pendingValue = 0;
		unsigned long pendingSinceMs = 0;
		Pattern pattern;
	};

	static constexpr unsigned long kDebounceMs = 30;

	static uint8_t readLevel(uint8_t pin) { return digitalRead(pin) == HIGH ? 1 : 0; }

	void pollChangeEvent(uint8_t pin, PinState& state, unsigned long now) {
		uint8_t level = readLevel(pin);
		if (level != state.pendingValue) {
			state.pendingValue = level;
			state.pendingSinceMs = now;
			return;
		}
		if (level != state.lastValue && (now - state.pendingSinceMs) >= kDebounceMs) {
			state.lastValue = level;
			if (changeEventCallback_) {
				changeEventCallback_(pin, level);
			}
		}
	}

	void stepPattern(uint8_t pin, PinState& state, unsigned long now) {
		Pattern& pattern = state.pattern;
		if (now < pattern.nextTransitionMs) {
			return;
		}
		pattern.stepIndex++;
		if (pattern.stepIndex >= pattern.stepDurationsMs.size()) {
			if (pattern.repeatForever) {
				pattern.stepIndex = 0;
			} else if (pattern.repeatsRemaining > 0) {
				pattern.repeatsRemaining--;
				pattern.stepIndex = 0;
			} else {
				pattern.active = false;
				digitalWrite(pin, LOW);
				state.lastValue = 0;
				return;
			}
		}
		pattern.currentLevel = pattern.currentLevel ? 0 : 1;
		digitalWrite(pin, pattern.currentLevel ? HIGH : LOW);
		state.lastValue = pattern.currentLevel;
		pattern.nextTransitionMs = now + pattern.stepDurationsMs[pattern.stepIndex];
	}

	const int* availablePins_;
	size_t availablePinsCount_;
	std::map<uint8_t, PinState> pins_;
	ChangeEventCallback changeEventCallback_;
};

}  // namespace crowpanel
