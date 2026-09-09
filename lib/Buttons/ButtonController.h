// Physical button press/release/long-press detection (doc/PROTOCOL.md §11), pushing BUTTON_EVENT.
// Mirrors GpioController's debounced-polling shape (§15's ChangeEventCallback) rather than sharing
// code with it directly - GPIO_EVENT reports a raw level change for a PC-configured pin, while
// BUTTON_EVENT reports PRESS/RELEASE/LONG_PRESS for a fixed, firmware-known set of buttons, which
// needs its own small per-button state machine (press-start timestamp, one-shot long-press flag).
#pragma once

#include <Arduino.h>
#include <functional>
#include <map>
#include <vector>

#include "Protocol.h"

namespace crowpanel {

class ButtonController {
public:
	// Pushed for every PRESS, RELEASE, and (once per press, after the hold threshold) LONG_PRESS -
	// caller wires this to whatever broadcasts a frame to the live transports.
	using EventCallback = std::function<void(uint8_t buttonId, uint8_t eventType)>;

	struct ButtonDef {
		int pin;
		uint8_t buttonId;
	};

	explicit ButtonController(std::vector<ButtonDef> buttons) : buttons_(std::move(buttons)) {}

	void setEventCallback(EventCallback callback) { callback_ = std::move(callback); }

	// The already-debounced current state for a listed button - doc/PROTOCOL.md §5.3's MENU+BACK
	// handshake override reads this rather than a raw digitalRead() (which would duplicate the
	// active-LOW polarity knowledge readPressed() already owns, and could catch a transient bounce).
	// false for a buttonId that was never registered with this controller.
	bool isPressed(uint8_t buttonId) const {
		auto it = states_.find(buttonId);
		return it != states_.end() && it->second.pressed;
	}

	// Call once from setup(), after Serial/display bring-up. Every listed pin is driven
	// INPUT_PULLUP regardless of whether the board schematic already has an external pull-up -
	// combining both is harmless and guarantees a defined idle-HIGH level either way.
	void begin() {
		unsigned long now = millis();
		for (const ButtonDef& def : buttons_) {
			pinMode(def.pin, INPUT_PULLUP);
			State& state = states_[def.buttonId];
			state.pin = def.pin;
			bool pressed = readPressed(def.pin);
			state.pressed = pressed;
			state.pendingPressed = pressed;
			state.pendingSinceMs = now;
		}
	}

	// Call frequently from loop(): debounces every button and fires PRESS/RELEASE/LONG_PRESS.
	void update() {
		unsigned long now = millis();
		for (auto& entry : states_) {
			uint8_t buttonId = entry.first;
			State& state = entry.second;
			bool pressed = readPressed(state.pin);
			if (pressed != state.pendingPressed) {
				state.pendingPressed = pressed;
				state.pendingSinceMs = now;
				continue;
			}
			if (pressed != state.pressed && (now - state.pendingSinceMs) >= kDebounceMs) {
				state.pressed = pressed;
				if (pressed) {
					state.pressStartMs = now;
					state.longPressFired = false;
					fire(buttonId, buttonEventType::kPress);
				} else {
					fire(buttonId, buttonEventType::kRelease);
					// doc/PROTOCOL.md §11 SHORT_PRESS: fires right after RELEASE whenever this cycle
					// never crossed the long-press threshold - the common "quick tap" case.
					if (!state.longPressFired) {
						fire(buttonId, buttonEventType::kShortPress);
					}
				}
			}
			if (state.pressed && !state.longPressFired && (now - state.pressStartMs) >= kLongPressThresholdMs) {
				state.longPressFired = true;
				fire(buttonId, buttonEventType::kLongPress);
			}
		}
	}

private:
	struct State {
		int pin = -1;
		bool pressed = false;
		bool pendingPressed = false;
		unsigned long pendingSinceMs = 0;
		unsigned long pressStartMs = 0;
		bool longPressFired = false;
	};

	static constexpr unsigned long kDebounceMs = 30;
	static constexpr unsigned long kLongPressThresholdMs = 800;

	// Wired active-LOW (pin -> switch -> GND): pressed reads LOW.
	static bool readPressed(int pin) { return digitalRead(pin) == LOW; }

	void fire(uint8_t buttonId, uint8_t eventType) {
		if (callback_) {
			callback_(buttonId, eventType);
		}
	}

	std::vector<ButtonDef> buttons_;
	std::map<uint8_t, State> states_;
	EventCallback callback_;
};

}  // namespace crowpanel
