// Command dispatch table (doc/PROTOCOL.md §4): maps COMMAND_ID -> handler, shared across all three
// transports. Replaces main.cpp's original bring-up stub (a single
// `if (commandId == kHandshakeRequest) ... else NACK`) - every later feature (drawing, storage,
// GPIO, OTA, config, power) registers a handler here instead of growing one giant function.
//
// Depends on StreamFrameTransport (Arduino Stream-based), so - unlike Protocol.h - this is not
// meant to be usable from the native/host unit-test build; it's firmware-only glue.
#pragma once

#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Protocol.h"
#include "StreamFrameTransport.h"

namespace crowpanel {

// Passed to every registered handler: what arrived, which transport it arrived on (doc/PROTOCOL.md
// §5.2 ACTIVE_TRANSPORT), the access level this connection currently holds (doc/PROTOCOL.md §5.3 -
// already checked against the handler's own required level by dispatch() below; a handler only
// ever sees this to layer an *additional*, more specific check, e.g. SET_POWER_MODE's
// transport-conditional HARD_SLEEP rule), and where/how to reply.
struct CommandContext {
	StreamFrameTransport& transport;
	uint8_t activeTransportValue;
	const Frame& request;
	uint8_t authLevel;

	// doc/PROTOCOL.md §10: ACK and NACK share one payload shape (REF_SEQ, REF_COMMAND_ID, STATUS) -
	// ack() is just nack() with STATUS=OK. REF_SEQ/REF_COMMAND_ID are always the request's own -
	// handlers never assemble these by hand.
	void ack() const { reply(cmd::kAck, refPayload(status::kOk)); }

	void nack(uint8_t statusCode) const { reply(cmd::kNack, refPayload(statusCode)); }

	// For commands whose own response IS the stop-and-wait unblock (§10) - e.g.
	// HANDSHAKE_REQUEST -> HANDSHAKE_RESPONSE - and thus need no separate ACK/NACK.
	void reply(uint16_t responseCommandId, std::vector<uint8_t> payload) const {
		Frame response;
		response.commandId = responseCommandId;
		response.seq = request.seq;  // placeholder pairing until real per-direction SEQ tracking exists
		response.payload = std::move(payload);
		transport.send(response);
	}

private:
	std::vector<uint8_t> refPayload(uint8_t statusCode) const {
		return {request.seq, static_cast<uint8_t>(request.commandId & 0xFF),
				static_cast<uint8_t>((request.commandId >> 8) & 0xFF), statusCode};
	}
};

using CommandHandler = std::function<void(const CommandContext&)>;

class Dispatcher {
public:
	// doc/PROTOCOL.md §5.3: the cascading-fallback rule for "what level does a connection actually
	// need to meet a command's own requiredLevel, given which PIN tiers are currently configured" -
	// a higher tier never becomes *more* open just because its own PIN wasn't set (configuring any
	// PIN expresses real intent to restrict the device). A pure function of its inputs - no
	// dependency on Preferences/globals - so both dispatch() below and main.cpp's own
	// SET_POWER_MODE HARD_SLEEP check call the exact same logic rather than risking two copies
	// drifting apart on a security-relevant rule.
	static uint8_t effectiveRequiredLevel(uint8_t requiredLevel, bool usagePinConfigured, bool adminPinConfigured) {
		if (requiredLevel == authLevel::kAdmin) {
			if (adminPinConfigured) {
				return authLevel::kAdmin;
			}
			return usagePinConfigured ? authLevel::kUsage : authLevel::kNone;
		}
		if (requiredLevel == authLevel::kUsage) {
			return usagePinConfigured ? authLevel::kUsage : authLevel::kNone;
		}
		return authLevel::kNone;
	}

	// Last registration for a given commandId wins - intentionally unchecked (no duplicate-
	// registration assert) since handlers are all registered once, at setup() time. `requiredLevel`
	// defaults to kNone (no gate) - only commands explicitly registered with a higher level are
	// restricted.
	void registerHandler(uint16_t commandId, CommandHandler handler, uint8_t requiredLevel = authLevel::kNone) {
		handlers_[commandId] = Entry{std::move(handler), requiredLevel};
	}

	// doc/PROTOCOL.md §4: "Unknown COMMAND_ID -> NACK(UNSUPPORTED_COMMAND), never silently dropped" -
	// enforced here once so no individual handler (or a commandId nobody registered yet) can forget
	// it. doc/PROTOCOL.md §5.3's access-control gate is enforced here too, for the same reason - one
	// central check a new handler can't accidentally skip, rather than 50 individual call sites each
	// remembering to check `ctx.authLevel` themselves. `usagePinConfigured`/`adminPinConfigured` are
	// passed in fresh per call (not stored) - see effectiveRequiredLevel's own doc for why.
	void dispatch(StreamFrameTransport& transport, uint8_t activeTransportValue, const Frame& frame,
			uint8_t grantedLevel, bool usagePinConfigured, bool adminPinConfigured) const {
		CommandContext ctx{transport, activeTransportValue, frame, grantedLevel};
		auto it = handlers_.find(frame.commandId);
		if (it == handlers_.end()) {
			ctx.nack(status::kUnsupportedCommand);
			return;
		}
		uint8_t required = effectiveRequiredLevel(it->second.requiredLevel, usagePinConfigured, adminPinConfigured);
		if (grantedLevel < required) {
			ctx.nack(status::kNotAuthorized);
			return;
		}
		it->second.handler(ctx);
	}

private:
	struct Entry {
		CommandHandler handler;
		uint8_t requiredLevel;
	};

	std::unordered_map<uint16_t, Entry> handlers_;
};

}  // namespace crowpanel
