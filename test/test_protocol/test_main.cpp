// Native (no-ESP32-toolchain) unit tests for the transport-independent protocol logic. Run with:
//   pio test -e native
//
// Mirrors pc-java-lib's Crc16Test / RlePackBitsTest / FrameTest so both implementations are
// checked against the same expectations (in particular the shared CRC16 test vector).
#include <unity.h>

#include <cstring>
#include <stdexcept>

#include "Crc16.h"
#include "Protocol.h"
#include "RlePackBits.h"

using namespace crowpanel;

// ---- Crc16 --------------------------------------------------------------------------------

void test_crc16_check_vector(void) {
	const uint8_t data[] = "123456789";
	TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16(data, 9));
}

void test_crc16_empty_is_init_value(void) {
	TEST_ASSERT_EQUAL_HEX16(0xFFFF, crc16(nullptr, 0));
}

// ---- RlePackBits ----------------------------------------------------------------------------

void test_rle_round_trips_all_zeros(void) {
	std::vector<uint8_t> data(15000, 0);
	auto encoded = rleEncode(data.data(), data.size());
	auto decoded = rleDecode(encoded.data(), encoded.size(), data.size());
	TEST_ASSERT_EQUAL(data.size(), decoded.size());
	TEST_ASSERT_EQUAL_UINT8_ARRAY(data.data(), decoded.data(), data.size());
}

void test_rle_round_trips_random_bytes(void) {
	std::vector<uint8_t> data(15000);
	uint32_t x = 42;  // small xorshift PRNG - deterministic, no <random> dependency needed
	for (auto& b : data) {
		x ^= x << 13;
		x ^= x >> 17;
		x ^= x << 5;
		b = static_cast<uint8_t>(x);
	}
	auto encoded = rleEncode(data.data(), data.size());
	auto decoded = rleDecode(encoded.data(), encoded.size(), data.size());
	TEST_ASSERT_EQUAL_UINT8_ARRAY(data.data(), decoded.data(), data.size());
}

void test_rle_encodes_long_run_compactly(void) {
	std::vector<uint8_t> data(15000, 0xFF);
	auto encoded = rleEncode(data.data(), data.size());
	// Repeat runs cap at 129 bytes/packet (2 wire bytes each): ceil(15000/129) * 2 = 234.
	TEST_ASSERT_EQUAL(234, static_cast<int>(encoded.size()));
}

void test_rle_decode_rejects_overrun(void) {
	const uint8_t malformed[] = {127};  // claims a 128-byte literal run but supplies none
	bool threw = false;
	try {
		rleDecode(malformed, 1, 128);
	} catch (const std::invalid_argument&) {
		threw = true;
	}
	TEST_ASSERT_TRUE(threw);
}

// ---- Frame ----------------------------------------------------------------------------------

void test_frame_round_trips_with_payload(void) {
	Frame frame;
	frame.commandId = cmd::kDrawText;
	frame.seq = 200;
	frame.payload = {'h', 'i'};

	auto encoded = frame.encode();
	TEST_ASSERT_EQUAL(kFrameHeaderSize + 2 + kFrameCrcSize, encoded.size());
	TEST_ASSERT_EQUAL_HEX8(kFrameMagic, encoded[0]);

	Frame decoded;
	FrameError err = Frame::decode(encoded.data(), encoded.size(), decoded);
	TEST_ASSERT_TRUE(err == FrameError::kNone);
	TEST_ASSERT_EQUAL_HEX16(cmd::kDrawText, decoded.commandId);
	TEST_ASSERT_EQUAL(200, decoded.seq);
	TEST_ASSERT_EQUAL(2, decoded.payload.size());
	TEST_ASSERT_EQUAL('h', decoded.payload[0]);
	TEST_ASSERT_EQUAL('i', decoded.payload[1]);
}

void test_frame_rejects_bad_magic(void) {
	Frame frame;
	frame.commandId = cmd::kAck;
	auto encoded = frame.encode();
	encoded[0] = 0x00;

	Frame decoded;
	FrameError err = Frame::decode(encoded.data(), encoded.size(), decoded);
	TEST_ASSERT_TRUE(err == FrameError::kBadMagic);
}

void test_frame_rejects_corrupted_crc(void) {
	Frame frame;
	frame.commandId = cmd::kAck;
	frame.payload = {1, 2, 3};
	auto encoded = frame.encode();
	encoded.back() ^= 0xFF;

	Frame decoded;
	FrameError err = Frame::decode(encoded.data(), encoded.size(), decoded);
	TEST_ASSERT_TRUE(err == FrameError::kCrcMismatch);
}

void test_frame_rejects_truncated_frame(void) {
	Frame frame;
	frame.commandId = cmd::kAck;
	frame.payload = {1, 2, 3};
	auto encoded = frame.encode();
	encoded.pop_back();

	Frame decoded;
	FrameError err = Frame::decode(encoded.data(), encoded.size(), decoded);
	TEST_ASSERT_TRUE(err == FrameError::kLengthMismatch);
}

void setup() {
	UNITY_BEGIN();
	RUN_TEST(test_crc16_check_vector);
	RUN_TEST(test_crc16_empty_is_init_value);
	RUN_TEST(test_rle_round_trips_all_zeros);
	RUN_TEST(test_rle_round_trips_random_bytes);
	RUN_TEST(test_rle_encodes_long_run_compactly);
	RUN_TEST(test_rle_decode_rejects_overrun);
	RUN_TEST(test_frame_round_trips_with_payload);
	RUN_TEST(test_frame_rejects_bad_magic);
	RUN_TEST(test_frame_rejects_corrupted_crc);
	RUN_TEST(test_frame_rejects_truncated_frame);
	UNITY_END();
}

void loop() {}

int main(int argc, char** argv) {
	setup();
	return 0;
}
