#include "RlePackBits.h"

#include <stdexcept>

namespace crowpanel {

namespace {
constexpr size_t kMaxLiteralRun = 128;
constexpr size_t kMaxRepeatRun = 129;

size_t runLengthAt(const uint8_t* data, size_t length, size_t pos) {
	size_t len = 1;
	while (pos + len < length && data[pos + len] == data[pos] && len < kMaxRepeatRun) {
		len++;
	}
	return len;
}
}  // namespace

std::vector<uint8_t> rleEncode(const uint8_t* data, size_t length) {
	std::vector<uint8_t> out;
	size_t i = 0;
	while (i < length) {
		size_t runLen = runLengthAt(data, length, i);
		if (runLen >= 2) {
			out.push_back(static_cast<uint8_t>(128 + (runLen - 2)));
			out.push_back(data[i]);
			i += runLen;
		} else {
			size_t litStart = i;
			size_t litLen = 0;
			while (i < length && litLen < kMaxLiteralRun && runLengthAt(data, length, i) < 2) {
				i++;
				litLen++;
			}
			out.push_back(static_cast<uint8_t>(litLen - 1));
			out.insert(out.end(), data + litStart, data + litStart + litLen);
		}
	}
	return out;
}

std::vector<uint8_t> rleDecode(const uint8_t* encoded, size_t encodedLength, size_t decodedLen) {
	std::vector<uint8_t> out(decodedLen);
	size_t oi = 0;
	size_t ei = 0;
	while (ei < encodedLength) {
		uint8_t c = encoded[ei++];
		if (c <= 127) {
			size_t len = static_cast<size_t>(c) + 1;
			if (ei + len > encodedLength || oi + len > decodedLen) {
				throw std::invalid_argument("RLE literal run overruns buffer");
			}
			for (size_t k = 0; k < len; k++) {
				out[oi + k] = encoded[ei + k];
			}
			ei += len;
			oi += len;
		} else {
			size_t runLen = static_cast<size_t>(c - 128) + 2;
			if (ei >= encodedLength || oi + runLen > decodedLen) {
				throw std::invalid_argument("RLE repeat run overruns buffer");
			}
			uint8_t value = encoded[ei++];
			for (size_t k = 0; k < runLen; k++) {
				out[oi + k] = value;
			}
			oi += runLen;
		}
	}
	if (oi != decodedLen) {
		throw std::invalid_argument("RLE decoded length mismatch");
	}
	return out;
}

}  // namespace crowpanel
