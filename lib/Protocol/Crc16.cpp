#include "Crc16.h"

namespace crowpanel {

uint16_t crc16(const uint8_t* data, size_t length) {
	uint16_t crc = 0xFFFF;
	for (size_t i = 0; i < length; i++) {
		crc ^= static_cast<uint16_t>(data[i]) << 8;
		for (int bit = 0; bit < 8; bit++) {
			if (crc & 0x8000) {
				crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
			} else {
				crc = static_cast<uint16_t>(crc << 1);
			}
		}
	}
	return crc;
}

}  // namespace crowpanel
