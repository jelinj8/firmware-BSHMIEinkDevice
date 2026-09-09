// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect, no xorout).
//
// Reference vector (shared with the Java implementation to guarantee both sides agree): ASCII
// "123456789" (9 bytes) -> 0x29B1.
#pragma once

#include <cstdint>
#include <cstddef>

namespace crowpanel {

uint16_t crc16(const uint8_t* data, size_t length);

}  // namespace crowpanel
