// Custom PackBits-inspired RLE codec for 1bpp bitmap payloads (doc/PROTOCOL.md §6). Mirrors
// pc-java-lib's RlePackBits.java byte-for-byte - keep both in sync.
//
// Control byte C:
//   C in [0,127]:   LITERAL run - next (C+1) bytes copied verbatim (1..128 bytes)
//   C in [128,255]: REPEAT run  - runLen = (C-128)+2 (2..129); next ONE byte repeated runLen times
#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

namespace crowpanel {

std::vector<uint8_t> rleEncode(const uint8_t* data, size_t length);

// Throws std::invalid_argument if the encoded stream overruns or underruns decodedLen.
std::vector<uint8_t> rleDecode(const uint8_t* encoded, size_t encodedLength, size_t decodedLen);

}  // namespace crowpanel
