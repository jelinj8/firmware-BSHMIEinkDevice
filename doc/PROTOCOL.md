# CrowPanel Wire Protocol

Version 1 (envelope `VERSION = 0x01`)

This document specifies the wire protocol between a PC-side control application (via the Java
library in `pc-java-lib/`) and CrowPanel firmware (`firmware/`) running on an ESP32-S3 + e-paper/
LCD board. The protocol is intentionally generic: it is not tied to the specific 400×300 1bpp
CrowPanel 4.2" panel, and a compliant device reports its own capabilities (display size, color
depth, available storage/GPIO, etc.) during a handshake rather than having them assumed by the PC
side.

Three transports are supported, all carrying the identical framed protocol described below:

- **WiFi** — a raw TCP socket (ESP32 as server, PC as client)
- **BLE** — via a custom GATT service (PC side built on the sibling `BSToolbox-BLE` library)
- **Serial** — a wired UART link

---

## 1. Layering overview

```
┌─────────────────────────────────────────────────────────┐
│ Layer 3: Commands (handshake, image transfer, drawing,   │  command-specific payloads
│          storage, config, GPIO, button events...)         │
├─────────────────────────────────────────────────────────┤
│ Layer 2: Logical Frame (common envelope, identical on    │  MAGIC/VERSION/CMD/SEQ/LEN/payload/CRC16
│          TCP, BLE, and Serial)                             │
├─────────────────────────────────────────────────────────┤
│ Layer 1: Transport                                        │
│   TCP    → frame bytes written straight to the stream,    │  no extra framing; PAYLOAD_LEN *is*
│            self-delimited by PAYLOAD_LEN in the header      │  the length prefix
│   Serial → same as TCP: frame bytes written straight to    │  no extra framing; full-duplex UART
│            the UART stream, self-delimited by PAYLOAD_LEN    │  needs no separate notify channel
│   BLE    → same as TCP/Serial: a byte stream, ATT/GATT      │  no wire-level framing added;
│            already fragments/reassembles to the MTU          │  write-with-response = flow control
└─────────────────────────────────────────────────────────┘
```

Reliability is **stop-and-wait**: only one logical message is ever in flight per direction. The
sender waits for an ACK/NACK (or, for commands with an inherent response, that response itself)
before sending its next logical frame; a timeout or NACK triggers a retransmit under the same
`SEQ`.

## 2. Common Logical Frame format

Identical byte layout on all three transports. **Little-endian** throughout (Java side must set
`ByteBuffer.order(ByteOrder.LITTLE_ENDIAN)` explicitly; on the ESP32/Xtensa/RISC-V firmware side
this matches the native byte order).

```
Offset  Size  Field         Type     Notes
------  ----  ------------  -------  --------------------------------------------------
0       1     MAGIC         u8       0xA5. Sync marker; lets a streamed-transport reader
                                      (TCP or Serial) resync after a corrupted frame.
1       1     VERSION       u8       Envelope version. v1 = 0x01.
2       2     COMMAND_ID    u16 LE   See §4 command-ID space.
4       1     SEQ           u8       Rolling counter, one per logical message (not per
                                      retry — a retry resends the same SEQ, enabling
                                      dedup at the receiver).
5       4     PAYLOAD_LEN   u32 LE   Length of PAYLOAD. u32 chosen over u16 for headroom
                                      on larger/generic displays and larger file transfers.
9       N     PAYLOAD       bytes    Command-specific, N = PAYLOAD_LEN.
9+N     2     CRC16         u16 LE   CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no
                                      reflect/xorout), computed over VERSION..end of
                                      PAYLOAD (excludes MAGIC and CRC itself).
```

Fixed overhead: **11 bytes/frame**.

Reference test vector for the CRC16 variant (CRC-16/CCITT-FALSE): the ASCII bytes `"123456789"`
(9 bytes, no terminator) → `0x29B1`. Both `Crc16.java` and the firmware `Crc16` implementation are
unit-tested against this vector to guarantee they agree.

### 2.1 Shared write-command FLAGS byte (working buffer + refresh timing)

The device maintains a single **working buffer** (an off-screen 1bpp bitmap the size of the panel)
distinct from what is currently physically shown on the **panel**. Every command that writes pixel
content — `FULL_IMAGE_TRANSFER` (§6), `PARTIAL_IMAGE_TRANSFER` (§7), and every local drawing
primitive (§12.2–§12.7) — writes into the working buffer, not directly to the panel, and shares
this one `FLAGS` byte to control when/how that content becomes visible:

```
bit0  REFRESH_NOW   0 = leave the working buffer updated but do not flip it to the panel yet
                         (deferred — batch with other writes, flip later via REFRESH, §12.8)
                     1 = flip the affected region to the panel immediately after this write
bit1  REFRESH_FULL   only consulted when bit0=1:
                     0 = fast partial e-ink update of just the affected bounding box
                     1 = full-panel refresh cycle (slower, clears more ghosting)
bits2-7  reserved (must be 0; must be tolerated if seen non-zero by a v1 receiver)
```

This is what makes buffered/batched drawing possible: send several drawing/image commands with
`REFRESH_NOW=0`, then a single explicit `REFRESH` (§12.8) to present them all in one visible
update.

## 3. Transport framing

### 3.1 BLE framing

Same treatment as TCP/Serial below — a raw ordered byte stream, **no wire-level chunk sub-header**.

*(This section originally specified a 2-byte `CHUNK_INDEX`/`CHUNK_FLAGS` sub-header framing each
BLE write/notify. Revised once actually implementing the transport showed it wasn't needed —
kept as a design note rather than silently erased, since a receiver implementation written against
the old text would be wire-incompatible with this one. Any implementation predating this revision
must be updated; there was never a shipped device depending on the old framing.)*

**Why no sub-header is needed**: BLE's ATT/GATT layer already fragments any write/notify to fit
the negotiated MTU and delivers it in order within one connection. The Logical Frame (§2) is
already self-delimiting via `PAYLOAD_LEN`, so reassembling one from a stream of arbitrarily-sized
byte chunks is exactly the problem TCP/Serial already solve (§3.2/§3.3) — no separate
chunk-sequence bookkeeping earns its cost on top of that. Concretely, each side gets this "just a
byte stream" shape from a different place:

- **Firmware**: one GATT characteristic (§20) with combined `WRITE | WRITE_NR | NOTIFY`
  properties, wrapped in NimBLE-Arduino's `NimBLEStreamServer`, which presents it as a plain
  Arduino `Stream` — internally fragmenting/reassembling to the negotiated MTU on its own.
  `firmware/lib/Transport/StreamFrameTransport.h` drives it completely unchanged from the
  Serial/TCP case (`Stream&` was chosen generic for exactly this reason).
- **PC** (`pc-java-lib`, via the sibling `BSToolbox-BLE` library): `BSToolbox-BLE` gives raw
  per-call `byte[]` read/write/notify with no stream abstraction or auto-fragmentation of its own
  (see the MTU discussion below), so a small adapter reproduces the same byte-stream shape:
  incoming notifications are appended to a buffer that `FrameStreamReader` reads from (identical
  reassembly/resync logic to TCP/Serial), and outgoing `Frame` bytes are split into
  `MAX_CHUNK_SIZE`-sized pieces, each sent as one `writeCharacteristic(..., withResponse=true)`
  call.

**Sizing**: `MAX_CHUNK_SIZE` (§5.2) is the max single-write size the PC should use — firmware
reports `negotiated ATT MTU - 3` (ATT opcode+handle overhead; there's no chunk-header overhead to
additionally subtract, unlike the superseded design). `BSToolbox-BLE` gives no way to query the
actually-negotiated ATT MTU itself (Windows negotiates a higher MTU transparently once a
`GattSession` is held, which the library does automatically; Linux/BlueZ negotiates it at the
kernel level with no app-level call needed) — so firmware should actively negotiate a larger ATT
MTU on connect (NimBLE-Arduino supports up to 247–517 B on ESP32-S3) and report what it actually
got via `MAX_CHUNK_SIZE`, rather than reporting the historical 23-byte default; the PC library must
trust this reported value rather than guessing. `MAX_CHUNK_SIZE` is advisory/unused on TCP and
Serial, which have no per-write size ceiling.

**Flow control**:
- PC→device writes always use GATT **write-with-response** — paces the sender for free and
  confirms each write reached the peripheral's BLE stack, without needing a wire-level chunk ACK.
- Device→PC notifications have no equivalent confirmed-delivery primitive at the ATT layer; a
  dropped notification isn't separately detected mid-stream — it surfaces as a `CRC_FAIL` (or an
  incomplete read) on the reassembled Frame, caught by the exact same resync logic already built
  for TCP/Serial, not a BLE-specific mechanism.
- Logical-frame-level ACK/NACK (§10) is unchanged and identical across all three transports —
  stop-and-wait, one logical message in flight per direction.

### 3.2 TCP framing

No separate chunking layer — `PAYLOAD_LEN` in the Logical Frame header is already a length prefix
on the reliable, ordered TCP stream.

### 3.3 Serial (UART) framing

Same treatment as TCP: no separate chunking layer. UART is full-duplex, so — unlike BLE — no
separate "notify-equivalent" channel is needed.

- **Baud rate**: not negotiated over the wire; both ends must already agree. Default: 115200 8N1.
- **Dedicated UART instance**: firmware must use a UART distinct from whatever carries Arduino
  debug logging (`Serial.print`) — mixing plain-text debug output into the binary protocol stream
  would corrupt frame parsing.
- **RST/BOOT control via DTR/RTS** (design note 71): on the CH340 USB-serial adapter this board
  uses, DTR drives the chip's GPIO0 (BOOT) line and RTS drives its EN (RST) line — the same lines
  the board's own physical RST/BOOT buttons pull, and the same convention `esptool.py` itself
  relies on for flashing. This isn't a wire-protocol feature (nothing crosses the framed-protocol
  envelope) — it's out-of-band control of the serial adapter's modem-control lines, available only
  over the Serial transport (BLE/TCP have no equivalent physical signal) — see
  `SerialFrameTransport`'s `pressReset()`/`releaseReset()`/`pressBoot()`/`releaseBoot()` and the
  composite `resetToRunMode()`/`resetToBootloader()`.

## 4. Command-ID space

2-byte `COMMAND_ID`, partitioned into 256-ID blocks:

```
0x0000            RESERVED (never valid; guards against garbage/zeroed buffers)
0x0001–0x00FF     Core / Handshake / ACK-NACK              — fully specified (§5, §10)
0x0100–0x01FF     Display / Image transfer / screen read /  — fully specified (§6, §7, §8, §9)
                  artifact clearing
0x0200–0x02FF     Device → Host events                       — button events specified (§11)
0x0300–0x03FF     Local drawing primitives                   — fully specified (§12)
0x0400–0x04FF     Device configuration                        — backup/restore, network config,
                                                                 device name, and access-control PINs
                                                                 specified (§13); 0x040E-0x04FF still
                                                                 RESERVED
0x0500–0x05FF     Sensor data / extensions (device→PC)         — RESERVED, not designed yet
0x0600–0x06FF     Storage / file management (SD + internal)    — fully specified (§14)
0x0700–0x07FF     GPIO configuration / control / events         — fully specified (§15)
0x0800–0x08FF     OTA firmware update                            — fully specified (§16)
0x0900–0x09FF     Power management                                — fully specified (§17)
0x0A00–0x0AFF     Macro recording and playback                    — fully specified (§18)
0x0B00–0xFEFF     Reserved for future core protocol expansion
0xFF00–0xFFFF     Vendor/private/experimental extensions

Every command below requires a minimum access level (§5.3): `NONE` unless noted, `USAGE`, or
`ADMIN` — shown inline as `[USAGE]`/`[ADMIN]` after the direction; unmarked means `NONE`.

Concrete IDs:
0x0001 HANDSHAKE_REQUEST         PC → device
0x0002 HANDSHAKE_RESPONSE        device → PC
0x0003 ACK                       either
0x0004 NACK                      either
0x0005 LOG_MESSAGE               either [USAGE] (§21, design note 86)

0x0100 FULL_IMAGE_TRANSFER       PC → device [USAGE]
0x0101 PARTIAL_IMAGE_TRANSFER    PC → device [USAGE]
0x0102 READ_SCREEN               PC → device [USAGE]
0x0103 SCREEN_DATA               device → PC
0x0104 CLEAR_ARTIFACTS           PC → device [USAGE]

0x0200 BUTTON_EVENT              device → PC

0x0300 DRAW_LINE                 PC → device [USAGE]
0x0301 DRAW_RECT                 PC → device [USAGE]
0x0302 DRAW_CIRCLE               PC → device [USAGE]
0x0303 CLEAR_REGION              PC → device [USAGE]
0x0304 DRAW_TEXT                 PC → device [USAGE]
0x0305 DRAW_IMAGE                PC → device [USAGE]
0x0306 REFRESH                   PC → device [USAGE]
0x0307 SHIFT_REGION              PC → device [USAGE]
0x0308 SET_CLIP_REGION           PC → device [USAGE]
0x0309 COPY_REGION               PC → device [USAGE]
0x030A SET_DRAW_OFFSET           PC → device [USAGE]
0x030B SET_ORIENTATION           PC → device [USAGE]
0x030C DRAW_IMAGE_ROW            PC → device [USAGE]
0x030D FILL_IMAGE                PC → device [USAGE]
0x030E FAST_CLEAR                PC → device [USAGE] (§12.16)
0x030F SET_CUSTOM_FONT_FOLDER    PC → device [USAGE] (§12.6.2)

0x0400 CONFIG_BACKUP_REQUEST     PC → device [USAGE]
0x0401 CONFIG_BACKUP_DATA        device → PC
0x0402 CONFIG_RESTORE            PC → device [ADMIN]
0x0403 SET_WIFI_CONFIG           PC → device [ADMIN]
0x0404 WIFI_STATUS_REQUEST       PC → device [USAGE]
0x0405 WIFI_STATUS_RESPONSE      device → PC
0x0406 SET_WIFI_ENABLED          PC → device [ADMIN]
0x0407 SET_BLE_ENABLED           PC → device [ADMIN]
0x0408 SET_BLE_PIN               PC → device [ADMIN]
0x0409 BLE_STATUS_REQUEST        PC → device [USAGE]
0x040A BLE_STATUS_RESPONSE       device → PC
0x040B SET_DEVICE_NAME           PC → device [ADMIN]
0x040C SET_USAGE_PIN             PC → device [ADMIN] (§5.3)
0x040D SET_ADMIN_PIN             PC → device [ADMIN] (§5.3)

0x0600 FILE_LIST_REQUEST         PC → device [USAGE]
0x0601 FILE_LIST_RESPONSE        device → PC
0x0602 FILE_DOWNLOAD_REQUEST     PC → device [USAGE]
0x0603 FILE_DATA                 device → PC
0x0604 FILE_UPLOAD               PC → device [ADMIN]
0x0605 FILE_DELETE               PC → device [ADMIN]
0x0606 STORAGE_INFO_REQUEST      PC → device [USAGE]
0x0607 STORAGE_INFO_RESPONSE     device → PC
0x0608 FILE_COPY                 PC → device [ADMIN]
0x0609 FILE_RENAME               PC → device [ADMIN]

0x0700 GPIO_CONFIGURE            PC → device [ADMIN]
0x0701 GPIO_WRITE                PC → device [USAGE]
0x0702 GPIO_READ_REQUEST         PC → device [USAGE]
0x0703 GPIO_READ_RESPONSE        device → PC
0x0704 GPIO_EVENT                device → PC
0x0705 GPIO_PLAY_PATTERN         PC → device [USAGE]

0x0800 OTA_INSTALL               PC → device [ADMIN]
0x0801 OTA_APPLY                 PC → device [ADMIN]
0x0802 OTA_STATUS_REQUEST        PC → device [USAGE]
0x0803 OTA_STATUS_RESPONSE       device → PC
0x0804 OTA_CONFIRM               PC → device [ADMIN]
0x0805 OTA_ROLLBACK              PC → device [ADMIN]

0x0900 SET_POWER_MODE            PC → device [USAGE] (own extra rule for HARD_SLEEP, §5.3)
0x0901 POWER_STATUS_REQUEST      PC → device [USAGE]
0x0902 POWER_STATUS_RESPONSE     device → PC

0x0A00 RECORD_MACRO              PC → device [ADMIN]
0x0A01 SAVE_MACRO                PC → device [ADMIN]
0x0A02 PLAY_MACRO                PC → device [USAGE]
0x0A03 PAUSE                     PC → device [USAGE]
```

Unknown `COMMAND_ID` → `NACK(UNSUPPORTED_COMMAND)`, never silently dropped.

## 5. Handshake / capability negotiation

On connect, PC sends `HANDSHAKE_REQUEST`; device replies `HANDSHAKE_RESPONSE` (this response
itself is the stop-and-wait unblock — no separate ACK for handshake). A `VERSION` mismatch →
`NACK(VERSION_MISMATCH)` instead.

`HANDSHAKE_REQUEST` payload (§5.3 access control — optionally offers a PIN):
```
0 1        PIN_TYPE  0x00 NONE (no pin offered), 0x01 USAGE, 0x02 ADMIN
1 1        PIN_LEN   u8, 0 if PIN_TYPE=NONE
2 PIN_LEN  PIN       UTF-8, present only if PIN_LEN>0
```
Minimum 2 bytes (`PIN_TYPE=NONE, PIN_LEN=0`) — always this fixed, self-describing shape, never a
truly empty payload. See §5.3 for what PIN_TYPE/PIN do and how HANDSHAKE_RESPONSE reports the
result.

### 5.1 TLV entry format (used for the handshake payload and §13 config backup)

```
Offset  Size    Field   Notes
0       1       TYPE    u8, capability identifier (own namespace)
1       1       LENGTH  u8, length of VALUE (0–255)
2       LENGTH  VALUE   raw bytes, semantics per TYPE
```

Unknown `TYPE` → skip via `LENGTH`, never an error. This is the extensibility template for any
future payload that needs to grow.

### 5.2 Capability TLV types

```
TYPE  Name                            Size      Meaning
0x01  PROTOCOL_VERSION                1         Envelope VERSION implemented
0x02  DISPLAY_WIDTH_PX                2 u16 LE  Panel width
0x03  DISPLAY_HEIGHT_PX               2 u16 LE  Panel height
0x04  COLOR_DEPTH                     1         Bits/pixel (1 = 1bpp B/W; future grayscale/color)
0x05  MAX_CHUNK_SIZE                  2 u16 LE  Max single BLE write/notify value size (negotiated
                                                 ATT MTU - 3), §3.1
0x06  FEATURE_BITMASK                 4 u32 LE  bit0 partial-refresh, bit1 RLE, bit2 button events,
                                                 bit3 drawing primitives, bit4 sensors,
                                                 bit5 SD card slot present (capability, not live
                                                 presence — see §14 STORAGE_INFO for that),
                                                 bit6 internal storage available,
                                                 bit7 GPIO control available,
                                                 bit8 power management available (§17),
                                                 bits9-31 reserved
0x07  DEVICE_MODEL                    ≤32       UTF-8, not null-terminated
0x08  FIRMWARE_VERSION                ≤16       UTF-8
0x09  MAX_FULL_IMAGE_BYTES            4 u32 LE  Max decoded payload accepted for full-image transfer
0x0A  PARTIAL_REFRESH_GRANULARITY_X   1         X/WIDTH alignment (8 for SSD1683-family)
0x0B  PARTIAL_REFRESH_GRANULARITY_Y   1         Y/HEIGHT alignment (1)
0x0C  ACTIVE_TRANSPORT                1         0x00 TCP, 0x01 BLE, 0x02 Serial — echoes which
                                                 transport this handshake was received over
0x0D  AVAILABLE_GPIO_PINS             ≤255      List of raw GPIO numbers (1 byte each) safe to use
                                                 with §15 commands; empty/absent = no GPIO exposed
0x0E  LAST_WAKE_REASON                1         Same enum as §17.2 POWER_STATUS_RESPONSE - lets a
                                                 client learn why the device is currently running
                                                 right on connect, no extra round trip needed
0x0F  DEVICE_NAME                    ≤32       UTF-8 - user-assigned label (§13.3), distinct from
                                                 the fixed DEVICE_MODEL string above
0x10  PIXEL_PITCH_X_UM                2 u16 LE  Physical pixel pitch, X axis, micrometers (e.g.
                                                 212 for CrowPanel's 0.212 mm pixel pitch) - a raw
                                                 hardware fact, not a display setting; a client
                                                 derives DPI as 25400 / PIXEL_PITCH_*_UM itself
                                                 (kept as pitch, not a pre-rounded DPI value, so
                                                 nothing lossy is baked into the wire format)
0x11  PIXEL_PITCH_Y_UM                2 u16 LE  Same, Y axis - reported separately rather than
                                                 assuming square pixels, even though every panel
                                                 this protocol has seen so far has them
0x12  GRANTED_LEVEL                   1         §5.3 - what THIS handshake actually achieved:
                                                 0x00 NONE, 0x01 USAGE, 0x02 ADMIN
0x13  USAGE_PIN_REQUIRED               1        §5.3 - is a usage PIN currently configured
                                                 (informational only, reported regardless of
                                                 whether this handshake's own PIN was correct -
                                                 "a lock exists" isn't itself sensitive)
0x14  ADMIN_PIN_REQUIRED               1        §5.3 - same, for the admin PIN
0x15+ reserved (battery, buttons-present bitmap, orientation, ...)
```

### 5.3 Access control

Two independent, **optional** PINs. Neither is configured by default, matching today's fully-open
behavior; setting one is a deliberate opt-in.

- **Usage PIN** — gates whether a connection can do anything beyond the handshake at all.
- **Admin PIN** — gates write/config operations specifically. Possessing it also grants usage
  access (admin implies usage).

Access level lives **per connection**, not per device: each of the three transports tracks its own
current connection's granted level independently, reset to NONE whenever that transport's
connection is (re-)established — a new TCP client, a new BLE central connecting, or (for Serial,
which has no separate "reconnect" event) every cold boot, which a real Serial reconnect already
triggers via the CH340 adapter's DTR/RTS auto-reset (§3.3). Closing a TCP or BLE connection and
opening a new one always starts back at NONE; there is no cross-connection "stay logged in".

**Presenting a PIN**: `HANDSHAKE_REQUEST`'s `PIN_TYPE`/`PIN` fields (§5). A **wrong** PIN is
treated identically to no PIN offered — the connection simply stays at whatever level it can
prove — `HANDSHAKE_REQUEST` itself never NACKs due to a bad/missing PIN, and there's no way to
distinguish "wrong PIN" from "no PIN sent" by response shape alone. `HANDSHAKE_RESPONSE`'s
`GRANTED_LEVEL` TLV reports exactly what was achieved; `USAGE_PIN_REQUIRED`/`ADMIN_PIN_REQUIRED`
report whether each tier is currently configured at all, so a client can decide whether to prompt
for a PIN without guessing.

**Managing the PINs**: `SET_USAGE_PIN`/`SET_ADMIN_PIN` (§13.4) — both **admin**-gated, including
`SET_USAGE_PIN` itself (a non-admin must never be able to tighten or loosen the usage gate). The
bootstrap case (neither PIN ever configured) needs no ceremony: admin is trivially open when
unconfigured, so the very first `SET_ADMIN_PIN` naturally succeeds without prior credentials.

**Per-command required level**: every command in §4's table now has an AUTH column —
`NONE` (`HANDSHAKE_REQUEST` only), `USAGE` (normal display-driving operations: all of §12
drawing, image transfer §6/§7, `CLEAR_ARTIFACTS`, macro *playback* `PLAY_MACRO`/`PAUSE`,
`GPIO_WRITE`/`GPIO_PLAY_PATTERN`/`GPIO_READ_REQUEST`, `LOG_MESSAGE`, `SET_POWER_MODE` (with its own
extra rule, below), and every plain status/read request), or `ADMIN` (§13 configuration including
`SET_BLE_PIN`/`SET_USAGE_PIN`/`SET_ADMIN_PIN`, §16 OTA, macro *recording* `RECORD_MACRO`/
`SAVE_MACRO`, `GPIO_CONFIGURE`, and storage writes `FILE_UPLOAD`/`FILE_DELETE`).

**Cascading fallback, not a simple "unconfigured tier = open" rule**: configuring *any* PIN
expresses real intent to restrict the device, so a higher tier never becomes *more* open just
because that specific tier's own PIN wasn't set. The *effective* required level for a command
whose table entry says `requiredLevel` is:
- `requiredLevel = ADMIN`: effective = ADMIN if an admin PIN is configured; else USAGE if a usage
  PIN is configured (falls back — an admin-gated command still needs *some* credential); else NONE.
- `requiredLevel = USAGE`: effective = USAGE if a usage PIN is configured; else NONE — an admin
  PIN existing *alone* does not gate USAGE-required commands.
- `requiredLevel = NONE`: always NONE.

Concretely: usage PIN set, admin PIN not set → admin-gated commands (including `SET_ADMIN_PIN`
itself) require the usage PIN, not open to everyone. Admin PIN set, usage PIN not set →
usage-gated commands are open to everyone, admin-gated ones need the admin PIN. Neither set →
everything open. A connection whose granted level is below the effective required level gets
`NACK(NOT_AUTHORIZED)`, §10.

**`SET_POWER_MODE`'s own extra rule**: registered at USAGE, but `MODE=HARD_SLEEP` requested over
TCP or BLE additionally requires the effective ADMIN level (same cascading rule as above) —
`HARD_SLEEP` kills whichever radio carries that very connection, so a non-admin severing their own
session that way, with no recovery but a timer/button, is the risk being gated. Over **Serial**,
`HARD_SLEEP` only needs the baseline USAGE gate — Serial isn't radio-dependent and isn't severed by
a sleep the way a live TCP/BLE session is.

**MENU+BACK physical-presence override, Serial only**: if both buttons are held at the instant a
`HANDSHAKE_REQUEST` arrives *over Serial*, the granted level for that connection is ADMIN
regardless of any PIN offered (or not) — TCP/BLE handshakes never check button state. This exists
specifically because Serial is gated too (unlike this protocol's usual "Serial is the physical
recovery path" default) — a deliberate two-button hold, not bare cable access, is what recovers a
forgotten admin PIN, addressing "prevent at least accidental or uninformed malicious usage" while
the device might be physically restrained and only reachable this way.

A macro replay (`ACTIVE_TRANSPORT=MACRO`, §18) bypasses the access-control gate entirely, for every
entry, regardless of the replaying connection's own granted level - `PLAY_MACRO` itself is only
USAGE-gated. This is safe by construction, not merely convenient: the only two ways a `.macro`
file can come to exist on the device are `RECORD_MACRO`/`SAVE_MACRO` (capturing a live session) and
`FILE_UPLOAD` (placing one directly) - both ADMIN-gated. A macro file can therefore only ever
contain what an admin already put there, so a USAGE-level `PLAY_MACRO` is delegation, not
escalation: it can trigger a script an admin pre-approved, but can never get a new ADMIN-level
command executed that wasn't already baked into that file.

## 6. Full-image transfer (`0x0100`, PC → device)

```
Offset  Size         Field         Notes
0       1            ENCODING      0x00 RAW, 0x01 RLE_PACKBITS, else reserved
1       1            FLAGS         shared write FLAGS byte, §2.1
2       4            DECODED_LEN   u32 LE, exact decoded bitmap size
6       4            ENCODED_LEN   u32 LE, must equal PAYLOAD_LEN-10
10      ENCODED_LEN  DATA          Packed bitmap (RAW) or RLE stream
```

Writes the full panel-sized image into the working buffer (§2.1); no explicit width/height field —
always targets the full panel at the resolution reported in the handshake.

**Wire pixel polarity convention**: `bit=1 = BLACK`, `bit=0 = WHITE`, independent of the driver
chip's native RAM polarity — translation to native polarity is firmware's job.

**RAW encoding**: packed 1bpp, row-major top-to-bottom, `ceil(width/8)` bytes/row, MSB-first
(bit 7 = leftmost pixel).

**RLE_PACKBITS encoding** (custom, PackBits-*inspired*, not off-the-shelf-compatible — operates on
whole bytes of the RAW bitmap, since e-ink content has long identical-byte runs):

```
Control byte C:
  C in [0,127]:   LITERAL run — next (C+1) bytes copied verbatim (1..128 bytes)
  C in [128,255]: REPEAT run — runLen = (C-128)+2 (2..129); next ONE byte repeated runLen times
```

Decode until output reaches `DECODED_LEN`; early end or overrun → `NACK(DECODE_FAIL)`. Encoder
algorithm is not normative — any conformant encoder is valid. This same RAW/RLE scheme is reused
verbatim by §7 (partial transfer), §8 (screen readback), and the `.epi` file format (§12.7).
Implemented by `RlePackBits` (Java: `pc-java-lib`; firmware: `firmware/lib/Protocol/RlePackBits.h`).

## 7. Partial-image transfer (`0x0101`, PC → device)

```
Offset  Size         Field         Notes
0       1            ENCODING      same as §6
1       1            FLAGS         shared write FLAGS byte, §2.1
2       2            X             u16 LE, must be multiple of PARTIAL_REFRESH_GRANULARITY_X
4       2            Y             u16 LE, must be multiple of PARTIAL_REFRESH_GRANULARITY_Y
6       2            WIDTH         u16 LE, must be multiple of PARTIAL_REFRESH_GRANULARITY_X
8       2            HEIGHT        u16 LE, must be multiple of PARTIAL_REFRESH_GRANULARITY_Y
10      4            DECODED_LEN   u32 LE
14      4            ENCODED_LEN   u32 LE
18      ENCODED_LEN  DATA          Same row-major/MSB-first layout as §6, scoped to the region
```

**Alignment violations or out-of-bounds regions → `NACK(BAD_PARAMETERS)`, never silently rounded.**
The PC-side Java library should proactively validate/round using the granularity capabilities
before sending, but the wire protocol itself stays strict.

Periodic full-refresh-for-ghosting policy is left to firmware (e.g. force a full refresh every N
partial writes) — see also §9 for an explicit on-demand cleaning cycle.

## 8. Screen / working-buffer readback (`0x0102` READ_SCREEN, `0x0103` SCREEN_DATA)

`READ_SCREEN` (PC → device):
```
Offset  Size  Field   Notes
0       1     SOURCE  0x00 PANEL (content last physically presented to the e-ink panel)
                       0x01 WORKING_BUFFER (current in-memory buffer, including deferred edits)
1       1     MODE    0x00 FULL, 0x01 REGION
2       2     X       u16 LE — present only if MODE=REGION
4       2     Y       u16 LE — present only if MODE=REGION
6       2     WIDTH   u16 LE — present only if MODE=REGION
8       2     HEIGHT  u16 LE — present only if MODE=REGION
```

`SCREEN_DATA` (device → PC, response), reusing the exact §6 RAW/RLE encoding:
```
Offset  Size         Field        Notes
0       1            ENCODING     0x00 RAW / 0x01 RLE_PACKBITS
1       2            X            u16 LE (0 for a FULL read)
3       2            Y            u16 LE (0 for a FULL read)
5       2            WIDTH        u16 LE (panel width for a FULL read)
7       2            HEIGHT       u16 LE (panel height for a FULL read)
9       4            DECODED_LEN  u32 LE
13      4            ENCODED_LEN  u32 LE
17      ENCODED_LEN  DATA
```

No region-alignment constraint here (unlike §7), since this only reads firmware-held memory rather
than addressing a panel RAM window.

## 9. Clear-artifacts / degauss cycle (`0x0104` CLEAR_ARTIFACTS, PC → device)

```
Offset  Size  Field   Notes
0       1     CYCLES  Number of full black/white flash cycles to run; 0 = firmware default
1       1     FLAGS   bit0 RESTORE_CONTENT: 0 = leave panel blank; 1 = re-flip the working
                       buffer to the panel once cycling finishes. bits1-7 reserved.
```

Distinct from `CLEAR_REGION` (§12.5), which only fills a working-buffer rectangle and does not
flash the panel. May take several seconds — PC clients should use a longer ACK timeout for it.

## 10. ACK / NACK (`0x0003` / `0x0004`)

Commands with an inherent data response use that response itself as the stop-and-wait unblock
(`HANDSHAKE_REQUEST`→`HANDSHAKE_RESPONSE`, `READ_SCREEN`→`SCREEN_DATA`,
`FILE_LIST_REQUEST`→`FILE_LIST_RESPONSE`, `FILE_DOWNLOAD_REQUEST`→`FILE_DATA`,
`STORAGE_INFO_REQUEST`→`STORAGE_INFO_RESPONSE`, `CONFIG_BACKUP_REQUEST`→`CONFIG_BACKUP_DATA`,
`GPIO_READ_REQUEST`→`GPIO_READ_RESPONSE`, `WIFI_STATUS_REQUEST`→`WIFI_STATUS_RESPONSE`,
`BLE_STATUS_REQUEST`→`BLE_STATUS_RESPONSE`, `POWER_STATUS_REQUEST`→`POWER_STATUS_RESPONSE`) — no
separate ACK for those. Everything else gets an
explicit ACK/NACK.

```
Offset  Size  Field          Notes
0       1     REF_SEQ        SEQ of the message being acknowledged/rejected
1       2     REF_COMMAND_ID u16 LE, for debugging/disambiguation
3       1     STATUS         see below
```

```
0x00 OK (ACK)                0x08 FILE_NOT_FOUND
0x01 CRC_FAIL                0x09 INSUFFICIENT_STORAGE
0x02 BUSY                    0x0A VOLUME_NOT_PRESENT
0x03 UNSUPPORTED_COMMAND     0x0B PIN_UNAVAILABLE
0x04 BAD_PARAMETERS          0x0C OTA_HASH_MISMATCH
0x05 DECODE_FAIL             0x0D OTA_NOT_STAGED
0x06 VERSION_MISMATCH        0x0E NOT_AUTHORIZED (§5.3)
0x07 CHUNK_SEQUENCE_ERROR*   0x0F–0xFE reserved
                             0xFF UNKNOWN_ERROR
```
\* No longer has a defined trigger — was for the BLE chunk-sub-header scheme superseded in §3.1.
Kept assigned (not reused for something else) since a status code, once shipped, isn't worth
recycling; a future transport with its own sequencing concept could still use it.

A PC-side timeout (no ACK/NACK at all) is not a wire value — it triggers the same
retransmit-under-same-SEQ path as an explicit NACK.

### 10.1 LOG_MESSAGE (`0x0005`, either direction) — bidirectional marker/echo

No wire-format structure — the payload is an opaque, PC-chosen marker (design note 86). Sent live
(PC → device), the device both ACKs it normally *and* echoes it straight back on every live
transport, unsolicited, as its own `LOG_MESSAGE` frame with a fresh `SEQ` (a separate counter from
ordinary requests). Recorded into a macro (§18) like any other command, replaying it later
re-triggers the exact same echo at that point in playback — letting a PC embed a marker mid-macro
and use `CommandClient#waitForLogMessage(marker, timeout)` to detect when playback actually reaches
(or finishes) that point, rather than relying only on `PLAY_MACRO`'s own ACK (which means "playback
started", not "finished"). Not registered in §10's "inherent data response" list — a macro-embedded
marker's echo can arrive long after the SEQ-based correlation window of the original live send (if
any) has already closed, so it always correlates by payload content via the event-listener path,
never by SEQ.

Two fixed markers, `"init_done"` and `"boot_done"`, are appended automatically as macro entries
around the two auto-run startup macros (§18) — `"init_done"` right after `/init.macro`'s own
entries, `"boot_done"` at the very end of the combined sequence (after `/boot.macro`'s entries, if
any) — so a PC can reliably learn when each stage of the cold-boot sequence has actually finished
without needing to embed its own markers for this common case.

## 11. Button event (`0x0200`, device → PC, via notify)

```
Offset  Size  Field         Notes
0       1     BUTTON_ID     enum below
1       1     EVENT_TYPE    0x00 PRESS, 0x01 RELEASE, 0x02 LONG_PRESS (fires once, not a
                             repeat stream), 0x03 SHORT_PRESS (fires immediately after
                             RELEASE whenever that press-release cycle never crossed the
                             LONG_PRESS threshold - the common "quick tap" case; a long
                             hold instead fires LONG_PRESS mid-hold, then RELEASE, with
                             no SHORT_PRESS)
2       4     TIMESTAMP_MS  u32 LE, device-local ms since boot (monotonic, NOT wall-clock)
```

```
0x00 UNKNOWN/RESERVED       0x08–0x0F reserved
0x01 DIAL_SWITCH (press)    0x10–0x1F GENERIC_BUTTON_0..15 (unlabeled GPIO buttons on other boards)
0x02 MENU                   0x20–0xFF reserved
0x03 BACK
0x04 BOOT
0x05 RESET
0x06 DIAL_UP
0x07 DIAL_DOWN
```

CrowPanel's "Dial switch" (per its spec sheet) is wired on real hardware as three separate GPIOs,
not a single rotary-encoder signal or a single press button — a scroll-up input, a scroll-down
input, and a press/confirm input. `DIAL_SWITCH` covers the press/confirm action (kept at its
original ID for compatibility); `DIAL_UP`/`DIAL_DOWN` (added from the range this table already
reserved for growth) cover the two scroll directions as their own discrete button IDs, each with
the usual PRESS/RELEASE/LONG_PRESS `EVENT_TYPE`s. A future board with a true rotary encoder or a
single combined dial signal can still just emit `DIAL_SWITCH` alone.

`BUTTON_EVENT` is for the device's own firmware-recognized physical buttons; `GPIO_EVENT` (§15.4)
is for arbitrary pins the PC has explicitly configured as inputs.

## 12. Local drawing primitives (`0x0300`–`0x03FF`)

Write into the shared working buffer (§2.1); no alignment constraint applies here (unlike §7) —
firmware computes and aligns the actual panel window itself when flipping a region to the display.

### 12.1 Shared enums

```
DRAW_MODE (1 byte): 0x00 REPLACE, 0x01 OR, 0x02 XOR, 0x03 AND, 0x04-0xFF reserved
COLOR     (1 byte): 0x00 WHITE, 0x01 BLACK  (matches §6 wire polarity: bit=1=black)
```

Compositing rule per covered pixel: `new_dst = combine(dst, src, DRAW_MODE)` where `REPLACE→src`,
`OR→dst|src`, `XOR→dst^src`, `AND→dst&src`. For solid-color geometry, `src` is the single `COLOR`
value applied uniformly; for text and masked images, `src` is per-pixel, and non-ink/transparent
pixels are skipped entirely regardless of `DRAW_MODE`.

Every command below carries the shared `FLAGS` byte from §2.1.

### 12.2 `0x0300` DRAW_LINE
```
0  2 X0   2  2 Y0   4  2 X1   6  2 Y1   (all u16 LE)
8  1 COLOR   9  1 DRAW_MODE   10 1 LINE_WIDTH (px, min 1)   11 1 FLAGS (§2.1)
```

### 12.3 `0x0301` DRAW_RECT
```
0 2 X   2 2 Y   4 2 WIDTH   6 2 HEIGHT
8 1 COLOR   9 1 DRAW_MODE   10 1 FILLED (0=outline,1=filled)
11 1 LINE_WIDTH (ignored if FILLED=1)   12 1 FLAGS (§2.1)
```

### 12.4 `0x0302` DRAW_CIRCLE
```
0 2 CENTER_X   2 2 CENTER_Y   4 2 RADIUS
6 1 COLOR   7 1 DRAW_MODE   8 1 FILLED   9 1 LINE_WIDTH   10 1 FLAGS (§2.1)
```

### 12.5 `0x0303` CLEAR_REGION
```
0 2 X   2 2 Y   4 2 WIDTH   6 2 HEIGHT
8 1 COLOR (fill value)   9 1 FLAGS (§2.1)
```

### 12.6 `0x0304` DRAW_TEXT
```
0  2 X          anchor = top-left of text bounding box
2  2 Y
4  2 WIDTH      u16 LE. 0 = unbounded single line (ALIGN/WRAP are then ignored); >0 is the
                 alignment/word-wrap reference box
6  1 FONT_ID    0x00 = the classic small embedded font; 0x01 = a larger embedded monospace font
                 (see below); 0xFF = a custom, folder-driven proportional font (§12.6.1) - any
                 other value NACKs BAD_PARAMETERS
7  1 COLOR
8  1 BACKGROUND 0x00 TRANSPARENT (non-ink pixels untouched), 0x01 OPAQUE (non-ink pixels filled
                 with the opposite of COLOR). "Inverted" text (light ink on a dark fill) is simply
                 COLOR=WHITE with BACKGROUND=OPAQUE - there's no separate inverted concept.
9  1 DRAW_MODE
10 1 ALIGN      0x00 LEFT, 0x01 CENTER, 0x02 RIGHT - meaningful only when WIDTH>0
11 1 WRAP       0x00 = no word-wrap (a line just runs past WIDTH - subject to whatever the
                 current clip region, §12.10, silently drops); 0x01 = word-wraps onto additional
                 lines so no line exceeds WIDTH. An embedded U+000A (LF) always starts a new line
                 regardless of WRAP.
12 1 FLAGS (§2.1)  bits0-1 REFRESH_NOW/REFRESH_FULL (§2.1); bit2 TEXT_IS_PATH (DRAW_TEXT-specific -
                    TEXT below is a UTF-8 path rather than literal text; the referenced file's own
                    content, read fresh every time this command runs, becomes the text to draw);
                    bits3-7 reserved
13 2 TEXT_LEN   u16 LE, byte length of the UTF-8 string (or, if FLAGS.TEXT_IS_PATH, path) that
                 follows
15 TEXT_LEN TEXT  UTF-8 bytes (accented characters via multi-byte sequences, e.g. Latin-1
                   Supplement/Latin Extended-A for European diacritics) for FONT_ID 0x00/0x01 - a
                   raw single-byte
                   codepage (no UTF-8 decoding, §12.6.1) for FONT_ID 0xFF - or, if
                   FLAGS.TEXT_IS_PATH, a UTF-8 path, optionally prefixed "R:"/"S:"/"F:" (see below)
```

`FLAGS.TEXT_IS_PATH` lets a recorded macro (§18) stay parametrized: write the text a macro should
display to a file, record/save a `DRAW_TEXT` that references that file by path instead of embedding
the text directly, and replaying the same saved macro later — after writing different content to
that file — draws the new text, with the macro itself never re-recorded or re-encoded.
`NACK(FILE_NOT_FOUND)` if the referenced file doesn't exist.

`DRAW_TEXT`'s own payload has no `VOLUME` field (unlike `DRAW_IMAGE`, §12.7) — instead, `TEXT` may
carry an optional 2-byte volume prefix: `R:` selects `VOLUME=PSRAM`, `S:` selects `VOLUME=SD`, `F:`
selects `VOLUME=INTERNAL` (flash); the prefix is stripped and the remainder used as the path. No
prefix (or any unrecognized one) defaults to `VOLUME=PSRAM`, with `TEXT` used exactly as given,
unstripped — matching that this feature was designed around PSRAM as the natural "macro parameter
slot" (session-scoped, freely rewritable) while still allowing SD/INTERNAL when durable/pre-staged
content is wanted instead.

Font rendering (glyph table keyed by Unicode codepoint) is a firmware detail — the protocol only
carries `FONT_ID` + UTF-8 bytes. Every non-ink pixel — including the embedded font's diacritic
rows, drawn separately from the main glyph cell, and the 1px inter-character spacer column both
share with it — goes through the same DRAW_MODE compositing as ink pixels when BACKGROUND=OPAQUE,
so REPLACE/OR/XOR/AND all apply uniformly to the whole glyph cell, not just its ink. A word longer
than WIDTH is never split mid-word (no hyphenation) — it simply overflows that one line. Text
height is always `lineCount × (embedded font's fixed line height)`; there's no HEIGHT field — use
the clip region (§12.10) to constrain how much vertical space a text block is allowed to actually
paint, independent of how many lines it lays out to.

FONT_ID 0x00's glyph coverage also includes the CP437 box-drawing and block-shade characters
(Unicode "Box Drawings"/"Block Elements" blocks — U+2500 ─, U+250C ┌, U+2510 ┐, U+2514 └, U+2518
┘, U+251C ├, U+2524 ┤, U+252C ┬, U+2534 ┴, U+253C ┼, both single- and double-line variants, and
░▒▓█▄▌▐▀), for drawing ASCII-art tables/borders. WIDTH=0 (or WRAP=0) preserves whitespace exactly
as sent — necessary for a box-drawing table's fixed internal spacing to survive; only genuine
word-wrap (WRAP=1 with WIDTH>0) treats runs of spaces as collapsible word separators. An embedded
`\n` always starts a new line either way.

FONT_ID 0x00's accented-letter coverage (`EmbeddedFont.h`'s `textGlyph::kAccentedGlyphs` - a base
ASCII letter plus a small diacritic mark drawn above it, both from the classic font) spans
essentially all of Latin-1 Supplement (grave/acute/circumflex/diaeresis/tilde/ring - French,
German, Spanish, Portuguese, Italian, Dutch, Nordic) plus Latin Extended-A's own acute/caron
letters (Polish/Slovak), not just Czech (design note 98). Three categories remain unmapped,
falling back to `?` like any other unsupported codepoint: diacritics below the baseline (cedilla
`Ç`/`ç`, ogonek `Ą`/`ą` `Ę`/`ę`), a handful of single-language diacritic shapes not yet designed
(breve, macron, dot-above, double-acute - Romanian/Latvian/Polish/Hungarian), and true
ligatures/stroke letters (`Æ` `Ø` `Ł` `Đ` `Þ` `Ð` `ß`) that aren't a base-letter-plus-overlay at
all.

**FONT_ID 0x01** is a larger font for the same use case (design notes 90/91) — `u8g2_font_
unifont_t_extended` (GNU Unifont, via the `U8g2_for_Adafruit_GFX` bridge library), a genuinely
monospace bitmap font for the Latin range it's sliced to (8px fixed advance, real precomposed
glyphs for the *entire* Latin-1 Supplement and Latin Extended-A blocks — every European accented
letter, cedilla/ogonek and ligature/stroke letters included — **no diacritic-overlay composition**,
unlike FONT_ID 0x00),
with the same `\n`/WIDTH/WRAP/ALIGN behavior and DRAW_MODE/BACKGROUND compositing semantics
described above. It does **not** include the CP437 box-drawing/block-shade glyphs (its sliced
codepoint range, U+0020–U+02BD, doesn't reach that far) — those codepoints render as `?` under
FONT_ID 0x01, same as any other unmapped codepoint under either font.

#### 12.6.1 FONT_ID 0xFF — custom folder-driven proportional font

Unlike `FONT_ID` 0x00/0x01 (built into firmware), 0xFF's glyphs are individual files loaded from
SD or INTERNAL storage, in a folder selected at runtime by `SET_CUSTOM_FONT_FOLDER` (§12.6.2) —
letting a glyph set be authored/replaced without reflashing firmware. Differences from 0x00/0x01,
everything else (WIDTH/ALIGN/WRAP/`\n`/DRAW_MODE/BACKGROUND compositing) working the same:

- **`TEXT` is a raw single-byte codepage, not UTF-8** — each byte 0x00-0xFF is directly a glyph
  index (256 glyph slots total, one file per byte value — see the `.gly` file format below). This
  also applies when combined with `FLAGS.TEXT_IS_PATH` — the referenced file's raw bytes are the
  codepage bytes, not UTF-8 text.
- **Proportional width, no extra spacing** — each glyph's own stored `WIDTH` (from its `.gly` file)
  is both its pixel width and its full horizontal advance; there is no inter-character spacing, so
  adjacent glyphs can be visually "welded" together (e.g. custom ligatures/icons split across
  files). Word-wrap and CENTER/RIGHT alignment are computed from each line's actual per-glyph width
  sum, not a fixed advance × character count.
- **Bottom-aligned within a shared line height** — every line's height is the mandatory `XX.gly`
  fallback glyph's own `HEIGHT` (see below), used as this font's line-height reference regardless
  of which glyphs actually appear on that line. A glyph shorter than that is pinned to the
  *bottom* of the line box (`glyphTop = lineTop + (lineHeight - glyph.height)`), not centered or
  top-aligned — there is no ascent/descent metadata, just literal bottom alignment.
- **`XX.gly` fallback** — any codepoint byte whose own `<NN>.gly` file is missing or fails to parse
  silently falls back to drawing `XX.gly` instead (not a NACK); `XX` can never collide with a real
  hex byte value (00-FF), so it's an unambiguous reserved filename.
- **NACK(FILE_NOT_FOUND)** if `FONT_ID=0xFF` is used before any `SET_CUSTOM_FONT_FOLDER` has
  succeeded this session, or if the configured folder's own `XX.gly` is itself missing/unparseable
  — both cases mean there is no valid line-height reference to lay text out with. Any other
  individual glyph file being missing/corrupt is not a NACK (see `XX.gly` fallback above).
- Glyphs load lazily (on first use by a `DRAW_TEXT`) and are cached in PSRAM for as long as the
  current folder stays selected; a new `SET_CUSTOM_FONT_FOLDER` discards the whole cache.

##### Glyph file format (`.gly`)

One file per codepage byte, named `<2 uppercase hex digits>.gly` (e.g. `41.gly` for byte `0x41`),
plus the mandatory fallback `XX.gly`. Mirrors `.epi`'s (§12.7) packed-1bpp convention, simplified —
glyphs are small enough that RLE and a separate transparency mask plane aren't worth it; a glyph
has no mask because *white is always transparent* for this format, with no way to express an
opaque white pixel.

```
Offset Size Field           Notes
0      4    MAGIC           ASCII "GLY1"
4      1    FORMAT_VERSION  0x01
5      2    WIDTH           u16 LE (px) — also this glyph's full horizontal advance (§12.6.1: no
                             extra spacing between glyphs)
7      2    HEIGHT          u16 LE (px)
9      ...  BITMAP           packed 1bpp, row-major, MSB-first, bytesPerRow=(WIDTH+7)/8, size =
                             bytesPerRow×HEIGHT. bit=1 = ink (drawn in COLOR), bit=0 = transparent
                             (white — the source pixel/background, if any, shows through)
```

#### 12.6.2 `0x030F` SET_CUSTOM_FONT_FOLDER

```
0 1        PATH_LEN   u8, max 255
1 PATH_LEN PATH       UTF-8, MUST start with a 2-byte prefix "R:", "S:", or "F:"
```

Points `DRAW_TEXT`'s `FONT_ID=0xFF` custom font (§12.6.1) at a folder of `.gly` glyph files.
Usage-level, and — unlike the `0x04xx` configuration commands (§13) — has no `FLAGS.PERSIST`
option at all: this is pure in-RAM session state, the same family as `SET_DRAW_OFFSET` (§12.13),
`SET_CLIP_REGION` (§12.10), and `SET_ORIENTATION` (§12.12), lost on reboot like the rest of them.

Reuses `DRAW_TEXT`'s own `R:`/`S:`/`F:` volume-prefix convention (§12.6), with one deliberate
difference: **the prefix is mandatory here** (no default-to-PSRAM for a missing/unrecognized one),
since `VOLUME=PSRAM` is flat (no subdirectories, §14) and can never be a valid folder target.

- **`S:`/`F:`** — direct: the prefix is stripped, the remainder is the folder path on `VOLUME=SD`/
  `INTERNAL` respectively.
- **`R:`** — one-time indirection, the "macro variable" mechanism: the prefix is stripped, the
  remainder is a path on `VOLUME=PSRAM` whose *content* (not `PATH` itself) is the real folder
  path — which must itself start with `S:` or `F:` (exactly one level of indirection; a `R:`-
  prefixed or unprefixed inner value is `NACK(BAD_PARAMETERS)`). Resolved **once, right here** —
  unlike `DRAW_TEXT`'s `FLAGS.TEXT_IS_PATH`, which re-reads its referenced file on every single
  draw, `SET_CUSTOM_FONT_FOLDER` reads the PSRAM pointer file exactly once, at the moment this
  command runs, and keeps using that resolved path until the next `SET_CUSTOM_FONT_FOLDER`. This
  lets a macro (§18) parametrize which font folder is active by writing the desired path into a
  PSRAM file before replaying a recorded `SET_CUSTOM_FONT_FOLDER|R:...`, without needing a
  different macro per folder.

`NACK(BAD_PARAMETERS)` if `PATH` is shorter than 2 bytes or its prefix isn't one of `R:`/`S:`/`F:`
(or, for `R:`, if the pointer file's own content isn't a valid `S:`/`F:`-prefixed path).
`NACK(FILE_NOT_FOUND)` if an `R:` pointer file doesn't exist, or if the final resolved folder
itself doesn't exist on its volume (individual glyph files inside it are checked lazily instead,
§12.6.1 — this only validates the directory). `NACK(VOLUME_NOT_PRESENT)` if the resolved volume is
SD and no card is currently inserted. On success, replaces the active custom-font folder and
discards every cached glyph (§12.6.1) from whatever folder was active before, then `ACK`s.

### 12.7 `0x0305` DRAW_IMAGE
```
0 2 X   2 2 Y   4 1 DRAW_MODE   5 1 FLAGS (§2.1)
6 1 VOLUME (§14): 0x00 SD, 0x01 INTERNAL, 0x02 PSRAM
7 1 PATH_LEN (u8, max 255)
8 PATH_LEN PATH   UTF-8 path, e.g. "/icons/wifi.epi" (SD) or "wifi" (INTERNAL/PSRAM)
```

Image dimensions come from the referenced file's own header.

#### Image file format (`.epi`, referenced by `DRAW_IMAGE` and §14 file transfer)

Reuses the exact §6 RAW/RLE encoder.

```
Offset  Size  Field              Notes
0       4     MAGIC              ASCII "EPI1"
4       1     FORMAT_VERSION     0x01
5       2     WIDTH              u16 LE
7       2     HEIGHT             u16 LE
9       1     ENCODING           0x00 RAW / 0x01 RLE_PACKBITS (applies to both streams below)
10      1     FLAGS              bit0 HAS_MASK
11      4     COLOR_DECODED_LEN  u32 LE
15      4     COLOR_ENCODED_LEN  u32 LE
19      ...   COLOR_DATA
[if HAS_MASK:]
        4     MASK_DECODED_LEN   u32 LE (always == COLOR_DECODED_LEN)
        4     MASK_ENCODED_LEN   u32 LE
        ...   MASK_DATA          1=opaque, 0=transparent; same RAW/RLE encoding
```

### 12.8 `0x0306` REFRESH
```
0 1 MODE   0x00 = partial refresh of the union of bounding boxes deferred since the last refresh
           0x01 = force full-panel refresh
```

### 12.9 `0x0307` SHIFT_REGION
```
0  2 X           region to shift within (u16 LE)
2  2 Y
4  2 WIDTH
6  2 HEIGHT
8  1 DIRECTION   0x00 LEFT, 0x01 RIGHT, 0x02 UP, 0x03 DOWN, 0x04-0xFF reserved
9  2 STEP        u16 LE, pixels to shift by (0 = no-op)
11 1 FILL_COLOR  COLOR value (§12.1) for the vacated strip
12 1 FLAGS (§2.1)
```

Shifts the region's content by STEP pixels in DIRECTION; content moved past the region's own edge
is discarded (never wraps to the opposite edge or outside the region), and the vacated strip
(STEP pixels wide/tall, depending on DIRECTION) is filled with FILL_COLOR. `STEP >= WIDTH` (for
LEFT/RIGHT) or `STEP >= HEIGHT` (for UP/DOWN) discards the entire region's content and fills it
solid. No DRAW_MODE field - like `CLEAR_REGION`, this always replaces (both the shifted content in
its new location and the fill are plain overwrites, not composited against what's underneath).
Primarily intended for scrolling (a ticker, a log panel, incrementally revealed text) without
resending the whole region's content for each step - typically paired with `FLAGS.REFRESH_NOW=0`
across several steps and one trailing `REFRESH` (§12.8), or a single immediate step with
`REFRESH_NOW=1`.

### 12.10 `0x0308` SET_CLIP_REGION
```
0 2 X        u16 LE
2 2 Y
4 2 WIDTH    0x0000 resets the clip to the full logical canvas (X/Y/HEIGHT ignored in that case)
6 2 HEIGHT
```

Constrains every subsequent write-capable command in this family (§12.2–§12.7, §12.9, §12.11) to
this rectangle: a pixel that command would otherwise touch outside the current clip region is
silently left untouched, the same way a pixel outside the panel already is — never a NACK, matching
this whole family's "overflow is just ignored" convention (§12's own "no alignment constraint"
note). This is device-global state, not a per-command flag like FLAGS/DRAW_MODE — it persists
across commands (and across which transport is used) until changed again or the device reboots,
starting as the full logical canvas by default. `FULL_IMAGE_TRANSFER`/`PARTIAL_IMAGE_TRANSFER`
(§6/§7) are **not** affected by the clip region — those already have their own explicit bounds
validation (NACK on violation), a different and deliberately stricter contract than "silently
clipped" that this doesn't change.

X/Y/WIDTH/HEIGHT are **logical** coordinates (§12.12) — the same space every other §12 command's
own X/Y is in — and are stored exactly as given, never pre-transformed. That has two consequences,
by design: the clip window automatically "follows" a later `SET_ORIENTATION` change (the same
numbers get reinterpreted against whatever the logical canvas's width/height are *now*, so a
portrait-mode clip box doesn't need to be re-issued after rotating), while staying completely
unaffected by `SET_DRAW_OFFSET` (§12.13) — an independent, orthogonal pan of drawing content within
this same logical frame, not a redefinition of where the clip window itself sits.

Typical use: narrow the clip to a sub-area (e.g. a scrolling log panel or status box) so `DRAW_TEXT`
and `SHIFT_REGION` operating on that area can't accidentally paint or scroll outside its bounds even
if their own X/Y/WIDTH math doesn't line up perfectly — then reset to the full logical canvas
(`WIDTH=0`) once done.

### 12.11 `0x0309` COPY_REGION
```
0  2 SRC_X   u16 LE
2  2 SRC_Y
4  2 DST_X
6  2 DST_Y
8  2 WIDTH
10 2 HEIGHT
12 1 FLAGS (§2.1)
```

Copies `[SRC_X,SRC_Y,WIDTH,HEIGHT)` to `[DST_X,DST_Y,WIDTH,HEIGHT)` within the working buffer (§2.1)
— a plain overwrite, like `SHIFT_REGION`/`CLEAR_REGION`, no DRAW_MODE field. Source and destination
rectangles may overlap in either direction; the result is as if the entire source were read before
any destination pixel was written (no visible corruption from reading-while-writing). The **source**
read is not constrained by the current clip region (§12.10) — only the destination write is,
consistent with every other command in this family; a source rectangle extending past the logical
canvas's own edges reads those out-of-bounds pixels as white. Both SRC and DST are subject to
`SET_DRAW_OFFSET`/`SET_ORIENTATION` (§12.12/§12.13), same as every other command here. Lets a shape
or icon already drawn once be duplicated elsewhere ("copy + paste") without resending its pixel
data from the PC.

### 12.12 `0x030B` SET_ORIENTATION
```
0 1 ROTATION   0x00 0°, 0x01 90° CW, 0x02 180°, 0x03 270° CW
1 1 FLAGS      bit0 MIRROR_H, bit1 MIRROR_V, bits2-7 reserved
```

Rotates and/or mirrors the **logical canvas** that every other §12 coordinate — every drawing
primitive's own X/Y, `SET_CLIP_REGION`'s window, `SET_DRAW_OFFSET`'s pan — is expressed in, relative
to the physical panel. `ROTATION=90/270` swap the logical canvas's width/height relative to the
physical panel's fixed `DISPLAY_WIDTH_PX`/`DISPLAY_HEIGHT_PX` (§5.2) — this is what makes a portrait
orientation possible without the panel itself changing shape. Mirroring is applied in logical space,
before rotation. This is device-global, persistent state, like `SET_CLIP_REGION` — it applies to
every command already queued as well as everything sent afterward (there's no "which orientation
was active when this was originally drawn" tracking; already-flushed pixels don't retroactively
move, only new writes use the new mapping), and persists until changed again or the device reboots,
starting at `ROTATION=0`/no mirror. `FULL_IMAGE_TRANSFER`/`PARTIAL_IMAGE_TRANSFER` (§6/§7) are
**not** affected — like the clip region, those keep addressing the physical panel directly.

The mapping is exact for all four rotations and both mirror axes since these are among a
rectangle's 8 symmetries (a rotation/mirror always maps an axis-aligned rectangle to another
axis-aligned rectangle) — verified on real hardware: a fixed logical anchor near one corner of the
logical canvas lands near a different physical corner under each `ROTATION` value (and each
rotation's glyphs render individually rotated in place, not just moved, since every pixel of a
shape goes through the same logical→physical mapping), and each mirror axis flips both the position
and the readability of text drawn under it.

### 12.13 `0x030A` SET_DRAW_OFFSET
```
0 2 DX   s16 LE
2 2 DY   s16 LE
```

A persistent `(dx,dy)` pan applied to every §12 read/write's own logical coordinate, before the clip
check and before `SET_ORIENTATION`'s rotation/mirror mapping. Lets a caller address logical
positions outside `[0,logicalWidth)x[0,logicalHeight)` — e.g. an anchor that, once shifted, lands
partly or fully off-canvas — without every individual drawing command's own X/Y wire field needing
to be signed: an out-of-canvas position (after the offset is applied) is simply silently dropped,
the same way any other off-panel/off-clip geometry already is. `(0,0)` is the identity value, so
there's no separate reset sentinel needed (unlike `SET_CLIP_REGION`'s `WIDTH=0`/`HEIGHT=0`) — just
send `(0,0)` to return to unshifted drawing. Device-global, persistent state, like `SET_CLIP_REGION`
and `SET_ORIENTATION`; starts at `(0,0)`. Typical use: smoothly scroll content partly off one edge
of the canvas by re-sending the same drawing command at the same logical anchor while decreasing
(or increasing) the offset each step, rather than needing negative wire coordinates.

### 12.14 `0x030C` DRAW_IMAGE_ROW
```
0  2 X          anchor = top-left of the row's bounding box
2  2 Y
4  2 WIDTH      0 = no reference box at all (images simply packed left-to-right from X, ALIGN
                 ignored — exactly like DRAW_TEXT's WIDTH=0); >0 = alignment reference box
6  1 ALIGN      0x00 LEFT, 0x01 CENTER, 0x02 RIGHT, 0x03 BLOCK (auto-spacing — meaningful only
                 when WIDTH>0, same as DRAW_TEXT's ALIGN)
7  2 SPACING    u16 LE — fixed px gap between images; ignored when ALIGN=BLOCK
9  1 DRAW_MODE
10 1 FLAGS (§2.1)
11 1 VOLUME (§14) — shared by every image in this row
12 1 COUNT      number of images, 1–255 (0 is invalid)
13 ... repeated COUNT times:
      1 PATH_LEN
      PATH_LEN PATH   UTF-8 path
```

Draws `COUNT` `.epi` images (§12.7, all from the same `VOLUME`) in a horizontal row — e.g. a strip
of status icons, or a "big number" composed from per-digit glyph images. Each image keeps its own
natural width/height from its own file header — **never scaled** — and all share one `Y` (top-
aligned) and one `DRAW_MODE`; total row height (for dirty-region/flush purposes) is the tallest
image's own height.

`BLOCK` distributes the leftover space (`WIDTH` minus the sum of every image's own width) evenly
across the gaps *between* images — the first image sits flush at `X`, the last image's right edge
sits flush at `X+WIDTH` — ignoring `SPACING` entirely; ties to plain left-packing when `COUNT==1`
(nothing to distribute across). Every other `ALIGN` value uses `SPACING` as a fixed gap instead. As
with `DRAW_TEXT`, an oversized row (wider than `WIDTH`) is never an error — it simply overflows the
box, subject to whatever the current clip region allows through.

### 12.15 `0x030D` FILL_IMAGE
```
0  2 X
2  2 Y
4  2 WIDTH      target fill width - used when TILE_MODE=HORIZONTAL/BOTH; ignored (the image's own
                 natural width is used, a single column) when TILE_MODE=VERTICAL
6  2 HEIGHT     target fill height - used when TILE_MODE=VERTICAL/BOTH; ignored (the image's own
                 natural height is used, a single row) when TILE_MODE=HORIZONTAL
8  1 TILE_MODE  0x00 HORIZONTAL, 0x01 VERTICAL, 0x02 BOTH
9  1 DRAW_MODE
10 1 FLAGS (§2.1)
11 1 VOLUME (§14)
12 1 PATH_LEN
13 PATH_LEN PATH   UTF-8 path to a `.epi` file (§12.7)
```

Tiles one `.epi` image repeatedly to cover `[X,Y,WIDTH,HEIGHT)` — a background pattern
(`TILE_MODE=BOTH`), or a decorative border strip (`HORIZONTAL`/`VERTICAL`, tiling along just one
axis while the other keeps the image's own natural size). A partial tile at the target rectangle's
far edge is simply clipped, never specially cropped or skipped — the same "overflow is just
ignored" convention every other §12 command already follows. Each tile is masked exactly like a
plain `DRAW_IMAGE` (§12.7's own transparent-pixel handling applies per tile), so a masked tile can
create textured/stippled overlays, not just opaque patterns. `WIDTH`/`HEIGHT` on the axis the
current `TILE_MODE` doesn't use are simply ignored, not validated.

### 12.16 `0x030E` FAST_CLEAR
```
0 1 COLOR   0x00 WHITE, 0x01 BLACK
1 1 FLAGS (§2.1)
```

Fills the *entire physical panel's* working buffer with a solid `COLOR` via a single raw write
(no per-pixel loop) — deliberately **the one §12 command that bypasses `SET_CLIP_REGION`
(§12.10), `SET_DRAW_OFFSET` (§12.13), and `SET_ORIENTATION` (§12.12) entirely**, unlike every
other primitive in this section, which all apply these three. No `X`/`Y`/`WIDTH`/`HEIGHT` field
exists on the wire — there is nothing to specify, since it always covers the whole panel; skipping
the clip/offset/orientation pipeline changes nothing about the result (every pixel ends up the
same value regardless of what any of those three would otherwise have done to a partial write) and
exists purely so this can be a true single-pass buffer fill rather than a per-pixel walk through
that pipeline. There is no persistent "current brush/background color" concept anywhere in this
protocol to default `COLOR` from — every command that draws a solid fill (`CLEAR_REGION`, §12.5;
this one) always takes it explicitly, the same as every other primitive's own `COLOR` field.

`0x0310`–`0x03FF` reserved.

## 13. Configuration (`0x0400`–`0x04FF`)

### 13.1 Backup / restore (`0x0400`–`0x0402`)

`0x0400 CONFIG_BACKUP_REQUEST` (PC → device): empty payload.

`0x0401 CONFIG_BACKUP_DATA` (device → PC): opaque, versioned TLV blob:
```
Offset  Size  Field           Notes
0       1     CONFIG_VERSION  Device-defined
1..     ...   TLV entries     Back-to-back §5.1 TLV entries, device-defined TYPE namespace
```

`0x0402 CONFIG_RESTORE` (PC → device): the exact blob previously received. Device applies entries
it recognizes and skips unknown `TYPE`s.

Every setting in §13.2 below that's saved with `FLAGS.PERSIST` is, by construction, already
included in this backup blob with no protocol changes needed — `CONFIG_BACKUP_DATA` just
serializes whatever persisted state exists.

**Layered config: SD overrides NVS** (design note 72), requested directly for easy device
repurposing. On boot, firmware checks `VOLUME=SD` for a fixed path, `/device.config` — if present,
its bytes are parsed as an ordinary `CONFIG_BACKUP_DATA` blob (the exact same `CONFIG_VERSION` +
TLV bytes this section already defines, just persisted as a file instead of sent over the wire) and
applied for that boot session, taking priority over whatever's in NVS. Not written back to NVS —
pull the card and the next boot falls back to the NVS-persisted value (or the hardcoded default, if
neither exists). This means backing up one unit's config (`CONFIG_BACKUP_REQUEST`), saving the blob
to `/device.config` on an SD card, and moving that card to a *different* physical unit re-identifies
it as the first unit on its next boot — a config layer that travels with the card, not the board.
No new file format was needed since the wire format already fit.

### 13.2 Network configuration (`0x0403`–`0x040A`)

WiFi and BLE are each independently configurable — enable/disable, WiFi credentials, and a BLE
pairing PIN — over **any** connected transport, including the one being reconfigured (see the
ACK-then-apply note below). **Serial has no enable/disable command at all, deliberately** — it's
always on, giving a guaranteed physical-access recovery path if WiFi/BLE ever end up
misconfigured into an unreachable state (wrong password, radio disabled, forgotten BLE PIN);
there's no separate "factory reset" command because reconfiguring over Serial already covers that.

Credentials and the BLE PIN are **write-only on the wire** — `SET_WIFI_CONFIG` and `SET_BLE_PIN`
set them, but neither status response ever echoes a stored password or PIN back, only whether one
is currently set. Firmware should persist these (and the enable/disable flags, when
`FLAGS.PERSIST` is set) in the `nvs` partition already present in `default_8MB.csv` (§16) —
distinct from the `LittleFS`-backed `VOLUME=INTERNAL` (§14), which is for user files, not device
settings.

**Firmware note, same ordering precedent as `OTA_APPLY` (§16.2)**: if `SET_WIFI_ENABLED(0)`,
`SET_BLE_ENABLED(0)`, or a `SET_WIFI_CONFIG` with new credentials arrives over the very
transport/network it's about to change, send the `ACK` and let it flush *before* applying the
change — otherwise the caller never sees confirmation before that connection drops.

#### `0x0403` SET_WIFI_CONFIG (PC → device)
```
Offset                    Size          Field         Notes
0                         1             SSID_LEN      u8, 1-32 (802.11 SSID max)
1                         SSID_LEN      SSID          UTF-8
1+SSID_LEN                1             PASSWORD_LEN  u8, 0-63 (0 = open network; WPA2-PSK max)
2+SSID_LEN                PASSWORD_LEN  PASSWORD      UTF-8, write-only
2+SSID_LEN+PASSWORD_LEN   1             FLAGS         bit0 PERSIST (save to NVS, becomes the
                                                       power-on default; 0 = this boot only)
                                                       bit1 CONNECT_NOW (attempt to (re)connect
                                                       immediately; 0 = store only)
                                                       bits2-7 reserved
```
`NACK(BAD_PARAMETERS)` if `SSID_LEN=0` or either length exceeds its max. `ACK` means "accepted and
stored/applied as requested" — **not** "connected": association can take several seconds, poll
`WIFI_STATUS_REQUEST` for the actual outcome rather than blocking the ACK on it.

#### `0x0404` WIFI_STATUS_REQUEST (PC → device) / `0x0405` WIFI_STATUS_RESPONSE (device → PC)
```
Request:  empty payload
Response: 0            1         ENABLED     radio on/off, independent of association state
          1            1         CONNECTED   currently associated to an access point
          2            1         SSID_LEN    length of the currently configured SSID (0 if none)
          3            SSID_LEN  SSID        UTF-8 - never the password (write-only)
          3+SSID_LEN   4         IP_ADDRESS  4 raw bytes, dotted-quad octets in order A.B.C.D;
                                              all zero if not connected
```

#### `0x0406` SET_WIFI_ENABLED (PC → device)
```
0 1 ENABLED (0x00 disable - drops any active TCP connection as a side effect; 0x01 enable)
1 1 FLAGS (bit0 PERSIST - becomes the power-on default)
```

#### `0x0407` SET_BLE_ENABLED (PC → device)
```
0 1 ENABLED   1 1 FLAGS (bit0 PERSIST)
```

#### `0x0408` SET_BLE_PIN (PC → device)
```
0 1 HAS_PIN  0x00 = remove (open/Just Works pairing), 0x01 = set a 6-digit numeric passkey
1 4 PIN      u32 LE, 000000-999999 (BLE Secure Simple Pairing passkey range); ignored if HAS_PIN=0
5 1 FLAGS    bit0 PERSIST
```
`NACK(BAD_PARAMETERS)` if `HAS_PIN=1` and `PIN > 999999`.

#### `0x0409` BLE_STATUS_REQUEST (PC → device) / `0x040A` BLE_STATUS_RESPONSE (device → PC)
```
Request:  empty payload
Response: 0 1 ENABLED
          1 1 CONNECTED     currently has a central connected
          2 1 HAS_PIN       whether a pairing PIN is set - never echoes the value itself
          3 6 BLE_ADDRESS   raw 6-byte BLE MAC address
```

### 13.3 Device name (`0x040B`)

A single user-assigned label, distinct from `DEVICE_MODEL` (§5.2, a fixed hardware/firmware
string like `"CrowPanel-4.2-EPD"`) — this is the name a *person* gave *this particular unit*
("Living Room Display"), used as the BLE advertised device name, the WiFi DHCP hostname, and
exposed in the handshake for general identification. Matters as soon as more than one unit exists
on the same network or in BLE range — without it every device would advertise/resolve identically.

**Default, before any `SET_DEVICE_NAME` is ever sent**: `"CrowPanel-XXXXXX"`, where `XXXXXX` is the
last 3 bytes of the device's BLE/WiFi MAC address in hex (e.g. `"CrowPanel-3851DC"`) — unique out
of the box across multiple deployed units, no configuration required first.

#### `0x040B` SET_DEVICE_NAME (PC → device)

```
Offset      Size      Field     Notes
0           1         NAME_LEN  u8, 0-32. 0 = clear any configured name, revert to the
                                 MAC-derived default above.
1           NAME_LEN  NAME      UTF-8, present only if NAME_LEN > 0
1+NAME_LEN  1         FLAGS     bit0 PERSIST (save to NVS, becomes the power-on default)
```

`NACK(BAD_PARAMETERS)` if `NAME_LEN > 32`.

**Firmware note**: applying this fully may not be instantaneous for every consumer of the name.
Renaming the BLE advertised name can generally take effect on the current advertising session; the
WiFi DHCP hostname is typically only sent to the router as part of establishing a lease, so a
name change may not actually reach the router until the next WiFi (re)connect. `ACK` means
"accepted and stored", not "every system that shows this name has already refreshed it" — same
reasoning as `SET_WIFI_CONFIG`'s `ACK` (§13.2, design note 27).

Exposed in the handshake as a new capability, `DEVICE_NAME` (§5.2), so a client learns the current
name on connect without a separate request.

### 13.4 Access-control PINs (`0x040C`–`0x040D`) — see §5.3 for the full access-control model

Both commands share one shape:
```
0            1         HAS_PIN  0x00 = remove (this tier becomes/stays open), 0x01 = set
1            1         PIN_LEN  u8, 0-64, meaningful only if HAS_PIN=1
2            PIN_LEN   PIN      UTF-8, present only if HAS_PIN=1
2+PIN_LEN    1         FLAGS    present for §13 shape consistency, not actually consulted (see
                                 below) - bit0 PERSIST is the usual convention elsewhere in §13
```
`NACK(BAD_PARAMETERS)` if `HAS_PIN=1` and `PIN_LEN > 64`, or if `HAS_PIN=0` and `PIN_LEN != 0`.

**`0x040C` SET_USAGE_PIN** / **`0x040D` SET_ADMIN_PIN** (both PC → device, both **ADMIN**-gated,
§5.3 — including `SET_USAGE_PIN` itself, so a non-admin can never tighten or loosen the usage
gate). Unlike `SET_BLE_PIN`'s numeric-only pairing code (§13.2), this PIN is an arbitrary UTF-8
string. Always persists regardless of `FLAGS` — the same documented exception `SET_BLE_PIN`
already establishes, since a session-only access-control PIN would be just as meaningless — and,
unlike the BLE PIN (only ever applied at the next boot), also takes effect immediately, live, on
the very next dispatched frame.

`0x040E`–`0x04FF` reserved (future: static IP config, WiFi AP-mode config — not designed now).

## 14. Storage / file management (`0x0600`–`0x06FF`)

```
VOLUME (1 byte, shared enum, used here, §12.7, and §18): 0x00 SD, 0x01 INTERNAL, 0x02 PSRAM,
                                                          0x03-0xFF reserved
```

Volume *capability* comes from handshake `FEATURE_BITMASK` bits 5/6; *live* presence (SD hot-plug)
is queried via `STORAGE_INFO_REQUEST`/`RESPONSE`. `PSRAM` is a universal third, flat (no
subdirectories), session-only volume — everything on it is lost on reboot — meant for
preloading/caching session-specific data or temporary saves without wearing SD/internal flash;
every command in this family (and `DRAW_IMAGE`/`DRAW_IMAGE_ROW`, §12.7/§12.14) accepts it exactly
like SD/INTERNAL, with no special-casing needed at the wire level. `STORAGE_INFO_RESPONSE` reports
`PRESENT=true` and a fixed capacity budget for it (not the whole chip's PSRAM, which is shared with
other subsystems).

### 14.1 `0x0600` FILE_LIST_REQUEST / `0x0601` FILE_LIST_RESPONSE
```
Request:  0 1 VOLUME   1 1 PATH_LEN   2 PATH_LEN PATH
Response: 0 2 ENTRY_COUNT (u16 LE), then repeated:
            1 NAME_LEN, NAME_LEN NAME (UTF-8), 1 ENTRY_TYPE (0=FILE,1=DIR), 4 SIZE (u32 LE)
```

### 14.2 `0x0602` FILE_DOWNLOAD_REQUEST / `0x0603` FILE_DATA
```
Request:  0 1 VOLUME   1 1 PATH_LEN   2 PATH_LEN PATH
Response: 0 4 FILE_LEN (u32 LE)   4 FILE_LEN DATA
```

### 14.3 `0x0604` FILE_UPLOAD
```
0 1 VOLUME   1 1 PATH_LEN   2 PATH_LEN PATH   (2+PATH_LEN) 4 FILE_LEN (u32 LE)
(2+PATH_LEN+4) FILE_LEN DATA
```
Overwrites any existing file. Missing parent directories (`VOLUME=SD`/`INTERNAL` — `PSRAM` is flat,
no subdirectories) are created automatically, as many levels deep as needed.
`NACK(INSUFFICIENT_STORAGE)` / `NACK(VOLUME_NOT_PRESENT)` as needed.

### 14.4 `0x0605` FILE_DELETE
```
0 1 VOLUME   1 1 PATH_LEN   2 PATH_LEN PATH
```

### 14.5 `0x0606` STORAGE_INFO_REQUEST / `0x0607` STORAGE_INFO_RESPONSE
```
Request:  0 1 VOLUME
Response: 0 1 VOLUME   1 1 PRESENT   2 4 TOTAL_BYTES (u32 LE)   6 4 FREE_BYTES (u32 LE)
```

`TOTAL_BYTES`/`FREE_BYTES` **saturate at `0xFFFFFFFF`** for a volume whose true byte count exceeds
what a u32 can hold (~4.29 GB) — real hardware testing with a 32 GB SD card confirmed this is a
routine case, not a hypothetical edge case, for any SD card sold today. A client should treat
`0xFFFFFFFF` as "at least ~4.29 GB", not an exact count (design note 54).

No resumable/segmented transfer is designed — a CRC failure on a large upload means a full retry.

### 14.6 `0x0608` FILE_COPY
```
0 1 SRC_VOLUME   1 1 SRC_PATH_LEN   2 SRC_PATH_LEN SRC_PATH
(2+SRC_PATH_LEN) 1 DST_VOLUME   (3+SRC_PATH_LEN) 1 DST_PATH_LEN
(4+SRC_PATH_LEN) DST_PATH_LEN DST_PATH
```
`ACK`/`NACK` only, no dedicated response payload. Copies a file, optionally across volumes (e.g.
`SD`/`INTERNAL` → `PSRAM`, to stage a macro for fast, session-only playback without a full
PC-mediated download-then-reupload round trip). Overwrites any existing file at
`DST_VOLUME`/`DST_PATH`, matching `FILE_UPLOAD`'s own semantics (§14.3). `NACK(FILE_NOT_FOUND)` if
`SRC_PATH` doesn't exist, `NACK(INSUFFICIENT_STORAGE)`/`NACK(VOLUME_NOT_PRESENT)` as needed
(design note 88).

### 14.7 `0x0609` FILE_RENAME
```
0 1 VOLUME   1 1 SRC_PATH_LEN   2 SRC_PATH_LEN SRC_PATH
(2+SRC_PATH_LEN) 1 DST_PATH_LEN   (3+SRC_PATH_LEN) DST_PATH_LEN DST_PATH
```
`ACK`/`NACK` only. Renames/moves a file within a single volume — unlike `FILE_COPY`, there is no
cross-volume form (matches POSIX `rename()` semantics; both volume backends implement this as a
cheap in-place move with no data copy). Overwrites any existing file at `DST_PATH`, matching
`FILE_UPLOAD`'s own semantics — the intended use is swapping in a new version of a staged file
(e.g. a macro) without a delete-then-reupload gap: upload the new content under a temporary name,
then `FILE_RENAME` it over the final name. `NACK(FILE_NOT_FOUND)`/`NACK(VOLUME_NOT_PRESENT)` as
needed (design note 88).

## 15. GPIO configuration and control (`0x0700`–`0x07FF`)

`PIN_ID` is the raw MCU GPIO number. The handshake's `AVAILABLE_GPIO_PINS` (§5.2) tells the PC
which numbers are safe to use.

### 15.1 `0x0700` GPIO_CONFIGURE
```
0 1 PIN_ID
1 1 MODE    0x00 INPUT, 0x01 INPUT_PULLUP, 0x02 INPUT_PULLDOWN, 0x03 OUTPUT,
            0x04 PWM_OUTPUT (reserved), 0x05 ANALOG_INPUT (reserved), 0x06-0xFF reserved
2 1 FLAGS   bit0 ENABLE_CHANGE_EVENTS (INPUT* modes only). bits1-7 reserved.
```
`NACK(PIN_UNAVAILABLE)` for an unexposed pin, `NACK(BAD_PARAMETERS)` for an unimplemented MODE.

### 15.2 `0x0701` GPIO_WRITE
```
0 1 PIN_ID   1 1 VALUE (0x00 LOW, 0x01 HIGH)
```
Only valid for a pin configured `OUTPUT`.

### 15.3 `0x0702` GPIO_READ_REQUEST / `0x0703` GPIO_READ_RESPONSE
```
Request:  0 1 PIN_ID
Response: 0 1 PIN_ID   1 1 VALUE   2 1 MODE (echoed)
```

### 15.4 `0x0704` GPIO_EVENT (device → PC, via notify)
```
0 1 PIN_ID   1 1 VALUE   2 4 TIMESTAMP_MS (u32 LE)
```
Pushed for a pin with `ENABLE_CHANGE_EVENTS` set.

### 15.5 `0x0705` GPIO_PLAY_PATTERN (PC → device)

Plays a timed on/off pulse sequence on an `OUTPUT` pin *asynchronously* — the device times it
internally (its own `millis()`/timer bookkeeping in `loop()`, not a blocking `delay()` sequence, so
polling the other transports keeps working while a pattern plays), so the PC fires one command and
moves on rather than driving the timing itself over the wire. One generic mechanism serves both an
LED "blink" and a buzzer "beep"/"double-beep"/"long flash" — which one it looks/sounds like is
purely a function of what's physically wired to the pin, not something the protocol needs to know
about; the PC-side library is expected to offer named convenience helpers (`beep()`,
`doubleBeep()`, `longFlash()`, ...) that just construct the right pulse sequence underneath.

```
Offset             Size           Field         Notes
0                  1              PIN_ID
1                  1              FLAGS         bit0 INITIAL_LEVEL (0=starts LOW, 1=starts HIGH)
                                                 bit1 REPEAT_FOREVER (ignore REPEAT_COUNT; loop
                                                 until cancelled, see below)
                                                 bits2-7 reserved
2                  1              STEP_COUNT    u8, 1-255 - number of timed steps
3                  1              REPEAT_COUNT  u8 - additional full-sequence replays after the
                                                 first (0 = play once). Ignored if REPEAT_FOREVER.
4                  STEP_COUNT*2   STEPS         STEP_COUNT × u16 LE `duration_ms` - how long to
                                                 hold before flipping to the next step's level.
                                                 Levels simply alternate starting from
                                                 INITIAL_LEVEL - no per-step level field needed.
```

Examples: a single beep/blink is one step (`STEP_COUNT=1`, e.g. 200 ms, `INITIAL_LEVEL=1`); a
double-beep is three steps (on 100 ms, off 100 ms, on 100 ms); a long beep/flash is one longer step
(e.g. 800 ms).

The pin always ends **LOW** once the pattern (and all repeats) finish — not whatever level the
last step happened to leave it at — so an indicator/buzzer has one predictable idle state. A
pattern in progress on a pin is cancelled and replaced by: a new `GPIO_PLAY_PATTERN` for that pin,
a `GPIO_WRITE` for that pin (manual control wins outright, and also sets a known level), or a
`GPIO_CONFIGURE` that reconfigures the pin away from `OUTPUT`. There is no separate "stop" command
— sending `GPIO_WRITE(pin, LOW)` is the stop command.

`NACK(PIN_UNAVAILABLE)` for an unexposed pin, `NACK(BAD_PARAMETERS)` if the pin isn't currently
configured `OUTPUT`, `STEP_COUNT=0`, or `STEP_COUNT` doesn't match the actual payload length. `ACK`
means "accepted and started", not "finished" — consistent with this being fire-and-forget by
design; there's no completion event in v1 (see design notes).

`0x0706`–`0x07FF` reserved (PWM, analog, pull-strength — not designed yet).

## 16. OTA firmware update (`0x0800`–`0x08FF`)

Updates firmware over whichever transport is connected (WiFi/TCP, BLE, or Serial), using the same
Logical Frame mechanism as everything else — a firmware image is just a large payload, no new
transfer mechanism is introduced. `PAYLOAD_LEN` is already a u32 (§2), so one `OTA_INSTALL` frame
carries the entire image; on BLE it's fragmented to the negotiated MTU exactly like every other
transport (§3.1), exactly like a large file upload (§14). Relies on the ESP32's standard
dual-OTA-partition scheme
(`esp_ota_ops`/Arduino `Update`): the running firmware writes the new image into the *inactive*
partition, so a failure mid-transfer leaves the currently running firmware completely untouched.

### 16.1 `0x0800` OTA_INSTALL (PC → device)

```
Offset       Size       Field       Notes
0            4          TOTAL_LEN   u32 LE — must equal the IMAGE_DATA length that follows
4            1          HASH_ALGO   0x00 NONE, 0x01 SHA-256, 0x02 MD5
5            1          HASH_LEN    u8 — 0 (NONE), 16 (MD5), or 32 (SHA-256)
6            HASH_LEN   HASH        Expected hash of IMAGE_DATA, verified before applying
6+HASH_LEN   1          FLAGS       bit0 APPLY_NOW: 1 = reboot into the new image immediately
                                     once written and verified; 0 = stage it and wait for an
                                     explicit OTA_APPLY (§16.2). bits1-7 reserved.
7+HASH_LEN   TOTAL_LEN  IMAGE_DATA  Raw firmware image (the same `firmware.bin` `pio run`
                                     produces)
```

`NACK(INSUFFICIENT_STORAGE)` if `TOTAL_LEN` exceeds the inactive OTA partition's size,
`NACK(BAD_PARAMETERS)` if `HASH_LEN` doesn't match `HASH_ALGO` or `TOTAL_LEN` disagrees with the
actual payload size, `NACK(BUSY)` if an OTA is already staged/in progress,
`NACK(OTA_HASH_MISMATCH)` (§10) if the received image's computed hash doesn't match `HASH`. `ACK`
only once the image is fully written and (if a hash was supplied) verified — with `APPLY_NOW=0`
this means "staged, ready for `OTA_APPLY`"; with `APPLY_NOW=1` the device sends this `ACK` and
*then* reboots (see the firmware note below).

**Wire-polarity-style convention, stated explicitly so it isn't assumed**: skipping the hash
(`HASH_ALGO=NONE`) is allowed, but strongly discouraged — per-frame `CRC16` (§2) only protects each
individual Logical Frame from bit corruption; it does not protect against, e.g., the PC library
sending a truncated or wrong file. A whole-image hash is the only thing that gives an end-to-end
guarantee the exact intended `firmware.bin` was received.

### 16.2 `0x0801` OTA_APPLY (PC → device)

Empty payload. Reboots into a previously staged image (from an `OTA_INSTALL` sent with
`APPLY_NOW=0`). `NACK(OTA_NOT_STAGED)` if nothing is staged.

**Firmware note**: for both this command and `OTA_INSTALL` with `APPLY_NOW=1`, the device must
send its `ACK` and let the transport actually flush it out *before* calling `ESP.restart()` —
otherwise the PC never sees confirmation that the update was accepted before the connection drops
for the reboot.

### 16.3 `0x0802` OTA_STATUS_REQUEST (PC → device) / `0x0803` OTA_STATUS_RESPONSE (device → PC)

Request: empty payload.
```
Response:
0  1                     RUNNING_SLOT           0x00 / 0x01 (which OTA partition is currently
                                                 running) — informational
1  1                     PENDING_VERIFICATION   0x01 if this boot is running firmware that
                                                 hasn't yet been confirmed via OTA_CONFIRM
                                                 (§16.4) and is still subject to automatic
                                                 rollback; 0x00 otherwise
2  1                     RUNNING_VERSION_LEN    u8
3  RUNNING_VERSION_LEN   RUNNING_VERSION        UTF-8 — same value as the handshake's
                                                 FIRMWARE_VERSION capability (§5.2), exposed here
                                                 too so a client can check it without a full
                                                 re-handshake
```

### 16.4 `0x0804` OTA_CONFIRM (PC → device)

Empty payload. Marks the currently running (freshly updated) firmware as valid — cancels ESP-IDF's
automatic rollback-on-next-boot for this image (`esp_ota_mark_app_valid_cancel_rollback()`). This
is the safety net that makes OTA over a wireless link low-risk: if a new image is bad enough that
the PC can never re-establish a connection to send `OTA_CONFIRM`, firmware should fall back to
auto-reverting to the previous slot after a boot/self-test timeout even without an explicit
`OTA_ROLLBACK` (§16.5) — the exact timeout and self-test criteria are a firmware policy choice, not
specified here.

### 16.5 `0x0805` OTA_ROLLBACK (PC → device)

Empty payload. Explicitly reverts to the previous OTA slot and reboots, without waiting for the
auto-rollback timeout — for a PC that has already decided the new firmware is bad (e.g. failed a
post-update self-check) and wants to recover immediately rather than wait.

`0x0806`–`0x08FF` reserved (future: OTA over a URL fetched by the device itself, staged/delta
updates — not designed now).

## 17. Power management (`0x0900`–`0x09FF`)

E-ink retains its image with zero power once flipped (§2.1) — neither mode below needs to blank
the panel first, or even touch it at all. That's the point: the display keeps showing whatever it
last showed while the MCU sleeps, which is what makes deep power savings actually useful for a
device whose entire job is to display something.

Two modes, mapping directly onto the ESP32's own sleep hardware:

- **`LOW_POWER`** (ESP32 *light sleep*): CPU paused, RAM/PSRAM retained, wakes quickly and resumes
  where it left off (not a reboot). Wakeable by Serial activity, BLE activity (if kept
  connectable), or a timer. WiFi is unconditionally suspended for the duration — regardless of its
  persisted `SET_WIFI_ENABLED` (§13.2) state — since staying associated to an AP is too
  power-hungry to justify through a low-power interval; it's restored to that persisted state on
  wake. This is also the mode to use purely to reduce heat/CPU power draw even when not
  battery-powered, not only for battery life.
- **`HARD_SLEEP`** (ESP32 *deep sleep*): everything powered down as hard as the chip allows.
  RAM/PSRAM contents are **lost** — waking is indistinguishable from a power-on reset except via
  `esp_sleep_get_wakeup_cause()`, so the PC should treat a device coming back from `HARD_SLEEP`
  exactly like one that just power-cycled (reconnect, re-handshake; any working-buffer edits that
  weren't flipped to the panel before sleeping are gone — the panel's own displayed image is
  unaffected, since that's held by the e-ink cells, not MCU memory). Wake sources are limited to a
  timer, one RTC-wake-capable button GPIO, or a literal external reset.

### 17.1 `0x0900` SET_POWER_MODE (PC → device)

```
Offset  Size  Field          Notes
0       1     MODE           0x00 ACTIVE (cancel/exit a low-power request; no-op if already
                              active), 0x01 LOW_POWER, 0x02 HARD_SLEEP
1       1     FLAGS          MODE-specific, see below
2       4     WAKE_AFTER_MS  u32 LE, 0 = no timer (wait indefinitely for another wake trigger) -
                              uses the chip's own timer wake source for the chosen mode
6       1     WAKE_BUTTON    BUTTON_ID (§11 enum), or 0x00 for none. HARD_SLEEP only - which
                              physical button (if wired to an RTC-capable pin on this board) also
                              wakes the device. Ignored for LOW_POWER, which already wakes on any
                              Serial/BLE activity.
```

`FLAGS` (`LOW_POWER` only): `bit0 KEEP_BLE_CONNECTABLE` — 1 = BLE keeps advertising/stays
connectable so a central can wake the device by connecting; 0 = BLE is also suspended and only
Serial activity or `WAKE_AFTER_MS` can wake it. `bits1-7` reserved. `FLAGS` is unused (must be 0)
for `HARD_SLEEP`.

`NACK(BAD_PARAMETERS)` for an invalid `MODE`, or a `WAKE_BUTTON` not wired to an RTC-wake-capable
pin on this device (exactly which buttons qualify is device-specific, not specified here).

**Access control (§5.3)**: this command is USAGE-gated overall (putting the device to sleep is
routine use), but `MODE=HARD_SLEEP` requested over TCP or BLE additionally requires the effective
ADMIN level — `NACK(NOT_AUTHORIZED)` otherwise — since it kills the very radio carrying that
connection. Over Serial, `HARD_SLEEP` only needs the baseline USAGE gate.

**Firmware note, same ACK-then-apply ordering as `OTA_APPLY` (§16.2) and the network-config
commands (§13.2)**: send the `ACK` and let it flush *before* actually entering the sleep mode —
otherwise the caller never sees confirmation the command was accepted.

**Waking `LOW_POWER` over Serial**: light-sleep UART-wake hardware typically needs to detect a few
edge transitions before the CPU is fully responsive again, so the first byte or two sent
immediately after triggering a wake may not be reliably captured. Clients should send a short
preamble of a few extra bytes (e.g. repeated `0xA5` `MAGIC` bytes) before the real frame that's
meant to wake it — the existing stream-transport resync logic (§3.2/§3.3) already discards leading
bytes that don't form a valid frame, so this preamble is silently harmless against an already-awake
device too. No dedicated "wake" command exists or is needed — any valid frame wakes the device from
`LOW_POWER`; `HANDSHAKE_REQUEST` is a natural, minimal choice for this.

### 17.2 `0x0901` POWER_STATUS_REQUEST (PC → device) / `0x0902` POWER_STATUS_RESPONSE (device → PC)

```
Request:  empty payload
Response: 0 1 CURRENT_MODE      always 0x00 ACTIVE - a device able to reply is by definition not
                                 asleep; included for symmetry/future-proofing rather than because
                                 it can currently report anything else
          1 1 LAST_WAKE_REASON  0x00 POWER_ON (fresh boot/power-on reset - never slept)
                                 0x01 HARD_SLEEP_TIMER            0x04 LOW_POWER_TIMER
                                 0x02 HARD_SLEEP_BUTTON           0x05 LOW_POWER_SERIAL_ACTIVITY
                                 0x03 HARD_SLEEP_EXTERNAL_RESET   0x06 LOW_POWER_BLE_ACTIVITY
                                 0x07-0xFF reserved
```

Also exposed as a new handshake capability, `LAST_WAKE_REASON` (§5.2, same 1-byte enum) — a client
learns it immediately on connect without an extra round trip, same rationale as `ACTIVE_TRANSPORT`.

`0x0903`–`0x09FF` reserved (future: battery voltage reporting, if this board's `BAT` input turns
out to be ADC-readable — not confirmed by the hardware spec available when this was designed, so
not assumed; scheduled wake windows; per-peripheral power gating).

## 18. Macro recording and playback (`0x0A00`–`0x0AFF`)

Lets a PC client capture a sequence of commands the device receives, save it as a file, and later
replay it — either on request or automatically on cold start — to reproduce a scripted sequence
(e.g. a boot-time demo, a preprogrammed status-icon cycle) without the PC needing to stay connected
or resend anything.

### 18.1 `0x0A00` RECORD_MACRO (PC → device)

Empty payload. Starts capturing every subsequent dispatched command (regardless of which transport
it arrives on) into an in-memory buffer, in the exact `.macro` format below, until `SAVE_MACRO`
stops it. `NACK(BUSY)` if already recording — a fresh recording is a deliberate act, never implicit.

`RECORD_MACRO`/`SAVE_MACRO`/`PLAY_MACRO` themselves are never captured into a recording (nesting
them would be meaningless); everything else is, including `PAUSE` (§18.4) — recording exactly what
was received, not filtering by whether it went on to ACK or NACK.

### 18.2 `0x0A01` SAVE_MACRO (PC → device)

```
0 1 VOLUME (§14)
1 1 PATH_LEN
2 PATH_LEN PATH
```

Stops recording and writes the finished `.macro` file to `PATH` on `VOLUME` — the exact same
`FILE_UPLOAD` semantics (§14.3: overwrites any existing file). `VOLUME=PSRAM` works here like any
other volume, turning the recording into a queryable/downloadable named file (§14) instead of
discarding it, without persisting it past reboot. `NACK(BAD_PARAMETERS)` if not currently recording.

### 18.3 `0x0A02` PLAY_MACRO (PC → device)

```
0 1 VOLUME (§14)
1 1 PATH_LEN
2 PATH_LEN PATH
```

Downloads and parses `PATH` from `VOLUME`, then starts replaying its entries as if each had just
been received — same dispatch, same handlers, same ACK/NACK generation, just with no live caller
listening for any of it. **The ACK means "playback started", not "playback finished"** — actually
stepping through the macro happens non-blocking, one entry per firmware `loop()` iteration, the
same "timed/sequenced operation is a loop()-driven state machine, never a blocking call inside a
command handler" pattern `GPIO_PLAY_PATTERN` (§15.5) already uses — the other transports keep being
polled throughout, however long the macro's own `PAUSE` entries (§18.4) make that take.
`NACK(BUSY)` if a macro is already playing (one at a time); `NACK(DECODE_FAIL)` for a malformed
file.

### 18.4 `0x0A03` PAUSE

```
0 4 DURATION_MS (u32 LE)
```

Always ACKs immediately, whether received live or replayed from a macro — it never itself blocks.
Its only effect is on a macro *currently playing*: it delays that player's next entry by
`DURATION_MS`, which is what "wait before interpreting the next command" means for a non-blocking
player. Sent live (outside macro playback), it's accepted but has no observable effect beyond the
ACK — there's no "next command" to delay for an already PC-paced, stop-and-wait live transport.

`0x0A04`–`0x0AFF` reserved.

### `.macro` file format (referenced by `SAVE_MACRO`/`PLAY_MACRO`, and the boot macro below)

```
Offset  Size  Field           Notes
0       4     MAGIC           ASCII "MAC1"
4       1     FORMAT_VERSION  0x01
5       ...   ENTRIES         repeated until EOF:
                2   COMMAND_ID   u16 LE
                4   PAYLOAD_LEN  u32 LE
                PAYLOAD_LEN PAYLOAD
```

No compression, CRC, or SEQ per entry — those are wire/session concerns (§2, §10) that don't apply
to a file the device only ever plays back to itself; `COMMAND_ID`+`PAYLOAD` is everything a replay
needs to re-dispatch an entry exactly as if it had just arrived live.

### Init macro and boot macro

Two fixed-name macros run automatically on cold start, in this fixed order (design note 85):
**system init** (the firmware's own hardware bring-up, §12.16/§9 note below) → **`/init.macro`** →
**`/boot.macro`** — each checked SD first, then INTERNAL, exactly like `PLAY_MACRO` would look them
up; neither volume having either is the ordinary case, not an error (there's no live caller to
NACK to at boot anyway). Both are chained into a single `MacroPlayer` run (it only ever holds one
playback at a time), so `/init.macro`'s entries always finish before `/boot.macro`'s begin.

`/init.macro` runs on *every* cold boot, not conditionally — if neither volume has one yet, the
device seeds one itself onto `INTERNAL`, pre-populated with a fixed default: `PAUSE` 1000ms,
`FAST_CLEAR` (§12.16, `COLOR=WHITE`), `CLEAR_ARTIFACTS` (§9, `CYCLES=1`) — giving the cold-boot
info screen (design note 83) a moment to be read before an opinionated but genuinely fast cleanup
flash clears it. Being a real `.macro` file (not a hardcoded routine) means it's just as
user-editable/replaceable as `/boot.macro` always was - place your own `/init.macro` on SD to
override the seeded default entirely, the same way any other macro works.

Two `LOG_MESSAGE` (§10.1) markers are appended directly as macro entries around this sequence,
design note 86: `"init_done"` right after `/init.macro`'s own entries (whether seeded or
user-supplied), and `"boot_done"` at the very end, after `/boot.macro`'s entries if any exist —
both fire even when `/boot.macro` doesn't exist, since the sequence always runs. A PC can wait on
either via `CommandClient#waitForLogMessage` to know precisely when each stage has finished,
without needing to embed its own markers for this common case.

### 18.5 Event-triggered macros (design note 89)

On *every* `BUTTON_EVENT` (§11) or `GPIO_EVENT` (§15.4), firmware checks for a fixed-name macro on
`VOLUME=PSRAM` and auto-plays it if present — no new command, no PC involvement needed, in the same
spirit as `/init.macro`/`/boot.macro`'s cold-boot auto-play above but keyed off a runtime event
instead. Purely opt-in: a missing file is the expected common case, never logged/reported.

**Filenames** (all `VOLUME=PSRAM` only):
```
/on_button_<BUTTON_ID>_press.macro
/on_button_<BUTTON_ID>_release.macro
/on_button_<BUTTON_ID>_shortpress.macro
/on_button_<BUTTON_ID>_longpress.macro
/on_gpio_<PIN_ID>_rising.macro
/on_gpio_<PIN_ID>_falling.macro
```
`<BUTTON_ID>`/`<PIN_ID>` are the same numeric values `BUTTON_EVENT`/`GPIO_EVENT` themselves carry,
formatted as plain decimal (e.g. `/on_button_2_press.macro` for `MENU`). This is a non-issue for
filesystem name-length limits — these lookups are hardcoded to `VOLUME=PSRAM`, a flat, session-only
`std::map`-backed volume with no filesystem-imposed name-length cap.

**Fallback + "last trigger" variable**: a fixed `/on_any_event.macro` is checked on every eligible
event too, regardless of whether a specific macro above also matches. If it exists,
`/trigger_id.txt` (also `VOLUME=PSRAM`) is overwritten with the plain-text path of the specific
macro that would apply (e.g. `/on_button_2_press.macro`) — whether or not that file actually
exists, and regardless of whether the resulting playback starts immediately or is queued (below).
This is a "universal variable" any macro can read via `DRAW_TEXT`'s `FLAGS.TEXT_IS_PATH` (§12.6) —
e.g. `on_any_event.macro` itself can `DRAW_TEXT` with `TEXT="R:/trigger_id.txt"` to show/log which
event actually fired, without needing a dedicated macro per button. Which macro actually plays: the
specific one if present; otherwise `/on_any_event.macro` as a fallback, if that exists; otherwise
nothing happens.

**Busy behavior — deliberately different from live `PLAY_MACRO`'s own contract**: if `MacroPlayer`
is already playing when a new event fires, the newly-resolved macro's entries are *queued* —
appended to the tail of the currently-playing sequence, playing immediately after it finishes —
rather than dropped or interrupting it. This reuses the player's own entry list as the queue, no
separate structure needed. Live `PLAY_MACRO` (§18.3) is unaffected by this and still `NACK(BUSY)`s
a concurrent request — that's a PC-facing request/response contract a live caller depends on
getting a definite answer from, unlike this fire-and-forget internal path.

## 19. Versioning / extensibility conventions

- Envelope `VERSION` is the escape hatch for changes too fundamental for the mechanisms below;
  expected to change rarely. Unsupported `VERSION` → `NACK(VERSION_MISMATCH)`.
- `COMMAND_ID` blocks pre-reserve whole ranges for individual config-setting commands, sensors, and
  PWM/analog GPIO so each becomes an independent addition later without renumbering anything
  already shipped.
- TLV-tail pattern (§5.1) is the template for any future payload that needs to grow; hot-path
  binary payloads (image transfer, drawing, file transfer) stay fixed-layout.
- Reserved bits/enum values must be sent as 0 and tolerated (ignored, not rejected) by v1
  receivers.

## 20. BLE GATT layout

One custom 128-bit Service UUID, one bidirectional Characteristic UUID (combined
`WRITE | WRITE_NR | NOTIFY` properties on the *same* characteristic — not the classic two-
characteristic Nordic-UART-style split — since `NimBLEStreamServer` (§3.1) presents one
characteristic with all three properties as a single `Stream`). PC always writes with
`withResponse=true` regardless of the characteristic also accepting write-without-response, and
subscribes to notifications on this same characteristic before sending `HANDSHAKE_REQUEST`.

```
Service:        748ac078-f79c-4b13-9ead-1a05597acb3f
Characteristic: 01940d49-522c-4b4e-b0c9-3be78c56c960   (WRITE | WRITE_NR | NOTIFY)
```

These are fixed, shared constants hardcoded in both firmware (`Protocol.h`) and the Java library
(`CommandId`-adjacent `Ble` constants class) — freshly generated v4 UUIDs, not placeholders (an
earlier draft of this section said UUIDs were still TBD; they no longer are).

**Pairing / security**: not yet wired up. `NimBLEStreamServer::begin(...)` takes a `secure` flag
that, when true, requires an encrypted (bonded) link to access the characteristic — the natural
mechanism for enforcing `SET_BLE_PIN` (§13.2) once that command is actually implemented, via
`NimBLEDevice::setSecurityPasskey(pin)` + `setSecurityIOCap(...)` + `secure=true`. Until then the
characteristic is open (no pairing required) — this is a real gap for anything security-sensitive,
not a wire-protocol concern the design leaves unaddressed on purpose.

## 21. Design notes / open considerations

These are judgment calls made while designing this protocol, kept here for implementers to revisit
if they turn out to matter in practice:

1. `PAYLOAD_LEN` is u32 rather than u16, for headroom on larger/generic displays and file transfers.
2. Stop-and-wait ACK/NACK applies to every logical frame, regardless of how many BLE
   writes/notifications it took to deliver.
3. Device→PC BLE notifications are unconfirmed at the ATT layer; loss is caught by a `CRC_FAIL` on
   the reassembled Frame (§3.1), not a dedicated sequence-tracking mechanism.
4. Full-image transfer and `READ_SCREEN` FULL reads omit explicit width/height, relying on the
   handshake-reported display size.
5. `TLV.LENGTH` is 1 byte (255-byte cap per value) — no extended-length variant in v1.
6. Local drawing primitives operate on an unconstrained (no alignment) working buffer; alignment is
   only enforced at the raw partial-image-transfer level (§7).
7. All pixel-writing commands default to deferred refresh + explicit `REFRESH` for batching.
8. Text has no opaque-background mode in v1.
9. No resumable/segmented file transfer.
10. `CLEAR_ARTIFACTS` may take several seconds; needs a longer client-side timeout.
11. Config backup/restore has no cross-firmware-version schema migration — unknown TLVs are
    skipped, not migrated.
12. Internal storage is modeled as a possibly-flat namespace (no guaranteed subdirectories).
13. Serial baud rate/framing is a fixed firmware convention (115200 8N1), not wire-negotiated.
14. `PIN_ID` uses raw MCU GPIO numbers rather than a board-agnostic logical pin abstraction.
15. GPIO pin configuration is session-only by default, not auto-persisted across reboots.
16. PWM output and analog input are reserved `MODE` values, not implemented in v1.
17. The PC library's Serial transport depends on `jSerialComm`, declared `provided` in
    `pc-java-lib/pom.xml` — same treatment as `BSToolbox-BLE` for the BLE transport: the consuming
    application supplies it on its own runtime classpath only if it actually uses
    `SerialTransport`, so the library stays free of it otherwise.
18. `OTA_INSTALL` buffers the entire reassembled firmware image before writing to flash (same
    "whole payload in one Logical Frame" model as everything else in this protocol) rather than
    streaming each BLE/TCP/Serial chunk straight into `Update.write()` as it arrives. Simpler and
    consistent with the rest of the protocol; costs a transient RAM buffer roughly the size of the
    firmware image while the transfer is in flight — comfortably affordable on this hardware
    (measured ~8 MB free PSRAM against a realistic ≤2 MB firmware image, see §22) but worth
    revisiting if a future target has much less RAM/PSRAM to spare.
19. Hashing the whole image (`OTA_INSTALL`'s `HASH_ALGO`/`HASH`) is optional on the wire but
    strongly recommended — per-frame `CRC16` alone doesn't protect against the PC sending a
    truncated or wrong image, only against bit corruption of one frame.
20. No delta/incremental OTA — every update transfers the full firmware image.
21. Auto-rollback timeout/self-test criteria (when firmware should revert on its own if
    `OTA_CONFIRM` never arrives) are a firmware policy choice, not specified in the wire protocol.
22. Stream-transport frame resync (§3.2/§3.3, implemented in `FrameStreamReader`/
    `StreamFrameTransport`) only recovers by dropping one byte at a time back to the next MAGIC —
    correct but O(n) in the size of the corrupted span; acceptable given corruption is expected to
    be rare on a wired UART/TCP link, not something to optimize preemptively.
23. `SerialFrameTransport.connect()` blocks for a fixed settle delay (default 3 s) after opening
    the port, because opening it resets the board (see §22's real-hardware findings) and the
    device needs time to reboot before it can be sent anything meaningful. A future
    `OTA_STATUS`-style "are you alive yet" poll could replace this fixed delay with something more
    responsive.
24. This bring-up firmware shares its one physical UART between plain-text self-test output (at
    boot, before the transport starts) and the binary framed protocol (from then on) rather than
    using a second UART, contrary to §3.3's general dedicated-UART guidance — a deliberate,
    temporary exception until this board's exact pinout/USB wiring is confirmed to have a second
    channel available; the two are not interleaved (see `firmware/src/main.cpp`).
25. Serial has no `SET_SERIAL_ENABLED`-style command at all (§13.2) — an intentional omission, not
    an oversight, so there's always a physical-access recovery path if WiFi/BLE config goes wrong.
26. WiFi/BLE credentials and the BLE PIN are write-only on the wire (§13.2) — status responses
    never echo a stored password or PIN back, only whether one is set. A deliberate security
    tradeoff: it means there's no way to retrieve a forgotten WiFi password *through the protocol*
    either (only to overwrite it), but avoids the wire protocol becoming a way to harvest stored
    credentials from a device an attacker briefly gets a connection to.
27. `SET_WIFI_CONFIG`'s `ACK` means "accepted", not "connected" — association is asynchronous and
    can take several seconds; a client must poll `WIFI_STATUS_REQUEST` for the actual outcome
    rather than expecting the initial `ACK` to reflect it. Same reasoning as OTA's status split
    (§16.3) between "command accepted" and "actual state".
28. The BLE PIN is a `u32` on the wire but semantically a 6-digit code (000000-999999, per BLE
    Secure Simple Pairing convention) — client UIs should treat it as a zero-padded 6-digit string,
    not a plain integer, so a PIN like `000042` displays/enters correctly.
29. `LOW_POWER` unconditionally suspends WiFi regardless of `SET_WIFI_ENABLED`'s persisted value
    (§17) — a hardcoded protocol-level policy (WiFi is assumed too power-hungry for any low-power
    mode), not a per-request choice, per the requirement that motivated this design.
30. `HARD_SLEEP` wake is indistinguishable from a power-on reset except via
    `esp_sleep_get_wakeup_cause()` — the working buffer (§2.1) and any other RAM/PSRAM state does
    NOT survive it, only the panel's physically-displayed image does. `LOW_POWER` (light sleep)
    retains RAM/PSRAM and is not a reboot.
31. No dedicated "wake" command — any valid frame wakes a `LOW_POWER` device, since the point of
    entering that mode is exactly to keep Serial/BLE responsive. Waking over Serial may need a
    short throwaway-byte preamble first (§17.1) due to UART-wake hardware limitations; this reuses
    the existing frame-resync logic rather than adding a protocol-level accommodation for it.
32. `POWER_STATUS_RESPONSE.CURRENT_MODE` can only ever report `ACTIVE` in practice (a sleeping
    device can't respond to the request that would report otherwise) — kept in the response for
    forward compatibility rather than removed as dead weight.
33. Battery voltage reporting was considered but not added — this protocol was designed without
    confirmation that the board's `BAT` input is wired to an ADC-capable pin, so the field was left
    out rather than speculatively defined against unconfirmed hardware.
34. §3.1's BLE framing was revised from a custom chunk-sub-header scheme to a plain byte-stream
    model (matching TCP/Serial) once actual implementation showed `NimBLEStreamServer` already
    provides that abstraction on firmware — a design correction made *before* any implementation
    shipped against the old framing, not a breaking change to something already deployed.
35. BLE pairing/security (`secure=true`, passkey via `SET_BLE_PIN`) is designed (§20) but not
    implemented — the GATT characteristic is currently open to any central that finds it. Treat the
    BLE transport as unauthenticated until this is built. Confirmed on real hardware this isn't
    blocking today (an unencrypted characteristic connects and works with zero OS-level pairing
    needed), but implementing `SET_BLE_PIN` for real will also need the PC side to trigger pairing
    *programmatically* (no OS system prompt) — currently not possible with `BSToolbox-BLE`, now
    tracked as a planned feature there (see its `CLAUDE.md`, "Planned: programmatic pairing").
36. The default device name (`"CrowPanel-XXXXXX"`, §13.3) is derived from the MAC address rather
    than being a fixed string, specifically so multiple deployed units stay distinguishable in a
    BLE scan or DHCP client list without requiring `SET_DEVICE_NAME` first — a fixed default would
    have been simpler but actively unhelpful for the "more than one unit nearby" case this feature
    exists for.
37. `SET_DEVICE_NAME`'s `ACK` doesn't guarantee every consumer of the name (BLE advertising, WiFi
    DHCP hostname) has already picked it up — same "accepted, not necessarily fully propagated yet"
    reasoning as `SET_WIFI_CONFIG` (design note 27).
38. `BleFrameTransport` takes a caller-owned `BleAdapter` rather than managing one internally
    (§22's real-hardware findings) — meaning whatever higher-level connection-management code gets
    built on top of it (not yet implemented) needs to own a `BleAdapter`'s lifecycle itself, scan
    on it, and pass both the adapter and the discovered address into `BleFrameTransport`. This is a
    real constraint of the underlying library, not a CrowPanel protocol design choice.
39. `GPIO_PLAY_PATTERN` (§15.5) is one generic timed pulse-train primitive rather than named
    opcodes for "beep"/"blink"/"double-beep" — whether a pattern reads as an audible beep or a
    visible blink depends entirely on what's wired to the pin, which the protocol has no visibility
    into and shouldn't need to; named presets belong in the PC-side library as convenience builders
    over this one wire command, not in the wire format itself.
40. `GPIO_PLAY_PATTERN` always ends the pin LOW after the pattern (and any repeats) finish, rather
    than leaving it at whichever level the last step happened to end on — a deliberate predictable-
    idle-state choice for indicator/buzzer use, at the cost of not being able to end a pattern
    "held HIGH" without a trailing `GPIO_WRITE`.
41. No pattern-completion event and no explicit "stop" command — cancellation is implicit (a new
    pattern, a `GPIO_WRITE`, or a `GPIO_CONFIGURE` mode change all supersede whatever's running) and
    completion notification wasn't requested; both are easy to add later in the still-reserved
    `0x0706`-`0x07FF` range without disturbing this command if a future need shows up.
42. `PIXEL_PITCH_X/Y_UM` (§5.2) reports raw pixel pitch, not a pre-computed DPI value — DPI is a
    trivial derived quantity (`25400 / pitch_um`) that belongs in a PC-library convenience helper,
    not baked into the wire format as a second, rounding-lossy representation of the same fact.
43. §11's `BUTTON_ID` gained `DIAL_UP`/`DIAL_DOWN` (0x06/0x07, from the range already reserved for
    growth) once the real board schematic showed CrowPanel's "Dial switch" is wired as three plain
    GPIOs (scroll up, scroll down, press), not one rotary-encoder or single-button signal —
    `DIAL_SWITCH` alone couldn't represent scroll direction. `board::kAvailableGpioPins` (the
    board header's list of the 2×10 GPIO expansion header's exposed pins) is recorded now that the
    real numbers are known, but is not yet wired into the handshake's `AVAILABLE_GPIO_PINS` TLV
    (§5.2) — that TLV documents what's *usable with §15 GPIO commands*, and §15's handlers don't
    exist yet (Phase 3, `plan.md`); sending it early would advertise a capability the device can't
    yet act on. Same reasoning applies to the onboard status LED at IO41
    (`board::kPinStatusLed`) — recorded, not yet wired to anything (no GPIO controller to drive it
    via `GPIO_PLAY_PATTERN`, §15.5, yet).
44. The working buffer (§2.1) is implemented as an explicit, persistent MCU-side copy
    (`firmware/lib/Display/WorkingBuffer.h`), not "whatever the SSD1683's own current/previous RAM
    banks currently hold" (which is what `FULL_IMAGE_TRANSFER`'s original implementation used,
    before `PARTIAL_IMAGE_TRANSFER`/`REFRESH` existed). REFRESH's "union of bounding boxes deferred
    since the last refresh" (§12.8) needs region-level state tracked across multiple deferred
    writes *before any of them touch the controller at all* — the controller's own RAM banks track
    a current-vs-previous *differential* for the fast-update LUT, a different kind of state than
    "what's pending flip", and conflating the two turned out not to work cleanly. This costs 15000
    bytes of MCU RAM (trivial given 8 MB PSRAM) in exchange for a much simpler, spec-literal
    implementation of §12.8's semantics.
45. §12's local drawing primitives reuse `Adafruit_GFX`'s Bresenham/midpoint-circle rasterizers
    (`firmware/lib/Display/WorkingBufferGfx.h` adapts `WorkingBuffer` to it) rather than
    hand-rolling line/circle drawing - confirmed by reading `Adafruit_GFX.cpp` that every
    higher-level primitive (`drawLine`, `fillRect`, `drawCircle`, `fillCircle`, ...) already
    funnels through the single virtual `drawPixel()`, so overriding only that one method to apply
    §12.1's DRAW_MODE compositing was sufficient - no other Adafruit_GFX method needed touching.
    `LINE_WIDTH>1` has no Adafruit_GFX equivalent, so that specific case (only) is hand-rolled: a
    line stamps a square brush at each Bresenham-stepped point; a rect/circle outline draws nested
    inward outlines instead of one thick one.
46. DRAW_TEXT's embedded font (§12.6) reuses `Adafruit_GFX`'s own bundled classic ASCII font
    (`glcdfont.c`, already shipped as part of the GxEPD2/Adafruit_GFX dependency, tested for
    decades across countless Arduino projects) for the base glyphs, rather than deriving a full
    ~125-glyph bitmap font from scratch - this environment has no FreeType/`fontconvert` available
    to generate a proper `GFXfont`. The ~30 accented Latin codepoints Czech text needs are each
    composed as their unaccented base letter (from that same classic font, unmodified) plus a
    hand-authored 2-row/5-column diacritic mark drawn above it, at the cost of not being
    typographically precise (an accented 'í', for instance, shows the classic font's own built-in
    tittle *and* the added mark, stacked, rather than one replacing the other) - an accepted
    simplification for a small HMI display's status text, not a design goal to improve on without
    a concrete reason to.
47. The clip region (§12.10) is enforced in exactly one place - `WorkingBuffer::getPixel`/
    `setPixel` - rather than separately in each command handler, so every caller (drawing
    primitives via `WorkingBufferGfx::drawPixel`, `SHIFT_REGION`, `COPY_REGION`'s destination side)
    gets it automatically and consistently. `FULL_IMAGE_TRANSFER`/`PARTIAL_IMAGE_TRANSFER` bypass
    this entirely (their `write()` path never calls getPixel/setPixel) since they already have
    their own explicit, stricter, NACK-on-violation bounds contract (§6/§7) that silently clipping
    would quietly change. `COPY_REGION`'s *source* read deliberately bypasses the clip too (a
    separate `getPixelUnclipped`) - the clip constrains what gets modified, not what existing
    content may be read back, and a copy's source rectangle has no reason to be limited by
    whatever the clip currently happens to be set to for unrelated destination-side reasons.
48. DRAW_TEXT's WIDTH/ALIGN/WRAP (§12.6) and the clip region (§12.10) are deliberately orthogonal,
    not the same mechanism wearing two names: WIDTH is a *layout* reference box (where to wrap and
    how to align within it), inherently per-command like every other geometry field; the clip
    region is separate, persistent, cross-command *enforcement* state. A client combines them for
    "wrapped/aligned text confined to exactly this area regardless of how many lines it takes" -
    useful directly after a `SHIFT_REGION` scroll, where only the freshly-vacated strip should
    receive new text even if the caller's own X/Y/line-count math is slightly off.
49. FONT_ID 0x00's CP437 box-drawing/block glyphs (added after initial DRAW_TEXT delivery, once
    checking `glcdfont.c` directly showed they were already present at codes 176-223, unmapped)
    surfaced two things worth recording. First, `Adafruit_GFX::_cp437` defaults to `false`, which
    makes the classic `drawChar()` path apply a legacy off-by-one shift to any code ≥176 (kept for
    backward compat with old sketches that relied on it) — without `gWorkingBufferGfx.cp437(true)`
    (called once, in `setup()`), every box-drawing glyph would silently render as its neighbor
    instead of itself. Second, adding them exposed a real inconsistency between this doc and the
    actual implementation: `WIDTH=0` originally drew everything as one line via a separate code
    path that never checked for `\n` at all (contradicting this section's own "an embedded `\n`
    always starts a new line" wording), and the word-wrap path collapsed runs of spaces into a
    single separator unconditionally — fine for prose, but it would have destroyed a box-drawing
    table's fixed internal spacing the moment `WRAP` was anything but carefully avoided. Both are
    fixed: `WIDTH<=0` now always goes through the same line-splitting logic as `WIDTH>0` (still
    skipping alignment, since there's no reference box to align within), and whitespace is only
    ever tokenized/collapsed when `WRAP=1` *and* `WIDTH>0` are both genuinely in effect.
50. DRAW_TEXT's `BACKGROUND` byte (§12.6, added after the initial delivery) needed no change at all
    to `WorkingBuffer::compositePixel`/`drawPixel` — only to what `bg` value gets passed into
    `Adafruit_GFX::drawChar()`. `bg==COLOR` is drawChar()'s own existing convention for "skip
    non-ink pixels" (transparent, the original v1 behavior); passing the opposite 1bpp value
    instead makes drawChar() paint every non-ink pixel through the exact same `writePixel()` →
    `drawPixel()` override → `compositePixel()` path the ink pixels already used, so DRAW_MODE
    ends up applying uniformly to the whole glyph cell for free. The embedded font's 2-row
    diacritic band, drawn separately from `drawChar()`'s own 8-row body since it sits above it, has
    no equivalent built-in opaque-fill logic and needed its own explicit fill loop when
    `BACKGROUND=OPAQUE` — real-hardware testing caught that this loop initially missed the 1px
    "spacer" column between characters (`drawChar()` itself fills that column across its own body
    when opaque, so the diacritic band's matching column needed the same explicit treatment, or
    every opaque/inverted character showed a small unfilled gap at its top-left).
51. `SET_DRAW_OFFSET`/`SET_ORIENTATION` (§12.12/§12.13, added after asking how off-canvas scrolling
    and portrait mounting could work) both slot into `WorkingBuffer`'s existing single-chokepoint
    pattern — `getPixel`/`setPixel`/`compositePixel`/`getPixelUnclipped` (widened from unsigned to
    signed logical coordinates) apply the offset, then the clip check, then a `toPhysical()`
    rotation/mirror mapping, in that fixed order, so every existing caller (drawing primitives via
    `WorkingBufferGfx::drawPixel`, `SHIFT_REGION`, `COPY_REGION`) gets both automatically without
    any change to their own code. `SET_CLIP_REGION`'s X/Y/WIDTH/HEIGHT deliberately stay untouched
    by this — stored exactly as given, in the same logical space `toPhysical()` re-derives its
    current width/height from — which is what makes the clip window "follow" a later
    `SET_ORIENTATION` change (same stored numbers, reinterpreted against new logical dimensions)
    while staying independent of `SET_DRAW_OFFSET` (an orthogonal pan, not a redefinition of where
    the window sits) — this exact interaction was confirmed with the user before implementing,
    since getting it backwards would have meant reworking the chokepoint. `finishDraw()` (main.cpp),
    which computes the physical region to flush/mark-dirty from a command's own logical geometry,
    needed a matching batch-rect version of the same pipeline (`computeAffectedPhysicalRegion()`) —
    exact, not an approximation, since every one of `SET_ORIENTATION`'s 8 rotation/mirror symmetries
    maps an axis-aligned rectangle's opposite corners to another rectangle's opposite corners, so
    transforming just those two corners and taking min/max is sufficient. Verified end-to-end on
    real hardware: `DrawOffsetManualCheck` (same `DRAW_RECT` at three different offsets, including
    one large enough negative offset to straddle the left panel edge — confirmed only the surviving
    portion rendered, flush against `x=0`) and `OrientationManualCheck` (a fixed logical anchor
    drawn once under each of the four `ROTATION` values landed near four different physical
    corners, each with its glyphs themselves visibly rotated in place, not just moved; `MIRROR_H`/
    `MIRROR_V` each flipped both the position and the readability of a text label relative to an
    unmirrored baseline) — all matching the predicted corner/readability mapping exactly on the
    first real-hardware attempt.
52. `READ_SCREEN`/`SCREEN_DATA` (§8) needed a second firmware-side buffer, `panelBuffer_` — the
    existing `WorkingBuffer::buffer_` is written to immediately by every §12 primitive regardless of
    `FLAGS.REFRESH_NOW`, so it already *is* `SOURCE=WORKING_BUFFER`'s semantics, but nothing tracked
    what had actually reached the physical panel (`SOURCE=PANEL`) once a write was deferred.
    `panelBuffer_` is kept in sync only inside `flush()` (copying exactly the region just written to
    the controller, bit-by-bit rather than by byte-range, since `flush()` itself has no alignment
    constraint) — everywhere else in this class is otherwise untouched. `CLEAR_ARTIFACTS`'s
    `RESTORE_CONTENT=0` path flashes the panel directly against the display driver (bypassing
    `WorkingBuffer` entirely, like the boot-time `displaySelfTest()`), so it needed one explicit
    `markPanelBlank()` call to keep `panelBuffer_` truthful, since no `flush()` call happens to do it
    automatically there.
53. `READ_SCREEN`'s correctness was verified *programmatically*, not just visually — decoding
    `SCREEN_DATA` and sampling specific pixels back in the PC library and asserting on the result is
    strictly stronger proof than eyeballing the panel, and this is one of only two commands so far
    (see also `CLEAR_ARTIFACTS` immediately below) where that's the more natural check anyway, since
    the whole point of `READ_SCREEN` is to give the PC-side code something to compare against.
    `CLEAR_ARTIFACTS`'s own visual confirmation, by contrast, turned out to be genuinely impractical:
    a single-cycle black/white flash is hard to visually distinguish from the boot self-test's own,
    near-identical flash that always happens immediately beforehand (opening the serial port resets
    the board) — noted honestly rather than claimed as visually confirmed; `READ_SCREEN` reading back
    exactly blank after the no-restore cycle and exactly restored after the restore cycle is the real
    evidence here, and is strictly better evidence than a human watching a fast flash sequence.
54. `STORAGE_INFO_RESPONSE`'s `TOTAL_BYTES`/`FREE_BYTES` (§14.5) are u32, capping representable
    volume sizes at ~4.29 GB — a real hardware surprise, not a hypothetical one: the first real-SD-card
    test run reported `FREE_BYTES` (4113563648) *larger* than `TOTAL_BYTES` (868220928), an
    impossible-looking result that an initial "clamp if used>total" fix didn't actually address,
    since `SD.totalBytes()`/`usedBytes()` (which return correct 64-bit byte counts) were each being
    independently truncated to u32 by a plain `static_cast` — losing different high bits from each
    and leaving two numbers that no longer relate to each other at all, on a card (32 GB, confirmed
    with the user) nowhere near an edge case. Fixed by saturating each value at `UINT32_MAX`
    independently instead of truncating, which keeps what's reported honest (§14.5's own text now
    says as much) without changing the already-documented 10-byte wire layout. Widening these two
    fields to u64 would be the complete fix, but that's a wire-format change to an already-designed
    command (Phase 0) — left as a candidate for a future protocol revision rather than done silently
    here.
55. `StorageManager` (§14) reuses one implementation for both volumes by writing every operation
    against Arduino's common `fs::FS` interface, which both `fs::SDFS` (`SD`) and `fs::LittleFSFS`
    (`LittleFS`) already implement — `VOLUME` just selects which `fs::FS*` an operation targets, so
    `list()`/`download()`/`upload()`/`remove()` are each written once, not duplicated per filesystem.
    `VOLUME=SD` is treated as hot-pluggable: every SD operation remounts fresh (`SD.end()` then
    `SD.begin()`) rather than trusting a cached "mounted" flag, since the ESP32 SD library has no
    card-detect callback — the simplest way to correctly notice a card inserted or removed between
    commands, at a small per-command cost that's a non-issue for occasional file operations.
    `INTERNAL` (`LittleFS`) is treated as supporting real subdirectories (it actually does) rather
    than the flat-namespace fallback §14 allows, kept uniform with SD for no real cost. **Verified
    end-to-end on real hardware** (`StorageManualCheck`, asserting programmatically like
    `ScreenReadbackManualCheck` rather than visually — file I/O has nothing to look at on the panel):
    for both `VOLUME=INTERNAL` and `VOLUME=SD` (a real 32 GB card), uploaded a small file, confirmed
    it appears in `FILE_LIST` with the right size, downloaded it back byte-for-byte identical,
    deleted it, and confirmed the listing no longer shows it — all passing on both volumes after the
    `TOTAL_BYTES`/`FREE_BYTES` saturation fix (design note 54).
56. `DRAW_IMAGE` (§12.7) is a §12 drawing primitive like any other — it goes through the same
    DRAW_MODE/clip/offset/orientation pipeline as `DRAW_LINE`/`DRAW_RECT`/etc (via
    `WorkingBufferGfx::drawPixel()`), unlike `FULL_IMAGE_TRANSFER`/`PARTIAL_IMAGE_TRANSFER`, which
    write the panel-sized buffer directly. It reuses `decodeImageData()` (the exact §6 RAW/RLE
    scheme already shared by full/partial image transfer) unchanged, since the `.epi` format's
    `COLOR_DATA`/`MASK_DATA` streams are byte-for-byte that same encoding — the only new code is
    parsing the `.epi` header and, per masked pixel, skipping the draw call entirely rather than
    compositing (matching §12.1's "non-ink/transparent pixels are skipped regardless of DRAW_MODE"
    convention already used for text). A sanity cap on `WIDTH*HEIGHT` guards the temporary decode
    buffer against a corrupt or adversarial `.epi` file claiming an enormous image — a real
    possibility since these are read from removable SD storage, not internal state.
57. `EpiImageCodec` (Java, `pc-java-lib`) converts to/from a plain `BufferedImage` rather than
    parsing a specific source format (PNG, etc.) itself — `encode()` thresholds each pixel's
    luminance for `COLOR_DATA` and, optionally, its alpha channel for `MASK_DATA`; any image
    decoding (PNG/JPEG/whatever) is left to `ImageIO`/`BufferedImage` upstream of this class, not
    duplicated here. Always RLE-encodes both streams (§6: "any conformant encoder is valid").
    Round-trip correctness (`EpiImageCodecTest`) is verified with plain JUnit assertions, no hardware
    needed — real-hardware verification (`DrawImageManualCheck`) instead exercises the actual wire
    path: encode → `FILE_UPLOAD` to `VOLUME=INTERNAL` → `DRAW_IMAGE` referencing that path →
    confirm on the panel. That check draws a striped background first, specifically so the masked
    icon's fully-transparent corners have something other than plain white behind them to reveal —
    a masked icon over a plain white background would look identical whether the mask worked or not.
58. `DRAW_IMAGE_ROW` (§12.14, added after asking for a way to lay out several images/icons in a
    line) needed `DRAW_IMAGE`'s single-image decode-and-draw logic split into two steps —
    `decodeEpiImage()` (download+parse+decode, no drawing) and `blitDecodedEpiImage()` (draw an
    already-decoded image at a given position) — since alignment (`CENTER`/`RIGHT`/`BLOCK`) needs
    every image's own width known *before* any image's X position can be computed, which a
    combined decode-and-draw-immediately function (the original `DRAW_IMAGE` shape) can't support.
    `DRAW_IMAGE` itself was refactored onto the same two functions rather than kept as a separate
    code path — reconfirmed byte-for-byte behaviorally unchanged via `DrawImageManualCheck` after
    the refactor, before adding the new command. `BLOCK` alignment (auto-distributing leftover
    space evenly across the gaps between images, CSS `justify-content: space-between`-style)
    deliberately ignores `SPACING` — the two are alternative ways of expressing the same concept
    (gap size), and having both apply simultaneously would just mean one silently overrides the
    other depending on evaluation order, worse than documenting one clean rule. **Verified on real
    hardware** (`DrawImageRowManualCheck`): drew the same three differently-sized icon tiles inside
    four outlined reference boxes, one per `ALIGN` value — confirmed `LEFT`/`CENTER`/`RIGHT` packed
    against the expected box edge and `BLOCK` split its leftover space evenly between the two gaps
    with the first/last tiles flush against the box edges, matching the predicted layout exactly on
    the first attempt.
59. `VOLUME=PSRAM` (§14, added alongside the macro system as "a universal third temporary storage
    option") is backed directly by a `std::map<std::string, PsramBuffer>` inside `StorageManager`
    rather than a real `fs::FS`-conformant filesystem — implementing Arduino's whole `FS`/`File`
    virtual interface just for a session-only, flat, no-subdirectory store wasn't worth it once
    `list()`/`download()`/`upload()`/`remove()`/`info()` could each just branch to their own small
    map-based implementation at the top, ahead of the existing SD/INTERNAL `fs::FS` path.
    `PsramBuffer` allocates via `ps_malloc()`/`free()` explicitly rather than trusting
    `std::vector`'s default allocator, since this volume is meant to guarantee genuine PSRAM
    residency, not "whichever heap the default allocator happens to pick". A fixed 2 MiB budget
    (deliberately not the whole chip's PSRAM, `ESP.getPsramSize()` — shared with other subsystems)
    bounds `STORAGE_INFO`/upload capacity honestly.
60. `RECORD_MACRO`/`SAVE_MACRO`/`PLAY_MACRO`/`PAUSE` (§18) deliberately don't touch `Dispatcher.h`
    or `StreamFrameTransport.h` at all. Recording is a thin wrapper, `dispatchAndMaybeRecord()`,
    that each live transport's own frame-handler lambda calls instead of `gDispatcher.dispatch()`
    directly; playback dispatches through the exact same `gDispatcher` every live command already
    uses, just with a `NullStream`-backed `StreamFrameTransport` standing in for a real caller
    (nobody is listening for a macro-replayed command's ACK/NACK) and a new `ACTIVE_TRANSPORT`
    value, `MACRO`, so a handler that cares which transport it's on (e.g. `HANDSHAKE_RESPONSE`'s own
    echo, §5.2) still reports something honest if a macro happens to contain one. Keeping this out
    of the core dispatch/transport files matches how every other feature so far has been layered on
    top of them without modifying them.
61. `PAUSE` always ACKs immediately and never itself blocks — it only actually delays anything when
    `ACTIVE_TRANSPORT=MACRO`, by telling `MacroPlayer` to hold its next entry. This makes a live
    `PAUSE` a well-defined (if inert) no-op rather than a rejected/special-cased command, and it
    means macro playback's own non-blocking-ness doesn't depend on `PAUSE`'s handler doing anything
    unusual — the actual "don't block `loop()`" property comes entirely from `stepMacroPlayback()`
    dispatching at most one entry per `loop()` iteration and returning immediately whenever
    `MacroPlayer::next()` reports it's still waiting.
62. Real-hardware testing of "does playback block other transports" caught a test-design flaw, not
    a firmware bug: the first attempt sent a live probe command immediately after `PLAY_MACRO`'s own
    (correctly instant) ACK, and it took ~430ms — failing an assumed "should be prompt" threshold.
    Root cause: entry 0 of that test's macro was itself a `DRAW_RECT` with `FLAGS.REFRESH_NOW=1`,
    and any `REFRESH_NOW` draw — live or replayed, unrelated to macros specifically — already blocks
    `loop()` for its own physical partial-refresh duration; the probe had landed mid-refresh, not
    inside the macro's `PAUSE` window at all. Fixed by waiting for entry 0's own refresh to finish
    before sending the probe, which then returned in the same ~10ms as a normal live command,
    confirming the actual claim (`PAUSE` itself doesn't block) rather than a different, false one
    (nothing in the whole system ever blocks, which was never true and isn't what non-blocking
    macro playback was meant to guarantee).
63. `FILL_IMAGE` (§12.15, added on request for a way to tile an image across a width/height/both,
    "creating background, constructing decorative borders") reuses `DecodedEpiImage`/
    `decodeEpiImage()`/`blitDecodedEpiImage()` (design note 58) completely unchanged — the only new
    logic is the tiling loop and where each tile lands, not how a tile itself gets drawn. Cropping
    a partial tile at the target rectangle's far edge is achieved by temporarily narrowing the clip
    region (§12.10) to the intersection of `FILL_IMAGE`'s own target rect and whatever clip was
    already active, then restoring the original clip afterward — every blitted tile's existing
    per-pixel clipping (already enforced for every other §12 command) crops it for free, so
    `FILL_IMAGE` needed no new per-pixel bounds-checking logic of its own, and it can never escape a
    caller's own already-active clip either (the intersection, not a plain overwrite, is what
    prevents that). **Verified on real hardware** (`FillImageManualCheck`): tiled one small,
    deliberately asymmetric 12×12 tile (a black square in one corner, so tile boundaries and
    orientation are both visually unambiguous) across three target rectangles, one per `TILE_MODE`,
    each sized to NOT be an exact multiple of the tile size — confirmed each run's last tile was
    cleanly cropped at the target edge rather than wrapping, overflowing, or being skipped, correct
    on the first real-hardware attempt.
64. `DRAW_TEXT`'s `FLAGS.TEXT_IS_PATH` (§12.6, added on request for "parametrized macros" - a macro
    that draws different text on different plays without being re-recorded) reuses one of §2.1's
    own reserved `FLAGS` bits (bit2) rather than growing the payload with a new field - the wire
    layout is unchanged size, only what bit2 means is new, and only for `DRAW_TEXT` specifically
    (bits0-1 keep their shared `REFRESH_NOW`/`REFRESH_FULL` meaning). Firmware-side, the change is
    minimal: `handleDrawText()` resolves `TEXT` to either the literal payload bytes or a
    freshly-downloaded file's bytes *before* anything else runs, so the rest of the function
    (layout, drawing, `finishDraw()`) is completely unaware which case it's in. Volume selection
    went through two iterations: the first pass hardcoded `VOLUME=PSRAM` (matching the literal
    request, "set text to a RAM file") to avoid growing `DRAW_TEXT`'s payload for a `VOLUME` field;
    asked directly whether other volumes should be supported too, the user proposed an optional
    2-byte prefix on `TEXT` itself (`R:`/`S:`/`F:`) instead of a wire-format change — strictly
    better than the field-based alternative floated first, since it needs no payload growth at all
    and defaults cleanly to PSRAM when omitted. **Verified end-to-end on real hardware**
    (`ParametrizedMacroManualCheck`): wrote `"HELLO"` to a PSRAM file, recorded a macro containing
    one `TEXT_IS_PATH` `DRAW_TEXT` referencing that file, saved it, and — before ever playing it —
    downloaded the saved macro and confirmed (via `MacroCodec`) it had captured the *path* itself,
    `"/label.txt"`, not the resolved text `"HELLO"`, proving the indirection is what actually got
    persisted. Played the macro, overwrote the PSRAM file with `"WORLD"` without touching the saved
    macro at all, played the *exact same* macro file again, and confirmed the panel showed
    `"WORLD"` the second time — the core claim, working correctly. `DrawTextFromFileManualCheck`
    separately confirmed all three prefixes (`R:`/`S:`/`F:`) resolve to the right volume, and that
    an unprefixed path defaults to PSRAM, by writing a distinct label to each volume and drawing one
    line per prefix.
65. Testing design note 64 caught a real, pre-existing bug, not something the new feature
    introduced: the panel showed `"WORLD"` with its last letter's rightmost column(s) missing. Root
    cause was in `WorkingBuffer::flush()`'s partial-refresh path, not `DRAW_TEXT` — GxEPD2's own
    `writeImagePart()` rounds a requested X down and W up to whole bytes internally (the SSD1683's
    RAM window is byte-addressed in X, `board::kPartialRefreshGranularityX=8`), but derives the
    rounded W from the *original* W, without accounting for how far X just shifted left rounding
    down — so a non-byte-aligned `x`/`w` (routine for §12's drawing primitives, which have no
    alignment constraint, unlike §7) could silently leave the rightmost columns of the requested
    region out of the physical write, even though they were already correct in `buffer_`. This had
    gone unnoticed through every earlier drawing-primitive manual check in this project, since none
    of them happened to combine a non-byte-aligned X/WIDTH with a genuinely partial (not full-panel)
    `REFRESH_NOW` flush *and* content sensitive enough (recognizable text) that a couple of missing
    edge columns were obviously wrong at a glance — `ParametrizedMacroManualCheck` was the first to
    hit all three at once. Fixed by pre-aligning `x`/`w` to whole bytes inside `flush()` itself,
    before calling into GxEPD2, so its own rounding becomes a no-op and the full requested region -
    including the "last character" case that surfaced this - is always actually written; a few
    extra already-correct pixels just outside the original region get harmlessly re-flushed too.
    Re-verified clean on the same test, plus a broader regression pass (`ScreenReadbackManualCheck`,
    `DrawPrimitivesManualCheck`, `ShiftRegionManualCheck`) confirming nothing else regressed.
66. §15 GPIO was implemented as its own small dedicated class, `firmware/lib/Gpio/GpioController.h`,
    following the `StorageManager`/`MacroPlayer` precedent (design notes for those) rather than
    inlining per-pin state into `main.cpp` — `GPIO_CONFIGURE`/`WRITE`/`READ_REQUEST`/`PLAY_PATTERN`
    handlers in `main.cpp` just validate payload shape and `PIN_ID` availability (against
    `board::kAvailableGpioPins[]`), then delegate. `GpioController::update()` is called every
    `loop()` iteration — the same non-blocking, state-machine pattern already established for
    `MacroPlayer` and originally flagged as the intended shape for `GPIO_PLAY_PATTERN` back when it
    was first designed, now actually built: per-pin `Pattern` state advances at most one step per
    `update()` call based on `millis()`, never blocking. Change-event detection (§15.4) is a simple
    `update()`-driven poll of every pin with `ENABLE_CHANGE_EVENTS` set, with a 30ms debounce
    (a candidate level must hold steady for that long before it's accepted and reported) — cheap
    enough at only twelve possible pins, and avoids a mechanical button/switch's contact bounce
    firing a burst of spurious `GPIO_EVENT`s. `GPIO_EVENT` is the **first device-initiated push this
    firmware sends unprompted by any request** — `BUTTON_EVENT` (§11) was designed earlier but never
    implemented, so there was no existing precedent to follow for "how does a spontaneous
    device→PC frame get sent when there's no request to reply to". Resolved by broadcasting the
    constructed `Frame` on every live transport (`gSerialTransport`, `gBleTransport`, and
    `gTcpTransport` if currently connected) rather than picking just one — each
    `StreamFrameTransport::send()` already tolerates a dead/absent peer gracefully via its own
    stall watchdog (see `StreamFrameTransport.h`), so broadcasting to a transport nobody's
    listening on right now is harmless, not an error; this same broadcast approach is the natural
    template for `BUTTON_EVENT` whenever that gets built. The `AVAILABLE_GPIO_PINS` TLV (§5.2, type
    `0x0D`) and `FEATURE_BITMASK` bit7 (§5.2) — both recorded in the board header/design note 43 but
    deliberately left unwired until §15's handlers actually existed — are now wired into
    `buildHandshakeResponsePayload()`. Cancellation semantics (§15.5: a new `GPIO_PLAY_PATTERN`, a
    `GPIO_WRITE`, or a `GPIO_CONFIGURE` reconfiguring the pin all supersede an in-progress pattern)
    are enforced by having `configure()`/`write()`/`playPattern()` each unconditionally clear the
    pin's `Pattern.active` flag before doing anything else, rather than tracking "was a pattern
    running" as a separate check. **Verified on real hardware** (`GpioManualCheck`, pins 15/16 on
    the 2×10 expansion header — LED on 15, button on 16 wired `INPUT_PULLUP`): `GPIO_WRITE`
    HIGH/LOW each confirmed via a following `GPIO_READ_RESPONSE`; a 5-blink `GPIO_PLAY_PATTERN`
    confirmed to end LOW via `GPIO_READ` after waiting out its total duration;
    `NACK(PIN_UNAVAILABLE)` for an unexposed `PIN_ID`, `NACK(BAD_PARAMETERS)` for
    `MODE=PWM_OUTPUT` and for a `GPIO_WRITE` on a non-`OUTPUT` pin, all confirmed programmatically;
    eleven real button presses during a live capture window produced eleven correctly-alternating
    (no duplicate-fire) `GPIO_EVENT` frames via `CommandEventListener`; the user independently
    confirmed the LED's on/off/blink behavior and button responsiveness visually matched. The
    board's physical header pins are schematic-labeled with `P$`-style pad designators distinct
    from the `IOxx` raw-GPIO net names `board::kAvailableGpioPins[]` uses — worth remembering when
    wiring test hardware to this or any other board: `PIN_ID` on the wire is always the raw MCU
    GPIO number (§15), never a header silkscreen/schematic pad label.
67. §11 `BUTTON_EVENT` was implemented as `firmware/lib/Buttons/ButtonController.h`, deliberately a
    separate small class from `GpioController.h` rather than sharing code with it, even though both
    are loop()-driven debounced-polling state machines: `GPIO_EVENT` reports a raw level for a
    PC-configured arbitrary pin, while `BUTTON_EVENT` reports PRESS/RELEASE/LONG_PRESS for a fixed,
    firmware-known set of five buttons (`board::kPinButtonMenu`/`kPinButtonBack`/
    `kPinButtonDialUp`/`kPinButtonDialDown`/`kPinButtonDialConfirm`, already recorded from the
    schematic per design note 43) and needs its own per-button state (press-start timestamp,
    one-shot long-press flag) that `GpioController`'s simpler level-only tracking doesn't carry.
    `BOOT`/`RESET` are intentionally never wired up or emitted — they're hardware strap/EN pins on
    this board, not observable GPIOs (`board_crowpanel_4_2.h`'s own comment). All five button pins
    are driven `INPUT_PULLUP` regardless of whether the schematic already has an external pull-up —
    harmless to combine, and guarantees a defined idle level either way. `BUTTON_EVENT` reuses
    `GPIO_EVENT`'s just-established broadcast-to-every-live-transport approach (design note 66)
    rather than inventing a second push mechanism. `FEATURE_BITMASK` bit2 (`BUTTON_EVENTS`,
    reserved in the bitmask layout since the very first handshake design pass) is now actually set.
    **Verified on real hardware** (`ButtonManualCheck`): a 40-second capture window with the user
    pressing each of the five buttons (one held long enough to trigger `LONG_PRESS`) produced 18
    `BUTTON_EVENT` frames covering all five `BUTTON_ID`s with plausible PRESS/LONG_PRESS/RELEASE
    sequences. **One unresolved oddity, noted honestly rather than swept under the rug**: several
    consecutive events landed suspiciously close to an exact ~2001ms grid regardless of which
    button, and `MENU`/`BACK` produced no `LONG_PRESS` despite an apparent ~4-second hold, while the
    dial buttons did at ~2 seconds. Asked the user directly what they'd actually done; they weren't
    tracking their own press timing closely enough to confirm either way. Weighed against this: the
    immediately preceding `GpioManualCheck` run exercised near-identical loop()-driven
    debounced-polling code (`GpioController::update()`) for the same physical button (wired as a
    plain `GPIO_EVENT` input at the time) and captured realistic, non-quantized human press
    intervals (2031–3236ms, all different) — evidence against a systemic polling/timing bug in the
    shared debounce approach itself. Most likely explanation is the user unconsciously falling into
    a steady per-instruction pace while working through a numbered on-screen list, not a firmware
    defect, but this is not proven either way — see `next-steps.md` item 20 if it needs revisiting
    with a controlled, narrated single-button hold test.
68. The onboard status LED (`board::kPinStatusLed`, IO41) was added to `board::kAvailableGpioPins[]`
    now that a `GpioController` exists to drive it — design note 43 originally recorded the pin but
    deliberately left it unwired pending exactly this. Gives every future GPIO-output check (and
    real usage) a built-in test target needing no external LED/wiring. **Verified on real hardware**
    (`StatusLedManualCheck`): `AVAILABLE_GPIO_PINS` includes `PIN_ID=41`; `GPIO_WRITE` HIGH/LOW and
    a 3-blink `GPIO_PLAY_PATTERN` (confirmed via `GPIO_READ` to end LOW) all behaved identically to
    an external LED on any other exposed pin, and the user visually confirmed the on/off/blink
    sequence.
69. `BUTTON_ID.BOOT` is now actually emitted — requested directly ("The boot button could probably
    also be used when already running"). GPIO0 is only sampled as a strapping pin at reset, to
    choose ROM-bootloader vs normal boot; once firmware is running it's a perfectly ordinary,
    safely-readable GPIO, the standard "boot button doubles as a user button" pattern used on many
    ESP32 boards. `board::kPinButtonBoot = 0` was added and wired into `ButtonController`'s list
    alongside the other five buttons; `RESET` remains genuinely unemittable, since it's the chip's
    EN pin and pulling it resets the whole MCU — there's no "after" in which firmware could ever
    observe that press. **Verified on real hardware** (`ButtonManualCheck`, extended to prompt for
    BOOT too): real PRESS/LONG_PRESS/RELEASE events captured for BOOT.

    Chasing this down surfaced the same ~2001ms-grid timing oddity flagged in design note 67, this
    time isolated to BOOT alone in one run (four repeated press/long-press/release cycles, each
    landing on almost exactly the same ~2001/2001/2001ms spacing) — worth taking seriously given it
    raised the question of whether GPIO0 might be picking up spurious electrical activity from the
    CH340 USB-serial adapter's auto-reset circuitry, which shares that same pin (`CLAUDE.md`'s own
    notes on `SerialFrameTransport` document DTR/RTS driving that same reset path). Ruled out with a
    focused negative-control test: a throwaway script watched `BUTTON_EVENT` passively for 30
    seconds with the board connected and *nobody touching any button at all* — zero events fired.
    That rules out spontaneous/electrical triggering on GPIO0 specifically (the CH340/DTR-RTS
    circuit is only touched once, at `connect()`, not periodically — see
    `SerialFrameTransport.java`'s own class doc — so this negative result is a real, informative
    control, not just an absence of bad luck). The remaining, still-open question from design note
    67 (why real human button presses land on such an exact ~2001ms grid) is therefore very likely
    ordinary human press-testing behavior after all, not a firmware timing defect — but that
    residual precision is still not fully explained and is left as a closed-with-caveat item rather
    than a confirmed non-issue.
70. **Design notes 67 and 69's "most likely human pacing" conclusion was wrong — this corrects it.**
    The user reported directly that the BOOT test clicks were short, and that most other button
    presses were short too, which flatly contradicts "the user held it long" and reopened the
    question with real information design notes 67/69 didn't have. Root-caused with real
    instrumentation rather than more guessing: a temporary per-call timing probe in `loop()`
    (broadcasting a synthetic diagnostic frame whenever any sub-call exceeded 50ms, since `Serial`
    is committed to the framed protocol and unavailable for plain debug prints) would have pinned
    it down directly, but reading `NimBLEStreamServer::write()`'s actual source first found the
    answer without needing to flash it: it returns 0 **immediately, every single call**, whenever no
    BLE central is currently subscribed — not an occasional stall, an unconditional zero. But
    `StreamFrameTransport::send()` treats *every* zero-byte `stream_.write()` as "peer's receive
    buffer is momentarily full, keep retrying" and only gives up after its full
    `kSendStallTimeoutMs` (2000ms) stall watchdog — correct behavior for a real but congested link,
    wrong for "there was never going to be a peer to send to." Since `pushGpioEvent()`/
    `pushButtonEvent()` call `gBleTransport.send()` synchronously from inside `loop()`, and BLE had
    no connected central throughout all of this session's Serial-transport testing, **every single
    GPIO_EVENT and BUTTON_EVENT push froze the entire single-threaded firmware for a full ~2
    seconds** — not just delaying the button feature's own event delivery, but blocking
    `gSerialTransport.poll()`/`pollTcp()`/`gBleTransport.poll()`/`stepMacroPlayback()`/
    `gGpioController.update()` too, for that whole window, on every push.

    This fully explains every observed symptom, once traced through carefully: firing `PRESS` calls
    `pushButtonEvent()`, which blocks ~2000ms on the doomed BLE send before returning — freezing
    `loop()` (and thus `ButtonController::update()`) for that whole window. `pressStartMs` was
    captured *before* the freeze; the next `update()` call afterward reads `millis()` *after* it —
    so the elapsed-time check `(now - pressStartMs) >= kLongPressThresholdMs` sees a ~2000ms gap
    that never really elapsed at the button, firing a spurious `LONG_PRESS` regardless of the actual
    (possibly very short) physical press. Firing that `LONG_PRESS` triggers its own ~2000ms freeze
    the same way, then firing `RELEASE` (once finally detected) does too — three self-inflicted
    ~2000ms delays in a row, exactly the observed PRESS→(~2001ms)→LONG_PRESS→(~2001ms)→RELEASE
    pattern, with zero relationship to how long the button was actually physically held. The earlier
    `GpioManualCheck` data (2031–3236ms gaps, read at the time as "realistic human variance,"
    design note 67) was completely misread — those are real human button-press gaps *plus* the same
    fixed ~2000ms tax added to every single `GPIO_EVENT` push, not evidence against a systemic bug
    at all.

    Fixed by checking `gBleStream.ready()` (`NimBLEStreamServer`'s own connected/subscribed-peer
    check, already public — no new API needed) before calling `gBleTransport.send()` in both push
    functions, skipping the doomed send outright instead of paying its 2-second timeout. `Serial`
    doesn't need the same guard (`HardwareSerial::write()` buffers into its TX FIFO regardless of
    whether a host is reading, so it won't zero-return the way a subscription-gated BLE
    characteristic does) and TCP was already guarded by its `if (gTcpTransport)` null-check for the
    no-client case. **Re-verified on real hardware**: quick real clicks on all six buttons produced
    realistic 150–250ms PRESS→RELEASE gaps and zero spurious `LONG_PRESS` across 18 events; three
    separate genuine 2+ second holds each fired `LONG_PRESS` at *exactly* 800ms after `PRESS` —
    matching `kLongPressThresholdMs` precisely — confirming both the false-positive bug and the real
    long-press path are now correct. This class of bug (any future device-initiated push blocking
    the whole firmware for up to 2 seconds whenever BLE has no connected central) would have hit
    `BUTTON_EVENT`/`GPIO_EVENT` identically regardless of what triggered it, so this fix matters well
    beyond the specific timing oddity that surfaced it.
71. RST/BOOT control via the Serial adapter's DTR/RTS lines (§3.3), requested directly: "as the RST
    and BOOT buttons can be controlled by the serial signals, we should document that and add
    tooling to the library." `SerialFrameTransport` already documented (and relied on) the fact
    that opening the port resets the board via these same lines; this exposes that same mechanism
    as first-class, on-demand methods rather than only an implicit side effect of `connect()`.
    `pressReset()`/`releaseReset()` (RTS→EN) and `pressBoot()`/`releaseBoot()` (DTR→GPIO0) are the
    primitives — modeled explicitly as pressing/releasing the physical buttons themselves, matching
    how the user described them, and each independently `boolean`-return-checked via jSerialComm's
    own API (`SerialPort.setRTS()`/`clearRTS()`/`setDTR()`/`clearDTR()`, already used for exactly
    this circuit). `resetToRunMode()` and `resetToBootloader()` compose them into the two sequences
    that matter in practice: a plain reset back into firmware, and forcing entry to the ROM
    bootloader (download mode) for a future OTA/flashing workflow.

    **A real, board-specific timing subtlety was found and fixed while verifying this.** An initial
    `resetToBootloader()` that simply held BOOT+RESET together for a pulse then released both
    reliably failed — the chip never actually reset at all (confirmed: the framed protocol kept
    responding normally throughout, with no reboot delay). Isolated `pressReset()`/`releaseReset()`
    on their own *did* correctly reset the chip (verified: an immediate handshake attempt timed out,
    then succeeded again only after waiting out a real boot sequence) — so RTS→EN itself works
    fine; the bug was specifically in `resetToBootloader()`'s DTR/RTS *sequencing*. Rewriting it to
    exactly match `esptool.py`'s own classic-reset transition order — release BOOT *before* the
    reset pulse starts, then press BOOT again at the same instant the reset pulse ends (so GPIO0 is
    already low the moment EN releases) — fixed it immediately, with no firmware changes needed at
    all (this is a purely PC-side signal-timing fix). This board's auto-program transistor circuit
    is transition-sensitive, not just level-sensitive: holding both lines steady for a pulse doesn't
    reproduce the same electrical sequence as flipping them at the right instants relative to each
    other, even though both sequences reach the same final "steady" DTR/RTS levels along the way -
    worth remembering for any board using this same class of 2-transistor auto-reset circuit.
    **Verified end-to-end on real hardware** (`BoardControlManualCheck`): `pressBoot()`/
    `releaseBoot()` while firmware is already running produced a real `BUTTON_EVENT` PRESS/RELEASE
    for `BUTTON_ID.BOOT`, indistinguishable from an actual physical press (GPIO0 is the same pin
    either way, per design note 69/`ButtonController`); `resetToBootloader()` left the device
    genuinely unresponsive to the framed protocol (a `HANDSHAKE_REQUEST` timed out, since the ROM
    bootloader speaks an entirely different protocol); `resetToRunMode()` recovered it back to
    normal, responsive firmware operation.

    **Follow-up, tested and ruled out**: asked whether the reset-on-connect itself (§3.3's other
    documented behavior — opening the port resets the board regardless of what `connect()` does
    afterward) could be prevented outright from the Java side. Checked `jSerialComm`'s actual
    source (pulled its sources JAR rather than guessing) for a config knob to suppress DTR/RTS
    assertion during `openPort()` — none exists; `disablePortConfiguration()` exists but is
    explicitly documented as a last resort for buggy drivers, disabling *all* of the library's own
    port configuration, not a targeted DTR/RTS option. Empirically tested the one plausible-looking
    lever anyway — calling `clearDTR()`/`clearRTS()` *before* `openPort()` instead of after, in case
    the native open path applied the already-requested deasserted state — on real hardware: **the
    board still reset**. This confirms the reset happens at the Windows COM-port level, below
    `jSerialComm` entirely (establishing the port handle itself asserts these lines transiently,
    before any application, `jSerialComm` included, can request otherwise) — a well-known limitation
    shared by essentially every serial library on Windows (`pySerial` has the identical,
    widely-reported issue against the same class of board), not something fixable in this library's
    Java-level API. The only real fixes are a hardware modification (some CH340 boards expose a
    jumper/removable component to disconnect the auto-reset transistors from DTR/RTS — not verified
    present on this board) or an OS/driver-level setting (not portable/scriptable). Left as-is per
    the user's own call — the existing mitigation (let the reset happen, steer it to normal run
    mode, wait it out) is already the practical ceiling for a pure software fix.
72. §13.1/§13.3's `SET_DEVICE_NAME`/`CONFIG_BACKUP_REQUEST`/`CONFIG_RESTORE` were implemented as the
    first slice of §13 (Config commands) — `SET_WIFI_CONFIG`/`WIFI_STATUS`/`SET_WIFI_ENABLED`/
    `SET_BLE_ENABLED`/`SET_BLE_PIN`/`BLE_STATUS` remain unimplemented, deliberately left for a
    later continuation. Device name was picked first since the handshake's `DEVICE_NAME` TLV and a
    MAC-derived-default `deviceName()` already existed; backup/restore was picked alongside it
    since it's the natural generic container for whatever gets persisted, and testing it needs at
    least one real persisted setting to round-trip. Introduces this firmware's first use of the
    `nvs` partition for actual settings (via Arduino's `Preferences` wrapper, namespace
    `"crowpanel"`) — distinct from `LittleFS`-backed `VOLUME=INTERNAL` (§14, user files). Every
    `SET_*` config command shares one `FLAGS.PERSIST` bit convention (new `configFlags` namespace in
    `Protocol.h`): unset applies for this boot only (a new `gRuntimeDeviceNameOverride`, kept
    separate from NVS), set also writes through to NVS and becomes the power-on default.
    `CONFIG_BACKUP_DATA`'s TLV entries reuse the existing generic `appendTlvString()` helper
    (previously only used for the handshake's own, independent TYPE namespace) rather than a second
    TLV writer; `CONFIG_RESTORE`'s parsing loop was factored into a shared `applyConfigTlvBlob()`
    (persist-or-not parameterized) specifically so design note 73's SD config layer could reuse the
    exact same parser instead of duplicating it. **Verified on real hardware** (`ConfigManualCheck`):
    a session-only `SET_DEVICE_NAME` took effect immediately but never appeared in
    `CONFIG_BACKUP_DATA`; a persisted one did, and survived being cleared-then-restored from the
    saved backup blob byte-for-byte; `NACK(BAD_PARAMETERS)` confirmed for an over-length name and a
    bad `CONFIG_VERSION`.
73. Layered config resolution — SD overrides NVS — requested directly right after device
    name/backup/restore landed: "we could use layered settings - the one stored on SD overriding
    the one internal flash, so we can repurpose device easily." Implemented as `loadSdConfigLayer()`
    in `main.cpp`, called once at boot (right after `gStorageManager.begin()`, before anything that
    calls `deviceName()`): downloads `/device.config` from `VOLUME=SD` if present and feeds its
    bytes through the *same* `applyConfigTlvBlob()` design note 72 already factored out, just with
    `persistToNvs=false` — the SD layer only ever lives in `gRuntimeDeviceNameOverride` for that
    boot session, never written to NVS, so pulling the card cleanly reverts to whatever's actually
    persisted on the next boot rather than leaving a stale copy behind. Choosing to reuse
    `CONFIG_BACKUP_DATA`'s exact wire format for the SD file (rather than inventing a config file
    format) means "back up unit A, drop the file on an SD card, move the card to unit B" *is* the
    entire repurposing workflow, no new tooling needed on either the firmware or PC side. Checked
    once at boot rather than polled live, so swapping cards while already running needs a reboot to
    take effect — matches how repurposing via a physical card swap normally happens with the device
    powered off anyway; polling for a live-hot-swap variant wasn't requested and would need its own
    design pass (when to re-check, how to signal a change) if ever wanted. **Verified on real
    hardware** (`ConfigManualCheck`'s SD-layering test, using `SerialFrameTransport.resetToRunMode()`
    from design note 71 to trigger real reboots mid-test): with `"SdLayerTest"` persisted in NVS and
    a `/device.config` naming `"FROM-SD-CARD"` uploaded to the SD card, a reboot showed
    `DEVICE_NAME="FROM-SD-CARD"` while `CONFIG_BACKUP_DATA` confirmed the NVS layer underneath still
    held `"SdLayerTest"`, untouched; deleting the SD file and rebooting again correctly fell back to
    `"SdLayerTest"`.
74. §13.2's `SET_WIFI_CONFIG`/`WIFI_STATUS_REQUEST`/`SET_WIFI_ENABLED` continue Phase 3's Config
    commands into WiFi. `resolveWifiCredentials()` checks NVS (`kPrefKeyWifiSsid`/`kPrefKeyWifiPass`)
    before falling back to the compile-time `secrets.h` pair, so a device configured over the wire
    keeps using its own credentials across reboots without `secrets.h` ever needing to know. The
    boot-time connect (`connectWifiAndStartTcpServer()`, still a blocking wait-with-timeout — kept
    exactly as-is, already verified working, not touched beyond swapping in the resolved credentials)
    stays separate from the *live* reconnect path (`SET_WIFI_CONFIG` with `FLAGS.CONNECT_NOW`,
    `SET_WIFI_ENABLED(1)`): both just call the inherently non-blocking `WiFi.begin()` and return
    immediately — never re-running the boot-time blocking loop, which would otherwise freeze the
    entire single-threaded firmware for up to `kWifiConnectTimeoutMs` (15s) on every live
    reconfiguration, the same class of mistake design note 70 already found and fixed for
    `GPIO_EVENT`/`BUTTON_EVENT`. A new `startTcpServerIfNeeded()` starts the TCP server the moment
    WiFi actually associates, called both at the end of the boot-time connect and every `pollTcp()`
    iteration, so a live reconnect picks up TCP the same way boot does, without any handler needing
    its own wait loop. `SET_WIFI_CONFIG`/`SET_WIFI_ENABLED(0)` ACK *before* applying the change
    (§13.2's own ordering note) with a short settle delay when the command arrived over TCP itself,
    since disabling WiFi tears down that same connection as a documented side effect.

    **A real Java-side gap was found and fixed while verifying this**: `WIFI_STATUS_REQUEST` timed
    out on the very first test, not because of anything on the firmware side, but because
    `CommandClient`'s `DIRECT_RESPONSE_COMMAND_IDS` map — the table telling it which requests have
    "inherent data responses" per §10, rather than a bare ACK/NACK — had never had a
    `WIFI_STATUS_REQUEST → WIFI_STATUS_RESPONSE` entry added, simply because nothing had needed one
    yet. Fixed by adding it (`BLE_STATUS_REQUEST` will need the same treatment once that handler
    exists). **Verified on real hardware** (`WifiConfigManualCheck`): `NACK(BAD_PARAMETERS)` for
    `SSID_LEN=0`, an over-length SSID, and a malformed `SET_WIFI_ENABLED` payload; a non-persisted,
    non-`CONNECT_NOW` `SET_WIFI_CONFIG` left live state untouched; a full live
    `SET_WIFI_ENABLED(0)` → confirmed disconnected → `SET_WIFI_ENABLED(1)` → polled
    `WIFI_STATUS_REQUEST` until reconnected to the same SSID as boot, all without ever persisting a
    test SSID — `SET_WIFI_CONFIG` has no "clear back to default" convention the way
    `SET_DEVICE_NAME`'s `NAME_LEN=0` does (`SSID_LEN=0` is a validation error, not a clear request),
    so a persisted test SSID would leave the board unable to rejoin the real network on its own with
    no clean way to undo it short of a fresh flash — deliberately avoided rather than risked.
75. §13.2's remaining WiFi work continued straight into `SET_BLE_ENABLED`/`BLE_STATUS_REQUEST`/
    `SET_BLE_PIN`, completing §13.2's wire-level surface. `SET_BLE_ENABLED` stops/starts advertising
    (`NimBLEDevice::stopAdvertising()`/`startAdvertising()`, plus disconnecting any connected
    central via `NimBLEServer::getPeerDevices()`/`disconnect()`) rather than a full
    `deinit()`/`init()` cycle — simpler, and sufficient for what "enabled" means here (mirrors WiFi's
    own `ENABLED`: the stack stays initialized either way, only external discoverability toggles). A
    new `gBleEnabled` flag (distinct from `NimBLEAdvertising::isAdvertising()`, which legitimately
    reads false while a central is connected even though BLE is still "enabled") gates whether
    `BleServerCallbacks::onDisconnect` restarts advertising, so an explicit disable sticks across a
    disconnect. `SET_BLE_PIN` persists `HAS_PIN`/`PIN` to NVS **unconditionally**, ignoring
    `FLAGS.PERSIST` — the one deliberate exception among all of §13's `SET_*` commands: pairing
    security can only be applied at the *next* boot (`NimBLEDevice::setSecurityAuth()`/
    `setSecurityIOCap(BLE_SM_IO_CAP_DISP_ONLY)`/`setSecurityPasskey()`, called from `setupBle()`
    before `gBleStream.begin(..., secure=hasPin)` — NimBLE doesn't support toggling a
    characteristic's encryption requirement live the way advertising start/stop is a simple runtime
    toggle), so a session-only PIN would silently never take effect at all. **Wire-level behavior
    verified on real hardware** (`BleConfigManualCheck`, run over Serial specifically so disabling
    BLE never risks disrupting the connection the test itself uses): validation NACKs, live
    enable/disable/re-enable, `HAS_PIN` correctly reflecting `SET_BLE_PIN`'s persisted state — always
    cleaning the PIN back off afterward so a real BLE central isn't left needing to satisfy pairing.
    Real pairing *enforcement* (does a central actually get prompted, does the fixed passkey actually
    work) was deliberately not exercised — see design note 76, which is also why it couldn't be.

76. **A pre-existing BLE connectivity issue was found while wrapping up §13.2 — not caused by this
    session's work, and not resolved by it.** Verifying the live BLE toggle work above surfaced that
    `BleHandshakeManualCheck` (unrelated to anything just written) couldn't find the device
    advertising at all, from either Windows or a phone. Bisected as thoroughly as the available
    tools allow, each step confirmed on real hardware:
    - Reverted every one of this session's BLE-specific `setupBle()` changes back to their exact
      pre-session form — still invisible.
    - Checked out `firmware/src/main.cpp` at the last commit before this session — still invisible.
    - Checked out `main.cpp` from the single squashed commit at the very start of this repository's
      tracked history (as far back as `git` can distinguish anything) — still invisible.
    - A genuine power cycle (not just a soft DTR/RTS reset) — still invisible.
    - `WiFi.mode(WIFI_OFF)` for the entire boot, ruling out WiFi/BT radio coexistence — still
      invisible.
    - A full flash erase (wiping NVS/calibration data, which a normal app reflash never touches),
      then a fresh flash — still invisible.
    - `SET_WIFI_CONFIG` with real credentials, confirmed via `WIFI_STATUS_REQUEST` to be genuinely
      associated to a real AP (not just idle STA mode) while BLE was scanned for — still invisible.

    With every controllable variable exhausted, the next step was checking whether NimBLE was
    silently swallowing a failure it never surfaced: none of this project's own manual-check tools
    had ever actually displayed the plain-text boot log (`SerialFrameTransport` only reads the
    *framed* protocol, which starts after boot completes), so a throwaway raw serial capture was
    used instead. It showed `NimBLEDevice::init()`, `NimBLEStreamServer::begin()`, and
    `NimBLEAdvertising::start()` **all returning `true`**, ending in firmware's own confirmation
    line ("BLE: advertising as ..."). The entire software stack believes it succeeded at every step.
    These three return-value checks were kept permanently in `setupBle()` (they were previously
    ignored entirely, on both branches of the `if`/`else` this design note didn't touch) since they're
    cheap and exactly this kind of "software says yes, reality says no" gap is what they now catch.

    Since every layer *this project's code touches* reports success, and the antenna is shared with
    WiFi (which works perfectly), the remaining explanation is something below what firmware-level
    diagnosis can distinguish — most likely a hardware fault specific to the BT radio/PA path inside
    the SoC (distinct from the shared antenna trace itself), or a very deep NimBLE/ESP-IDF
    controller-level bug neither this project's tooling nor a raw boot-log capture can see into
    further. Left open, `BLE_ADDRESS` in `BLE_STATUS_RESPONSE` at least confirms the controller
    initializes far enough to derive a real address (standard ESP32 WiFi-MAC-plus-one convention) -
    this is not a total BLE stack failure, just an advertising/RF-visibility one.

    **One more lead was tested and also ruled out**: the user noted BLE connectivity was last
    actually verified back when the communication layer/protocol was originally being built, and
    suggested testing against a non-`SNAPSHOT` release of `BSToolbox-BLE` (`pc-java-lib`'s `pom.xml`
    currently pins `bstoolbox-ble.version=0.3.0-SNAPSHOT`) in case the PC-side library itself had
    regressed. Swapped the runtime classpath's BLE jar (no `pom.xml` change) across all three
    locally-available versions — `0.1.0`, `0.2.0` (both real releases), and the pinned
    `0.3.0-SNAPSHOT` — and reran `BleHandshakeManualCheck` against each: **identical failure every
    time**. Since `BleFrameTransport`/`BleHandshakeManualCheck` themselves are untouched by this
    session and behave the same across three separate library versions, this rules out a PC-side
    library regression as the explanation too — reinforcing that the fault lies below what either
    the firmware or the PC-side Java code can influence.
77. §17 Power management (`SET_POWER_MODE`/`POWER_STATUS_REQUEST`) — command IDs and every enum
    (`powerMode`, `wakeReason`, `setPowerModeFlags`) were already fully scaffolded on both sides
    from the original design pass, so this was purely a handlers-and-firmware-logic pass.
    `LOW_POWER` (light sleep) and `HARD_SLEEP` (deep sleep) map directly onto `esp_light_sleep_start()`/
    `esp_deep_sleep_start()`; `LAST_WAKE_REASON` is computed once at boot (`computeBootWakeReason()`,
    using `esp_reset_reason()` *first* to tell a genuine `HARD_SLEEP` wake — `ESP_RST_DEEPSLEEP` —
    apart from an ordinary reset, since `esp_sleep_get_wakeup_cause()` alone can return a stale value
    on an unrelated reset) and exposed both via `POWER_STATUS_RESPONSE` and a new handshake TLV
    (`LAST_WAKE_REASON`, type `0x0E`, already reserved in both sides' TLV type lists). A new
    `SerialFrameTransport.sendWakePreamble()` (a few repeated `MAGIC` bytes, reusing the same
    write lock as `send()` via a new protected `sendRawBytes()` on the shared abstract base) gives PC
    clients the preamble §17.1's own text recommends before waking a `LOW_POWER` device over Serial.

    **Two real, silent bugs were found on real hardware, not from reading the code** — this is
    exactly the kind of gap `esp_light_sleep_start()`/`esp_deep_sleep_start()`'s own return values
    can't reveal, since those calls behave correctly regardless of whether the *wake* half of the
    round trip actually works:
    - **Wake sources are sticky across separate `SET_POWER_MODE` calls** — `esp_sleep_enable_*_wakeup()`
      calls aren't implicitly cleared just because a *later* call didn't ask for that source again.
      A first `LOW_POWER` call with `WAKE_AFTER_MS=3000` correctly woke on the timer; a *second* call
      right after, with `WAKE_AFTER_MS=0` (meant to wait indefinitely for Serial activity instead),
      woke on the **first call's leftover timer** instead — confirmed by `LAST_WAKE_REASON` reporting
      `LOW_POWER_TIMER` when `LOW_POWER_SERIAL_ACTIVITY` was expected. Fixed with
      `esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL)` at the start of every `SET_POWER_MODE`
      sleep path, before re-enabling exactly the sources this specific call asked for.
    - **`esp_sleep_enable_uart_wakeup()` alone never actually wakes the chip** — light-sleep UART wake
      works by counting RX-pin positive edges against a threshold (`uart_set_wakeup_threshold()`,
      `driver/uart.h`), and that threshold isn't configured to a working value by default. Without it,
      a `LOW_POWER` device with only Serial-activity as its wake source stayed asleep indefinitely —
      confirmed via a raw byte-level capture (bypassing `SerialFrameTransport` to watch the actual
      wire) showing zero response even minutes after sending a wake preamble and a real command.
      Fixed by calling `uart_set_wakeup_threshold(UART_NUM_0, 3)` (the driver's own documented minimum
      — enough edges for one 8n1 byte's start+stop bits) before enabling UART wake.

    **Verified end-to-end on real hardware**, including the two fixes above (`PowerManagementManualCheck`
    plus two throwaway raw-serial scripts for the paths a reboot would otherwise drop the test's own
    connection through): `NACK(BAD_PARAMETERS)` for an invalid `MODE` and an unresolvable `WAKE_BUTTON`;
    `LOW_POWER` waking correctly via both `WAKE_AFTER_MS` (timer) and real Serial activity (with the
    preamble), each reporting the correct `LAST_WAKE_REASON`; `HARD_SLEEP` waking correctly via both
    `WAKE_AFTER_MS` and a real physical button press (`HARD_SLEEP_BUTTON`) — the latter needing the
    user to actually press a button mid-sleep. `KEEP_BLE_CONNECTABLE`'s BLE-activity wake path
    (`ESP_SLEEP_WAKEUP_BT`) could not be verified — blocked by design note 76's still-unresolved BLE
    connectivity issue, unrelated to this work.

78. §16 OTA (`OTA_INSTALL`/`OTA_APPLY`/`OTA_STATUS_REQUEST`/`OTA_CONFIRM`/`OTA_ROLLBACK`) — handlers
    built on `esp_ota_ops.h`'s dual-partition API (`esp_ota_begin`/`esp_ota_write`/`esp_ota_end`,
    `esp_ota_set_boot_partition`, `esp_ota_get_running_partition`/`esp_ota_get_next_update_partition`,
    `esp_ota_mark_app_valid_cancel_rollback`, `esp_ota_get_state_partition`) and `mbedtls_sha256_ret`/
    `mbedtls_md5_ret` for hash verification before any flash write. `OTA_INSTALL` stages into whatever
    `esp_ota_get_next_update_partition()` returns and only calls `esp_ota_set_boot_partition()`
    immediately when `FLAGS.APPLY_NOW` is set; otherwise a later `OTA_APPLY` does that switch. Chose
    the safest possible real-hardware test methodology: OTA the board with its own currently-running
    `firmware.bin` — a functional no-op (both OTA partitions end up holding the exact same,
    already-proven-working image either way) that still exercises the entire real pipeline end to end.

    **Two real, silent bugs were found here too, both invisible from reading the handler code and
    both specific to a single ~1 MB payload — the size class this project hadn't exercised before
    OTA** (every prior command's payload was at most a few KB):
    - **Plain `malloc()`/`new` — and so every default-allocator `std::vector<uint8_t>`, notably
      `StreamFrameTransport`'s receive buffer and `Frame::payload` (`firmware/lib/Transport/
      StreamFrameTransport.h`, `firmware/lib/Protocol/Protocol.h`) — stayed confined to the
      ESP32-S3's ~512 KB internal SRAM despite PSRAM being present and initialized.** This board's
      `qio_opi` PSRAM profile (`platformio.ini`) makes PSRAM *available* but never configures the
      *default* allocator to draw from it — that needs a separate, explicit call this project never
      made. Real-hardware bisection (sending progressively larger payloads with a deliberately-wrong
      hash and watching the raw wire) found the firmware simply stopped producing any output at all
      — no crash, no reset, no NACK, nothing — somewhere between a 450 KB payload (worked) and a
      600 KB one (silently hung indefinitely), matching the internal-SRAM ceiling suspiciously
      exactly. Fixed with one call, `heap_caps_malloc_extmem_enable(8 * 1024)`, in `main.cpp::setup()`
      right after the existing PSRAM self-test line — routes any allocation of 8 KB or more through
      PSRAM automatically from then on, for every default-allocator container in the firmware, not
      just this one buffer.
    - **That fix alone wasn't sufficient — a second, independent bug remained**, found by
      temporarily instrumenting `StreamFrameTransport::poll()`'s buffer-growth loop with
      `Serial.printf()` heap/capacity checkpoints (safe to do only because this diagnostic
      deliberately didn't care about corrupting the concurrent binary frame stream on the same
      UART): the frame's declared-size reallocation itself succeeded correctly and cleanly in PSRAM,
      but the transfer still silently stalled a few tens of KB short of completion, with the
      checkpoint sequence just stopping — no error, nothing. Root cause: the default Arduino
      `HardwareSerial` RX ring buffer is only 256 bytes, which fills in ~22 ms at 115200 baud: if
      `loop()` is ever kept away from calling `poll()` for longer than that (WiFi/BLE housekeeping,
      macro playback, etc. all compete for the same single-threaded `loop()`), the UART driver
      silently drops whatever overflowed — and the frame parser, having already latched the
      declared `PAYLOAD_LEN` from the header, just waits forever for bytes that will never arrive.
      Small payloads rarely hit this (a short transfer gives few chances for an unlucky scheduling
      gap); a 90+ second transfer gives many. Fixed with `Serial.setRxBufferSize(16384)` immediately
      before `Serial.begin(115200)` in `main.cpp::setup()` (must be called before `begin()`).
      Confirmed via the same bisection technique both before (600 KB and 1 MB payloads both hung)
      and after (both completed correctly, reaching the correct `NACK(OTA_HASH_MISMATCH)` in each
      case) applying this second fix on top of the first.

    **Verified end-to-end on real hardware** with both fixes in place (`OtaManualCheck`, staging and
    installing the board's own current ~1 MB `firmware.bin`): `NACK(OTA_NOT_STAGED)` for `OTA_APPLY`
    with nothing staged; `NACK(BAD_PARAMETERS)` for a mismatched `HASH_LEN`; `NACK(OTA_HASH_MISMATCH)`
    for a full-size transfer with a deliberately-corrupted hash; a real `OTA_INSTALL` staging
    successfully (~99 s for ~1 MB at 115200 baud); `NACK(BUSY)` for a second `OTA_INSTALL` while one
    is already staged; `OTA_APPLY` rebooting the device and `OTA_STATUS_RESPONSE.RUNNING_SLOT`
    correctly reflecting the switch to the other OTA partition; `OTA_CONFIRM` clearing
    `PENDING_VERIFICATION`; `OTA_ROLLBACK` rebooting again and `RUNNING_SLOT` correctly reverting to
    the original partition.

79. §17 `SET_POWER_MODE` gained per-peripheral power-rail gating for the display and SD card,
    requested directly: "We should revisit the power management. We have IO07 for screen power
    control and IO42 for SD power control. For full powerdown we can turn them off completely, for
    light sleep it should be recoverable, in both cases carefully to not breaking filesystem. Also
    inactivity powering down the SD would be nice (would probably need unmounting?)" - and, as a
    direct follow-up once this was verified working: "So do the soft sleep keep screen content on
    wake (restore it to previous state)? Could the lazy screen reinit be done in the update method
    (when anything is to be really drawn on the screen from buffer?)" and "So we can add a screen
    controller power timeout too." No wire-format change - §17's own "future work" list already
    named "per-peripheral power gating" as reserved, and both `board::kPinDisplayPowerCtl` (GPIO7)
    and `board::kPinSdPowerCtl` (GPIO42) already existed in the board header, asserted once at boot
    and never touched again until now. All display-side logic below lives in
    `WorkingBuffer` (`firmware/lib/Display/WorkingBuffer.h`) rather than `main.cpp` - it needs
    `panelBuffer_` (this class's own always-correct shadow of "what's really on screen") to do the
    resync described below correctly, and `main.cpp`'s own free-function version of this (an earlier
    draft, briefly shipped mid-session before this redesign) had no access to it.

    - **`HARD_SLEEP`**: both rails are simply cut (`WorkingBuffer::powerDown()`/
      `StorageManager::powerDownSd()`) right before `esp_deep_sleep_start()` - no matching restore
      needed on this path, since a `HARD_SLEEP` wake is a full reboot and `setup()` already
      re-initializes both from scratch (`displaySelfTest()`, `StorageManager::begin()`) exactly as
      if this were a fresh power-on. (Separately, and *not* caused by this power-rail work:
      `displaySelfTest()`'s own unconditional bring-up write - originally a black-then-white flash,
      now the cold-boot info screen (design note 83) - runs on every single boot including a
      `HARD_SLEEP` wake, so the panel's visible content does *not* actually survive a `HARD_SLEEP`
      cycle today, even though the hardware itself would allow it - this is an existing, orthogonal
      design choice about when the bring-up screen runs, not a `SET_POWER_MODE` bug; flagged here
      since it directly bears on "does sleep preserve screen content" but left alone unless asked
      for, since skipping it specifically on a `HARD_SLEEP`-wake reboot, identifiable via
      `LAST_WAKE_REASON`/`esp_reset_reason()`, is its own separate decision.)
    - **`LOW_POWER`**: both rails are cut before `esp_light_sleep_start()`, but this mode resumes in
      the same function call on wake (no reboot), so recovery is this project's own responsibility -
      and, unlike an earlier version of this work mid-session, **both rails are now left powered down
      on wake, symmetrically, recovered lazily** rather than the display being restored eagerly. The
      reason the earlier eager version existed at all - "drawing/refresh commands assume the display
      is already initialized, with no lazy reinit path of their own" - turned out to be wrong once
      actually checked against GxEPD2's own driver source
      (`gdey/GxEPD2_420_GDEY042T81.cpp`, not assumed): every one of its write functions already calls
      `if (!_init_display_done) _InitDisplay();` internally, and `hibernate()` already sets
      `_init_display_done = false` - meaning GxEPD2 itself *already* lazily reinitializes (a genuine
      hardware RST-pin reset + `SWRESET`) the moment anything is next written, with no explicit
      `init()` call needed from this project at all. `WorkingBuffer::ensureControllerReady()`, now
      called as the very first thing `flush()` does, only has to handle the two things GxEPD2 has no
      way to know about itself: restoring the external VCI rail (`board::kPinDisplayPowerCtl`, wholly
      outside GxEPD2's CS/DC/RST/BUSY pins) before that automatic reinit runs, and reseeding the
      controller's own "current"/"previous" RAM banks - which the automatic `SWRESET` unconditionally
      clears, even though the *physical* pixels are completely unaffected (e-ink retains its image
      with zero power, §2.1) - from `panelBuffer_`, via a `writeImagePartAgain()` call carrying no
      `refresh()`, so the resync itself is never visible. Without this reseed, the *first* partial
      update's diff after any reinit would be computed against the controller's just-cleared RAM
      instead of the panel's real prior content, risking a visibly wrong/ghosted result specifically
      in that update's own region - confirmed as a real risk by reading the driver source, then
      confirmed fixed by directly provoking exactly that scenario on real hardware (see Verified,
      below) rather than only reasoning about it. SD recovery is unchanged from the original version:
      `StorageManager::mountSd()` already remounts fresh on every single `VOLUME=SD` operation
      (existing class doc, "hot-pluggable" design), so restoring GPIO42 + a settle delay folded
      directly into that one existing chokepoint already gives every future SD command a lazy,
      correct recovery path for free.
    - **Inactivity power-down, SD only**: `StorageManager` tracks `millis()` of its own last real
      access (refreshed inside `mountSd()`, the one chokepoint every `VOLUME=SD` operation already
      goes through) and exposes `updateIdlePowerDown()`, polled once per `loop()` iteration, cutting
      SD power on its own after 30 s idle - independent of any `SET_POWER_MODE` call, so a long
      `ACTIVE` stretch that never touches the card also saves power. Recovery is the same lazy
      `mountSd()` path the `LOW_POWER`-wake case above already uses. Not yet exposed over the wire
      (no command asked for it) - a firmware-only constant for now.

      A matching `WorkingBuffer::updateIdlePowerDown()` was also built, as a direct follow-up once
      the above was verified working: "we can add a screen controller power timeout too" - then
      **deliberately removed again** once actually measured against the SSD1683 datasheet
      (electrical characteristics, p.43, in response to "How much power does the controller powerdown
      save? As the wake might be expensive... Maybe we'd extend the timeout"): `powerDown()`'s
      `hibernate()` call already runs *unconditionally*, regardless of any idle timeout, and alone
      already drops the controller to its Deep Sleep Mode 1 current (3uA typ/5uA max from VCI,
      *with* VCI still supplied) - so an ACTIVE-mode idle timeout could only ever additionally save
      that same few uA (the level-shifter's own leakage when actually cut is unmeasured, no schematic
      part data available). The wake side isn't the limiting factor either - `ensureControllerReady()`
      never calls `refresh()`, so it stays nowhere near the 1000 uA operating-mode figure regardless
      of idle-timeout duration; on these numbers a longer timeout wouldn't have helped. What actually
      settled it: this timeout only ever fires in `ACTIVE` mode, where the ESP32-S3 itself is fully
      awake running `loop()` - tens of mA on its own, orders of magnitude past the few uA at stake -
      making the real-world payoff immeasurable there, unlike `LOW_POWER`/`HARD_SLEEP` (kept, see
      above), where the rest of the system is also minimized and that same delta becomes
      proportionally real. Requested, built, measured, and reverted within the same session - kept
      here as a record of the reasoning, not a suggestion to reintroduce it without new information.
    - **"Carefully to not breaking filesystem"**: every SD power-down path (`HARD_SLEEP`,
      `LOW_POWER`, the idle timeout) calls `SD.end()` before physically cutting GPIO42, never yanks
      power out from under a mounted card. This matters because `mountSd()`'s existing per-command
      remount design otherwise leaves the SD library's internal state assuming the card stays
      continuously mounted between commands - an unmounted card can't be left in whatever
      half-finished state a bare `digitalWrite(..., LOW)` might otherwise catch it in, even though in
      practice no command handler ever leaves an operation in flight across a `loop()` iteration
      boundary anyway (every `StorageManager` method opens, does its I/O, and closes synchronously
      within one handler call - see design note 47), so idle power-down can never fire mid-write.

    **Verified end-to-end on real hardware**: `PowerRailManualCheck` (run over Serial) - a small
    `DRAW_RECT` (`FLAGS.REFRESH_NOW`) and a full SD `FILE_UPLOAD`/`FILE_DOWNLOAD_REQUEST`/
    `FILE_DELETE` round trip both succeed at four points - fresh after boot; immediately after a
    `LOW_POWER` wake (`WAKE_AFTER_MS` timer); after a real 35 s idle wait with zero SD/display
    activity (comfortably past SD's 30 s idle-power-down threshold, and - while the now-reverted
    display idle timeout still existed - past its matching threshold too, confirming its lazy
    recovery also worked correctly before it was removed again); and after a real `HARD_SLEEP`
    reboot. All four passed, both before and after the `LOW_POWER` display-recovery redesign above.
    Separately, `DisplayResyncManualCheck` directly
    targeted the ghosting risk the redesign's RAM-reseed fix addresses: drew a filled black square,
    forced a `LOW_POWER` sleep/wake cycle, then immediately drew a smaller white square *inside* the
    still-black region - exactly the "partial update right after a reinit, into a region with real
    prior content" scenario the fix exists for. Visually confirmed clean by the user (a sharp white
    cutout with no ghosting or stray pixels) - the one part of this that has no programmatic
    equivalent, since nothing exposes the controller's own internal RAM banks over the wire to check
    directly; only this class's own (potentially-wrong-if-the-bug-were-present) `panelBuffer_` shadow
    is readable via `READ_SCREEN`.

80. **A real, previously-unnoticed WiFi bug was found live, from the user's own current-draw
    measurement while working through the power-rail work above**: "Is the wifi currently enabled?
    As it currently draws about 120mA when not suspended." Traced to `connectWifiAndStartTcpServer()`
    (boot): `WiFi.mode(WIFI_STA)` ran unconditionally, but the early-return for "no credentials
    configured" - the exact state this whole session's test board has been in throughout every
    earlier check - never called the matching `WiFi.mode(WIFI_OFF)` the way the sibling
    "WiFi explicitly disabled" branch right above it already did, leaving the radio sitting in STA
    mode, unassociated, indefinitely - consistent with a continuous ~120 mA draw (an unassociated
    STA-mode radio's own scanning/probe activity). The exact same shape of bug existed independently
    at two more call sites - `SET_WIFI_ENABLED(1)` and the `LOW_POWER`-wake WiFi-restoration path -
    each also setting STA mode unconditionally and only conditionally calling `WiFi.begin()`.
    Consolidated all three into one shared `startWifiOrPowerOff(ssid, password)` (also adopted by
    `SET_WIFI_CONFIG`'s own connect-now path, for consistency, though that one was never actually
    vulnerable - its `ssid` is already validated non-empty) rather than patching each site
    individually, so this class of bug can't recur at a fourth call site later. Boot's own
    `WiFi.mode(WIFI_STA)` driver warm-up (needed so `WiFi.macAddress()` - `deviceName()`'s
    MAC-derived fallback - works even when never actually associating) is kept as its own explicit
    line rather than folded into the shared helper, to avoid the risk of silently changing that
    already-debugged-once behavior. **Verified fixed on real hardware**: `WIFI_STATUS_REQUEST`
    reported `ENABLED=0` immediately after boot with no credentials configured, where it would
    previously have reported `ENABLED=1` (and drawn the ~120 mA) despite `CONNECTED` correctly
    already reading 0 either way.

    A parallel BLE power-management consolidation was considered (design note 79's own
    per-peripheral pattern prompted the question directly: "The same consolidation might be usefull
    for bluetooth power management") but no equivalent bug was found - `handleSetBleEnabled()`,
    `setupBle()`, and the `LOW_POWER`-wake BLE-restoration path are already a clean, symmetric
    enable/disable toggle (`startAdvertising()`/`stopAdvertising()`+disconnect peers) with no
    "credentials"-shaped third dependency the way WiFi has; there's no intermediate state where BLE
    ends up "enabled" but radio-idle-yet-still-costly the way an unassociated STA-mode WiFi radio is.

    **mDNS added** (`#include <ESPmDNS.h>`, part of the Arduino-ESP32 core, no new `platformio.ini`
    dependency needed), requested directly as a follow-up: "We should also add mDNS for the wifi with
    the device name and wifi on (or is that already present?)" - it wasn't. `startTcpServerIfNeeded()`
    (already the established "WiFi just associated, first-time setup" chokepoint, called from both
    the boot-time blocking connect and every `pollTcp()` iteration) now also calls
    `MDNS.begin(deviceName().c_str())` and `MDNS.addService("crowpanel", "tcp", kTcpPort)` the same
    moment it starts the TCP server; `stopTcpServerAndClient()` calls `MDNS.end()` symmetrically,
    so a later reconnect re-begins it fresh with whatever `deviceName()`/IP apply at that time (same
    "reflects state as of that specific connection, not live-updated" precedent `WiFi.setHostname()`
    already established). No `loop()`-driven polling needed - this core's `ESPmDNS` runs on the IDF
    mDNS component internally, unlike the older ESP8266 core's `MDNS.update()`-driven version.
    **Verified end-to-end on real hardware**: connected the device to a real WiFi network live
    (`SET_WIFI_CONFIG`, session-only), then `ping CrowPanel-3851DC.local` from this PC resolved to
    the device's actual DHCP-assigned IP and got real replies.

81. **Real current-draw measurements across every power state**, taken live with a physical
    multimeter while working through design notes 79-80 above, requested directly: "test the low
    power modes." **Important methodology caveat, stated directly by the user and worth repeating
    prominently: this board was powered via USB for every measurement below, so the CH340
    USB-serial adapter (used for flashing and the Serial transport, §3.3) was continuously active
    and drawing its own current the entire time, on the same measured rail** - none of these
    figures isolate the ESP32-S3 module's own consumption in isolation. The *relative* drops
    between states should still be broadly indicative (the CH340's own draw is a roughly constant
    offset present in every row), but the *absolute* `LOW_POWER`/`HARD_SLEEP` floors below are not
    what a battery-powered deployment with no USB/CH340 attached at all would see - such a
    deployment should draw meaningfully less than the absolute numbers here in those two rows,
    though this hasn't itself been measured (no such non-USB test rig existed for this session).

    Synchronization method: each state change first drew a short status label on the panel
    (`CLEAR_REGION` then `DRAW_TEXT`, `FLAGS.REFRESH_NOW`) describing exactly what was about to be
    measured, *then* applied the state change - giving the user an unambiguous on-panel cue for
    which reading corresponds to which row, addressed directly: "For synchronization a message for
    next status on the display would work." For the two sleep-mode rows, the label was drawn
    immediately before sending `SET_POWER_MODE`, timed so it was already on the panel and the ACK
    already received (sleep entry follows the ACK, §17.1) before the user needed to read the meter.

    ```
    State                                    Reading
    ----------------------------------------------------------------------
    ACTIVE, WiFi OFF, BT ON                  ~115 mA
    ACTIVE, WiFi ON (associated), BT OFF     80-115 mA (oscillating)
    ACTIVE, WiFi OFF, BT OFF                 128 mA, settling to 115 mA after ~5s
    LOW_POWER (light sleep)                  10.7 mA
    HARD_SLEEP (deep sleep)                  0.88 mA
    ```

    **Findings**:
    - **In `ACTIVE` mode, WiFi/BT state made no clearly attributable difference** - all three
      readings cluster around 115 mA, with more variation *within* a single state (the 80-115 mA
      oscillation while WiFi was genuinely associated; the 128→115 mA settling over ~5s with both
      radios off) than *between* states. This is consistent with - and a direct real-hardware
      confirmation of - the reasoning behind dropping the display's own `ACTIVE`-mode idle timeout
      (design note 79/80): in `ACTIVE` mode the ESP32-S3 CPU (continuously running `loop()`) plus
      fixed board-level draw (the CH340 above included) dominates the total, and a few-mA-or-less
      radio delta gets lost in that noise floor. The first of these three ACTIVE-mode readings was
      briefly, mistakenly reported as "1128-1130 mA" (a typo, corrected by the user moments later,
      "the previous 1230 was a typo") - flagged here only so a future reader doesn't take that
      number as real; the actual first ACTIVE-mode reading was ~115 mA like the other two.
    - **`LOW_POWER` (light sleep) drew ~10.7 mA - roughly an 11x drop from the ~115 mA `ACTIVE`
      baseline.** Expected: the CPU itself pauses, and by this point in the session `SET_POWER_MODE`
      also explicitly cuts the display and SD power rails before entering light sleep (design note
      79).
    - **`HARD_SLEEP` (deep sleep) drew ~0.88 mA - roughly a 12x further drop from `LOW_POWER`, and
      about 130x below the `ACTIVE` baseline.** Expected for ESP32-S3 deep sleep: everything is
      powered down as hard as the chip allows, leaving only RTC-domain leakage (the timer/wake
      circuitry) plus whatever the CH340 and other always-on board components still draw on their
      own (see the methodology caveat above - a meaningful fraction of even this small figure is
      likely not the ESP32-S3 itself).

    **Overall**: a clean, textbook power curve confirming this project's power-management work is
    aimed at the right place - `LOW_POWER`/`HARD_SLEEP` deliver large, clearly-measurable savings,
    while `ACTIVE`-mode peripheral/radio gating (deliberately not pursued further for the display,
    design note 79/80) would be chasing savings this measurement can't even reliably distinguish
    from noise.

82. **A real PlatformIO board definition** (`firmware/boards/crowpanel_4_2.json`), requested via
    "Continue with Phase 4 next" - the last item in that phase not blocked on something else: BLE
    pairing needs design note 76's still-unresolved BLE connectivity issue investigated first, and
    `pio test -e native` still has no system C++ compiler available in this dev environment.
    Replaces the generic `esp32-s3-devkitc-1` stand-in `platformio.ini` used since this project's
    very start. Turned out this didn't actually need the exact pin mapping (Phase 1, still only
    schematic-derived, never physically probed) confirmed first, contrary to what `plan.md` had
    assumed when this item was originally deferred - that concern only applies to this project's
    own `firmware/include/boards/board_crowpanel_4_2.h` (GPIO wiring, a different thing with a
    similar name), never to a PlatformIO board JSON, which only ever carries MCU/flash/PSRAM/upload
    metadata.

    Folds in everything `platformio.ini` previously had to override on top of the generic
    stand-in, all already independently confirmed correct on real hardware earlier this project:
    `build.arduino.memory_type=qio_opi` (the ESP32-S3-WROOM-1-N8R8 module's Octal-SPI PSRAM - the
    devkitc-1 profile's own `name` field literally says "No PSRAM", the opposite of this module),
    `upload.speed=115200` (CH340 USB-serial reliability, design note - see §3.3's own history),
    `upload.flash_size`/`maximum_size` (8MB). Reuses the generic `esp32s3` pin variant rather than
    defining a board-specific one - unneeded, since this project addresses GPIOs directly by
    number (`board::kPin*`) rather than through Arduino `Dx`-style aliases a custom variant would
    exist to provide.

    PlatformIO board JSONs require a real `url` field with no way around it (`pio run` refuses to
    proceed without one) - asked the user directly for the CrowPanel product/wiki page rather than
    guess one, per this session's own standing rule never to fabricate URLs.

    **Verified on real hardware**: rebuilt against the new profile - identical RAM/Flash usage to
    the last known-good build (21.3%/32.9%), confirming the new profile produces the exact same
    build configuration, not just a same-looking one - and reflashed; the plain-text boot log was
    unchanged (PSRAM found at the same size, the same OTA partition addresses/sizes as every
    earlier boot this project, LittleFS mounted, display self-test passed); and a full
    `StorageManualCheck` round trip (`VOLUME=INTERNAL`/`SD`/`PSRAM`, upload/list/download/delete)
    passed over the real framed protocol, not just the raw boot log.

83. **Cold-boot info screen**, requested directly: "We'd add a cold boot screen (before the full
    cleanup, don't worry about remaining picture, use opaque drawing) to identify the loaded
    firmware, device and resolution." Replaces `displaySelfTest()`'s old full-black-then-full-white
    fill (which left the panel blank, showing nothing useful) with three lines of actual
    information - `FW <version>`, `DEV <device name>`, `RES <width>x<height>` - drawn through the
    same path every `DRAW_TEXT` command already uses (`WorkingBufferGfx`/`drawText()`,
    `EmbeddedFont.h`), not a separate Adafruit_GFX-level implementation, so the boot screen renders
    with the exact same embedded font a client would see from any other text draw.
    `kFirmwareVersion` was previously duplicated inline in two places (the handshake TLV builder,
    `OTA_STATUS_RESPONSE`) with a comment asking the reader to keep them in sync by hand - now a
    single shared constant, since this is a third use.

    **"Is the multistep redraw init needed? (specially the bordered redraw)?"**, asked directly -
    answered by reading GxEPD2's own driver source rather than assumed, then confirmed on real
    hardware: no. `gdey/GxEPD2_420_GDEY042T81.cpp`'s `_writeImagePart`/`_writeImage`/
    `_writeScreenBuffer` all check `_initial_write` and call `writeScreenBuffer()` (a full clean
    pass) if it's still set, and `refresh()` checks `_initial_refresh` and forces a full update
    (never partial) for the very first refresh after `_InitDisplay()` - both flags GxEPD2 manages
    entirely on its own. A full update's own waveform redraws the *entire* panel from the buffer
    regardless of what was physically there before, not just the pixels that logically changed -
    so a separate preliminary black-then-white pass was never actually required to get a clean
    result on the *first* write after init; the old two-step sequence was only ever useful as its
    own manual hardware self-test (visually confirming the panel could reach both color extremes
    correctly), a distinct purpose from what this cold-boot screen is for, and not one it
    attempts to replace. (The SSD1683's own "border" concept, `BorderWaveform Control`/VBD, §9.1
    step 3 of the datasheet's init flow, is a *register setting* for the physical border strip
    around the active area, configured once as part of `_InitDisplay()` - not a separate redraw
    step at all, and unaffected by any of this.)

    **Updated by design note 85**: this screen's own `flush()` call was originally full-panel
    (`full=true`); once `/init.macro` (design note 85) took over doing a real full-panel
    clear+degauss right after it anyway, the boot screen's own flush was narrowed to just the
    text's own bounding box (`full=false`) - faster on its own, and no longer redundant with what
    immediately follows it.

    "Use opaque drawing": `drawText()`'s existing `opaqueBackground=true` parameter (already
    exposed over the wire as `DRAW_TEXT.BACKGROUND=OPAQUE`, §12.6) fills each glyph cell's non-ink
    pixels with white, so old panel content directly under each line of text can't show through
    around the letters - combined with the full-panel update redrawing everything else in
    `WorkingBuffer`'s `buffer_` (still all-white outside these three lines, its power-on-default
    state) regardless, nothing needs a separate/manual clear pass first ("don't worry about
    remaining picture").

    **Verified on real hardware**: the single-pass boot screen took ~4.1s (`Display: cold-boot
    info screen shown (4055 ms)`) - faster than the old two-step sequence's ~6.1s
    (4026ms + 2064ms) it replaced - and was visually confirmed clean and legible by the user, with
    no ghosting or artifacts anywhere on the panel.

84. **`FAST_CLEAR` (§12.16, `0x030E`)**, requested directly as a follow-up to the "clear command"
    question above: "the idea was just fast buffer filling with 1 or 0, the opposite to brush
    color that we already have (do we?), skipping all clipping and mapping guards." Checked first
    (as the "(do we?)" asked) - confirmed there is no persistent "current brush/background color"
    concept anywhere in this protocol; every fill command (`CLEAR_REGION`, §12.5; this one) already
    takes `COLOR` explicitly, so `FAST_CLEAR` does too. `WorkingBuffer::fastClear()` writes
    `buffer_`'s raw bytes directly via a single `std::fill` - no `getPixel()`/`setPixel()` per-pixel
    calls, and critically, its handler calls `finishWrite()` directly rather than `finishDraw()`,
    which is what applies `SET_CLIP_REGION`/`SET_DRAW_OFFSET`/`SET_ORIENTATION` (`computeAffected
    PhysicalRegion()`) for every other §12 primitive - `FAST_CLEAR` is the one command in this
    section that deliberately skips all three, exactly as asked ("skipping all clipping and
    mapping guards"), correct specifically because it always covers the whole physical panel
    regardless (see §12.16's own doc for why that makes skipping them safe, not just fast).

    **A first verification attempt looked like a real firmware bug and wasn't one** - worth
    recording since it's exactly the kind of thing this session has otherwise caught for real: a
    quick `READ_SCREEN`-based check found `BLACK` fills reading back as not-black at every sampled
    pixel while `WHITE` fills read back correctly, which looked like a black-specific polarity bug
    in `fastClear()`. Tracing it down (`WorkingBuffer::syncPanelSnapshot()`/`packRegion()`, both
    checked directly and found correct) instead found the bug in the *test script*: it read
    `SCREEN_DATA`'s payload as if always `RAW`-encoded, never checking the `ENCODING` byte - the
    device's actual (correct) `RLE_PACKBITS`-encoded response for a solid-black 1×1 region doesn't
    happen to look like a raw `0x80` byte at the same offset, while a solid-white region's RLE
    encoding coincidentally did, masking the test bug for that half of the comparison. Fixed the
    test to actually RLE-decode; all checks then passed.

    **Verified end-to-end on real hardware**: validation `NACK(BAD_PARAMETERS)` for both a wrong
    payload size and an invalid `COLOR`; a full-panel `BLACK` fill and a full-panel `WHITE` fill
    each read back correctly at three sampled points (two corners plus center) via `READ_SCREEN`
    (with correct RLE decoding this time); and, with an active `SET_CLIP_REGION` deliberately set
    to a small 50×50 sub-rectangle beforehand, a `FAST_CLEAR` still correctly filled a point well
    outside that clip region - confirming the clip is genuinely ignored, not just untested. Timing:
    ~2.0s for `BLACK`, ~2.0s for `WHITE` - both notably faster than `displaySelfTest()`'s old
    per-pixel `gDisplay.fillScreen()`-based fills (~4.0s black, ~2.1s white, design note 83),
    consistent with skipping per-pixel Adafruit_GFX overhead being a real, measurable speedup, not
    just a theoretical one.

85. **`/init.macro`, chained before `/boot.macro` on every cold boot**, requested directly: "add
    additional 'init' optional macro, run first, flash only. Set it to sleep 1s, clear and
    degauss. That would be 'fixed' boot init procedure." - refined twice in immediate follow-ups:
    "Not flash dependent, just three step boot init on cold boot" (dropped the original
    "flash only" gating entirely - no `esp_reset_reason()`-based detection was actually needed,
    let alone built; see the abandoned approach below) and "The sequence should be system init -
    init macro - boot macro" (chained before the regular boot macro, not run instead of it - the
    two are appended into a single `MacroPlayer` run via a shared `appendBootMacroEntries()`
    helper, since `MacroPlayer` only ever holds one playback at a time).

    **An abandoned approach, worth recording since real design work went into it before it was
    cut**: the original ask's "flash only" was going to be implemented via a per-build identity
    string (`__DATE__ " " __TIME__`, changes on every compile) persisted to NVS and compared at
    boot - `esp_reset_reason()` alone can't distinguish "just flashed" from any other reset on this
    board, since the CH340's DTR/RTS auto-reset circuit produces an identical `POWERON`-style reset
    whether esptool just flashed a new image or a PC client's `SerialFrameTransport` merely opened
    a connection (§3.3's own class doc covers this same behavior). Removed entirely once "flash
    only" was dropped - but the `__DATE__ " " __TIME__` identity string itself was kept and
    repurposed, see below.

    `/init.macro` is a real file (same `.macro` format, §18), not a hardcoded routine - the device
    seeds it onto `INTERNAL` with a fixed default the first time neither volume has one:
    `PAUSE(1000ms)` → `FAST_CLEAR(WHITE)` (§12.16) → `CLEAR_ARTIFACTS(CYCLES=1)` (§9). Reuses
    `MacroRecorder` (a local instance) to build the seed bytes rather than hand-rolling the entry
    layout a second time. Deliberately user-editable/replaceable afterward, same as `/boot.macro`
    always was.

    **Build-date versioning**: "The build identity would be nice for versioning (build date info)"
    - the abandoned fresh-flash-detection identity string above got a second life here instead.
    `kFirmwareVersion` (already shared across the handshake TLV, `OTA_STATUS_RESPONSE`, and the
    cold-boot screen, design note 83) now reads `"0.1.0-dev (" __DATE__ " " __TIME__ ")"` - adjacent
    string-literal concatenation, resolved entirely at compile time, no runtime cost - since the
    hand-maintained `"0.1.0-dev"` part hasn't actually been bumped all project and so can't tell
    two different builds apart on its own.

    **A real, measured slowness, found from a direct comparison against the original firmware's
    own behavior**: "the original firmware was us[]ing sort of fast flashing instead of slow
    redraws, doesnt the driver support something like that? Now it redraws filled screen several
    times and the init is even slower than before." Checked the actual driver source rather than
    guessed: `GxEPD2_420_GDEY042T81::useFastFullUpdate = true` is already this panel class's own
    default, applied automatically in its constructor - every full update here already uses the
    fastest waveform this panel supports; there was no missing "fast mode" to turn on. The real
    cause was `CLEAR_ARTIFACTS`'s own `kDefaultClearArtifactsCycles = 3` - `CYCLES × 2` full
    updates per the handler (one black, one white, per cycle), each measured at ~4s in real use
    (design note 83's own boot-log timing, well above this driver's nominal `full_refresh_time`
    constant of 1200ms) - meaning the *default* degauss alone was doing 6 full updates, roughly
    12-24s, well past the 2-full-update old self-test this whole sequence replaced. `CLEAR_ARTIFACTS`'s
    general-purpose default is reasonably aimed at a long-running device's own accumulated
    ghosting; a fresh boot has none of that yet, so the seeded `/init.macro` explicitly passes
    `CYCLES=1` (2 full updates) instead of relying on the default. A true partial-update mode
    (`hasFastPartialUpdate`, `partial_refresh_time = 400ms`) exists too, but isn't a substitute
    here - a degauss cycle's whole point is the more thorough full-update waveform's ghosting
    removal, which a partial update doesn't provide, so switching `CLEAR_ARTIFACTS` itself to
    partial mode would have defeated its own purpose rather than just making it faster.

    **Verified on real hardware**: the seeded `/init.macro`'s bytes matched the intended format
    exactly, byte for byte, both before and after the `CYCLES` fix (deleted the stale
    already-seeded file and reflashed to force a fresh re-seed for the second check);
    `HANDSHAKE_RESPONSE.FIRMWARE_VERSION` correctly reported the real build timestamp; the panel
    read back correctly blank (`READ_SCREEN`) at every sampled point - including inside the boot
    screen's own text region - after the full sequence ran; and, after the cycle-count fix, the
    entire boot screen → pause → clear → degauss sequence completed within `SerialFrameTransport`'s
    own 12s connect-time settle window (previously not confirmed to fit inside it). User confirmed
    both that the intended visual sequence (text visible, then flashed away by cleanup) looked
    correct, and, separately after the cycle-count fix, that it now feels appropriately fast again.

86. **`LOG_MESSAGE` (§10.1, `0x0005`) — bidirectional debug/sync marker, plus `init_done`/
    `boot_done` boot markers**, requested directly: "For debugging we could make the log frame
    bidirectional, so it could be put inside a macro by the PC and when executed just repeated
    back - the PC than could wait for receiving that frame (e.g. for timing or for waiting for
    macro completion before sending more commands)" (an earlier-session idea for a firmware
    logging channel, narrowed to this shape), followed by: "We can add init_done and boot_done
    messages after running the startup macros."

    No wire-format structure — payload is an opaque PC-chosen marker. `handleLogMessage()`
    (main.cpp) both ACKs normally and unconditionally broadcasts an echo (`pushLogMessage()`) on
    every live transport, exactly like `pushGpioEvent`/`pushButtonEvent` already did — pushed
    directly to `gSerialTransport`/`gBleStream`(guarded by `.ready()`)/`gTcpTransport`, never via
    `ctx.reply()`, since a macro-replayed command's `ctx.transport` is the null-sink
    `gMacroTransport` (§18) and would otherwise discard it. Deliberately **not** added to
    `DIRECT_RESPONSE_COMMAND_IDS` on the PC side — a macro-embedded marker's echo can arrive long
    after the SEQ-based correlation window of the original send (if any) has closed, so it always
    correlates by payload content via the event-listener path instead.

    **`CommandClient` gained multiple event listeners**: `setEventListener()` (single, silently
    clobbered by a second call) replaced with `addEventListener()`/`removeEventListener()` over a
    `CopyOnWriteArrayList`, so `waitForLogMessage()`'s own temporary listener can coexist with
    whatever the caller already registered for `BUTTON_EVENT`/`GPIO_EVENT` — the four existing call
    sites (`CommandClientTest`, `BoardControlManualCheck`, `ButtonManualCheck`, `GpioManualCheck`)
    updated to the new name with no behavior change. `waitForLogMessage(marker, timeoutMillis)`
    arms a temporary listener, blocks on a `CompletableFuture` until a `LOG_MESSAGE` frame's
    payload exactly matches `marker`, and always removes the listener again (success or timeout).

    **A real race, found and fixed in the new `LogMessageManualCheck` test itself, not in
    `CommandClient` or firmware**: the live round-trip section originally called `send()` then
    `waitForLogMessage()` - but firmware sends the echo *before* its ACK
    (`pushLogMessage(); ctx.ack();`), so the echo could arrive and find no listener registered yet;
    fixed by arming the wait on a background thread before sending. The macro-replay section hit
    the same race in a tighter form and initially went undiagnosed as a full 10s timeout with no
    echo ever received: `handlePlayMacro()` calls `gMacroPlayer.start()` synchronously before
    `ctx.ack()`, and `stepMacroPlayback()` runs later in that *same* `loop()` iteration, so a
    macro's first entry (here, the `"start"` marker) can echo within microseconds of `PLAY_MACRO`'s
    own ACK - reliably faster than the test thread waking from `send()`'s blocking call and only
    then registering its listener. Diagnosed by inspecting `handlePlayMacro`/`stepMacroPlayback`/
    `Dispatcher::dispatch`/`MacroPlayer`/the PSRAM-volume `StorageManager::download` path in turn
    (all correct) before re-reading `CommandClient.correlates()`/`handleFrame()` and recognizing the
    exact same listener-registration-order problem already fixed once for the live case. Fixed by
    arming both `"start"` and `"end"` marker listeners (as one shared `CommandEventListener`
    completing per-marker `CompletableFuture`s) *before* calling `send(PLAY_MACRO, ...)`.

    **Verified on real hardware**: `LogMessageManualCheck` — live round trip (ACK 9ms, echo 9ms),
    macro-recorded round trip (`"start"` at t+8ms, `"end"` at t+1006ms, ~998ms gap matching the
    recorded 1000ms `PAUSE`) — all pass. A separate throwaway check confirmed `init_done`/
    `boot_done` both fire on a real cold boot (t+8898ms/t+8904ms, back-to-back since this device has
    no `/boot.macro`), matching the ~9s total observed for the init-macro sequence's `PAUSE` +
    `FAST_CLEAR` + two full updates.

87. **Two-tier PIN access control (§5.3) — usage/admin PINs, gating every command, on all three
    transports.** Requested directly, pointing out the protocol had no access control at all:
    "There is no lock on configuration or usage. We should add two optional configurable PINs to
    the firmware... One for usage, one for persistent writing and config... Handshake would
    optionally take pin type and value to unlock a feature level, admin pin unlocking also the
    usage functions. Closed TCP or BLE connection would relock... new COM connection reboots
    anyway. The CLI would need a pin parameter too."

    Three rounds of direct correction refined the design before implementation, each addressing a
    real gap in the first draft:
    - **The USAGE/ADMIN boundary** ("Same as 2), but allow user to control power mode") settled on
      USAGE = normal display-driving (drawing, image transfer, macro playback, GPIO actuation) vs.
      ADMIN = configuration/security/OTA/macro-recording/storage-writes — with `SET_POWER_MODE`
      specifically at USAGE, since sleeping the device is routine use.
    - **The fallback rule** ("If only usage pin is configured, admin commands require that too (not
      open). Only when just admin pin is set, usage is open.") replaced an initial, less secure
      "unconfigured tier = always open" draft with the cascading rule in §5.3: configuring *any*
      PIN expresses real intent to restrict the device, so ADMIN falls back to requiring the USAGE
      credential when no admin PIN exists, rather than leaving ADMIN commands wide open just
      because that one tier's own PIN was never set.
    - **The MENU+BACK override scope** ("The button pin override was supposed to be only for
      serial connection (might be physically restrained, to be used just as recovery)") corrected
      an initial draft that checked button state on every transport - it's Serial-only, a
      deliberate two-button hold standing in for the physical-access recovery path Serial usually
      gets "for free" on this project (§3.3), now that Serial itself is gated too instead of
      exempted.

    A macro replay bypasses the gate entirely for every entry (`stepMacroPlayback()` dispatches
    with a hardcoded ADMIN grant, ignoring live PIN configuration) - confirmed directly as a
    deliberate design property, not an oversight: "Does the macro content bypass the gating? (that
    would be OK as macro can be prepared only with required privileges)". Safe by construction: the
    only ways a `.macro` file can exist are `RECORD_MACRO`/`SAVE_MACRO` and `FILE_UPLOAD`, both
    ADMIN-gated, so `PLAY_MACRO` (USAGE-gated) can only ever trigger a script an admin already
    approved - delegation, not escalation.

    **A second real timing bug was found and fixed along the way**, unrelated to access control
    itself but surfaced by testing it: opening a *new* Serial connection reboots the board (already
    documented, DTR/RTS auto-reset), so `Cli`'s new auto-handshake-on-connect could arrive before
    firmware had actually finished booting - the exact "real fix... still open" gap
    `SerialFrameTransport`'s own class doc had flagged, now closed by having `SerialHmiDevice`
    additionally wait for the `boot_done` marker (design note 86) before `connect()` returns,
    layered on top of (not replacing) the existing fixed settle delay.

    Implementation: `Dispatcher` (`firmware/lib/Dispatcher/Dispatcher.h`) gained a pure static
    `effectiveRequiredLevel()` implementing the cascading rule once, shared by the generic
    per-commandId gate and `handleSetPowerMode`'s own extra `HARD_SLEEP` check, so the
    security-relevant formula exists in exactly one place; `registerHandler()` now takes a required
    level per command; `CommandContext` gained an `authLevel` field. `ButtonController` gained a
    public `isPressed()` reading its already-debounced state (not a raw `digitalRead()`, avoiding
    duplicated polarity knowledge). Three new per-transport granted-level globals
    (`gTcpAuthLevel`/`gBleAuthLevel`/`gSerialAuthLevel`), reset at each transport's own
    connection-established point - a **new** `BleServerCallbacks::onConnect` override had to be
    added, since none existed before this. PINs persist via `Preferences`, following `SET_BLE_PIN`'s
    established convention (always persists regardless of `FLAGS`) but - unlike the BLE PIN, only
    ever applied at boot - additionally updating a live in-RAM mirror immediately, since this gate
    is checked on every dispatch, not just at boot.

    **Verified on real hardware** (Serial, `COM5`, full round trip via `Cli`): baseline (no PINs)
    behaves exactly as before; bootstrapping an admin PIN with nothing configured succeeds
    unauthenticated, as designed; an admin-gated command NACKs `NOT_AUTHORIZED` (`0x0E`) without
    the PIN and succeeds with it; with only a usage PIN configured, USAGE commands succeed and
    ADMIN commands NACK; with only an admin PIN configured, USAGE commands stay open (cascading
    rule); `SET_POWER_MODE|HARD_SLEEP` succeeds over Serial with only a usage PIN (device woke via
    its own timer and reconnected normally); clearing both PINs (`HAS_PIN=0`) fully reopens the
    device. The MENU+BACK override was verified live with the user physically holding both buttons
    during a real handshake: an admin-gated command (`SET_DEVICE_NAME`) succeeded with no PIN
    offered at all while an (unknown to that connection) admin PIN was configured - the one check
    in this whole feature that can't be simulated in software, unlike `BOOT`/`RESET` (not wired to
    the CH340's DTR/RTS lines the way those two are).

88. **`FILE_COPY`/`FILE_RENAME` (§14.6/§14.7)**, added as preparation for a still-undesigned
    future feature (event-triggered macros: auto-playing a same-named `VOLUME=PSRAM` macro on a
    `BUTTON_EVENT`/`GPIO_EVENT`) which needs a way to stage a macro durably on SD/INTERNAL and then
    get it into fast, session-only PSRAM, and to atomically swap in a new version later without a
    delete-then-reupload gap — both previously just noted as unimplemented future ideas.

    `FILE_COPY` is cross-volume-capable (separate `SRC_VOLUME`/`DST_VOLUME` fields) since the
    motivating use case is specifically SD/INTERNAL → PSRAM; implemented in `StorageManager` as
    `download()` then `upload()` rather than a new per-filesystem copy path, reusing two
    already-tested primitives (and getting PSRAM's existing 2 MiB budget enforcement, already
    inside `upload()`, for free when PSRAM is the destination). `FILE_RENAME` is deliberately
    same-volume only (one `VOLUME` field) — matches POSIX `rename()` semantics, and the "swap in a
    new version" use case only ever needs to rename within one volume; PSRAM does this as a plain
    `std::map` key move, SD/INTERNAL via the underlying `fs::FS::rename()`. Both commands overwrite
    an existing destination, matching `FILE_UPLOAD`'s own established overwrite semantics — the
    "atomic swap without a gap" motivation only makes sense if the destination can already exist.
    No new NACK status codes were needed; both are `ADMIN`-gated like `FILE_UPLOAD`/`FILE_DELETE`,
    since they mutate storage.

    A new `parseVolumeAndPathAt()`/`parsePathAt()` pair in `main.cpp` generalizes the existing
    `parseVolumeAndPath()` (which always starts at payload offset 0) to parse a `VOLUME(1)+
    PATH_LEN(1)+PATH` or `PATH_LEN(1)+PATH` field at an arbitrary offset, so `FILE_COPY`'s two
    volume+path fields and `FILE_RENAME`'s shared-volume two-path fields can each be parsed by
    calling the appropriate helper twice rather than hand-rolling the parse per handler.

    Verified: `pc-java-lib`'s `CommandSchemaRoundTripTest` (schema-driven, covers both commands
    automatically) and `CommandIdTest` pass; `firmware` builds clean for `esp32-s3-crowpanel`.
    `StorageManualCheck` was extended with same-volume copy/rename (including the
    overwrite-existing-destination case) for every volume, plus a dedicated cross-volume
    (`INTERNAL`→`PSRAM`) copy check — **verified end-to-end on real hardware** (Serial, `COM5`):
    every check passed on `VOLUME=INTERNAL`, a real SD card, `VOLUME=PSRAM`, and the cross-volume
    `INTERNAL`→`PSRAM` copy, first attempt, no bugs found.

89. **Event-triggered macros (§18.5) are done**, requested directly ("Go on with the macros") as a
    follow-up once `FILE_COPY`/`FILE_RENAME` (design note 88) had made staging a macro into
    `VOLUME=PSRAM` practical. The user asked directly how macro playback currently queues a second
    trigger arriving before the first finishes, which led to reading `MacroPlayer`/`handlePlayMacro`
    together before any design decisions were made: strictly single-slot, live `PLAY_MACRO`
    `NACK(BUSY)`s a concurrent request, never queues.

    The design itself went through several rounds of direct correction, each changing real
    behavior:
    - **Trigger scope**: initially scoped to `PRESS`-only (buttons)/rising-edge-only (GPIO), then
      widened directly ("go for all events, the macros just won't exist") to every `BUTTON_EVENT`
      type and both GPIO edges — safe to widen freely since the whole mechanism is opt-in per
      filename's existence.
    - **`SHORT_PRESS`** (`0x03`, a real new `BUTTON_EVENT.EVENT_TYPE` value, §11), the user's own
      addition mid-design — fires right after `RELEASE` whenever a press-release cycle never
      crossed the `LONG_PRESS` threshold, giving buttons a dedicated "quick tap" macro distinct
      from a plain `RELEASE`.
    - **Busy behavior — revised from "drop" to "queue via append"**, the user's own proposal
      ("Could we just add the new macro content to the entries_ to queue them?"): `MacroPlayer`
      gained `enqueue()`, which starts playback if idle exactly like `start()`, or appends the new
      entries onto the tail of `entries_` if already playing — reusing the existing entry list as
      the queue rather than building a separate structure. Deliberately scoped to only this
      internal auto-play path; live `PLAY_MACRO` keeps its existing `NACK(BUSY)` contract unchanged,
      since a live PC caller depends on a definite answer in a way this fire-and-forget path
      doesn't.
    - **The `/on_any_event.macro` fallback + `/trigger_id.txt` "last trigger" variable**, entirely
      the user's own proposal: check for a universal fallback macro before resolving the specific
      one, and if it exists, record which specific macro *would* have applied into a fixed file —
      reusing `DRAW_TEXT`'s already-existing `FLAGS.TEXT_IS_PATH` (§12.6) as the read side, so a
      single `on_any_event.macro` can show/log which event fired without a dedicated macro per
      button.
    - A direct question ("Won't there be a problem with file name lengths?") was answered rather
      than assumed away: the filenames involved are hardcoded to `VOLUME=PSRAM`, a `std::map`-backed
      volume with no filesystem-imposed name-length cap, and even the worst case here
      (`/on_button_255_shortpress.macro`) is ~31 bytes — a non-issue.

    Implementation: `StorageManager::exists()` (new, a lightweight presence check reused to decide
    which macro to play without a wasted `download()`), `MacroPlayer::enqueue()` (above),
    `ButtonController`'s release branch now fires `SHORT_PRESS` alongside `RELEASE`, and
    `main.cpp`'s `pushButtonEvent`/`pushGpioEvent` each call a new `tryAutoPlayEventMacro()` after
    broadcasting their frame — reusing `handlePlayMacro`'s own download+parse sequence rather than
    duplicating it. No new command IDs; `ButtonEventType.SHORT_PRESS` was mirrored into
    `pc-java-lib` (picked up automatically by the schema's existing `u8Enum` reflection, no
    `CommandSchema`/`CommandId` change needed since `EVENT_TYPE` was already a generic enum field).

    Verified: `firmware` builds clean for `esp32-s3-crowpanel`; `pc-java-lib`'s `mvn test` passes.
    A new `EventTriggeredMacroManualCheck` (real hardware, Serial `COM5`, simulating `BOOT`
    press/release via `SerialFrameTransport.pressBoot()`/`releaseBoot()` rather than physical touch)
    asserts programmatically throughout (`LOG_MESSAGE` echoes and downloading `/trigger_id.txt`,
    like `StorageManualCheck`'s own style) that: a quick tap auto-plays `SHORT_PRESS`'s dedicated
    macro; a second trigger arriving mid-playback queues rather than dropping (two distinct
    `LOG_MESSAGE` markers both arrive, in order); and `/on_any_event.macro` correctly falls back for
    both `PRESS` and (isolated via a long hold, so no `SHORT_PRESS` follows) `RELEASE`, with
    `/trigger_id.txt` reading the correct specific path each time.

90. **`DRAW_TEXT` `FONT_ID 0x01`, a larger embedded font (§12.6), is done** — asked directly ("Do we
    have space for a larger font to include in the firmware?"), answered by checking real numbers
    first (33.2% flash used of the 3.19 MB OTA partition, ~2.1 MB free; font glyph bitmaps live in
    flash, not RAM) before designing anything, then built once the user confirmed. Uses
    Adafruit_GFX's bundled `Fonts/FreeMono12pt7b.h` — chosen specifically because it's genuinely
    monospace (confirmed from its own glyph table: all 95 glyphs share `xAdvance=14`), letting it
    slot into `EmbeddedFont.h`'s existing fixed-advance layout code (`splitTextLines()`/`drawText()`,
    now parametrized per font rather than hardcoded to FONT_ID 0x00's constants) without a real
    proportional-font layout rewrite — a `FreeSans` family also ships with Adafruit_GFX but is
    proportionally spaced, which would have needed one. A real, load-bearing finding from reading
    Adafruit_GFX's own source before writing any code: its `drawChar()` for a custom `GFXfont` has
    **no background-fill option at all** ("NO 'BACKGROUND' COLOR OPTION ON CUSTOM FONTS. THIS IS ON
    PURPOSE AND BY DESIGN", its own comment — proportional glyphs can overlap, so a uniform fill
    isn't generally safe) — meaning `BACKGROUND=OPAQUE` (already an existing, tested FONT_ID 0x00
    feature) could not simply come "for free" from calling `drawChar()` for the new font too. Fixed
    by not calling `drawChar()` at all for FONT_ID 0x01 — `EmbeddedFont.h`'s new `largeFont`
    namespace walks `FreeMono12pt7b`'s bitmap/glyph tables directly (safe direct struct/array access,
    no `pgm_read_byte`/`word` needed, since neither of this project's real targets — ESP32-S3/Xtensa
    or the native host build — is AVR, confirmed by reading how `Adafruit_GFX.cpp`'s own
    `pgm_read_glyph_ptr()` takes the same direct-access path on non-AVR), and fills the whole
    fixed-width character cell with the background color first when opaque — safe specifically
    because this font is used at a fixed advance width here, unlike Adafruit_GFX's general-purpose
    proportional-font caution. Reuses the exact same accented-glyph codepoint mapping
    (`textGlyph::resolveGlyph()`/`diacriticRows()`) as FONT_ID 0x00 for Czech coverage — codepoint
    mapping is font-agnostic — just drawn as 2×2px blocks instead of 1×1px to stay proportionate at
    the larger size, centered within the wider advance width. No CP437 box-drawing coverage (the
    bundled `GFXfont`s don't include those bitmaps) — those codepoints fall back to `?` under
    FONT_ID 0x01, the same honest unmapped-codepoint behavior already used elsewhere. Cost: ~2.9 KB
    flash (1109593→1112489 bytes, confirmed via a real rebuild), matching the font header's own
    "Approx. 2132 bytes" comment plus the small amount of new dispatch code — negligible against the
    ~2.1 MB headroom that motivated the question in the first place. Verified: `firmware` builds
    clean for `esp32-s3-crowpanel`, `RAM`/`Flash` usage confirmed via the real build output;
    `java-bshmidriver`'s `DrawTextManualCheck` was extended with two FONT_ID=0x01 lines (Czech
    accented glyphs, opaque background) and run against real hardware over Serial (`COM5`) — both
    `ACK`ed and the following `REFRESH` succeeded. Visual correctness (glyph shape, diacritic
    proportions, opaque-fill coverage) still needs eyes on the physical panel, not yet confirmed.
    **Superseded by design note 91** — the diacritic-overlay approach and the `FreeMono12pt7b` font
    itself were both later replaced.

91. **`FONT_ID 0x01`'s diacritic overlay was redesigned, then replaced entirely with a font that
    doesn't need one.** First round: the physical panel showed real accented text, and it was
    reported directly ("The diacritics is ugly") — the 2×2px-scaled reuse of FONT_ID 0x00's tiny
    5×2 dot patterns (design note 90) looked like disconnected blobs at this larger size, not real
    accent marks. Fixed by hand-drawing three purpose-sized 8×6 bitmaps instead (a diagonal stroke
    for acute, a downward chevron for caron/háček, a small ring outline for ring/kroužek) —
    verified as a real visual improvement on real hardware.

    A further, more fundamental request followed: real precomposed accented glyphs instead of any
    diacritic-overlay composition at all ("I'd prefer getting a proper font including the accented
    characters... avoid the manual workaround. If not possible, keep as is"). Investigated before
    committing to anything: no bundled Adafruit_GFX `GFXfont` (`FreeMono`/`FreeSans`/etc.) has Latin
    Extended-A coverage — checked directly, every one is 0x20-0x7E only — so achieving this meant a
    different font-rendering library entirely. Found `U8g2_for_Adafruit_GFX`, a real, established
    bridge that renders u8g2's own font data onto any Adafruit_GFX-derived object; confirmed by
    reading its source (`U8g2_for_Adafruit_GFX.cpp`) that it draws via `Adafruit_GFX::
    drawFastHLine()`/`drawFastVLine()`, both of which — like every other Adafruit_GFX primitive this
    project already relies on — fall through to `WorkingBufferGfx`'s own overridden `drawPixel()`
    with no hardware-accelerated override in between, so `DRAW_MODE` compositing keeps working.
    u8g2's own bundled `u8g2_font_unifont_t_extended` (GNU Unifont, sliced to codepoints
    U+0020–U+02BD) genuinely includes every Czech Latin Extended-A letter as a real precomposed
    glyph — no FreeType, no custom font-building, shipped as-is.

    This was only achievable for the *larger* font — no small (~6-10px) u8g2 font has confirmed
    Latin Extended-A coverage (only Cyrillic-extended variants exist at that size); building one
    would mean running u8g2's own font-build tooling locally against a source font whose Czech
    glyph coverage isn't even confirmed to exist. Presented as a real, asked-then-answered choice —
    the user chose to swap the larger font and leave FONT_ID 0x00 with its existing diacritic
    overlay (both fonts covered by Latin Extended-A wasn't achievable off-the-shelf).
    `U8g2_for_Adafruit_GFX@1.8.0` added as a `platformio.ini` dependency. Its own background-fill
    feature (`setFontMode(0)`) was checked and deliberately *not* used — reading its decoder
    (`u8g2_font_decode_glyph()`) confirmed a background fill there only covers each glyph's own
    tight ink bounding box, not the full fixed advance cell, which would leave visible gaps around
    narrow glyphs (e.g. '.', 'i') — the same reason `FreeMono12pt7b`'s own opaque fill had to be
    hand-rolled in design note 90; `EmbeddedFont.h`'s `largeFont::drawCodepointLine()` still fills
    the full cell by hand before drawing ink in transparent mode. Ascent/descent are queried from
    the font itself (`getFontAscent()`/`getFontDescent()`) rather than hand-computed, unlike
    `FreeMono12pt7b`'s constants before it. An unmapped codepoint (still includes box-drawing —
    this font's sliced range doesn't reach the Box Drawings/Block Elements blocks) is checked via
    `u8g2_IsGlyph()` and falls back to `?`, matching FONT_ID 0x00's own honest-fallback philosophy.
    Cost: ~11.8 KB flash (1112477→1124285 bytes, confirmed via a real rebuild) — still negligible
    against the multi-MB headroom that motivated the original question. Verified: `firmware` builds
    clean for `esp32-s3-crowpanel`; `DrawTextManualCheck` (updated to describe the new font) ACKed
    all eight `DRAW_TEXT` lines and the `REFRESH` on real hardware over Serial (`COM5`) after both
    the diacritic-bitmap fix and the full font swap.

92. **`FONT_ID 0x01`'s accented capitals were clipped at the top — a real bug in design note 91's
    own ascent calculation, found and fixed on real hardware.** Reported directly after the font
    swap: "there is no space for accents above the capitals," with a concrete repro requested
    ("Try the PŘÍLIŠ ŽLUŤOUČKÝ KŮŇ ÚPLĚL in capitals, also with the small font"). Root cause:
    `largeFont::metrics()` used `getFontAscent()` (`u8g2_font_info_t::ascent_A`), which — as its
    name and offset comment in `U8g2_for_Adafruit_GFX.h` both say — is anchored specifically to a
    plain, unaccented capital `A`, not the tallest glyph the font actually contains. Every Czech
    accented capital (Č Ž Ř Š Ď Ě Ň Ý Á É Í Ó Ú Ů) is taller than that, since its diacritic sits
    above a plain cap's own height — so the baseline (computed as `y + ascent_A`) was placed too
    low, and the diacritic ink above it landed above `y`, into whatever was drawn immediately
    before this line rather than this line's own reserved space. Fixed by using
    `max_char_height + y_offset` instead — `u8g2_font_info_t`'s own documented formula (see its
    struct comment) for the true font-wide worst-case ascent across every glyph, not just `A` —
    read directly off the public `u8g2_font_t` member since `U8G2_FOR_ADAFRUIT_GFX` has no wrapper
    getter for it. `FONT_ID 0x00` was checked too (its fixed-height glyph cell doesn't vary by
    case, so this specific failure mode doesn't apply there, but worth confirming directly rather
    than assuming) — `DrawTextManualCheck` extended with an ALL-CAPS Czech pangram for both fonts
    (placed in an already-unused gap for FONT_ID 0x00; FONT_ID 0x01's existing lowercase
    accented-glyph line was replaced with it, since that case was already confirmed working one
    round earlier). Verified end-to-end on real hardware over Serial (`COM5`) after the fix — user
    confirmed both all-caps lines render with their diacritics fully visible, no clipping.

93. **Custom, folder-driven proportional font — `DRAW_TEXT FONT_ID=0xFF` + new command
    `SET_CUSTOM_FONT_FOLDER` (§12.6.1/§12.6.2).** Requested directly: a third font whose glyphs
    live as individual files on SD/flash (so a glyph set can be authored/replaced without
    reflashing firmware), proportional-width (unlike `FONT_ID` 0x00/0x01, both fixed-advance),
    bottom-aligned against a shared line-height reference, with zero inter-character spacing so
    adjacent glyphs can be "welded" together, and a session-only "set the font folder" command
    supporting a one-time PSRAM-variable indirection for macro parametrization — explicitly
    *not* re-read on every draw the way `DRAW_TEXT`'s own `FLAGS.TEXT_IS_PATH` is, a deliberate
    semantic difference from that existing mechanism.

    Implementation: a new header-only module, `lib/Display/CustomFont.h`, owns the active
    folder/volume, an in-RAM (PSRAM-backed) glyph cache keyed by codepage byte, the new `.gly`
    file format's parser (§12.6.1 — packed 1bpp, no RLE/mask plane, simpler than `.epi` since
    glyphs are tiny and white is unconditionally transparent), and the actual bottom-aligned,
    zero-spacing glyph drawing (`drawByteLine()`). `EmbeddedFont.h`'s `splitTextLines()`/
    `drawText()` — previously hard-coded around one fixed `advanceWidth` per font, since both
    existing embedded fonts are monospace — were generalized to take a per-codepoint width-lookup
    callable instead; the two existing fonts plug in a constant-returning lambda (mathematically
    identical to the old fixed-multiply math, confirmed by inspection, so no regression risk),
    while `FONT_ID 0xFF` plugs in a real per-glyph lookup backed by the new cache. `TEXT` for this
    font is deliberately a raw single-byte codepage, not UTF-8 — only 256 glyph slots exist, and
    reusing UTF-8 would make bytes ≥0x80 unrepresentable as single glyph indices; `drawText()`
    detects `fontId==0xFF` and skips `decodeUtf8String()` entirely, widening each raw byte
    directly, which works unmodified through the rest of the line-splitting/alignment pipeline
    since it only ever compares codepoints against `'\n'`/`' '`, identical values either way.
    `handleDrawText()` (`main.cpp`) gained a pre-flight `customFont::ensureReady()` check — NACKs
    `FILE_NOT_FOUND` if no folder was ever configured this session or the folder's mandatory
    `XX.gly` fallback/line-height-reference glyph itself can't be loaded, since there is no valid
    line height to lay text out with in either case; any other individual glyph file being
    missing/corrupt is not an error, it silently substitutes `XX.gly` at draw time (and caches
    that substitution, so a missing glyph is never re-attempted from disk on every subsequent
    draw). The new `handleSetCustomFontFolder()` handler implements §12.6.2's `R:`/`S:`/`F:`
    resolution (one deliberate deviation from `DRAW_TEXT`'s own convention: the prefix is
    mandatory here, since `VOLUME=PSRAM` — flat, no subdirectories — can never be a valid folder
    target, so there's no sensible default to fall back to), a `StorageManager::list()` courtesy
    check that the resolved folder actually exists (fails fast at `SET` time rather than at the
    next draw), and hands the resolved volume/path to `customFont::setFolder()`, which also
    discards the entire glyph cache — every cached glyph belonged to whichever folder was active
    before. Registered `authLevel::kUsage`, opcode `0x030F` (next free ID in the `0x03xx` drawing
    block), with no `FLAGS`/`PERSIST` byte at all, matching `SET_DRAW_OFFSET`/`SET_CLIP_REGION`/
    `SET_ORIENTATION`'s existing "pure in-RAM session state" pattern rather than the `0x04xx`
    config commands' persist-to-NVS option.

    Verified: `firmware` builds clean for `esp32-s3-crowpanel` (RAM 21.4%/Flash 33.8% used, no
    regression in either budget). Real-hardware verification (a hand-authored small `.gly` glyph
    set uploaded via `FILE_UPLOAD`, `SET_CUSTOM_FONT_FOLDER` with both a direct `S:`/`F:` path and
    a `R:`-indirected one, visual confirmation of proportional widths/bottom-alignment/wrap/
    zero-spacing welding on the real panel, and the NACK edge cases) is still pending a physical
    CrowPanel run — see `java-bshmidriver`'s new `CustomFontManualCheck` (§22).

94. **`FILE_UPLOAD` (§14.3) now auto-creates missing parent directories.** Found while designing a
    client-side recursive folder-sync feature for `java-bshmidriver` (pushing a local asset tree —
    fonts, graphics, macros — onto device storage): `StorageManager::upload()` opened files via the
    2-argument `fs::FS::open(path, FILE_WRITE)` overload, whose `create` parameter (whether to
    auto-create missing parent directories) defaults to `false` in the Arduino-ESP32 core's own
    `VFSImpl::open()` (confirmed by reading `vfs_api.cpp` directly, both `SD` and `INTERNAL` share
    this same VFS-mounted `open()` implementation) — so uploading to a not-yet-existing
    subdirectory silently produced a "successfully opened" `File` handle whose subsequent write
    would fail, rather than the directory actually getting created. Fixed by passing the 3-argument
    overload's `create=true` explicitly. `PSRAM` is unaffected (its own upload path never touches
    `fs::FS` at all — flat, no subdirectories, §14). No wire format change — this is a pure
    server-side behavior fix, existing `FILE_UPLOAD` clients targeting a path whose parent already
    exists see no difference. Not yet verified on real hardware (queued alongside the custom-font
    feature's own pending hardware run, design note 93).

95. **`StorageManager::mountSd()` remounts lazily instead of on every single `VOLUME=SD`
    operation.** Found while exercising `java-bshmidriver`'s new recursive folder-sync feature
    against real hardware: pushing ~460 small generated font glyph files drove a burst of
    individual `FILE_UPLOAD`/`FILE_LIST_REQUEST` commands, each previously paying a full
    `SD.end()`+`SD.begin()` remount (the original "hot-pluggable, remount every operation" design,
    §14/class doc) - slow enough in aggregate to blow past a client's 5s+1-retry timeout on a
    second, otherwise-trivial re-sync pass. Asked directly: "Why do we need to repeatedly remount?
    There should be some lazy delay unmounting mechanism to speed this up." Fixed by adding a
    `sdMounted_` flag: `mountSd()` now only does the real remount once per "session" - first access
    after boot, or after the card was last powered down - and returns immediately otherwise.
    `powerDownSd()` (called by `SET_POWER_MODE` `HARD_SLEEP`/`LOW_POWER` and the existing 30s
    idle-power-down timeout, §17) clears `sdMounted_` alongside its existing `sdPowered_`, so every
    one of those remains an unconditional real remount point - confirmed directly with the user
    ("Will still need unmounting when going to powersave modes") that this was the intended
    boundary, not an oversight. The accepted tradeoff, stated explicitly rather than silently
    changed: a card hot-swapped while the device stays continuously active (no power-cycle point in
    between) now goes undetected until the next one, instead of on the very next command - no
    wire-protocol change, purely an internal latency fix. Verified: firmware builds clean; real
    hardware re-test of the same sync workload that originally timed out (see design note 96)
    confirms the fix.

96. **Custom font (`DRAW_TEXT FONT_ID=0xFF`) + `SET_CUSTOM_FONT_FOLDER` (design note 93) and the
    `FILE_UPLOAD` auto-mkdir fix (design note 94) verified end-to-end on real hardware**, together
    with `java-bshmidriver`'s new recursive folder-sync feature and TTF-to-`.gly` glyph generator
    (both new, `cz.bliksoft.hmieink.protocol.sync`/`cz.bliksoft.hmieink.protocol.font`) - the
    hardware verification both of those design notes had left pending, plus BSToolbox added as a
    `provided` dependency (its own `IconSpecEngine` turned out to be a UI icon-compositing/theming
    DSL, not a font rasterizer, so the glyph generator rasterizes TTF glyphs itself via plain
    `java.awt.Font`/`Graphics2D`/`FontMetrics` rather than through it). Sequence: flashed this build
    to a real CrowPanel over Serial; ran `CustomFontManualCheck` (after fixing two real bugs in the
    *test* itself - ambiguous same-shaped `send`/`expectNack` helper overloads that silently
    defaulted to the wrong `COMMAND_ID`, not firmware bugs) - all NACK/ACK assertions passed;
    generated three real glyph sets (CP1250 16px proportional and fixed-width from
    LiberationSans-Regular.ttf, a 48pt numeric-and-punctuation set from LiberationMono-Regular.ttf,
    both vendored in `BSToolbox-resources`) via the new `GlyphGenerator`; `SYNC`'d them onto
    `VOLUME=SD:/fonts` in `PC_MASTER` mode - which surfaced and fixed two real Java-side bugs, not
    firmware ones: (a) `RemoteFileStore.list()` propagated a not-yet-existing device folder's
    `NACK(FILE_NOT_FOUND)` as a hard error instead of treating it as "empty," breaking every
    first-ever sync into a new folder; (b) `HmiDevice`'s fixed 5s default command timeout was
    routinely too short for a real, ~220-entry `FILE_LIST_REQUEST`/bulk `FILE_UPLOAD` burst against
    real SD hardware (every `VOLUME=SD` operation walks the FAT directory over SPI, §14) - fixed by
    adding timeout-overriding overloads to `HmiDevice`'s file methods and having
    `HmiDeviceRemoteFileStore` use a generous (30s) one, unrelated to and additional on top of
    design note 95's lazy-remount fix (that one only removes the *mount* overhead per operation, not
    the underlying per-operation SD directory-walk cost itself). Then drew real text with
    `FONT_ID=0xFF` against the synced fonts and read the panel back via `READ_SCREEN` to a PNG for
    visual confirmation (not just programmatic assertions) - proportional widths, bottom-alignment,
    zero-spacing "welding," word-wrap, and full Czech CP1250 diacritics ("Příliš žluťoučký kůň úpěl
    ďábelské ódy") all rendered correctly, and the large numeric font rendered cleanly at 48pt. The
    first fixed-width-variant render exposed a real *generator* parameter issue, not a rendering
    bug - asked directly ("Is the fixed font interleaved with spaces, or the glyphs are too wide?")
    after it visually looked like extra inter-character gaps: the fixed cell width had been sized to
    the single widest glyph across the *entire* 218-character CP1250 set (`‰` PER MILLE SIGN, 17px)
    while the median glyph is only 9px wide, so ordinary letters ended up centered in a cell nearly
    twice their natural width; refixed to size off the ASCII printable range only (widest there:
    `@`, 16px) - a real, if only partial, improvement, since monospacing an inherently proportional
    typeface always leaves *some* visible padding around its narrowest letters (`i`/`l`) relative to
    its widest (`M`/`W`) - that's an inherent property of the source font, not a defect to chase
    further. `java-bshmidriver`'s `FolderSyncTest` (14 cases, in-memory fake) and
    `GlyphGeneratorTest`/`CodepagesTest` (11 cases) all still pass. See design note 95 for the one
    genuine firmware-side fix (SD mount latency) this same hardware session's own sync workload
    surfaced.

97. **`StorageManager::mountSd()`'s lazy remount (design note 95) is now debounced (5s) instead of
    fully sticky-per-session.** Design note 95's fix skipped the real `SD.end()`+`SD.begin()`
    remount entirely once mounted, until the next power-down - fast, but left an unbounded staleness
    window: a card hot-swapped while the device stayed continuously active (never idle-timed-out or
    slept) would never be re-detected. Flagged directly: "The SD remounting should be still done,
    just with some timeout to prevent too frequent remounts in sync or font drawing (e.g. don't
    remount if it was remounted/accessed in last 5 seconds or similar)." `mountSd()` now compares
    `millis()` against `lastAccessMs_` (already tracked, previously only for the idle-power-down
    timer) *before* updating it, and only skips the remount if under `kRemountDebounceMs` (5000)
    has elapsed since the last access - otherwise it remounts for real, same as before design note
    95 existed. `powerDownSd()`'s own unconditional remount-on-next-access (via `sdMounted_ = false`)
    is unchanged and still takes priority regardless of the debounce window. Verified: firmware
    builds clean; real-hardware re-sync of the (by then unchanged) font set from design note 96
    completed in the same ~49s as before the debounce fix (expected - individual `FILE_UPLOAD`s in
    a continuous burst are well under 5s apart, so the debounce window was never the bottleneck for
    that workload either way; its purpose is bounding hot-swap staleness, not this case's own
    throughput).

98. **FONT_ID 0x00's diacritic-overlay accent coverage extended from Czech-only to most of
    Latin-script Europe.** Requested directly: "extend the accents workaround for the builtin
    fonts to whole europe unicode characters (latin), not limiting to Czech set." Before this,
    `textGlyph::kAccentedGlyphs` (`EmbeddedFont.h`) only mapped the ~30 codepoints Czech text
    needs (acute/caron/ring); extended to essentially all of Latin-1 Supplement plus Latin
    Extended-A's own remaining acute/caron letters (Polish/Slovak) - four new diacritic shapes
    added to the existing 2-row/5-column composition mechanism (§12.6): grave, circumflex,
    diaeresis, tilde. Two of the four were deliberately designed as the vertical mirror of an
    existing shape, matching their real typographic relationship rather than being arbitrary: grave
    mirrors acute (falling vs. rising diagonal tick), circumflex mirrors caron (a `^` peak vs. a `ˇ`
    valley, literally the same two dots swapped between the top/bottom row) - keeping each pair
    readable as *related-but-different* rather than adding unrelated noise to an already-tight
    10-bit space. Diaeresis is two dots confined to the top row only (distinct from ring, which fills
    both rows); tilde is an approximated 3-dot spread across the full width, the closest a
    continuous wavy line gets to being legible as a discrete 2×5 bitmap - confirmed on real hardware
    to read unambiguously as "some diacritic here," though less immediately as "specifically a
    tilde," an accepted tradeoff at this resolution.

    Deliberately out of scope, explained in both this file (§12.6) and the firmware's own header
    comment rather than silently absent: diacritics that sit *below* the baseline (cedilla, ogonek)
    - the existing cell layout has no room below the glyph body for them; diacritic shapes not yet
    designed (breve, macron, dot-above, double-acute) - each serves only one or two languages
    (Romanian/Latvian/Polish/Hungarian respectively) versus every shape actually implemented
    covering several apiece, and each additional shape increases the risk of the whole set
    becoming harder to tell apart, not just adding one more; and true ligature/stroke-through
    letters (`Æ` `Ø` `Ł` `Đ` `Þ` `Ð` `ß`), which aren't a base-letter-plus-overlay composition at
    all and would need dedicated hand-drawn glyphs. All three remain honest `?` fallbacks, same as
    any other unmapped codepoint - not a regression, the same behavior these codepoints already had.
    `FONT_ID 0x01` (real precomposed Unifont glyphs, not composition) already covered every one of
    these codepoints, cedilla/ogonek/ligatures included, and needed no changes.

    Verified on real hardware: flashed, then `DRAW_TEXT FONT_ID=0x00` with sample text in French,
    German, Spanish, Portuguese, Slovak, and Polish, read back via `READ_SCREEN` to a PNG for visual
    confirmation. Every new diacritic shape renders distinguishable from every other (including the
    three pre-existing ones) at native resolution; sample sentences ("À bientôt, été!", "schön,
    über", "mañana está", "Camões é ótimo", "vŕba, ľúbim", "ćma, źle, śpi") all read correctly.

99. **Design note 76's "BLE undiscoverable" issue is resolved — it was never a hardware/RF/NimBLE-
    internal problem, just a missing advertised name.** Root cause, found by reading
    `setupBle()` (`src/main.cpp`) against the vendored NimBLE-Arduino 2.5.1 source
    (`NimBLEAdvertising.cpp`) side by side: the firmware called `addServiceUUID()` and `start()`
    but never `setName()`, and `NimBLEAdvertising` defaults `m_scanResp=false` — so scan response was
    never even sent (`start()` only sends it `if (m_scanResp && m_scanData has content)`), and no
    name was broadcast anywhere, in either the primary packet or a scan response. Devices advertising
    only flags + a bare 128-bit service UUID with zero name are exactly the ones many scanners/OS
    pairing UIs are least likely to surface prominently, which reads as "undiscoverable" without
    actually being an RF/hardware fault — consistent with note 76's own observation that
    `BLE_ADDRESS` came back correct (the controller was fine all along).
    Fix: `setupBle()` now calls `advertising->enableScanResponse(true)` then
    `advertising->setName(deviceName())` before `start()`. Deliberately *not* put in the primary
    packet instead: that packet is already tight (flags + the custom 128-bit service UUID = 21 of
    the legacy 31-byte budget), leaving only ~8 bytes for a name inline — not enough for
    `deviceName()`'s typical length (e.g. `"CrowPanel-3851DC"`) or a longer user-set
    `SET_DEVICE_NAME` override, so the scan response (its own full 31-byte budget) is the correct
    home for it, not a workaround.
    **Verified on real hardware**: flashed, then confirmed the device shows up with its name
    (`"CrowPanel-3851DC"`) in nRF Connect on Android and via `bluetoothctl scan on` on Linux/BlueZ.
    Also newly verified while re-testing this — not just handshake this time, but a full local
    drawing round trip: `pc-java-lib`'s `DrawPrimitivesBleManualCheck` (new manual check, mirroring
    the existing Serial-only `DrawPrimitivesManualCheck`) ran end-to-end over BLE — `DRAW_RECT`
    (REPLACE and XOR), `DRAW_CIRCLE` (outline and filled), `DRAW_LINE`, and a final `REFRESH` — all
    ACKed and confirmed rendered correctly on the panel. This is the first time any local drawing
    primitive (§12), not just the handshake, was exercised over BLE specifically.
    One dead end worth recording so it isn't re-walked: this same verification pass hit a *second*,
    unrelated problem that looked identical from the outside — a from-Windows scan (via
    `BSToolbox-BLE`'s Rust `ble-bridge` sidecar) found nothing, with or without a service-UUID
    filter. Bypassing every layer of this project's own code and querying Windows'
    `BluetoothLEAdvertisementWatcher` WinRT API directly (PowerShell, no Java/Rust involved)
    reproduced the same zero-results outcome for *every* BLE peripheral, not just this device
    (already-bonded phone/earbuds included) — proving the PC's own Bluetooth stack was wedged, not a
    regression in this project's BLE code, `pc-java-lib`, or `BSToolbox-BLE`. A Bluetooth radio
    toggle didn't clear it (a full reboot was the next step, not yet tried). The actual verification
    above was done instead from a second machine (Linux/aarch64, BlueZ 5.82) reachable over the LAN,
    which saw the device immediately via plain `bluetoothctl scan on` — a useful fallback to remember
    for future BLE-transport verification if the usual Windows dev machine's BLE stack is ever
    wedged again like this.

## 22. Implementation status

- **`pc-java-lib/`** — **BSHMIProtocol** (`cz.bliksoft.hmieink:bshmiprotocol`, package
  `cz.bliksoft.hmieink.protocol`, LGPL-2.1; renamed from an earlier CrowPanel-specific package to
  reflect that it's a generic client for this wire protocol, not tied to one device — see
  `CLAUDE.md`) (Maven, Java 8 target): `Crc16`, `RlePackBits`, `Frame` (§2 envelope
  encode/decode), and the `CommandId`/`Status`/`WriteFlags`/`ChunkHeader`/`PowerMode`/`WakeReason`/
  `Ble`/`DrawMode`/`Color`/`ShiftDirection`/`TextAlign`/`TextBackground` constant classes (mirroring firmware's
  `Protocol.h`/`EmbeddedFont.h`, covering every command family designed so far) are implemented and
  unit-tested. **Transport layer (§3) is now implemented for all three
  transports**: `FrameStreamReader` (stateful, resyncing byte-stream frame parser shared by TCP and
  Serial), `AbstractStreamFrameTransport` (background reader thread + listener dispatch +
  serialized sends), `TcpFrameTransport`, `SerialFrameTransport` (built on `jSerialComm`), and
  `BleFrameTransport` + `BleOutputStream` (built on the sibling `BSToolbox-BLE`, fragmenting writes
  to `MAX_CHUNK_SIZE` and feeding notifications through the same `FrameStreamReader` via a
  `PipedInputStream` bridge — no BLE-specific reassembly logic needed, matching §3.1's design).
  **The real stop-and-wait command/response client layer (§10) is now implemented**: `Tlv`/
  `TlvCodec` (§5.1 TLV entries — decode + a `Builder` for encoding), `HandshakeCapabilities` (a
  typed view over a decoded `HANDSHAKE_RESPONSE`, §5.2, with a `getDpiX()`/`getDpiY()` helper
  derived from `PIXEL_PITCH_X/Y_UM` per design note 42), and `CommandClient` (SEQ assignment,
  send-and-block, one retry on timeout, ACK/NACK-vs-direct-response correlation per §10's own
  rules — not just "matching SEQ", since a device-initiated event frame like `BUTTON_EVENT` has its
  own independent SEQ counter that could coincidentally collide with a pending request's; an
  optional `CommandEventListener` receives anything that doesn't correlate — including `GPIO_EVENT`
  and now `BUTTON_EVENT`, both exercised against real hardware, see design notes 66-67). All
  thirty manual check tools (`SerialHandshakeManualCheck`, `TcpHandshakeManualCheck`,
  `BleHandshakeManualCheck`, `FullImageTransferManualCheck`, `PartialImageTransferManualCheck`,
  `DrawPrimitivesManualCheck`, `ShiftRegionManualCheck`, `DrawTextManualCheck`,
  `ClipRegionManualCheck`, `CopyRegionManualCheck`, `BoxDrawingManualCheck`, `DrawOffsetManualCheck`,
  `OrientationManualCheck`, `ScreenReadbackManualCheck`, `StorageManualCheck`,
  `DrawImageManualCheck`, `DrawImageRowManualCheck`, `MacroManualCheck`, `BootMacroManualCheck`,
  `FillImageManualCheck`, `ParametrizedMacroManualCheck`, `DrawTextFromFileManualCheck`,
  `GpioManualCheck`, `ButtonManualCheck`, `StatusLedManualCheck`, `BoardControlManualCheck`,
  `ConfigManualCheck`, `WifiConfigManualCheck`, `BleConfigManualCheck`, `PowerManagementManualCheck`)
  now use this instead of ad hoc `CountDownLatch`/hex-dump code. `WifiConfigManualCheck` caught a real gap in
  `DIRECT_RESPONSE_COMMAND_IDS` (`WIFI_STATUS_REQUEST` had no `WIFI_STATUS_RESPONSE` entry, so
  requests just timed out) — fixed, see design note 74; `BLE_STATUS_REQUEST` got the same fix
  proactively while adding it. **`BleHandshakeManualCheck` itself currently fails** — not from
  anything in `pc-java-lib`, see design note 76: the device isn't discoverable over BLE at all right
  now, a pre-existing hardware/RF-level issue unrelated to this session's work.
  `DrawMode`/`Color`/`ShiftDirection`/`TextAlign`/`TextBackground`/`Rotation`/`OrientationFlags`/
  `ReadScreenSource`/`ReadScreenMode`/`ClearArtifactsFlags`/`Volume`/`EntryType`/`ImageRowAlign`/
  `FillTileMode`/`DrawTextFlags`/`GpioMode`/`GpioConfigureFlags`/`GpioPatternFlags`/`ButtonId`/
  `ButtonEventType`/`ConfigFlags`/`WifiConfigFlags` constant classes mirror the corresponding
  firmware enums;
  `EpiImageCodec` encodes/decodes the `.epi` format and `MacroCodec` the `.macro` format, each
  to/from plain Java types (`BufferedImage`, a small `Entry`
  list) rather than requiring a live device round trip to construct or inspect one. 68 unit tests
  pass (`mvn test`), including a real loopback-socket round-trip for TCP, a corrupted-frame resync
  case for the shared reader, and `CommandClient`'s correlation/timeout/retry logic against a
  synchronous in-memory `FakeFrameTransport`. OTA client helpers are not yet implemented.

  **`SerialFrameTransport.DEFAULT_RESET_SETTLE_DELAY_MS` bug fixed**: it had gone stale at its
  original 3000ms once firmware's `displaySelfTest()` (SSD1683 bring-up) made boot take noticeably
  longer (~6.1s for that self-test alone, plus a variable-length WiFi connect on top) — `connect()`
  was regularly returning before firmware had actually reached "framed-protocol mode", so a request
  sent immediately after could silently go unanswered (not NACKed — the bytes just arrived too
  early and sat in the UART buffer while firmware was still busy in `setup()`), surfacing as
  `CommandClient` retrying once and then throwing `CommandTimeoutException` even though nothing was
  actually wrong with the request. Bumped the default to 12000ms (covers the common case with
  margin; a board slow to associate to WiFi, up to its own ~15s timeout, needs an explicit larger
  value via the 3-arg constructor). The real fix — firmware signaling "ready" some other way than a
  fixed delay, rather than a magic number that can go stale again — is still open, and documented as
  such directly in the class's own javadoc rather than silently re-tuned and forgotten.

  **`SerialFrameTransport` gained RST/BOOT control** (§3.3, design note 71): `pressReset()`/
  `releaseReset()` (RTS→EN) and `pressBoot()`/`releaseBoot()` (DTR→GPIO0) drive the same lines the
  board's physical buttons pull; `resetToRunMode()`/`resetToBootloader()` compose them into a plain
  reset and a forced ROM-bootloader entry, respectively. Fixing `resetToBootloader()` to actually
  work required matching `esptool.py`'s exact DTR/RTS transition order (not just reaching the same
  final levels) — a purely PC-side signal-timing fix, no firmware involved. **Verified on real
  hardware** (`BoardControlManualCheck`): simulated BOOT press/release produced real `BUTTON_EVENT`s
  indistinguishable from a physical press; forced bootloader entry left the device genuinely
  unresponsive to the framed protocol; `resetToRunMode()` recovered it back to normal operation.

  **`SerialFrameTransport` also gained `sendWakePreamble()`** (§17.1, design note 77): writes a few
  repeated `MAGIC` bytes directly to the port, sharing `send()`'s own write lock via a new protected
  `sendRawBytes()` on the shared `AbstractStreamFrameTransport` base (so it can't interleave with a
  concurrent frame send) — exactly the preamble §17.1's own text recommends before waking a
  `LOW_POWER` device over Serial.
- **`firmware/`** (PlatformIO, Arduino framework for the ESP32-S3 target): `Crc16`, `RlePackBits`,
  and `Frame` are implemented as a portable-C++17 library at `firmware/lib/Protocol/`, mirroring
  the Java code field-for-field, with a Unity test suite at `firmware/test/test_protocol/test_main.cpp`
  covering the same cases as the Java tests (including the shared CRC16 check vector).
  **`firmware/lib/Transport/StreamFrameTransport.h`** mirrors the Java side's stream-framing/resync
  logic, but as a non-blocking `poll()` (Arduino's single-threaded `loop()` model, not a reader
  thread) driven off any Arduino `Stream&` — the same code drives Serial, TCP, *and* BLE (via
  `NimBLEStreamServer`, §3.1/§20) unchanged, the payoff of building it generic over `Stream&` up
  front. `src/main.cpp` now brings up WiFi (STA mode; credentials from a gitignored
  `include/secrets.h`, see `secrets.h.example`), a `WiFiServer` on TCP port 5577, and a NimBLE
  peripheral (one service/characteristic, MAC-derived device name, §13.3), all three wired to a
  real command dispatch table (**`firmware/lib/Dispatcher/Dispatcher.h`**, §4): a
  `COMMAND_ID -> handler` map with a `CommandContext` (request frame, `ACTIVE_TRANSPORT`, and
  `ack()`/`nack(status)`/`reply(commandId, payload)` helpers so REF_SEQ/REF_COMMAND_ID are never
  hand-assembled), replacing the earlier single-`if` bring-up stub — every later command family
  just registers a handler instead of growing one giant function, and any commandId nobody has
  registered yet still correctly falls through to `NACK(UNSUPPORTED_COMMAND)`, enforced centrally
  by the dispatcher rather than by each handler remembering to do it. Two handlers are registered
  so far: `HANDSHAKE_REQUEST` → `HANDSHAKE_RESPONSE` (correctly reporting `ACTIVE_TRANSPORT` and
  `MAX_CHUNK_SIZE` per the transport it arrived on) and `FULL_IMAGE_TRANSFER` (§6, see below).
  **Bug fixed**: `ack()` originally sent an empty payload instead of this section's own §10-defined
  `REF_SEQ`/`REF_COMMAND_ID`/`STATUS=OK` shape — a real spec-vs-implementation mismatch, caught only
  once `pc-java-lib`'s `CommandClient` (§22's `pc-java-lib` bullet) started actually correlating ACK
  payloads instead of just waiting for "any frame back". `ack()` is now `nack()` with `STATUS=OK`,
  sharing a `refPayload()` helper — verified against real hardware post-fix. **Hardware definition
  is externalized**: display size/depth, product
  naming, and pin assignments live in `firmware/include/boards/board_crowpanel_4_2.h` (selected via
  `BoardConfig.h` + a `-DBOARD_*` flag in `platformio.ini`), not hardcoded in `main.cpp` — a
  different ESP32-S3 + e-paper/LCD board needs a new header (`boards/board_template.h` documents
  the required fields) and a new `platformio.ini` env, not a fork of the firmware. Display fields
  are real and hardware-verified (handshake TLVs unchanged byte-for-byte after this refactor); every
  pin constant (display SPI, SD card SPI, buttons, status LED, GPIO expansion header) is now a real
  value read off the board's own schematic — no `-1` placeholders left, though none have been
  confirmed by probing the physical board itself yet. The handshake now also
  reports `PIXEL_PITCH_X/Y_UM` (§5.2) — verified on real hardware as `212`/`212` (matching the
  spec sheet's 0.212 mm pixel pitch, ≈120 DPI once a client computes it). **The SSD1683 e-paper
  driver is brought up**: `GxEPD2`'s `GxEPD2_420_GDEY042T81` panel class (400×300, SSD1683) is an
  exact match for this board's display — confirmed independently by GxEPD2's own example file
  having a `// CrowPanel wiring` line for these exact pin values. `main.cpp`'s `displaySelfTest()`
  drives the display power-enable pin (GPIO7, active HIGH), initializes the driver (no custom SPI
  pin remapping needed — the display's SCK/MOSI, 12/11, are this chip's *default* hardware SPI
  pins), and fills the panel full-black then full-white via `setFullWindow()`/`fillScreen()`/
  `display(false)`, then `hibernate()`s the controller. Verified on real hardware both in the boot
  log (no BUSY-timeout or SPI hang, refresh timing in line with the datasheet's ~1200ms
  full-refresh spec) and visually (panel confirmed to actually redraw black then white) — one
  observed difference from the board's factory firmware: no rapid multi-flash cleaning cycle before
  the fill, just a single direct pass, since `useFastFullUpdate=true` uses the panel's OTP waveform
  LUT directly rather than a manual multi-cycle ghosting-reduction flash; `CLEAR_ARTIFACTS` (§9) is
  exactly where that belongs once it has a handler.

  **`FULL_IMAGE_TRANSFER` (§6), `PARTIAL_IMAGE_TRANSFER` (§7), and `REFRESH` (§12.8) are all wired
  end to end — Phase 1 (`plan.md`) is complete, `FULL_IMAGE_TRANSFER` having been the project's
  "hello world" milestone.** All three share **`firmware/lib/Display/WorkingBuffer.h`**: an
  explicit, persistent MCU-side copy of the working buffer (§2.1) with union-rectangle
  dirty-region tracking. This is a deliberate departure from `FULL_IMAGE_TRANSFER`'s original
  implementation (which used the SSD1683's own RAM directly, no MCU-side copy) — that worked fine
  for a command that always overwrites the whole panel, but `REFRESH`'s "union of bounding boxes
  deferred since the last refresh" inherently needs region-level state tracked across *multiple*
  deferred writes *before* any of them touch the controller, which the controller's own
  current/previous RAM banks aren't a natural fit for; `handleFullImageTransfer` was refactored
  onto `WorkingBuffer` too, for consistency. `WorkingBuffer::flush()` mirrors
  `GxEPD2_BW::display()`'s own write/refresh/resync sequence (full mode: both RAM banks via
  `writeImagePartAgain`, slow whole-panel LUT via `refresh(false)`, power off; fast mode: only
  "current" via `writeImagePart`, region-scoped diff LUT via `refresh(x,y,w,h)`, then
  `writeImagePartAgain` to resync "previous") using the `*Part` GxEPD2 call family uniformly for
  both whole-panel and sub-region writes (its indexing already accounts for the source bitmap
  being wider than the region written), with the same `invert=true` wire-vs-controller polarity
  translation as before. `handlePartialImageTransfer` validates `X`/`Y`/`WIDTH`/`HEIGHT` against
  `board::kPartialRefreshGranularityX/Y` and panel bounds, NACKing `BAD_PARAMETERS` on any
  violation per §7 (never silently rounded/clamped); both image-transfer handlers now share one
  `decodeImageData()` RAW/RLE helper. `handleRefresh`: `MODE=0x01` flushes the whole panel and
  clears dirty tracking; `MODE=0x00` flushes just the tracked dirty union, or is a harmless no-op
  ACK if nothing is pending. **Verified end-to-end on real hardware**: `pc-java-lib`'s
  `FullImageTransferManualCheck` (re-run as a regression check post-refactor) still sends a real
  400×300 horizontal-stripe test pattern correctly; the new `PartialImageTransferManualCheck` sends
  two black boxes to opposite corners as *deferred* writes (`FLAGS.REFRESH_NOW=0` — no visible
  change from either individually), then a single `REFRESH(MODE=0x00)` — confirmed both boxes
  appeared together only after that one call, proving the dirty-region union correctly spans
  multiple deferred writes.

  **Local drawing primitives are wired end to end — `plan.md` Phase 2's first item.**
  `DRAW_LINE`/`DRAW_RECT`/`DRAW_CIRCLE`/`CLEAR_REGION` (§12.2–§12.5) via a new
  **`firmware/lib/Display/WorkingBufferGfx.h`**, an `Adafruit_GFX` adapter over `WorkingBuffer` -
  only `drawPixel()` needed overriding (to apply §12.1's DRAW_MODE compositing via
  `WorkingBuffer::compositePixel()`, new bit-level pixel access alongside the existing byte-level
  `write()`), since every higher-level Adafruit_GFX primitive already funnels through it (confirmed
  by reading `Adafruit_GFX.cpp`) — reusing its well-tested Bresenham/midpoint-circle rasterizers
  rather than hand-rolling them (design note 45). `LINE_WIDTH>1` on a line hand-rolls Bresenham
  stepping + a square-brush stamp (no Adafruit_GFX equivalent); on a rect/circle outline it draws
  nested inward outlines instead. **Verified end-to-end on real hardware**: a new
  `DrawPrimitivesManualCheck` sends a filled rect, a second rect XORed on top (the overlap correctly
  turned white, proving real per-pixel compositing, not just REPLACE), an outline circle, a filled
  circle, and a thick diagonal line, all deferred, then one `REFRESH` — confirmed all five shapes
  rendered correctly. **A real bug was caught along the way**: all four new handlers were initially
  missing their final `ctx.ack()` call — every request was validated and drawn correctly, then
  silently got no reply at all. Properly diagnosed (not guessed): a raw-byte serial dump proved
  zero bytes came back for `DRAW_RECT`, but a `HANDSHAKE_REQUEST` sent moments later on the same
  still-open connection got a normal reply — ruling out both a boot-timing race (disproven by a
  control run with an artificially long 25s settle delay that still failed) and a firmware
  crash/hang (ruled out by the successful handshake) before finding the actual missing-`ack()`
  cause. Fixed, rebuilt, reflashed, reverified.

  **`SHIFT_REGION` (§12.9, `0x0307`) is wired end to end** - a new command, added for scrolling use
  cases (a ticker, a log panel, incrementally revealed text): shifts a region's content
  LEFT/RIGHT/UP/DOWN by a pixel step in place, discarding content shifted past the region's own
  edge and filling the vacated strip with a plain color (no `DRAW_MODE`, like `CLEAR_REGION` -
  always an overwrite). New `WorkingBuffer::shift()` picks its iteration direction (ascending when
  reading from a higher index, descending from a lower one) so the shift works in place with no
  temporary copy of the region. **Verified end-to-end on real hardware**:
  `ShiftRegionManualCheck` draws four black vertical stripes, refreshes to show them, then shifts
  the whole panel LEFT by 50px with a white fill and refreshes again — confirmed the leftmost
  stripe vanished off the edge, the remaining three moved exactly 50px left, and a blank strip
  appeared on the right.

  **`DRAW_TEXT` (§12.6, `0x0304`), `SET_CLIP_REGION` (§12.10, `0x0308`), and `COPY_REGION`
  (§12.11, `0x0309`) are all wired end to end.** `DRAW_TEXT`: new `firmware/lib/Display/
  EmbeddedFont.h` (design notes 46/48/49) reuses `Adafruit_GFX`'s own bundled classic ASCII font
  for base glyphs, composing the ~30 accented Latin codepoints Czech text needs as base letter +
  hand-authored 2-row diacritic mark; supports WIDTH=0 (unbounded, still `\n`-aware) or WIDTH>0
  (word-wrap + LEFT/CENTER/RIGHT alignment within that reference box; an embedded `\n` always
  breaks a line regardless of WRAP). Also exposes the classic font's own CP437 box-drawing/block
  glyphs (codes 176-223, already present in `glcdfont.c`, mapped from their real Unicode
  codepoints — no new bitmaps needed), for ASCII-art tables; getting these right required disabling
  `Adafruit_GFX`'s legacy `_cp437` off-by-one shift (`cp437(true)`, `setup()`) and fixing a real
  inconsistency where `WIDTH=0` didn't respect embedded `\n` and word-wrap unconditionally
  collapsed whitespace runs, which would have destroyed a table's fixed spacing (design note 49).
  `DRAW_TEXT` also gained a `BACKGROUND` byte (§12.6, design note 50) after initial delivery:
  OPAQUE fills every non-ink glyph pixel with the opposite of `COLOR`, routed through
  `Adafruit_GFX::drawChar()`'s own existing `bg`-parameter mechanism so DRAW_MODE keeps applying
  uniformly across ink and background alike; "inverted" text is simply `COLOR=WHITE` with an
  OPAQUE background, no separate concept needed. `SET_CLIP_REGION`: new `WorkingBuffer` state enforced once, in
  `getPixel`/`setPixel` (design note 47), so it automatically covers every §12 drawing primitive,
  `SHIFT_REGION`, and `COPY_REGION`'s destination side without each needing its own check;
  `FULL_IMAGE_TRANSFER`/`PARTIAL_IMAGE_TRANSFER` are deliberately unaffected (their own stricter
  NACK-on-violation contract, §6/§7, is untouched). `COPY_REGION`: reads its whole source into a
  temporary buffer first (correct regardless of source/destination overlap, like `memmove` vs
  `memcpy`), and its source read bypasses the clip region via a separate `getPixelUnclipped`
  (design note 47) — only the destination write is clip-constrained.

  **Verified end-to-end on real hardware**: `pc-java-lib`'s new `DrawTextManualCheck` drew a Czech
  pangram (`Příliš žluťoučký kůň úpěl ďábelské ódy` — exercising nearly every accented glyph the
  font supports), a word-wrapped English paragraph, and centered/right-aligned lines — all
  confirmed rendering correctly, diacritics included. `DrawTextManualCheck` also now draws an
  opaque-background line and an inverted (`COLOR=WHITE`+OPAQUE) line — the first real-hardware
  attempt showed a small unfilled 1px×2px gap at the top-left of every character (the diacritic
  band's background fill loop missed the inter-character spacer column `drawChar()` itself fills
  for its own 8-row body), fixed per design note 50 and reconfirmed clean on a second attempt.
  `ClipRegionManualCheck` drew a border showing
  a clip box, set the clip to it, drew text deliberately longer than the box (confirmed truncated
  at the box edge, never spilling outside), shifted the box's content up (scroll), drew a second
  overflowing line the same way, then reset the clip to the full panel and confirmed drawing
  outside the old box worked again — all confirmed correct on the *second* attempt: the first
  attempt's border rectangle exactly coincided with the shift/clip rectangle, so `SHIFT_REGION`'s
  own fill legitimately overwrote the border's edge pixels (a test-script bug, not a firmware one —
  fixed by drawing the border 2px outside the actual clip/shift box instead). `CopyRegionManualCheck`
  drew one filled circle and copied its bounding box to two other locations — confirmed three
  identical circles appeared without the circle being redrawn from the PC. `BoxDrawingManualCheck`
  sent a real 2×2 ASCII-art table (corners, tees, cross-junction, `WIDTH=0` so the cells' fixed
  internal spacing wasn't touched) as a single pre-formatted `DRAW_TEXT` with embedded `\n`s —
  confirmed the table rendered with correct borders and intact alignment.

  **`SET_DRAW_OFFSET` (§12.13, `0x030A`) and `SET_ORIENTATION` (§12.12, `0x030B`) are both wired end
  to end** (design note 51): a persistent `(dx,dy)` pan and a persistent rotation/mirror of the
  logical canvas, both enforced at the exact same single chokepoint `SET_CLIP_REGION` already used
  (`WorkingBuffer::getPixel`/`setPixel`/`compositePixel`, widened to signed logical coordinates),
  plus a matching batch-rect version (`computeAffectedPhysicalRegion()`) for `finishDraw()`'s own
  flush/dirty-region bookkeeping. `SET_CLIP_REGION`'s own X/Y/WIDTH/HEIGHT deliberately stay stored
  exactly as given — never pre-transformed — so the clip window automatically follows a later
  `SET_ORIENTATION` change while staying independent of `SET_DRAW_OFFSET`, an interaction confirmed
  with the user before implementing. **Verified end-to-end on real hardware**: `DrawOffsetManualCheck`
  drew the same `DRAW_RECT` three times at three different offsets (baseline, shifted 60px right,
  and shifted far enough left to straddle the panel's left edge) — confirmed only the surviving
  portion of the third rectangle rendered, flush against `x=0`, with the first two fully intact and
  correctly positioned; `OrientationManualCheck` drew the same small text label under each of the
  four `ROTATION` values at a fixed logical anchor — confirmed each landed near a different physical
  corner (top-left/top-right/bottom-right/bottom-left for 0°/90°/180°/270° respectively) with its
  own glyphs visibly rotated in place, not just moved, and separately confirmed `MIRROR_H`/`MIRROR_V`
  each flip both the position and the on-panel readability of a text label — all matching the
  predicted mapping exactly on the first real-hardware attempt, no fixes needed.

  **`READ_SCREEN`/`SCREEN_DATA` (§8) and `CLEAR_ARTIFACTS` (§9) are both wired end to end** (design
  notes 52/53): `WorkingBuffer` gained a second buffer, `panelBuffer_`, kept in sync with the actual
  physical panel content only inside `flush()` (bit-by-bit, since `flush()` has no alignment
  constraint) — `SOURCE=WORKING_BUFFER` reads the existing `buffer_` directly (already exactly this
  semantics, since every §12 write lands there immediately regardless of `FLAGS.REFRESH_NOW`),
  `SOURCE=PANEL` reads `panelBuffer_`. Both bypass the §12 offset/clip/orientation pipeline entirely
  — `X`/`Y`/`WIDTH`/`HEIGHT` here are physical panel coordinates, matching what `SCREEN_DATA` itself
  echoes back. `CLEAR_ARTIFACTS` reuses the same direct-display-driver black/white flashing technique
  as the boot-time `displaySelfTest()`; `FLAGS.RESTORE_CONTENT=0` additionally calls a new
  `markPanelBlank()` to keep `panelBuffer_` truthful (nothing else does, since this path
  deliberately bypasses `WorkingBuffer::flush()`), while `=1` just re-flushes the current working
  buffer, which keeps `panelBuffer_` in sync on its own. **Verified end-to-end on real hardware**:
  `ScreenReadbackManualCheck` draws one rect immediately and a second one deferred, then checks
  `SOURCE=PANEL` vs `SOURCE=WORKING_BUFFER` disagree exactly as expected (working buffer shows both,
  panel shows only the first, until an explicit `REFRESH` catches panel up) — then runs
  `CLEAR_ARTIFACTS` both without and with `RESTORE_CONTENT`, confirming via `READ_SCREEN` that the
  panel reads back exactly blank, then exactly restored, while the working buffer stays untouched
  throughout. Unlike every other manual check so far, this one asserts programmatically (decoded
  pixel samples compared against expected values in Java, not eyeballed on the panel) — all checks
  passed on the first real-hardware attempt. `CLEAR_ARTIFACTS`'s own visual flash was inconclusive
  to the human eye (see design note 53) — a single-cycle flash is hard to tell apart from the boot
  self-test's own near-identical flash immediately beforehand — but the programmatic `READ_SCREEN`
  evidence is strictly stronger anyway.

  **The storage manager (§14) is wired end to end for both volumes** (design note 55): new
  `firmware/lib/Storage/StorageManager.h`, writing every operation once against Arduino's common
  `fs::FS` interface rather than duplicating logic per filesystem — `VOLUME=SD` (`fs::SDFS`, its own
  independent SPI bus, remounted fresh on every operation since it's hot-pluggable and the ESP32 SD
  library has no card-detect callback) and `VOLUME=INTERNAL` (`fs::LittleFSFS`, mounted once at
  boot). `FILE_LIST_REQUEST`/`FILE_DOWNLOAD_REQUEST`/`FILE_UPLOAD`/`FILE_DELETE`/
  `STORAGE_INFO_REQUEST` are all registered handlers now; the handshake's `FEATURE_BITMASK` (§5.2)
  was also corrected while touching this code — it previously reported every bit as 0 regardless of
  what was actually already implemented (partial refresh, RLE, drawing primitives), not just the
  new storage bits. **Verified end-to-end on real hardware** (`StorageManualCheck`, asserting
  programmatically rather than visually, like `ScreenReadbackManualCheck`): upload/list/download/
  delete all confirmed correct, byte-for-byte, on both `VOLUME=INTERNAL` and a real 32 GB
  `VOLUME=SD` card. **A real bug was caught and fixed along the way** (design note 54): the first
  attempt against the actual SD card reported `FREE_BYTES` larger than `TOTAL_BYTES` — an
  impossible-looking result. Root cause was `TOTAL_BYTES`/`FREE_BYTES`'s u32 wire width (max
  ~4.29 GB) silently truncating each of `SD.totalBytes()`/`usedBytes()`'s correct 64-bit values
  independently, losing different high bits from each; fixed by saturating each at `UINT32_MAX`
  instead of truncating, confirmed correct (both fields read back as exactly `0xFFFFFFFF` on the
  32 GB card) on the next real-hardware attempt.

  **`FILE_COPY`/`FILE_RENAME` (§14.6/§14.7) are wired end to end** (design note 88), added as
  preparation for a still-undesigned future feature (event-triggered macros): `FILE_COPY` is
  cross-volume-capable (`download()` then `upload()`, reused from the two operations above rather
  than a new per-filesystem copy path), `FILE_RENAME` is same-volume only (a `std::map` key move on
  `VOLUME=PSRAM`, `fs::FS::rename()` on SD/INTERNAL), and both overwrite an existing destination,
  matching `FILE_UPLOAD`'s own semantics. `CommandSchemaRoundTripTest`/`CommandIdTest` pass and the
  firmware builds clean; `StorageManualCheck` was extended with copy/rename coverage (including a
  dedicated cross-volume case) and **verified end-to-end on real hardware** — every check passed
  across `VOLUME=INTERNAL`, a real SD card, `VOLUME=PSRAM`, and the cross-volume copy, first
  attempt, no bugs found.

  **`DRAW_IMAGE` (§12.7) and the `.epi` file format are both wired end to end** (design notes
  56/57): firmware reuses `decodeImageData()` (the exact §6 RAW/RLE scheme already shared by
  full/partial image transfer) for the `.epi` file's `COLOR_DATA`/`MASK_DATA` streams unchanged,
  going through the same `WorkingBufferGfx::drawPixel()` pipeline as every other §12 drawing
  primitive (DRAW_MODE/clip/offset/orientation all apply) rather than writing the buffer directly
  like full/partial image transfer do; masked (transparent) pixels are skipped entirely, never
  composited, matching the same convention already used for text. New `EpiImageCodec` (Java)
  converts to/from a plain `BufferedImage` — thresholding luminance for `COLOR_DATA` and, optionally,
  alpha for `MASK_DATA` — with any actual image *decoding* (PNG/JPEG/etc) left to `ImageIO`/
  `BufferedImage` upstream, not duplicated here. **Verified on real hardware**
  (`DrawImageManualCheck`): drew a striped background, encoded a small rounded-square "badge" icon
  (a filled black circle on a white rounded-rect, with fully transparent corners outside the
  rounding) with a mask, `FILE_UPLOAD`ed it to `VOLUME=INTERNAL`, then `DRAW_IMAGE`d it on top of the
  stripes — confirmed the badge's four corners showed the stripes showing through (mask genuinely
  working, not just "happens to look white already" — the whole reason the background is striped
  rather than plain white), while the rest of the badge fully replaced the stripes underneath it,
  correct on the first real-hardware attempt.

  **`DRAW_IMAGE_ROW` (§12.14, `0x030C`) is wired end to end** (design note 58), added after a
  request for a way to lay out several images/icons in a line — e.g. a strip of indicator icons, or
  a "big number" composed from per-digit glyph images. Draws `COUNT` `.epi` images (all from one
  `VOLUME`) left-to-right, each keeping its own natural width/height, never scaled; `ALIGN`
  (`LEFT`/`CENTER`/`RIGHT`/`BLOCK`) and `SPACING` work like `DRAW_TEXT`'s `WIDTH`/`ALIGN` layout
  concept, with `BLOCK` auto-distributing leftover space evenly across the gaps between images
  (ignoring `SPACING`) rather than needing it specified by hand. Required splitting `DRAW_IMAGE`'s
  decode-and-draw-immediately logic into two steps (`decodeEpiImage()`/`blitDecodedEpiImage()`),
  since alignment needs every image's own width known before any image's position can be computed —
  `DRAW_IMAGE` itself was refactored onto the same two functions, reconfirmed behaviorally unchanged
  via `DrawImageManualCheck` before the new command was added. **Verified on real hardware**
  (`DrawImageRowManualCheck`): the same three differently-sized icon tiles drawn inside four
  outlined reference boxes, one per `ALIGN` value — `LEFT`/`CENTER`/`RIGHT` each packed against the
  expected box edge, `BLOCK` split its leftover space evenly between the two gaps with the first/
  last tiles flush against the box edges — matching the predicted layout exactly on the first
  attempt.

  **`VOLUME=PSRAM` (§14) and the full macro system (§18) are both wired end to end** (design notes
  59-62), requested together — a "universal third temporary storage option" and record/save/play/
  pause commands to preprogram a sequence, plus an auto-playing boot macro. `StorageManager` gained
  a `std::map`-backed PSRAM volume (`ps_malloc`-allocated, a fixed 2 MiB budget) that every existing
  file command, and `DRAW_IMAGE`/`DRAW_IMAGE_ROW`, now accepts exactly like SD/INTERNAL with no
  special-casing. New `firmware/lib/Macro/` (`MacroRecorder`, `MacroPlayer`): recording is a thin
  wrapper each live transport's frame handler calls instead of `Dispatcher::dispatch()` directly;
  playback dispatches through that exact same `Dispatcher`, one entry per `loop()` iteration, via a
  `NullStream`-backed transport standing in for a real caller — neither `Dispatcher.h` nor
  `StreamFrameTransport.h` needed any changes. `PAUSE` always ACKs immediately and only actually
  delays anything when replayed from a macro, keeping the whole player non-blocking without needing
  its own handler to do anything unusual. **Verified end-to-end on real hardware**
  (`MacroManualCheck`): recorded two `DRAW_RECT`s with a `PAUSE` between them, downloaded the saved
  `.macro` file and confirmed (via `MacroCodec`) it captured exactly those three entries byte-for-
  byte, then played it back — confirming `PLAY_MACRO`'s ACK arrives in ~10ms (means "started", not
  "finished"), that a live `HANDSHAKE_REQUEST` sent during the macro's `PAUSE` window also gets a
  ~10ms response (genuinely non-blocking), and that `READ_SCREEN` shows both rectangles were
  actually drawn once playback finished. The first attempt at that responsiveness check itself
  hit a test-timing flaw, not a firmware bug (design note 62) — fixed and reconfirmed clean.
  `BootMacroManualCheck` uploaded a one-entry `/boot.macro` to `INTERNAL`, closed and reopened the
  connection (which resets the board), and confirmed the rectangle appeared with no `PLAY_MACRO`
  sent that run — proving the boot macro was found and played automatically — then deleted it so
  later manual checks' own initial connects don't keep replaying it.

  **`FILL_IMAGE` (§12.15) is wired end to end** (design note 63), requested for tiling a `.epi`
  image across a width/height/both — backgrounds and decorative borders. Reuses `DecodedEpiImage`/
  `decodeEpiImage()`/`blitDecodedEpiImage()` (design note 58) completely unchanged; the only new
  logic is the tiling loop itself and a temporary clip-region narrowing (intersected with whatever
  clip was already active, then restored) that makes every blitted tile's own existing per-pixel
  clipping crop a partial edge tile for free, with no new bounds-checking code needed. **Verified on
  real hardware** (`FillImageManualCheck`): tiled one small, deliberately asymmetric tile across
  three target rectangles — one per `TILE_MODE` — each sized to not be an exact multiple of the tile
  size, confirming every run's last tile was cleanly cropped at the target edge rather than
  wrapping, overflowing, or being skipped, correct on the first real-hardware attempt.

  **`DRAW_TEXT`'s `FLAGS.TEXT_IS_PATH` is wired end to end** (design note 64), requested for
  "parametrized macros" — a saved macro that draws different text on different plays without being
  re-recorded, by reading its text fresh from a file each time rather than embedding it directly.
  Reuses one of `DRAW_TEXT`'s own reserved `FLAGS` bits, so the wire payload's size is unchanged;
  volume selection (which started as a hardcoded `VOLUME=PSRAM`, matching the literal "RAM file"
  request) was extended, after asking whether other volumes should be supported, to an optional
  `R:`/`S:`/`F:` prefix on `TEXT` itself (PSRAM/SD/INTERNAL) rather than a `VOLUME` wire field —
  the user's own proposal, and a strictly better fit than the field-based alternative, since it adds
  no payload growth and still defaults cleanly to PSRAM when omitted. **Verified end-to-end on real
  hardware**: `ParametrizedMacroManualCheck` recorded a macro referencing a PSRAM file, confirmed
  (via `MacroCodec`) the saved macro captured the *path*, not the resolved text, then proved the
  same saved macro drew different text on two separate plays purely because the PSRAM file changed
  in between; `DrawTextFromFileManualCheck` confirmed all three prefixes resolve to the correct
  volume and an unprefixed path defaults to PSRAM.

  **A real, pre-existing bug was caught and fixed along the way** (design note 65), not introduced
  by this feature: the first `ParametrizedMacroManualCheck` attempt showed the replayed text with
  its last character's rightmost column(s) missing. Root cause was in `WorkingBuffer::flush()`'s
  partial-refresh path — GxEPD2's own byte-alignment rounding of a non-byte-aligned X/WIDTH (routine
  for §12 primitives, which have no alignment constraint) derives its rounded width from the
  *original* width without accounting for how far X just shifted left, silently leaving the
  requested region's rightmost columns out of the physical write even though they were already
  correct in the working buffer. This had gone unnoticed through every earlier manual check, since
  none combined a non-byte-aligned region with a genuine partial `REFRESH_NOW` flush *and* content
  precise enough that a couple of missing edge pixels were obviously wrong. Fixed by pre-aligning
  the region inside `flush()` itself, before calling into GxEPD2, so its own rounding becomes a
  no-op; reconfirmed clean on the same test plus a broader regression pass.

  **GPIO (§15) is wired end to end** (design note 66): `GPIO_CONFIGURE`/`WRITE`/`READ_REQUEST`/
  `PLAY_PATTERN` handlers plus the `GPIO_EVENT` push path, backed by a new
  `firmware/lib/Gpio/GpioController.h` (per-pin mode/state tracking, non-blocking pulse-train
  playback, debounced change-event polling — all driven from `loop()` via `update()`, the same
  pattern `MacroPlayer` established). `AVAILABLE_GPIO_PINS` (§5.2 TLV `0x0D`) and `FEATURE_BITMASK`
  bit7 are now included in the handshake response. `GPIO_EVENT` is the first device-initiated push
  this firmware sends with no request to reply to — broadcast on every live transport rather than
  routed to one. **Verified on real hardware** (`GpioManualCheck`): LED write/read round-trips, a
  5-blink pattern confirmed to end LOW, `PIN_UNAVAILABLE`/`BAD_PARAMETERS` validation paths, and
  eleven live button presses producing eleven correctly-alternating `GPIO_EVENT` frames, plus the
  user's own visual/physical confirmation of the LED and button behavior.

  **`BUTTON_EVENT` (§11) is wired end to end** (design notes 67, 69): a new
  `firmware/lib/Buttons/ButtonController.h` debounces and detects PRESS/RELEASE/LONG_PRESS for the
  six observable buttons (Menu, Back, the dial switch's Up/Down/Confirm, and BOOT — GPIO0, safe to
  read as an ordinary button once running since it's only sampled as a strap pin at reset; RESET
  stays genuinely unemittable, since pulling it resets the whole MCU), pushed via `GPIO_EVENT`'s
  broadcast-to-every-live-transport mechanism. **A real bug was found and fixed here** (design note
  70, correcting design notes 67/69's wrong "likely human pacing" conclusion): every single
  `GPIO_EVENT`/`BUTTON_EVENT` push froze the entire firmware for ~2 seconds whenever BLE had no
  connected central, because `NimBLEStreamServer::write()` returns 0 immediately with no peer
  subscribed, and `StreamFrameTransport::send()`'s stall watchdog spent its full 2-second timeout
  retrying a send that was never going to succeed — explaining every symptom of the earlier
  ~2001ms-grid timing oddity exactly, including spurious `LONG_PRESS` events on genuinely short
  clicks. Fixed by checking `gBleStream.ready()` before sending on it. **Re-verified on real
  hardware**: real short clicks now produce realistic 150–250ms PRESS→RELEASE gaps with zero
  spurious `LONG_PRESS`, and three genuine 2+ second holds each fired `LONG_PRESS` at exactly 800ms
  after `PRESS`. The onboard status LED (IO41) was also added to `board::kAvailableGpioPins[]`
  (design note 68), giving GPIO output a built-in test target with no external LED needed —
  verified with `StatusLedManualCheck`.

  **§13 Config commands begun**: `SET_DEVICE_NAME`/`CONFIG_BACKUP_REQUEST`/`CONFIG_RESTORE` (design
  note 72) are implemented, using this firmware's first NVS-backed settings storage (Arduino
  `Preferences`, distinct from the `LittleFS`-backed `VOLUME=INTERNAL`). **A layered config
  resolution was added on top** (design note 73): a `/device.config` file on `VOLUME=SD`, if
  present, overrides whatever's persisted in NVS for that boot session only — reusing
  `CONFIG_BACKUP_DATA`'s exact wire format as the file's own format, so backing up one unit and
  dropping the file on an SD card moved to a different unit re-identifies it on next boot, no new
  tooling needed. **Verified on real hardware**
  (`ConfigManualCheck`): session-only vs. persisted `SET_DEVICE_NAME` behave correctly per
  `FLAGS.PERSIST`; a saved `CONFIG_BACKUP_DATA` blob round-tripped byte-for-byte through
  `CONFIG_RESTORE`; the SD layer correctly overrode NVS after a real reboot
  (`SerialFrameTransport.resetToRunMode()`, design note 71) while leaving the NVS value underneath
  untouched, and correctly fell back once the SD file was removed.

  **§13.2 WiFi config also done** (design note 74): `SET_WIFI_CONFIG`/`WIFI_STATUS_REQUEST`/
  `SET_WIFI_ENABLED`, with persisted credentials (NVS) taking priority over the compile-time
  `secrets.h` fallback. The live reconnect path never blocks `loop()` (unlike the boot-time connect,
  left as its existing blocking wait-with-timeout) — the same non-blocking discipline design note
  70 already established for event pushes. **Verified on real hardware**
  (`WifiConfigManualCheck`): validation NACKs, a live disable→confirm-disconnected→re-enable→poll-
  until-reconnected cycle against the real network, all without ever persisting a test SSID (no wire
  convention exists yet to clear WiFi credentials back to default the way `SET_DEVICE_NAME`'s
  `NAME_LEN=0` does, so that path was deliberately not exercised).

  **§13.2 is now fully implemented, closing out Config commands** (design note 75):
  `SET_BLE_ENABLED`/`BLE_STATUS_REQUEST`/`SET_BLE_PIN` complete it, mirroring `SET_WIFI_ENABLED`'s
  live-toggle shape. `SET_BLE_PIN` is the one `SET_*` command that always persists regardless of
  `FLAGS.PERSIST` — pairing security only ever applies at the next boot (NimBLE has no live
  characteristic-encryption toggle), so a session-only PIN would never take effect at all. **Wire-
  level behavior verified on real hardware** (`BleConfigManualCheck`, run over Serial so disabling
  BLE mid-test can't disrupt the test's own connection): validation, live enable/disable, and
  `HAS_PIN` tracking all correct, PIN always cleared back off afterward.

  **A pre-existing BLE connectivity issue surfaced while verifying the above — not caused by, or
  fixed by, anything in this session** (design note 76): the device isn't discoverable over BLE at
  all, from either Windows or Android, despite `NimBLEDevice::init()`/`NimBLEStreamServer::begin()`/
  `NimBLEAdvertising::start()` all reporting success in a raw boot-log capture. Bisected exhaustively
  — every one of this session's BLE changes reverted, the firmware checked out as far back as git
  history goes, a real power cycle, WiFi forced fully off, and a full flash erase all failed to
  change the outcome; a genuine live WiFi connection while scanning also made no difference. Left
  open as a hardware/RF-level (or very deep NimBLE/ESP-IDF-internal) issue beyond what firmware-side
  diagnosis can resolve further. `BLE_STATUS_RESPONSE.BLE_ADDRESS` at least confirms the controller
  initializes correctly (a real, standard WiFi-MAC-plus-one derived address) — not a total BLE stack
  failure, specifically an advertising/RF-visibility one. A follow-up test swapping `pc-java-lib`'s
  `BSToolbox-BLE` dependency across all three locally-available versions (`0.1.0`, `0.2.0`, the
  pinned `0.3.0-SNAPSHOT`) also ruled out a PC-side library regression — identical failure on all
  three.

  **§17 Power management is done** (design note 77): `SET_POWER_MODE`/`POWER_STATUS_REQUEST` map
  onto `esp_light_sleep_start()`/`esp_deep_sleep_start()`; `LAST_WAKE_REASON` is computed once at
  boot via `esp_reset_reason()` + `esp_sleep_get_wakeup_cause()` and exposed both in
  `POWER_STATUS_RESPONSE` and a new handshake TLV. **Two real, silent bugs were found and fixed on
  real hardware**: wake sources are sticky across separate `SET_POWER_MODE` calls (a second call
  with no timer woke on the *first* call's leftover one — fixed with
  `esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL)` at the start of every sleep path), and
  `esp_sleep_enable_uart_wakeup()` alone never actually wakes the chip without also calling
  `uart_set_wakeup_threshold()`, which isn't configured to a working value by default (confirmed via
  a raw byte-level capture showing zero response, even minutes later, without this fix). **Verified
  end-to-end on real hardware** (`PowerManagementManualCheck` plus targeted raw-serial scripts for
  the paths a reboot would drop the test's own connection through): validation NACKs; `LOW_POWER`
  waking correctly via both timer and real Serial activity; `HARD_SLEEP` waking correctly via both
  timer and an actual physical button press. `KEEP_BLE_CONNECTABLE`'s BLE-activity wake path could
  not be verified — blocked by the still-unresolved BLE connectivity issue above, unrelated to this
  work.

  §16 OTA (`OTA_INSTALL`/`OTA_APPLY`/`OTA_STATUS_REQUEST`/`OTA_CONFIRM`/`OTA_ROLLBACK`) is now fully
  implemented on `esp_ota_ops.h`'s dual-partition API, with hash verification (SHA-256/MD5) before
  any flash write (design note 78). **Two real, silent bugs were found and fixed on real hardware**,
  both specific to the ~1 MB single-frame payload class OTA introduced (nothing earlier came close):
  plain `malloc()`/`new` — and so every default-allocator `std::vector<uint8_t>`, including
  `StreamFrameTransport`'s receive buffer and `Frame::payload` — stayed confined to the ESP32-S3's
  ~512 KB internal SRAM despite PSRAM being present, silently hanging (no crash, no NACK, nothing)
  once a payload passed roughly that size; fixed with one `heap_caps_malloc_extmem_enable(8 * 1024)`
  call in `setup()`. Separately, the default 256-byte Arduino UART RX ring buffer could silently
  drop bytes during a long (90+ second) transfer whenever `loop()` went more than ~22ms without
  polling the transport (WiFi/BLE housekeeping competing for the same single thread), leaving the
  frame parser waiting forever for bytes that would never arrive; fixed with
  `Serial.setRxBufferSize(16384)`. **Verified end-to-end on real hardware** (`OtaManualCheck`,
  staging and installing the board's own current firmware.bin): every validation NACK, a real
  ~1 MB `OTA_INSTALL` (staged in ~99s), `NACK(BUSY)` on a concurrent install, `OTA_APPLY` +
  reboot + confirmed `RUNNING_SLOT` switch, `OTA_CONFIRM`, and `OTA_ROLLBACK` + reboot + confirmed
  revert to the original slot.

  **§17 `SET_POWER_MODE` gained per-peripheral power-rail gating** for the display (GPIO7) and SD
  card (GPIO42) — no wire-format change, both pins already existed in the board header (design note
  79). `HARD_SLEEP` simply cuts both rails before deep sleep (no restore needed, boot re-inits from
  scratch). `LOW_POWER` cuts both before light sleep and restores both **lazily**, symmetrically, on
  the next real use of each — `WorkingBuffer::ensureControllerReady()` (called from `flush()`) for
  the display, `StorageManager::mountSd()`'s existing per-command remount path for SD. The display's
  lazy path relies on a real finding from reading GxEPD2's own driver source: every write function
  already calls `_InitDisplay()` (a hardware reset + `SWRESET`) automatically once `hibernate()` has
  run, so this project only needs to restore VCI first and then reseed the controller's own
  "current"/"previous" RAM banks (which that automatic `SWRESET` clears) from `panelBuffer_` — a
  write-only, invisible resync — before trusting any partial-update diff again; skipping this reseed
  risked visible ghosting specifically in the first post-wake partial update's own region.
  `StorageManager` also cuts its own rail after 30s of inactivity via `updateIdlePowerDown()`,
  polled from `loop()`, recovered the same lazy way. A matching display-side idle timeout was built
  too, then **deliberately reverted** once measured against the SSD1683 datasheet: `hibernate()`
  already runs unconditionally on every `powerDown()` and alone already drops the controller to
  ~3-5uA from VCI, so an ACTIVE-mode-only idle timeout could only ever save that same few uA against
  the ESP32-S3's own tens-of-mA ACTIVE-mode draw — immeasurable in practice, unlike `LOW_POWER`/
  `HARD_SLEEP` where the rest of the system is also minimized (design note 79). Every SD power-down
  path calls `SD.end()` first, never yanking power out from under a mounted card. **Verified
  end-to-end on real hardware**: `PowerRailManualCheck` — a small draw and a full SD upload/download/
  delete round trip both succeed fresh after boot, immediately after a `LOW_POWER` wake, after a
  real 35s idle wait, and after a real `HARD_SLEEP` reboot; `DisplayResyncManualCheck` — drew real
  content, forced a `LOW_POWER` cycle, then drew directly into the still-real (not blank) region
  immediately on wake, visually confirmed clean (no ghosting) by the user — the one part with no
  programmatic equivalent, since the controller's own RAM banks aren't readable over the wire.

  **A real WiFi bug was found live from the user's own current-draw measurement** (~120mA
  unexpectedly, while idle): boot's `connectWifiAndStartTcpServer()` set `WiFi.mode(WIFI_STA)`
  unconditionally but never powered it back off when no credentials were configured — this whole
  session's test board's exact state throughout — leaving an unassociated STA-mode radio running
  indefinitely. The same shape of bug existed independently in `SET_WIFI_ENABLED(1)` and the
  `LOW_POWER`-wake WiFi-restoration path too; all three (plus `SET_WIFI_CONFIG`'s connect-now path,
  for consistency) now go through one shared `startWifiOrPowerOff()` (design note 80). **Verified
  fixed**: `WIFI_STATUS_REQUEST.ENABLED` now correctly reads 0 right after a no-credentials boot,
  where it previously read 1. A parallel BLE consolidation was considered but no equivalent bug
  found — BLE's enable/disable is already a clean symmetric toggle with no credentials-shaped third
  state. **mDNS added**: `startTcpServerIfNeeded()` now also calls `MDNS.begin(deviceName())` +
  `MDNS.addService("crowpanel","tcp",kTcpPort)`, torn down symmetrically in
  `stopTcpServerAndClient()`. Verified live: `ping CrowPanel-3851DC.local` resolved and replied
  correctly after a real (session-only) WiFi connection.

  **Real multimeter current-draw measurements across every power state** (design note 81; USB-powered,
  so the CH340 adapter's own draw is included throughout, not isolated): `ACTIVE` sat flat around
  ~115mA regardless of WiFi/BT state (no attributable difference at this resolution); `LOW_POWER`
  dropped to ~10.7mA (~11x); `HARD_SLEEP` dropped further to ~0.88mA (~130x below `ACTIVE`) — a
  clean confirmation that this project's power-management work targets the states where savings are
  actually large and measurable.

  **Cold-boot info screen and `FAST_CLEAR` (§12.16, `0x030E`) are done** (design notes 83-84):
  `displaySelfTest()` now shows `FW`/`DEV`/`RES` info via the same `drawText()` path `DRAW_TEXT`
  itself uses, a single partial (not full) write - the old full-black-then-full-white pass wasn't
  actually needed, confirmed by reading GxEPD2's own driver source (`_initial_write`/
  `_initial_refresh` already force a full update on the very first write regardless). `FAST_CLEAR`
  is a new, deliberately guard-bypassing full-panel solid fill (skips `SET_CLIP_REGION`/
  `SET_DRAW_OFFSET`/`SET_ORIENTATION` entirely, unlike every other §12 primitive) - about 2x faster
  than the old per-pixel fill path it's modeled on. Both verified on real hardware, including a
  test-script bug (missing RLE decoding) that briefly looked like a real firmware bug before being
  traced to the test itself.

  **`/init.macro`, chained before `/boot.macro` on every cold boot, is done** (design note 85):
  seeds a fixed default (`PAUSE` 1s → `FAST_CLEAR` → `CLEAR_ARTIFACTS`) onto `INTERNAL` the first
  time neither volume has one - a real, user-replaceable `.macro` file, not a hardcoded routine.
  `kFirmwareVersion` now also carries a compile-time build timestamp
  (`"0.1.0-dev (" __DATE__ " " __TIME__ ")"`), since the hand-maintained semver part alone can't
  tell two different builds apart. A real, measured slowness compared to the original firmware's
  own behavior led to checking (not assuming) that this panel's fast full-update waveform was
  already the default, then finding and fixing the actual cause: `CLEAR_ARTIFACTS`'s general
  default (3 cycles = 6 full updates, ~4s each in practice) was excessive for a fresh boot with no
  real ghosting yet to clear; the seeded macro now explicitly requests 1 cycle instead. Verified on
  real hardware: seeded file bytes matched exactly (before and after the cycle-count fix), the
  panel read back correctly blank at every sampled point after the full sequence, the whole
  sequence now completes within the existing 12s connect-time settle window, and the user confirmed
  both the visual sequence and, after the fix, that it feels appropriately fast again.

  **`LOG_MESSAGE` (§10.1, `0x0005`) and the `init_done`/`boot_done` boot markers are done** (design
  note 86): a bidirectional debug/sync marker — sent live it ACKs and echoes back on every live
  transport; recorded into a macro, replaying it re-triggers the same echo at that point in
  playback, letting a PC detect exactly when playback reaches or finishes a point via the new
  `CommandClient#waitForLogMessage(marker, timeout)`. `init_done`/`boot_done` are appended as macro
  entries around the init-macro/boot-macro sequence (§18) automatically. `CommandClient` gained
  multiple event listeners (`addEventListener`/`removeEventListener` replacing the old single
  `setEventListener`) so this temporary listener can coexist with a caller's own. Found and fixed a
  real race in the new manual-check test itself (not in firmware or `CommandClient`): a macro's
  first entry can echo within microseconds of `PLAY_MACRO`'s own ACK — faster than the test thread
  could return from `send()` and only then arm its wait — fixed by arming both marker listeners
  before sending. Verified on real hardware: live round trip (ACK 9ms, echo 9ms), macro-recorded
  round trip (`"start"` t+8ms, `"end"` t+1006ms, ~998ms gap matching the recorded 1000ms `PAUSE`),
  and `init_done`/`boot_done` both firing on a real cold boot (t+8898ms/t+8904ms).

  **Two-tier PIN access control (§5.3, design note 87) is done** — usage/admin PINs gating every
  command on all three transports, a cascading fallback rule (an admin-gated command needs *some*
  credential once any PIN is configured, never left wide open just because that one tier's PIN
  wasn't set), a Serial-only MENU+BACK physical-presence override, and a transport-conditional
  extra check on `SET_POWER_MODE`'s `HARD_SLEEP` (admin over TCP/BLE, usage-only over Serial).
  `Dispatcher` gained a pure, shared `effectiveRequiredLevel()` helper and a required-level
  parameter on every `registerHandler()` call; `BleServerCallbacks` gained its first-ever
  `onConnect` override. Along the way, fixed a second real bug unrelated to access control itself:
  `SerialHmiDevice.connect()` now waits for the `boot_done` marker (design note 86) before
  returning, closing the "real fix... still open" gap `SerialFrameTransport`'s own class doc had
  flagged for its fixed settle-delay guess. **Verified on real hardware** (Serial, full round trip
  via `Cli`): every PIN-configuration combination behaves per the cascading rule, `NOT_AUTHORIZED`
  (`0x0E`) NACKs correctly, `HARD_SLEEP`'s transport-conditional rule holds, clearing PINs reopens
  the device, and the MENU+BACK override was confirmed live with the user physically holding both
  buttons during a real handshake - the one check in this feature that can't be simulated in
  software.

  BLE pairing/security is designed (§20) but not implemented (and, per the above, not currently
  testable either way).

  **Build status**: `pio run -e esp32-s3-crowpanel` (the real target, PlatformIO's bundled
  Espressif xtensa toolchain) **succeeds** — RAM/Flash percentages below have drifted upward
  repeatedly as features were added since this paragraph was first written; see each feature's own
  design note for the size at the time it landed (currently 21.3%/32.9%, design note 82). Now
  builds against a real board profile, `firmware/boards/crowpanel_4_2.json` (design note 82),
  rather than the generic `esp32-s3-devkitc-1` stand-in used since this project's start — verified
  to produce an identical build (same RAM/Flash usage) and identical real-hardware boot behavior.
  Verified via the PlatformIO install at
  `%USERPROFILE%\.platformio\penv\Scripts\pio.exe`. `pio test -e native` (the host-native unit
  tests, no ESP32 involved) could **not** be verified in the environment this was authored in — no
  system C++ compiler (MSVC/LLVM/MinGW) was available there; install one (e.g. MSYS2 MinGW-w64, or
  Visual Studio Build Tools) to run that suite locally.

  **Real-hardware status**: flashed to an actual CrowPanel board (ESP32-S3, rev v0.2, WiFi+BLE,
  identified by `esptool` over the CH340 USB-serial adapter on the board's programming port) and
  boot-log-verified:
  - `CRC16("123456789") = 0x29B1` — matches the reference vector, and matches `pc-java-lib`'s
    result for the same input, confirming the two independent implementations actually agree, not
    just compile.
  - `Frame` round-trip (encode → decode) succeeds on-device.
  - PSRAM initializes correctly and reports ~8 MB free (`ESP.getPsramSize()`/`getFreePsram()`),
    matching the WROOM-1-N8R8 spec — this required setting
    `board_build.arduino.memory_type = qio_opi` in `platformio.ini` (the generic
    `esp32-s3-devkitc-1` board profile defaults to no PSRAM; without this setting, boot logged
    `PSRAM ID read error: ... chip not found`, a real hardware-config bug this caught, not a
    cosmetic one). Also note: this board's CH340 adapter needed `upload_speed = 115200` in
    `platformio.ini` — esptool's default higher upload speed produced intermittent
    "Invalid head of packet" / "chip stopped responding" failures on it.
  - `board_build.partitions = default_8MB.csv` (§16) is confirmed in effect:
    `esp_ota_get_running_partition()` reports `app0` (3,342,336 bytes) as running,
    `esp_ota_get_next_update_partition()` correctly identifies the inactive `app1` slot (same
    size) as where `OTA_INSTALL` would write, and `ESP.getFreeSketchSpace()` reports that same
    ~3.19 MB as the ceiling for `OTA_INSTALL`'s `NACK(INSUFFICIENT_STORAGE)` check.
  - `LittleFS.begin(true)` mounts successfully (~1.5 MB total) — the data partition backing
    `VOLUME=INTERNAL` (§14) exists and is usable, though no `StorageManager` code reads/writes it
    yet.

  **Real end-to-end round trip (Serial transport)**: `pc-java-lib`'s `SerialFrameTransport`
  successfully exchanged a live `HANDSHAKE_REQUEST`/`HANDSHAKE_RESPONSE` with the board over COM5
  and decoded a correct 53-byte TLV payload matching the firmware's handshake builder exactly
  (`PROTOCOL_VERSION=1`, `400×300`, `COLOR_DEPTH=1`, `FEATURE_BITMASK=0`,
  `DEVICE_MODEL="CrowPanel-4.2-EPD"`, `FIRMWARE_VERSION="0.1.0-dev"`, `ACTIVE_TRANSPORT=Serial`).
  Getting there surfaced two real hardware/driver quirks, now fixed in `SerialFrameTransport` (see
  its class doc comment for the full story) rather than just worked around in a test:
  - Opening the port resets the board (the CH340 adapter's DTR/RTS lines drive the same
    auto-reset circuit flashing tools use) — left alone, the device stayed silently in the ROM
    bootloader (zero bytes ever received); explicitly clearing both DTR and RTS right after
    `openPort()` is what lets it boot and run normally instead.
  - `jSerialComm`'s `TIMEOUT_READ_BLOCKING` mode intermittently returned -1 (misread by
    `FrameStreamReader` as the stream having closed) on this adapter/OS combination.
    `TIMEOUT_READ_SEMI_BLOCKING` with a finite per-call timeout was reliable instead, at the cost
    of it throwing `SerialPortTimeoutException` on every ordinary "nothing arrived yet" instead of
    just blocking — `RetryingBlockingInputStream` absorbs that and presents a normal blocking
    stream so `FrameStreamReader` itself stays jSerialComm-agnostic.

  **Real end-to-end round trip (WiFi/TCP transport)**: worked on the first attempt, no bugs
  surfaced. `pc-java-lib`'s `TcpFrameTransport` connected to the board's `WiFiServer` at its
  DHCP-assigned LAN address and exchanged `HANDSHAKE_REQUEST`/`HANDSHAKE_RESPONSE`, decoding the
  same 53-byte TLV payload as the Serial case but with `ACTIVE_TRANSPORT=TCP` — confirming the
  per-transport dispatch (`gDispatcher.dispatch(source, activeTransport, frame)`) correctly
  attributes which channel a request arrived on. Verified both transports live and independently functional
  at the same time (Serial handshake re-run immediately after the TCP one, same firmware image,
  both succeeded).

  **Real end-to-end round trip (BLE transport)**: `pc-java-lib`'s `BleFrameTransport` scanned for,
  connected to, and exchanged `HANDSHAKE_REQUEST`/`HANDSHAKE_RESPONSE` with the board, decoding a
  75-byte TLV payload (now including `MAX_CHUNK_SIZE=244` and `DEVICE_NAME="CrowPanel-3851DC"`
  alongside the fields already seen over Serial/TCP) with `ACTIVE_TRANSPORT=BLE` — with all three
  transports confirmed live on the same firmware image (BLE handshake immediately followed by a
  Serial handshake re-run, both succeeded). Firmware needed no changes to get here beyond the BLE
  bring-up itself; getting the PC side working surfaced one real bug, entirely in
  `pc-java-lib`'s own code, not the sidecar library: `BleFrameTransport.connect()` originally
  constructed its own `BleAdapter` internally rather than accepting the caller's — each
  `BleAdapter` spawns an independent `ble-bridge` sidecar process with its own peripheral cache, so
  connecting via a second, freshly constructed adapter that never scanned reliably failed with
  "unknown peripheral address ... scan for it first", even for an address a *different* adapter had
  just discovered moments earlier. Diagnosing this took a real detour: it was initially
  (reasonably, on Windows, where a never-before-seen BLE peripheral often needs some OS-level
  interaction first) suspected to be a Windows pairing requirement, until a direct WinRT
  `GetGattServicesAsync()` call outside either library entirely proved the device was reachable
  and connectable with **no pairing needed** — which correctly redirected the investigation back to
  this project's own transport code instead of the sidecar. Fixed by having `BleFrameTransport`
  take a caller-owned `BleAdapter` (see its class doc) rather than managing one internally; the
  manual check (below) now shares one adapter between scanning and connecting, as any real
  application must. Also improved, in the sibling `BSToolbox-BLE` project itself (Java-side
  javadoc changes verified compiling here; the Rust sidecar's error message was also clarified but
  **not locally rebuilt/verified** — that project's binary rebuild goes through its own GitHub CI
  pipeline, no local Rust toolchain was available in this session) so this same trap is
  documented/harder to hit for future consumers of that library.

  **Design note 76's "BLE undiscoverable" issue is fixed (design note 99)**: root cause was
  `setupBle()` never calling `NimBLEAdvertising::setName()` — no scan response was ever sent (NimBLE
  defaults `m_scanResp=false`) and no name was broadcast anywhere, which is what made the device
  read as unreliably-discoverable rather than any RF/hardware/NimBLE-internal fault as note 76 had
  guessed. Fixed by enabling scan response and setting the name there (the primary advertising
  packet is already near its 31-byte budget with flags + the custom 128-bit service UUID, leaving no
  room for a name of realistic length). **Verified on real hardware**: the device now shows up named
  in nRF Connect (Android) and `bluetoothctl scan` (Linux/BlueZ). Also used this as the opportunity
  to close a real gap: `DrawPrimitivesManualCheck` (above) had only ever been run over Serial —
  `pc-java-lib` gained a new `DrawPrimitivesBleManualCheck` mirroring it exactly, and a real
  `DRAW_RECT`/`DRAW_CIRCLE`/`DRAW_LINE`/`REFRESH` sequence now round-trips correctly over BLE too,
  ACKed and confirmed rendered on the panel — the first time a local drawing primitive, not just the
  handshake, was exercised over that transport. Also see design note 99 for a Windows-Bluetooth-
  stack red herring hit mid-verification (unrelated to this project's code) and its workaround
  (verify from a Linux/BlueZ machine instead).

See the repository root `CLAUDE.md` for the project brief and hardware specs, and the plan file
this design originated from for the full rationale behind each decision.
