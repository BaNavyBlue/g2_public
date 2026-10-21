# Pi link protocol, v0

Binary link between a host (Raspberry Pi running LinuxCNC) and g2core on a TinyG v9
board, over the board's J17 header. USB and its JSON/G-code interface are unchanged.

v0 covers bring-up only: echo and status. No motion is commanded over this link yet.

## Physical layer

| Item | Value |
| --- | --- |
| Board UART | SAM3X USART2 |
| Board TX | J17 "CS1" (PB20) -> host RX |
| Board RX | J17 "CS2" (PB21) <- host TX. The factory GND-CS2 jumper must be removed |
| Ground | J17 GND <-> host GND |
| Levels | 3.3 V both sides |
| Format | 8N1, no flow control |
| Baud | `PI_LINK_BAUD`, default 115200 (bring-up). The board sets the USART's fractional divider, so 1, 2 and 3 Mbaud are exact at MCK 84 MHz |

## Framing

Each frame is COBS-encoded and followed by a single `0x00` delimiter. Decoded frames
are at most 128 bytes. Frames that fail COBS decoding, are too long or too short, or
fail the CRC are dropped silently and counted (see STATUS).

Decoded frame layout (all multi-byte fields little-endian):

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 1 | version, 0 |
| 1 | 1 | type |
| 2 | 2 | seq: chosen by the host; the board echoes it in its reply |
| 4 | n | payload |
| 4+n | 4 | CRC-32 of bytes 0 .. 3+n (IEEE 802.3: reflected poly 0xEDB88320, init and final XOR 0xFFFFFFFF; same as zlib.crc32) |

## Frame types

| Type | Direction | Payload | Reply |
| --- | --- | --- | --- |
| 0x01 PING | host -> board | 0-64 bytes, any content | 0x81 PONG with the same payload |
| 0x02 STATUS_REQ | host -> board | none | 0x82 STATUS |
| 0xFF NAK | board -> host | reason (u8), offending type (u8) | none |

NAK reasons: 1 = unknown type, 2 = bad payload length.

## STATUS payload (40 bytes)

| Offset | Type | Field | Same meaning as JSON |
| --- | --- | --- | --- |
| 0 | u8 | combined machine state | `stat` |
| 1 | u8 | interlock and ESC safety bits | `safe` |
| 2 | u8 | E-stop state | `estp` |
| 3 | u8 | homed: bit0 X, bit1 Y, bit2 Z, bit3 A | `homx` .. `homa` |
| 4 | u16 | input states: bit (N-1) = `inN` | `in1` .. `in16` |
| 6 | u8 | spindle direction | `spc` |
| 7 | u8 | reserved, 0 | |
| 8 | f32 | spindle speed | `sps` |
| 12 | f32 x4 | machine position X, Y, Z (mm), A (deg) | `mpox` .. `mpoa` |
| 28 | u32 | board uptime, ms | |
| 32 | u16 | frames received OK | |
| 34 | u16 | frames dropped: bad CRC | |
| 36 | u16 | frames dropped: bad COBS, too long or too short | |
| 38 | u16 | receive ring overflows | |

Counters wrap at 65535.

## Host rules

- Keep one request in flight: wait for the reply (or a timeout, e.g. 50 ms at 115200) before
  sending the next frame. The board sends one reply at a time and drops a reply that would
  start while the previous one is still being sent.
- A dropped frame gets no reply. The host retries after its timeout; `seq` tells a late
  reply from the current one.

## Test tool

`host/pi_link_test.py` (Python 3 + pyserial) speaks v0 from the host:
`ping [-n N] [-s BYTES]`, `status [--watch S]`, and `errors` (sends malformed frames and
checks that the board drops and counts each one).
