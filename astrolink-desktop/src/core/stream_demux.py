"""
Serial stream demultiplexer for AstroLink.

The Heltec RX firmware writes:
  1) a raw telemetry payload (starts with 0xA4 or 0xA5, fixed length)
  2) then a RF META frame (starts with 0xD3 0xD4, length-prefixed; v1 is 28 bytes)

This module splits a byte stream into telemetry frames and meta frames and
counts dropped/desync bytes.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

# Import CRC validator
try:
    from utils.crc import crc16_ccitt
    CRC_AVAILABLE = True
except ImportError:
    CRC_AVAILABLE = False


SYNC_FULL = 0xA5
SYNC_DELTA = 0xA4
FULL_LEN = 38
DELTA_LEN = 23

META_H0 = 0xD3
META_H1 = 0xD4
META_MIN_LEN = 8
META_V1_LEN = 28
META_V1_VER = 1


@dataclass
class DemuxResult:
    telemetry_frames: List[bytes]
    meta_frames: List[bytes]
    dropped_bytes: int


class SerialStreamDemux:
    """Split mixed serial stream into telemetry and RF META frames."""

    def __init__(self):
        self._buf = bytearray()
        self.dropped_total = 0

    def reset(self):
        self._buf = bytearray()
        self.dropped_total = 0

    def ingest(self, chunk: bytes) -> DemuxResult:
        self._buf.extend(chunk)
        telemetry: List[bytes] = []
        meta: List[bytes] = []
        dropped = 0

        while True:
            if not self._buf:
                break

            b0 = self._buf[0]

            # PRIORITY 1: Telemetry frames (fixed size, HIGHEST priority)
            # CRITICAL: Must check FULL/DELTA first because telemetry data can contain
            # 0xD3 0xD4 bytes that look like META headers but are actually payload!
            if b0 == SYNC_FULL:
                if len(self._buf) < FULL_LEN:
                    break

                frame = bytes(self._buf[:FULL_LEN])

                # CRITICAL FIX: Validate CRC before accepting frame
                # This prevents false sync byte detection in payload data!
                if CRC_AVAILABLE:
                    crc_calculated = crc16_ccitt(frame[:36])
                    crc_received = frame[36] | (frame[37] << 8)

                    if crc_calculated != crc_received:
                        # Bad CRC → false sync, skip this byte and rescan
                        del self._buf[0]
                        dropped += 1
                        continue

                telemetry.append(frame)
                del self._buf[:FULL_LEN]
                continue

            if b0 == SYNC_DELTA:
                if len(self._buf) < DELTA_LEN:
                    break

                frame = bytes(self._buf[:DELTA_LEN])

                # CRITICAL FIX: Validate CRC before accepting frame
                if CRC_AVAILABLE:
                    crc_calculated = crc16_ccitt(frame[:21])
                    crc_received = frame[21] | (frame[22] << 8)

                    if crc_calculated != crc_received:
                        # Bad CRC → false sync, skip this byte and rescan
                        del self._buf[0]
                        dropped += 1
                        continue

                telemetry.append(frame)
                del self._buf[:DELTA_LEN]
                continue

            # PRIORITY 2: META frames (only if NOT telemetry)
            # Double-check: META can only start if b0 is exactly 0xD3 (not 0xA4 or 0xA5!)
            if b0 == META_H0 and len(self._buf) >= 2 and self._buf[1] == META_H1:
                # Need at least 4 bytes to read header
                if len(self._buf) < 4:
                    break

                ver = self._buf[2]
                total_len = int(self._buf[3])

                # Strict validation: must be exact v1 format (prevents false positives)
                if ver != META_V1_VER or total_len != META_V1_LEN:
                    # Not a valid META v1 header, skip this byte
                    del self._buf[0]
                    dropped += 1
                    continue

                # Wait for complete frame
                if len(self._buf) < total_len:
                    break

                frame = bytes(self._buf[:total_len])

                # Additional CRC validation (byte 27 should be XOR of bytes 2-26)
                crc_calc = 0
                for b in frame[2:27]:
                    crc_calc ^= b
                crc_recv = frame[27]

                if crc_calc != crc_recv:
                    # Invalid META CRC, skip this byte and rescan
                    del self._buf[0]
                    dropped += 1
                    continue

                meta.append(frame)
                del self._buf[:total_len]
                continue

            # Unknown byte: resync (corrupted data, boot messages, etc.)
            # REMOVED: ASCII text detection was too aggressive and dropped valid telemetry!
            # Now we rely on CRC validation above to prevent false sync detection.
            del self._buf[0]
            dropped += 1

        self.dropped_total += dropped
        return DemuxResult(telemetry_frames=telemetry, meta_frames=meta, dropped_bytes=dropped)

