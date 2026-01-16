"""
Telemetry Frame Decoder
Decodes FULL and DELTA frames from LoRa telemetry stream
"""

from typing import List, Tuple, Optional
from dataclasses import dataclass
from config import SYNC_FULL, SYNC_DELTA, FULL_FRAME_LEN, DELTA_FRAME_LEN
from utils.crc import crc16_ccitt
from utils.helpers import extract_bits_le


@dataclass
class TelemetrySample:
    """Single telemetry sample"""

    index: int
    met_tick: int  # 0.5s/tick (from frame base_MET + intra-frame offset)
    temperature: float  # Celsius
    humidity: float  # Percent
    pressure: float  # hPa
    is_keyframe: bool
    crc_valid: bool


@dataclass
class FrameInfo:
    """Frame decoding information"""

    frame_number: int
    frame_type: str  # "FULL" or "DELTA"
    sequence: int
    ref_sequence: Optional[int]
    crc_valid: bool
    error_reason: Optional[str]
    status: str  # "ACCEPTED", "CRC_ERROR", "NO_REF", "REF_MISMATCH", "TRUNCATED"
    raw_data: bytes
    crc_calculated: Optional[int] = None
    crc_received: Optional[int] = None


@dataclass
class SignalMetadata:
    """LoRa signal quality metadata"""

    rssi: float  # Received Signal Strength Indicator (dBm, -140 to 0)
    snr: float  # Signal-to-Noise Ratio (dB)
    freq_error: float  # Frequency offset (Hz)
    packet_rssi: float  # Per-packet RSSI (dBm)
    timestamp_us: int  # Reception timestamp (microseconds)


class TelemetryDecoder:
    """
    Decodes mixed stream of FULL (38B) and DELTA (23B) telemetry frames

    FULL Frame Structure (38 bytes):
      [0]      = 0xA5 (SYNC_FULL)
      [1..2]   = base_MET (LE)
      [3]      = FLAGS
      [4]      = SEQ
      [5]      = VALIDITY
      [6..35]  = bitstream: 8×30-bit samples (LSB-first)
      [36..37] = CRC16 (LE) over bytes 0..35

    DELTA Frame Structure (23 bytes):
      [0]      = 0xA4 (SYNC_DELTA)
      [1..2]   = base_MET (LE)
      [3]      = FLAGS
      [4]      = SEQ
      [5]      = REF_KEY_SEQ
      [6..20]  = bitstream: 8×(dT5 + dRH4 + dP6) = 120 bits (LSB-first)
      [21..22] = CRC16 (LE) over bytes 0..20

    Sample encoding (30 bits):
      - T_code:  11 bits (temperature)
      - H_code:   7 bits (humidity)
      - P_int:    8 bits (pressure integer)
      - P_frac:   4 bits (pressure fraction)
    """

    def __init__(self):
        self.samples: List[TelemetrySample] = []
        self.frames: List[FrameInfo] = []
        self.signal_metrics: List[SignalMetadata] = []  # Track signal quality over time
        self._sample_index = 0
        self._frame_number = 0

        # State for delta decoding
        self._have_ref = False
        self._last_abs30 = 0
        self._last_key_seq: Optional[int] = None
        self._last_signal_metadata: Optional[SignalMetadata] = None

    def decode_frames(
        self, raw_data: bytes
    ) -> Tuple[List[float], List[float], List[float], List[int]]:
        """
        Decode raw telemetry stream into temperature, humidity, pressure arrays

        Args:
            raw_data: Raw byte stream from LoRa

        Returns:
            Tuple of (temperatures, humidities, pressures, keyframe_indices)
        """
        temperatures = []
        humidities = []
        pressures = []
        keyframe_indices = []

        i = 0
        n = len(raw_data)

        while i < n:
            sync = raw_data[i]

            # Determine frame type
            if sync == SYNC_FULL:
                frame_len = FULL_FRAME_LEN
                frame_type = "FULL"
            elif sync == SYNC_DELTA:
                frame_len = DELTA_FRAME_LEN
                frame_type = "DELTA"
            else:
                # Not a valid sync byte, skip
                i += 1
                continue

            # Check if we have enough data
            if i + frame_len > n:
                # Record truncated frame
                self.frames.append(
                    FrameInfo(
                        frame_number=self._frame_number,
                        frame_type=frame_type,
                        sequence=raw_data[i + 4] if (i + 5) <= n else -1,
                        ref_sequence=raw_data[i + 5] if (frame_type == "DELTA" and (i + 6) <= n) else None,
                        crc_valid=False,
                        error_reason="Truncated frame",
                        status="TRUNCATED",
                        raw_data=raw_data[i:],
                    )
                )
                self._frame_number += 1
                break

            frame = raw_data[i : i + frame_len]
            base_met = frame[1] | (frame[2] << 8)

            # Extract signal metadata from FLAGS byte [3]
            flags_byte = frame[3]
            signal_meta = self.extract_signal_metadata(flags_byte)
            self._last_signal_metadata = signal_meta
            self.signal_metrics.append(signal_meta)

            # Verify CRC
            if frame_type == "FULL":
                crc_calculated = crc16_ccitt(frame[:36])
                crc_received = frame[36] | (frame[37] << 8)
            else:  # DELTA
                crc_calculated = crc16_ccitt(frame[:21])
                crc_received = frame[21] | (frame[22] << 8)

            crc_valid = crc_calculated == crc_received
            frame_bad_crc = not crc_valid

            # Extract sequence numbers
            sequence = frame[4]
            ref_sequence = frame[5] if frame_type == "DELTA" else None

            current_frame = FrameInfo(
                frame_number=self._frame_number,
                frame_type=frame_type,
                sequence=sequence,
                ref_sequence=ref_sequence,
                crc_valid=crc_valid,
                error_reason=None if crc_valid else "CRC mismatch",
                status="ACCEPTED" if crc_valid else "CRC_ERROR",
                crc_calculated=crc_calculated if not crc_valid else crc_calculated,
                crc_received=crc_received if not crc_valid else crc_received,
                raw_data=frame,
            )

            # For CRC errors: decode anyway for diagnostics, but do NOT update reference state.
            decode_even_if_crc_bad = True

            # Decode frame
            if frame_type == "FULL":
                # FULL keyframe: 8×30-bit absolute samples
                bit_position = 6 * 8  # Start after header (6 bytes)
                for sample_offset in range(8):
                    v30 = extract_bits_le(frame, bit_position, 30)
                    bit_position += 30

                    if crc_valid:
                        self._last_abs30 = v30
                        self._have_ref = True
                        self._last_key_seq = sequence

                    temp, hum, press = self._decode_sample(v30)
                    temperatures.append(temp)
                    humidities.append(hum)
                    pressures.append(press)

                    self.samples.append(
                        TelemetrySample(
                            index=self._sample_index,
                            met_tick=int(base_met) + int(sample_offset),
                            temperature=temp,
                            humidity=hum,
                            pressure=press,
                            is_keyframe=True,
                            crc_valid=crc_valid,
                        )
                    )

                    if crc_valid:
                        keyframe_indices.append(self._sample_index)
                    self._sample_index += 1

            else:  # DELTA frame
                # Check if we can decode (need previous keyframe reference)
                if not self._have_ref:
                    # Haven't received a keyframe yet
                    self.frames.append(
                        FrameInfo(
                            frame_number=self._frame_number,
                        frame_type=frame_type,
                        sequence=sequence,
                        ref_sequence=ref_sequence,
                        crc_valid=True,
                        error_reason="No reference keyframe",
                            status="NO_REF",
                            raw_data=frame,
                        )
                    )
                    self._frame_number += 1
                    i += frame_len
                    continue

                if (
                    self._last_key_seq is not None
                    and ref_sequence != self._last_key_seq
                ):
                    # Delta references different keyframe, skip
                    self.frames.append(
                        FrameInfo(
                            frame_number=self._frame_number,
                            frame_type=frame_type,
                            sequence=sequence,
                            ref_sequence=ref_sequence,
                            crc_valid=True,
                            error_reason="Delta references different keyframe",
                            status="REF_MISMATCH",
                            raw_data=frame,
                        )
                    )
                    self._frame_number += 1
                    i += frame_len
                    continue

                # Decode 8 delta samples
                bit_position = 6 * 8  # Start after header
                prev = self._last_abs30

                for sample_offset in range(8):
                    dT = self._twos_complement(
                        extract_bits_le(frame, bit_position, 5), 5
                    )
                    bit_position += 5
                    dRH = self._twos_complement(
                        extract_bits_le(frame, bit_position, 4), 4
                    )
                    bit_position += 4
                    dP = self._twos_complement(
                        extract_bits_le(frame, bit_position, 6), 6
                    )
                    bit_position += 6

                    # Unpack previous absolute codes
                    T_code = (prev & 0x7FF) + dT
                    RH_code = ((prev >> 11) & 0x7F) + dRH
                    P_int = (prev >> 18) & 0xFF
                    P_frac = (prev >> 26) & 0x0F

                    if P_frac > 9:
                        P_frac = 9

                    P_code12 = (P_int * 10 + P_frac) + dP

                    # Clamp to valid ranges (prevents runaway on corrupted delta)
                    T_code = max(0, min(T_code, 0x7FF))
                    RH_code = max(0, min(RH_code, 0x7F))
                    P_code12 = max(0, min(P_code12, 2550))

                    # Repack into 30-bit value
                    P_int = P_code12 // 10
                    P_frac = P_code12 % 10

                    v30 = (
                        (T_code & 0x7FF)
                        | ((RH_code & 0x7F) << 11)
                        | ((P_int & 0xFF) << 18)
                        | ((P_frac & 0x0F) << 26)
                    )

                    prev = v30
                    if crc_valid:
                        self._last_abs30 = v30
                        self._have_ref = True

                    temp, hum, press = self._decode_sample(v30)
                    temperatures.append(temp)
                    humidities.append(hum)
                    pressures.append(press)

                    self.samples.append(
                        TelemetrySample(
                            index=self._sample_index,
                            met_tick=int(base_met) + int(sample_offset),
                            temperature=temp,
                            humidity=hum,
                            pressure=press,
                            is_keyframe=False,
                            crc_valid=crc_valid,
                        )
                    )

                    self._sample_index += 1

            # Move to next frame (accepted)
            self.frames.append(current_frame)
            self._frame_number += 1
            i += frame_len

        return temperatures, humidities, pressures, keyframe_indices

    def _decode_sample(self, v30: int) -> Tuple[float, float, float]:
        """
        Decode 30-bit sample into physical values

        Args:
            v30: 30-bit encoded sample

        Returns:
            Tuple of (temperature, humidity, pressure)
        """
        T_code = (v30 >> 0) & ((1 << 11) - 1)
        H_code = (v30 >> 11) & ((1 << 7) - 1)
        P_int = (v30 >> 18) & 0xFF
        P_frac = (v30 >> 26) & 0x0F

        # Convert to physical values
        temperature = -40.0 + T_code * 0.1
        humidity = float(H_code)
        pressure = 822.0 + (P_int * 10 + P_frac) * 0.1

        return temperature, humidity, pressure

    @staticmethod
    def _twos_complement(value: int, bits: int) -> int:
        """
        Convert unsigned value to signed using two's complement

        Args:
            value: Unsigned integer value
            bits: Number of bits

        Returns:
            int: Signed integer
        """
        sign_bit = 1 << (bits - 1)
        if value & sign_bit:
            return value - (1 << bits)
        return value

    def extract_signal_metadata(self, flags_byte: int) -> SignalMetadata:
        """
        Extract RF signal quality metadata from FLAGS byte

        FLAGS byte layout (8 bits):
          Bits [7:5] - RSSI indicator (3 bits, 0-7 scale)
          Bits [4:2] - SNR indicator (3 bits, 0-7 scale)
          Bits [1:0] - Reserved

        RSSI scale: 0 = -140dBm, 7 = -60dBm (linear interpolation)
        SNR scale:  0 = 0dB, 7 = 15dB (linear interpolation)

        Args:
            flags_byte: FLAGS byte from frame [3]

        Returns:
            SignalMetadata: Signal quality information
        """
        # Extract bit fields
        rssi_code = (flags_byte >> 5) & 0x07  # Bits [7:5]
        snr_code = (flags_byte >> 2) & 0x07  # Bits [4:2]

        # Convert to physical values
        # RSSI: scale from -140dBm (code=0) to -60dBm (code=7)
        rssi = -140.0 + (rssi_code / 7.0) * 80.0

        # SNR: scale from 0dB (code=0) to 15dB (code=7)
        snr = (snr_code / 7.0) * 15.0

        return SignalMetadata(
            rssi=rssi,
            snr=snr,
            freq_error=0.0,  # Not available in current frame format
            packet_rssi=rssi,  # Use same as RSSI
            timestamp_us=0,  # Not available in current frame format
        )

    def reset(self):
        """Reset decoder state"""
        self.samples.clear()
        self.frames.clear()
        self.signal_metrics.clear()
        self._sample_index = 0
        self._frame_number = 0
        self._have_ref = False
        self._last_abs30 = 0
        self._last_key_seq = None
        self._last_signal_metadata = None

    def get_frame_stats(self) -> dict:
        """
        Get statistics about decoded frames

        Returns:
            dict: Frame statistics
        """
        total_frames = len(self.frames)
        if total_frames == 0:
            return {
                "total_frames": 0,
                "full_frames": 0,
                "delta_frames": 0,
                "crc_errors": 0,
                "ref_mismatch": 0,
                "no_ref": 0,
                "truncated": 0,
                "total_samples": 0,
            }

        full_count = sum(1 for f in self.frames if f.frame_type == "FULL")
        delta_count = sum(1 for f in self.frames if f.frame_type == "DELTA")
        crc_errors = sum(1 for f in self.frames if f.status == "CRC_ERROR")
        ref_mismatch = sum(1 for f in self.frames if f.status == "REF_MISMATCH")
        no_ref = sum(1 for f in self.frames if f.status == "NO_REF")
        truncated = sum(1 for f in self.frames if f.status == "TRUNCATED")

        return {
            "total_frames": total_frames,
            "full_frames": full_count,
            "delta_frames": delta_count,
            "crc_errors": crc_errors,
            "ref_mismatch": ref_mismatch,
            "no_ref": no_ref,
            "truncated": truncated,
            "total_samples": len(self.samples),
        }

    def get_sample_met_seconds(self, sample_index: int) -> Optional[float]:
        """
        Return MET for a given sample index in seconds (0.5s per tick).
        """
        if sample_index < 0 or sample_index >= len(self.samples):
            return None
        return float(self.samples[sample_index].met_tick) * 0.5

    def get_recent_frames(self, limit: int = 20) -> List[FrameInfo]:
        """Return last N frames for diagnostics."""
        if limit <= 0:
            return []
        return self.frames[-limit:]
