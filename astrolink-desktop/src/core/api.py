"""
Core API wrapper for stable programmatic access to telemetry decoding.
"""

from typing import Dict, Any, List

from .telemetry import TelemetryDecoder


def decode_stream(raw_data: bytes) -> Dict[str, Any]:
    """Decode a raw telemetry stream and return structured results.

    Args:
        raw_data: Raw bytes from telemetry stream

    Returns:
        dict with keys: `temperatures`, `humidities`, `pressures`,
        `keyframes`, `frame_stats`.
    """
    decoder = TelemetryDecoder()
    temps: List[float]
    hums: List[float]
    pres: List[float]
    keyframes: List[int]

    temps, hums, pres, keyframes = decoder.decode_frames(raw_data)
    stats = decoder.get_frame_stats()
    return {
        "temperatures": temps,
        "humidities": hums,
        "pressures": pres,
        "keyframes": keyframes,
        "frame_stats": stats,
    }
