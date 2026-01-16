"""
Utility helper functions
"""

from datetime import datetime
from pathlib import Path
from typing import List, Optional


def format_file_size(size_bytes: int) -> str:
    """
    Format file size in human-readable format

    Args:
        size_bytes: Size in bytes

    Returns:
        str: Formatted size (e.g., "1.5 MB")
    """
    if size_bytes < 1024:
        return f"{size_bytes} B"

    size = float(size_bytes)
    for unit in ("KB", "MB", "GB", "TB"):
        size /= 1024.0
        if size < 1024.0 or unit == "TB":
            return f"{size:.1f} {unit}"

    return f"{size:.1f} TB"


def get_timestamp_filename(prefix: str = "raw", extension: str = ".bin") -> str:
    """
    Generate timestamped filename

    Args:
        prefix: Filename prefix
        extension: File extension

    Returns:
        str: Filename like "raw_20251216_143022.bin"
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{prefix}_{timestamp}{extension}"


def find_files_by_extension(directory: Path, extension: str) -> List[str]:
    """
    Find all files with given extension in directory

    Args:
        directory: Directory to search
        extension: File extension (e.g., ".bin")

    Returns:
        List[str]: List of filenames (not full paths)
    """
    if not directory.exists():
        return []

    files = []
    for file_path in directory.iterdir():
        if file_path.is_file() and file_path.suffix.lower() == extension.lower():
            files.append(file_path.name)

    return sorted(files, reverse=True)  # Newest first


def extract_bits_le(buffer: bytes, bit_position: int, num_bits: int) -> int:
    """
    Extract bits from byte buffer (little-endian bit order)

    Args:
        buffer: Byte buffer
        bit_position: Starting bit position
        num_bits: Number of bits to extract

    Returns:
        int: Extracted value
    """
    value = 0

    for i in range(num_bits):
        byte_index = (bit_position + i) >> 3  # Divide by 8
        bit_index = (bit_position + i) & 7  # Modulo 8

        if buffer[byte_index] & (1 << bit_index):
            value |= 1 << i

    return value


def format_timestamp(timestamp: Optional[datetime] = None) -> str:
    """
    Format timestamp for display

    Args:
        timestamp: Datetime object (None = current time)

    Returns:
        str: Formatted timestamp
    """
    if timestamp is None:
        timestamp = datetime.now()

    return timestamp.strftime("%Y-%m-%d %H:%M:%S")


def clamp(value: float, min_value: float, max_value: float) -> float:
    """
    Clamp value between min and max

    Args:
        value: Value to clamp
        min_value: Minimum value
        max_value: Maximum value

    Returns:
        float: Clamped value
    """
    return max(min_value, min(value, max_value))
