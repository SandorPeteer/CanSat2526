"""
CRC16-CCITT Implementation
Standard CRC-16 used in telemetry frames
"""


def crc16_ccitt(data: bytes) -> int:
    """
    Calculate CRC16-CCITT for given data

    Polynomial: 0x1021
    Initial value: 0xFFFF

    Args:
        data: Bytes to calculate CRC for

    Returns:
        int: 16-bit CRC value
    """
    crc = 0xFFFF

    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc


def verify_crc(data: bytes, expected_crc: int) -> bool:
    """
    Verify CRC of data against expected value

    Args:
        data: Data bytes (without CRC)
        expected_crc: Expected CRC value

    Returns:
        bool: True if CRC matches
    """
    calculated = crc16_ccitt(data)
    return calculated == expected_crc
