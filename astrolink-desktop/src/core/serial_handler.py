"""
Serial Port Handler for LoRa Communication
Manages serial port connections and data streaming
"""

import logging
from typing import List, Optional

from PyQt6.QtCore import QObject, QTimer, pyqtSignal

from config import DEFAULT_BAUD_RATE, SERIAL_TIMEOUT, HELTEC_KEYWORDS

try:
    import serial
    import serial.tools.list_ports

    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    serial = None

logger = logging.getLogger(__name__)


class SerialHandler(QObject):
    """
    Handles serial port communication with LoRa receiver

    Signals:
        data_received: Emitted when new data arrives (bytes)
        connection_changed: Emitted when connection state changes (bool)
        error_occurred: Emitted on error (str)
    """

    data_received = pyqtSignal(bytes)
    connection_changed = pyqtSignal(bool)
    error_occurred = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._serial_port: Optional[serial.Serial] = None
        self._is_connected = False

        # Polling timer for reading data
        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._poll_serial)
        self._poll_interval = 5  # milliseconds (faster polling to prevent buffer overflow)

        # Statistics
        self._bytes_received = 0
        self._last_error: Optional[str] = None

    @property
    def is_connected(self) -> bool:
        """Check if serial port is connected"""
        return self._is_connected

    @property
    def bytes_received(self) -> int:
        """Get total bytes received"""
        return self._bytes_received

    @staticmethod
    def list_ports() -> List[str]:
        """
        List available serial ports

        Returns:
            List[str]: List of port names (e.g., "COM3", "/dev/ttyUSB0")
        """
        if not SERIAL_AVAILABLE:
            logger.warning("pyserial not installed")
            return []

        ports = []
        try:
            for port_info in serial.tools.list_ports.comports():
                ports.append(port_info.device)
        except Exception as e:
            logger.error(f"Error listing ports: {e}")

        return ports

    @staticmethod
    def find_heltec_port() -> Optional[str]:
        """
        Auto-detect Heltec LoRa board by USB chip identifiers

        Returns:
            Optional[str]: Port name if found, None otherwise
        """
        if not SERIAL_AVAILABLE:
            return None

        try:
            for port_info in serial.tools.list_ports.comports():
                description = port_info.description.upper()
                manufacturer = (port_info.manufacturer or "").upper()

                # Check if any Heltec keyword matches
                for keyword in HELTEC_KEYWORDS:
                    if (
                        keyword.upper() in description
                        or keyword.upper() in manufacturer
                    ):
                        logger.info(
                            f"Found Heltec board: {port_info.device} ({port_info.description})"
                        )
                        return port_info.device

        except Exception as e:
            logger.error(f"Error detecting Heltec port: {e}")

        return None

    def connect(self, port: str, baud_rate: int = DEFAULT_BAUD_RATE) -> bool:
        """
        Connect to serial port

        Args:
            port: Serial port name
            baud_rate: Baud rate (default 115200)

        Returns:
            bool: True if connection successful
        """
        if not SERIAL_AVAILABLE:
            error_msg = "pyserial not installed. Install: pip install pyserial"
            logger.error(error_msg)
            self.error_occurred.emit(error_msg)
            return False

        if self._is_connected:
            logger.warning("Already connected, disconnecting first")
            self.disconnect()

        try:
            # Open port in a “read-only friendly” way: no HW/SW flow control, no auto-reset lines.
            self._serial_port = serial.Serial(
                port=port,
                baudrate=baud_rate,
                timeout=SERIAL_TIMEOUT,
                write_timeout=0,  # we are not writing; prevents blocking
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )

            # Explicitly deassert control lines (prevents some boards from resetting on open).
            try:
                self._serial_port.setDTR(False)
                self._serial_port.setRTS(False)
            except Exception:
                pass

            # Drop any stale bytes that arrived while closed
            try:
                self._serial_port.reset_input_buffer()
                self._serial_port.reset_output_buffer()
            except Exception:
                pass

            self._is_connected = True
            self._bytes_received = 0
            self._last_error = None

            # Start polling for data
            self._poll_timer.start(self._poll_interval)

            logger.info(f"Connected to {port} at {baud_rate} baud")
            self.connection_changed.emit(True)
            return True

        except serial.SerialException as e:
            error_msg = f"Failed to connect to {port}: {e}"
            logger.error(error_msg)
            self._last_error = error_msg
            self.error_occurred.emit(error_msg)
            return False

        except Exception as e:
            error_msg = f"Unexpected error connecting to {port}: {e}"
            logger.error(error_msg)
            self._last_error = error_msg
            self.error_occurred.emit(error_msg)
            return False

    def disconnect(self):
        """Disconnect from serial port"""
        if not self._is_connected:
            return

        # Stop polling
        self._poll_timer.stop()

        # Close port
        if self._serial_port and self._serial_port.is_open:
            try:
                self._serial_port.close()
                logger.info("Serial port closed")
            except Exception as e:
                logger.error(f"Error closing serial port: {e}")

        self._serial_port = None
        self._is_connected = False
        self.connection_changed.emit(False)

    def _poll_serial(self):
        """
        Poll serial port for incoming data
        Called by timer at regular intervals
        """
        if not self._is_connected or not self._serial_port:
            return

        try:
            # Check if data is available
            if self._serial_port.in_waiting > 0:
                # Read available data (limit to 4096 bytes per poll to prevent buffer overflow)
                bytes_to_read = min(self._serial_port.in_waiting, 4096)
                data = self._serial_port.read(bytes_to_read)

                if data:
                    self._bytes_received += len(data)
                    self.data_received.emit(data)

        except serial.SerialException as e:
            error_msg = f"Serial read error: {e}"
            logger.error(error_msg)
            self._last_error = error_msg
            self.error_occurred.emit(error_msg)

            # Auto-disconnect on error
            self.disconnect()

        except Exception as e:
            error_msg = f"Unexpected error reading serial: {e}"
            logger.error(error_msg)
            self._last_error = error_msg
            self.error_occurred.emit(error_msg)

    def write(self, data: bytes) -> bool:
        """
        Write data to serial port

        Args:
            data: Bytes to send

        Returns:
            bool: True if write successful
        """
        if not self._is_connected or not self._serial_port:
            logger.warning("Cannot write: not connected")
            return False

        try:
            self._serial_port.write(data)
            return True

        except Exception as e:
            error_msg = f"Error writing to serial port: {e}"
            logger.error(error_msg)
            self.error_occurred.emit(error_msg)
            return False

    def reset_statistics(self):
        """Reset byte counter"""
        self._bytes_received = 0

    def get_port_info(self) -> Optional[dict]:
        """
        Get information about current port

        Returns:
            Optional[dict]: Port information or None if not connected
        """
        if not self._is_connected or not self._serial_port:
            return None

        return {
            "port": self._serial_port.port,
            "baudrate": self._serial_port.baudrate,
            "is_open": self._serial_port.is_open,
            "bytes_received": self._bytes_received,
        }

    def __del__(self):
        """Cleanup on deletion"""
        try:
            self.disconnect()
        except Exception:
            pass
