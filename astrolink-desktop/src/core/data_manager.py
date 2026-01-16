"""
Data Manager
Handles file I/O for telemetry data (load/save .bin files, export CSV/JSON)
"""

import csv
import json
import logging
from pathlib import Path
from typing import List, Optional
from datetime import datetime

from config import (
    TELEMETRY_FILE_EXTENSION,
    EXPORT_CSV_EXTENSION,
    EXPORT_JSON_EXTENSION,
    WORKING_DIR,
)
from utils.helpers import get_timestamp_filename, find_files_by_extension

logger = logging.getLogger(__name__)


class DataManager:
    """Manages telemetry data file operations"""

    def __init__(self, working_directory: Optional[Path] = None):
        """
        Initialize data manager

        Args:
            working_directory: Directory for telemetry files (default: current dir)
        """
        self.working_dir = working_directory or WORKING_DIR
        self._ensure_directory_exists()

        # Current recording
        self._recording = False
        self._recording_file: Optional[Path] = None
        self._last_recording_file: Optional[Path] = None
        self._recording_handle = None

    def _resolve_path(self, filename: str) -> Path:
        """
        Resolve a user-provided filename to a full path.

        - Absolute paths are used as-is.
        - Relative paths are resolved under working_dir.
        """
        candidate = Path(filename).expanduser()
        if candidate.is_absolute():
            return candidate
        return self.working_dir / candidate

    def _ensure_directory_exists(self):
        """Create working directory if it doesn't exist"""
        if not self.working_dir.exists():
            self.working_dir.mkdir(parents=True, exist_ok=True)
            logger.info(f"Created working directory: {self.working_dir}")

    def list_telemetry_files(self) -> List[str]:
        """
        List all .bin telemetry files in working directory

        Returns:
            List[str]: Filenames (sorted, newest first)
        """
        return find_files_by_extension(self.working_dir, TELEMETRY_FILE_EXTENSION)

    def load_file(self, filename: str) -> Optional[bytes]:
        """
        Load telemetry data from .bin file

        Args:
            filename: Filename or full path

        Returns:
            Optional[bytes]: File contents or None on error
        """
        file_path = self._resolve_path(filename)

        if not file_path.exists():
            logger.error(f"File not found: {file_path}")
            return None

        try:
            data = file_path.read_bytes()
            logger.info(f"Loaded {len(data)} bytes from {filename}")
            return data

        except Exception as e:
            logger.error(f"Error loading file {filename}: {e}")
            return None

    def save_file(self, filename: str, data: bytes) -> bool:
        """
        Save telemetry data to .bin file

        Args:
            filename: Filename or full path
            data: Binary data to save

        Returns:
            bool: True if successful
        """
        file_path = self._resolve_path(filename)

        try:
            file_path.parent.mkdir(parents=True, exist_ok=True)
            file_path.write_bytes(data)
            logger.info(f"Saved {len(data)} bytes to {file_path}")
            return True

        except Exception as e:
            logger.error(f"Error saving file {file_path}: {e}")
            return False

    def start_recording(self, filename: Optional[str] = None) -> bool:
        """
        Start recording telemetry to file

        Args:
            filename: Optional filename (auto-generated if None)

        Returns:
            bool: True if recording started
        """
        if self._recording:
            logger.warning("Already recording")
            return False

        if filename is None:
            filename = get_timestamp_filename("raw", TELEMETRY_FILE_EXTENSION)

        self._recording_file = self._resolve_path(filename)

        try:
            self._recording_file.parent.mkdir(parents=True, exist_ok=True)
            self._recording_handle = open(self._recording_file, "wb")
            self._recording = True
            logger.info(f"Started recording to {self._recording_file}")
            return True

        except Exception as e:
            logger.error(f"Error starting recording: {e}")
            self._recording = False
            self._recording_file = None
            return False

    def append_recording(self, data: bytes) -> bool:
        """
        Append data to current recording

        Args:
            data: Data to append

        Returns:
            bool: True if successful
        """
        if not self._recording or not self._recording_handle:
            return False

        try:
            self._recording_handle.write(data)
            self._recording_handle.flush()  # Ensure data is written immediately
            return True

        except Exception as e:
            logger.error(f"Error appending to recording: {e}")
            return False

    def stop_recording(self) -> Optional[str]:
        """
        Stop current recording

        Returns:
            Optional[str]: Filename of recorded file, or None if not recording
        """
        if not self._recording:
            return None

        filename = self._recording_file.name if self._recording_file else None

        try:
            if self._recording_handle:
                self._recording_handle.close()

            logger.info(f"Stopped recording: {filename}")

        except Exception as e:
            logger.error(f"Error stopping recording: {e}")

        finally:
            self._recording = False
            self._last_recording_file = self._recording_file
            self._recording_file = None
            self._recording_handle = None

        return filename

    def is_recording(self) -> bool:
        """Check if currently recording"""
        return self._recording

    def get_last_recording_path(self) -> Optional[Path]:
        """Return the most recently finished recording path (if any)."""
        return self._last_recording_file

    def delete_file(self, filename: str) -> bool:
        """
        Delete telemetry file

        Args:
            filename: Filename to delete

        Returns:
            bool: True if successful
        """
        file_path = self._resolve_path(filename)

        if not file_path.exists():
            logger.error(f"File not found: {filename}")
            return False

        try:
            file_path.unlink()
            logger.info(f"Deleted file: {filename}")
            return True

        except Exception as e:
            logger.error(f"Error deleting file {filename}: {e}")
            return False

    def export_to_csv(
        self,
        filename: str,
        temperatures: List[float],
        humidities: List[float],
        pressures: List[float],
    ) -> bool:
        """
        Export telemetry data to CSV file

        Args:
            filename: Output filename
            temperatures: Temperature values
            humidities: Humidity values
            pressures: Pressure values

        Returns:
            bool: True if successful
        """
        if not filename.endswith(EXPORT_CSV_EXTENSION):
            filename += EXPORT_CSV_EXTENSION

        file_path = self._resolve_path(filename)

        try:
            file_path.parent.mkdir(parents=True, exist_ok=True)
            with open(file_path, "w", newline="") as csvfile:
                writer = csv.writer(csvfile)

                # Header
                writer.writerow(
                    ["Index", "Temperature (°C)", "Humidity (%)", "Pressure (hPa)"]
                )

                # Data rows
                for i, (temp, hum, press) in enumerate(
                    zip(temperatures, humidities, pressures)
                ):
                    writer.writerow([i, f"{temp:.1f}", f"{hum:.1f}", f"{press:.1f}"])

            logger.info(f"Exported {len(temperatures)} samples to {file_path}")
            return True

        except Exception as e:
            logger.error(f"Error exporting to CSV: {e}")
            return False

    def export_to_json(
        self,
        filename: str,
        temperatures: List[float],
        humidities: List[float],
        pressures: List[float],
        metadata: Optional[dict] = None,
    ) -> bool:
        """
        Export telemetry data to JSON file

        Args:
            filename: Output filename
            temperatures: Temperature values
            humidities: Humidity values
            pressures: Pressure values
            metadata: Optional metadata dictionary

        Returns:
            bool: True if successful
        """
        if not filename.endswith(EXPORT_JSON_EXTENSION):
            filename += EXPORT_JSON_EXTENSION

        file_path = self._resolve_path(filename)

        # Prepare data structure
        data = {
            "metadata": metadata
            or {
                "export_time": datetime.now().isoformat(),
                "sample_count": len(temperatures),
            },
            "samples": [
                {
                    "index": i,
                    "temperature": round(temp, 2),
                    "humidity": round(hum, 2),
                    "pressure": round(press, 2),
                }
                for i, (temp, hum, press) in enumerate(
                    zip(temperatures, humidities, pressures)
                )
            ],
        }

        try:
            file_path.parent.mkdir(parents=True, exist_ok=True)
            with open(file_path, "w") as jsonfile:
                json.dump(data, jsonfile, indent=2)

            logger.info(f"Exported {len(temperatures)} samples to {file_path}")
            return True

        except Exception as e:
            logger.error(f"Error exporting to JSON: {e}")
            return False

    def get_file_info(self, filename: str) -> Optional[dict]:
        """
        Get information about a telemetry file

        Args:
            filename: Filename to inspect

        Returns:
            Optional[dict]: File information or None if not found
        """
        file_path = self._resolve_path(filename)

        if not file_path.exists():
            return None

        try:
            stat = file_path.stat()
            return {
                "filename": filename,
                "size_bytes": stat.st_size,
                "created": datetime.fromtimestamp(stat.st_ctime),
                "modified": datetime.fromtimestamp(stat.st_mtime),
            }

        except Exception as e:
            logger.error(f"Error getting file info for {filename}: {e}")
            return None
