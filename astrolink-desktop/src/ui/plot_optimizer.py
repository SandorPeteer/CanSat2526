"""
Plot Performance Optimizer for AstroLink Mission Control

Provides automatic downsampling, chunked rendering, and adaptive update rates
to ensure smooth plotting on all hardware configurations.
"""

import numpy as np
from typing import Tuple, List
import time


class PlotOptimizer:
    """Automatic plot performance optimization"""

    def __init__(
        self,
        max_points: int = 10000,
        downsample_threshold: int = 5000,
        update_interval_ms: int = 50,
    ):
        """
        Initialize plot optimizer

        Args:
            max_points: Maximum points to render
            downsample_threshold: Start downsampling above this count
            update_interval_ms: Plot update interval in milliseconds
        """
        self.max_points = max_points
        self.downsample_threshold = downsample_threshold
        self.update_interval_ms = update_interval_ms

        # Performance tracking
        self._last_update_time = 0
        self._frame_times = []
        self._adaptive_enabled = True

    def downsample_decimate(
        self, data: np.ndarray, target_points: int
    ) -> np.ndarray:
        """
        Downsample data using decimation (fastest method)

        Args:
            data: Input data array
            target_points: Target number of points

        Returns:
            Downsampled array
        """
        if len(data) <= target_points:
            return data

        step = len(data) // target_points
        return data[::step]

    def downsample_lttb(
        self, x: np.ndarray, y: np.ndarray, threshold: int
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Largest-Triangle-Three-Buckets (LTTB) downsampling algorithm

        Preserves visual features better than simple decimation.
        Best for time-series data where visual fidelity matters.

        Args:
            x: X-axis data (e.g., time or index)
            y: Y-axis data (e.g., temperature)
            threshold: Target number of points

        Returns:
            Tuple of (downsampled_x, downsampled_y)
        """
        n = len(x)

        if n <= threshold:
            return x, y

        # Always include first and last points
        sampled_x = [x[0]]
        sampled_y = [y[0]]

        # Bucket size
        bucket_size = (n - 2) / (threshold - 2)

        # Previous point (initially first point)
        prev_x = x[0]
        prev_y = y[0]

        for i in range(1, threshold - 1):
            # Calculate point range for this bucket
            range_start = int((i - 1) * bucket_size) + 1
            range_end = int(i * bucket_size) + 1

            # Calculate average point of NEXT bucket (for triangle calculation)
            next_range_start = int(i * bucket_size) + 1
            next_range_end = min(int((i + 1) * bucket_size) + 1, n)

            avg_x = np.mean(x[next_range_start:next_range_end])
            avg_y = np.mean(y[next_range_start:next_range_end])

            # Find point in current bucket that forms largest triangle
            max_area = -1
            max_idx = range_start

            for idx in range(range_start, min(range_end, n)):
                # Calculate triangle area
                area = abs(
                    (prev_x - avg_x) * (y[idx] - prev_y)
                    - (prev_x - x[idx]) * (avg_y - prev_y)
                )

                if area > max_area:
                    max_area = area
                    max_idx = idx

            sampled_x.append(x[max_idx])
            sampled_y.append(y[max_idx])

            prev_x = x[max_idx]
            prev_y = y[max_idx]

        # Always include last point
        sampled_x.append(x[-1])
        sampled_y.append(y[-1])

        return np.array(sampled_x), np.array(sampled_y)

    def downsample_min_max(
        self, x: np.ndarray, y: np.ndarray, threshold: int
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Min-Max downsampling for line plots

        Preserves peaks and valleys by including both min and max
        values in each bucket. Good for preserving signal features.

        Args:
            x: X-axis data
            y: Y-axis data
            threshold: Target number of buckets

        Returns:
            Tuple of (downsampled_x, downsampled_y)
        """
        n = len(x)

        if n <= threshold * 2:
            return x, y

        sampled_x = []
        sampled_y = []

        bucket_size = n // threshold

        for i in range(threshold):
            start = i * bucket_size
            end = start + bucket_size if i < threshold - 1 else n

            bucket_x = x[start:end]
            bucket_y = y[start:end]

            if len(bucket_y) == 0:
                continue

            # Find min and max indices
            min_idx = start + np.argmin(bucket_y)
            max_idx = start + np.argmax(bucket_y)

            # Add min and max in chronological order
            if min_idx < max_idx:
                sampled_x.extend([x[min_idx], x[max_idx]])
                sampled_y.extend([y[min_idx], y[max_idx]])
            else:
                sampled_x.extend([x[max_idx], x[min_idx]])
                sampled_y.extend([y[max_idx], y[min_idx]])

        return np.array(sampled_x), np.array(sampled_y)

    def optimize_data(
        self,
        x: np.ndarray | List[float] | None,
        y: np.ndarray | List[float],
        method: str = "lttb",
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Optimize plot data with automatic downsampling

        Args:
            x: X-axis data (optional, will use indices if None)
            y: Y-axis data
            method: Downsampling method ('decimate', 'lttb', 'minmax')

        Returns:
            Tuple of (optimized_x, optimized_y)
        """
        # Convert to numpy arrays
        y = np.asarray(y)

        if x is None:
            x = np.arange(len(y))
        else:
            x = np.asarray(x)

        # Check if downsampling needed
        if len(y) <= self.downsample_threshold:
            return x, y

        # Determine target points
        target_points = min(self.max_points, len(y))

        # Apply downsampling
        if method == "decimate":
            step = len(y) // target_points
            return x[::step], y[::step]

        elif method == "lttb":
            return self.downsample_lttb(x, y, target_points)

        elif method == "minmax":
            return self.downsample_min_max(x, y, target_points)

        else:
            # Fallback to decimation
            step = len(y) // target_points
            return x[::step], y[::step]

    def track_frame_time(self):
        """
        Track frame rendering time for adaptive performance

        Call this after each plot update to monitor performance.
        """
        now = time.time()

        if self._last_update_time > 0:
            frame_time = (now - self._last_update_time) * 1000  # Convert to ms
            self._frame_times.append(frame_time)

            # Keep only last 60 frames
            if len(self._frame_times) > 60:
                self._frame_times.pop(0)

        self._last_update_time = now

    def get_avg_frame_time(self) -> float:
        """Get average frame time in milliseconds"""
        if not self._frame_times:
            return 0.0
        return sum(self._frame_times) / len(self._frame_times)

    def get_fps(self) -> float:
        """Get current FPS"""
        avg_frame_time = self.get_avg_frame_time()
        if avg_frame_time <= 0:
            return 0.0
        return 1000.0 / avg_frame_time

    def should_reduce_quality(self) -> bool:
        """
        Check if rendering quality should be reduced

        Returns:
            True if average frame time exceeds update interval
        """
        if not self._adaptive_enabled or len(self._frame_times) < 10:
            return False

        avg_frame_time = self.get_avg_frame_time()
        return avg_frame_time > self.update_interval_ms * 1.5

    def adaptive_adjust(self):
        """
        Automatically adjust settings based on performance

        Reduces max_points if rendering is too slow.
        """
        if not self._adaptive_enabled:
            return

        if self.should_reduce_quality():
            # Reduce max points by 20%
            self.max_points = int(self.max_points * 0.8)
            self.downsample_threshold = int(self.downsample_threshold * 0.8)

            print(
                f"⚠️  Adaptive quality reduction: max_points={self.max_points}, "
                f"threshold={self.downsample_threshold}"
            )

    def get_performance_stats(self) -> dict:
        """
        Get current performance statistics

        Returns:
            Dictionary with performance metrics
        """
        return {
            "avg_frame_time_ms": self.get_avg_frame_time(),
            "fps": self.get_fps(),
            "max_points": self.max_points,
            "downsample_threshold": self.downsample_threshold,
            "update_interval_ms": self.update_interval_ms,
            "quality_reduction_needed": self.should_reduce_quality(),
        }


# Convenience function for quick downsampling
def downsample_for_plot(
    data: List[float] | np.ndarray, max_points: int = 5000, method: str = "lttb"
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Quick downsample data for plotting

    Args:
        data: Input data (y-axis values)
        max_points: Maximum points to render
        method: Downsampling method ('decimate', 'lttb', 'minmax')

    Returns:
        Tuple of (x, y) ready for plotting
    """
    optimizer = PlotOptimizer(max_points=max_points, downsample_threshold=max_points)
    return optimizer.optimize_data(None, data, method=method)


# Example usage
if __name__ == "__main__":
    # Generate test data
    n = 100000
    x = np.linspace(0, 100, n)
    y = np.sin(x) + np.random.normal(0, 0.1, n)

    print(f"Original data: {len(y)} points")

    # Test different downsampling methods
    optimizer = PlotOptimizer(max_points=1000, downsample_threshold=5000)

    x_dec, y_dec = optimizer.optimize_data(x, y, method="decimate")
    print(f"Decimation: {len(y_dec)} points")

    x_lttb, y_lttb = optimizer.optimize_data(x, y, method="lttb")
    print(f"LTTB: {len(y_lttb)} points")

    x_mm, y_mm = optimizer.optimize_data(x, y, method="minmax")
    print(f"MinMax: {len(y_mm)} points")

    # Performance stats
    for _ in range(100):
        optimizer.track_frame_time()
        time.sleep(0.02)  # Simulate 50ms update interval

    stats = optimizer.get_performance_stats()
    print(f"\nPerformance stats:")
    for key, value in stats.items():
        print(f"  {key}: {value}")
