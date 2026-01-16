"""
Responsive UI Utilities for AstroLink Mission Control

Provides automatic scaling and responsive layout helpers for different screen sizes.
Supports:
- Small laptops (1366x768, 1280x800)
- Standard displays (1920x1080, 1440x900)
- Large displays (2560x1440, 3840x2160)
"""

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QSize
from dataclasses import dataclass
from typing import Tuple
import math


@dataclass
class ScreenProfile:
    """Screen size profile with responsive scaling factors"""

    name: str
    min_width: int
    min_height: int
    scale_factor: float
    button_scale: float
    font_scale: float
    spacing_scale: float


# Screen profiles sorted by size (smallest to largest)
SCREEN_PROFILES = [
    ScreenProfile(
        name="tiny",
        min_width=0,
        min_height=0,
        scale_factor=0.65,
        button_scale=0.75,
        font_scale=0.85,
        spacing_scale=0.75,
    ),
    ScreenProfile(
        name="small",
        min_width=1280,
        min_height=720,
        scale_factor=0.8,
        button_scale=0.85,
        font_scale=0.9,
        spacing_scale=0.85,
    ),
    ScreenProfile(
        name="medium",
        min_width=1440,
        min_height=900,
        scale_factor=1.0,
        button_scale=1.0,
        font_scale=1.0,
        spacing_scale=1.0,
    ),
    ScreenProfile(
        name="large",
        min_width=1920,
        min_height=1080,
        scale_factor=1.2,
        button_scale=1.0,
        font_scale=1.0,
        spacing_scale=1.0,
    ),
    ScreenProfile(
        name="xlarge",
        min_width=2200,  # Retina iMacs (2240×1260 physical, ~2240×1171 available)
        min_height=1100,  # Account for macOS dock/menubar (171px - 90px = ~1080px usable)
        scale_factor=1.5,
        button_scale=1.0,
        font_scale=1.1,
        spacing_scale=1.0,
    ),
]


class ResponsiveScaling:
    """Automatic responsive scaling based on screen size"""

    def __init__(self):
        self._profile: ScreenProfile = SCREEN_PROFILES[2]  # default: medium
        self._detect_screen_size()

    def _detect_screen_size(self):
        """Detect current screen size and select appropriate profile"""
        app = QApplication.instance()
        if not app:
            return

        # Get primary screen geometry
        screen = app.primaryScreen()
        if not screen:
            return

        geometry = screen.availableGeometry()
        width = geometry.width()
        height = geometry.height()

        # Find matching profile (largest that fits)
        for profile in reversed(SCREEN_PROFILES):
            if width >= profile.min_width and height >= profile.min_height:
                self._profile = profile
                break

    @property
    def profile(self) -> ScreenProfile:
        """Get current screen profile"""
        return self._profile

    def scale(self, value: int) -> int:
        """Scale a dimension by the current profile's scale factor"""
        return round(value * self._profile.scale_factor)

    def scale_button(self, value: int) -> int:
        """Scale button dimensions (separate factor for better UX)"""
        return round(value * self._profile.button_scale)

    def scale_font(self, value: int) -> int:
        """Scale font sizes"""
        return round(value * self._profile.font_scale)

    def scale_spacing(self, value: int) -> int:
        """Scale layout spacing"""
        return max(1, round(value * self._profile.spacing_scale))

    def scale_size(self, width: int, height: int) -> QSize:
        """Scale a QSize"""
        return QSize(self.scale(width), self.scale(height))

    def scale_button_size(self, width: int, height: int) -> QSize:
        """Scale button QSize"""
        return QSize(self.scale_button(width), self.scale_button(height))

    def get_window_size(
        self, base_width: int = 1440, base_height: int = 900
    ) -> Tuple[int, int]:
        """
        Get responsive window size based on screen

        Args:
            base_width: Base window width (at medium profile)
            base_height: Base window height (at medium profile)

        Returns:
            Tuple of (width, height) scaled to current screen
        """
        app = QApplication.instance()
        if not app:
            return (base_width, base_height)

        screen = app.primaryScreen()
        if not screen:
            return (base_width, base_height)

        geometry = screen.availableGeometry()
        screen_width = geometry.width()
        screen_height = geometry.height()

        # Calculate proportional size (90% of screen for comfortable margins)
        target_width = min(self.scale(base_width), int(screen_width * 0.9))
        target_height = min(self.scale(base_height), int(screen_height * 0.9))

        return (target_width, target_height)

    def get_splitter_sizes(
        self, total_size: int, ratios: list[float]
    ) -> list[int]:
        """
        Calculate splitter sizes based on proportional ratios

        Args:
            total_size: Total available size
            ratios: List of size ratios (e.g., [0.2, 0.8] for 20%/80% split)

        Returns:
            List of absolute sizes
        """
        if not ratios:
            return []

        # Normalize ratios to sum to 1.0
        total_ratio = sum(ratios)
        normalized = [r / total_ratio for r in ratios]

        # Convert to absolute sizes
        sizes = [round(total_size * r) for r in normalized]

        # Ensure total matches (adjust last element for rounding)
        diff = total_size - sum(sizes)
        if diff != 0 and sizes:
            sizes[-1] += diff

        return sizes

    def get_recommended_plot_points(self) -> int:
        """
        Get recommended max plot points based on screen profile

        Returns:
            Maximum number of plot points for optimal performance
        """
        points_map = {
            "tiny": 2000,
            "small": 5000,
            "medium": 10000,
            "large": 15000,
            "xlarge": 20000,
        }
        return points_map.get(self._profile.name, 10000)

    def should_use_opengl(self) -> bool:
        """
        Determine if OpenGL should be used for plotting

        Returns:
            True if screen is large enough to benefit from OpenGL
        """
        # OpenGL is beneficial for large screens with many plot points
        # but can cause issues on low-end laptops
        return self._profile.name in ["large", "xlarge"]

    def get_plot_update_interval(self) -> int:
        """
        Get recommended plot update interval based on performance

        Returns:
            Update interval in milliseconds
        """
        interval_map = {
            "tiny": 100,  # 10 Hz on very small screens
            "small": 67,  # 15 Hz
            "medium": 50,  # 20 Hz (default)
            "large": 33,  # 30 Hz
            "xlarge": 33,  # 30 Hz
        }
        return interval_map.get(self._profile.name, 50)

    def get_downsampling_threshold(self) -> int:
        """
        Get threshold for automatic plot downsampling

        Returns:
            Number of points before downsampling kicks in
        """
        threshold_map = {
            "tiny": 1000,
            "small": 2500,
            "medium": 5000,
            "large": 10000,
            "xlarge": 15000,
        }
        return threshold_map.get(self._profile.name, 5000)


# Global singleton instance
_responsive_scaling: ResponsiveScaling | None = None


def get_responsive_scaling() -> ResponsiveScaling:
    """
    Get the global ResponsiveScaling instance

    Returns:
        ResponsiveScaling: Singleton instance
    """
    global _responsive_scaling
    if _responsive_scaling is None:
        _responsive_scaling = ResponsiveScaling()
    return _responsive_scaling


def reset_responsive_scaling():
    """Reset and re-detect screen size (call after screen change)"""
    global _responsive_scaling
    _responsive_scaling = None


# Convenience functions for common operations
def scale(value: int) -> int:
    """Scale a dimension value"""
    return get_responsive_scaling().scale(value)


def scale_button(value: int) -> int:
    """Scale a button dimension"""
    return get_responsive_scaling().scale_button(value)


def scale_font(value: int) -> int:
    """Scale a font size"""
    return get_responsive_scaling().scale_font(value)


def scale_spacing(value: int) -> int:
    """Scale layout spacing"""
    return get_responsive_scaling().scale_spacing(value)


def scale_size(width: int, height: int) -> QSize:
    """Scale a QSize"""
    return get_responsive_scaling().scale_size(width, height)


def scale_button_size(width: int, height: int) -> QSize:
    """Scale a button QSize"""
    return get_responsive_scaling().scale_button_size(width, height)


def get_window_size(base_width: int = 1440, base_height: int = 900) -> Tuple[int, int]:
    """Get responsive window size"""
    return get_responsive_scaling().get_window_size(base_width, base_height)


def get_splitter_sizes(total_size: int, ratios: list[float]) -> list[int]:
    """Calculate splitter sizes"""
    return get_responsive_scaling().get_splitter_sizes(total_size, ratios)


# Example usage:
if __name__ == "__main__":
    from PyQt6.QtWidgets import QApplication
    import sys

    app = QApplication(sys.argv)

    rs = get_responsive_scaling()
    print(f"Screen profile: {rs.profile.name}")
    print(f"Scale factor: {rs.profile.scale_factor}")
    print(f"Window size: {get_window_size()}")
    print(f"Button (42x42) scaled: {scale_button_size(42, 42)}")
    print(f"Splitter [180, 720] scaled: {get_splitter_sizes(900, [0.2, 0.8])}")
    print(f"Recommended plot points: {rs.get_recommended_plot_points()}")
    print(f"Use OpenGL: {rs.should_use_opengl()}")
    print(f"Plot update interval: {rs.get_plot_update_interval()}ms")
