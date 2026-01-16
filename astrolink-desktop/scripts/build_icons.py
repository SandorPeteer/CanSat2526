#!/usr/bin/env python3
from pathlib import Path

from PIL import Image


BASE_DIR = Path(__file__).resolve().parent.parent
ASSETS_DIR = BASE_DIR / "assets"
SRC_PNG = ASSETS_DIR / "astrolink_icon.png"
OUT_ICO = ASSETS_DIR / "astrolink_icon.ico"
OUT_ICNS = ASSETS_DIR / "astrolink_icon.icns"


def main() -> None:
    img = Image.open(SRC_PNG).convert("RGBA")

    # Windows icon (multi-size)
    ico_sizes = [(16, 16), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)]
    img.save(OUT_ICO, sizes=ico_sizes)

    # macOS icon
    icns_sizes = [(16, 16), (32, 32), (64, 64), (128, 128), (256, 256), (512, 512)]
    img.save(OUT_ICNS, sizes=icns_sizes)


if __name__ == "__main__":
    main()
