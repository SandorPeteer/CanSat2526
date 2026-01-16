# -*- mode: python ; coding: utf-8 -*-

block_cipher = None

import os
import sys

base_dir = os.path.dirname(os.path.abspath(sys.argv[0]))
assets_dir = os.path.join(base_dir, "assets")
icon_icns = os.path.join(assets_dir, "astrolink_icon.icns")
icon_ico = os.path.join(assets_dir, "astrolink_icon.ico")
icon_png = os.path.join(assets_dir, "astrolink_icon.png")

if sys.platform == "darwin":
    icon_path = icon_icns if os.path.exists(icon_icns) else icon_png
else:
    icon_path = icon_ico if os.path.exists(icon_ico) else icon_png

a = Analysis(
    ["src/main.py"],
    pathex=["src"],
    binaries=[],
    datas=[(icon_png, "assets")],
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)
pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name="AstroLink",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    icon=icon_path,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

app = BUNDLE(
    exe,
    name="AstroLink.app",
    icon=icon_path if sys.platform == "darwin" else None,
    bundle_identifier=None,
)
