#!/usr/bin/env python3
"""
Automated migration script to apply responsive UI to main_window.py

This script will:
1. Backup the original main_window.py
2. Apply responsive scaling to all hardcoded sizes
3. Update splitter sizing to use ratios
4. Add resizeEvent handler
5. Apply performance optimizations

Usage:
    python APPLY_RESPONSIVE_UI.py [--dry-run] [--backup-dir PATH]
"""

import re
import shutil
from pathlib import Path
from datetime import datetime
import argparse


class ResponsiveUIConverter:
    """Convert hardcoded UI sizes to responsive equivalents"""

    def __init__(self, dry_run=False, backup_dir=None):
        self.dry_run = dry_run
        self.backup_dir = backup_dir or Path("backups")
        self.changes_made = 0

    def backup_file(self, file_path: Path) -> Path:
        """Create backup of original file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = self.backup_dir / f"{file_path.name}.{timestamp}.bak"
        self.backup_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(file_path, backup_path)
        print(f"✅ Backup created: {backup_path}")
        return backup_path

    def add_imports(self, content: str) -> str:
        """Add responsive module imports"""
        import_block = """from ui.responsive import (
    get_responsive_scaling,
    scale,
    scale_button,
    scale_spacing,
    scale_button_size,
    scale_size,
    get_window_size,
    get_splitter_sizes,
)
"""

        # Find where to insert (after other ui imports)
        pattern = r"(from ui\.\w+ import.*?\n)"
        match = re.search(pattern, content)

        if match:
            insert_pos = match.end()
            content = content[:insert_pos] + "\n" + import_block + content[insert_pos:]
            self.changes_made += 1
            print("✅ Added responsive imports")
        else:
            # Fallback: add after PyQt imports
            pattern = r"(from PyQt6\.\w+ import.*?\n)(?!\nfrom PyQt6)"
            match = re.search(pattern, content)
            if match:
                insert_pos = match.end()
                content = content[:insert_pos] + "\n" + import_block + content[insert_pos:]
                self.changes_made += 1
                print("✅ Added responsive imports (after PyQt6)")

        return content

    def convert_window_size(self, content: str) -> str:
        """Convert self.resize() to responsive"""
        pattern = r"self\.resize\(DEFAULT_WINDOW_WIDTH,\s*DEFAULT_WINDOW_HEIGHT\)"
        replacement = """width, height = get_window_size(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT)
        self.resize(width, height)"""

        if re.search(pattern, content):
            content = re.sub(pattern, replacement, content)
            self.changes_made += 1
            print("✅ Converted window resize to responsive")

        return content

    def convert_fixed_sizes(self, content: str) -> str:
        """Convert setFixedSize() calls to responsive"""
        # Pattern: .setFixedSize(width, height)
        pattern = r"(\w+)\.setFixedSize\((\d+),\s*(\d+)\)"

        def replace_fixed_size(match):
            widget = match.group(1)
            width = match.group(2)
            height = match.group(3)
            self.changes_made += 1
            return f"{widget}.setFixedSize(scale_button_size({width}, {height}))"

        content = re.sub(pattern, replace_fixed_size, content)
        count = len(re.findall(pattern, content))
        if count > 0:
            print(f"✅ Converted {count} setFixedSize() calls")

        return content

    def convert_fixed_widths(self, content: str) -> str:
        """Convert setFixedWidth() calls to responsive"""
        # Pattern: .setFixedWidth(value)
        pattern = r"(\w+)\.setFixedWidth\((\d+)\)"

        def replace_fixed_width(match):
            widget = match.group(1)
            width = match.group(2)
            self.changes_made += 1

            # Use scale_button for button-like widgets, scale for others
            if any(
                keyword in widget.lower()
                for keyword in ["btn", "button", "combo", "led"]
            ):
                return f"{widget}.setFixedWidth(scale_button({width}))"
            else:
                return f"{widget}.setFixedWidth(scale({width}))"

        content = re.sub(pattern, replace_fixed_width, content)
        count = len(re.findall(pattern, content))
        if count > 0:
            print(f"✅ Converted {count} setFixedWidth() calls")

        return content

    def convert_spacing_and_margins(self, content: str) -> str:
        """Convert layout spacing and margins to responsive"""
        # Pattern: .setSpacing(value)
        pattern1 = r"\.setSpacing\((\d+)\)"
        content = re.sub(pattern1, r".setSpacing(scale_spacing(\1))", content)

        # Pattern: .setContentsMargins(a, b, c, d)
        pattern2 = r"\.setContentsMargins\((\d+),\s*(\d+),\s*(\d+),\s*(\d+)\)"

        def replace_margins(match):
            self.changes_made += 1
            return f".setContentsMargins(scale_spacing({match.group(1)}), scale_spacing({match.group(2)}), scale_spacing({match.group(3)}), scale_spacing({match.group(4)}))"

        content = re.sub(pattern2, replace_margins, content)

        print("✅ Converted spacing and margins")
        return content

    def convert_splitter_sizes(self, content: str) -> str:
        """Convert splitter.setSizes() to proportional ratios"""
        # Pattern: splitter.setSizes([value1, value2, ...])
        pattern = r"self\.main_splitter\.setSizes\(\[(\d+),\s*(\d+)\]\)"

        match = re.search(pattern, content)
        if match:
            val1 = int(match.group(1))
            val2 = int(match.group(2))
            total = val1 + val2
            ratio1 = val1 / total
            ratio2 = val2 / total

            # Add instance variable for ratios in __init__
            init_pattern = r"(def __init__\(self.*?\):.*?)(def \w+)"
            init_match = re.search(init_pattern, content, re.DOTALL)

            if init_match:
                init_end = init_match.end(1)
                ratio_code = f"\n        self._splitter_ratios = [{ratio1:.2f}, {ratio2:.2f}]  # {val1}px / {val2}px = {ratio1:.0%} / {ratio2:.0%}\n"
                content = content[:init_end] + ratio_code + content[init_end:]
                self.changes_made += 1

            # Replace setSizes call
            replacement = """self._update_splitter_sizes()"""
            content = re.sub(pattern, replacement, content)
            self.changes_made += 1

            print(f"✅ Converted splitter sizes to ratios ({ratio1:.0%}/{ratio2:.0%})")

        return content

    def add_resize_event(self, content: str) -> str:
        """Add resizeEvent handler for dynamic splitter adjustment"""
        resize_method = '''
    def resizeEvent(self, event):
        """Handle window resize - update splitter proportions"""
        super().resizeEvent(event)
        if hasattr(self, '_splitter_ratios') and hasattr(self, 'main_splitter'):
            self._update_splitter_sizes()

    def _update_splitter_sizes(self):
        """Update splitter sizes based on current window height"""
        if not hasattr(self, '_splitter_ratios'):
            return
        total_height = self.height()
        sizes = get_splitter_sizes(total_height, self._splitter_ratios)
        self.main_splitter.setSizes(sizes)
'''

        # Check if resizeEvent already exists
        if "def resizeEvent" not in content:
            # Find end of class (before if __name__ == "__main__")
            pattern = r"(class \w+.*?)((?:\n\nif __name__|$))"
            match = re.search(pattern, content, re.DOTALL)

            if match:
                insert_pos = match.end(1)
                content = content[:insert_pos] + resize_method + content[insert_pos:]
                self.changes_made += 1
                print("✅ Added resizeEvent handler")
        else:
            print("ℹ️  resizeEvent already exists, skipping")

        return content

    def add_responsive_plot_config(self, content: str) -> str:
        """Add responsive plot configuration in __init__"""
        plot_config = """
        # Responsive plot configuration
        self.rs = get_responsive_scaling()
        self.max_plot_points = self.rs.get_recommended_plot_points()
        self.plot_update_interval = self.rs.get_plot_update_interval()
        self.use_opengl = self.rs.should_use_opengl()
        self.downsample_threshold = self.rs.get_downsampling_threshold()

        print(f"📊 Screen profile: {self.rs.profile.name}")
        print(f"📊 Max plot points: {self.max_plot_points}")
        print(f"📊 Update interval: {self.plot_update_interval}ms")
        print(f"📊 Use OpenGL: {self.use_opengl}")
"""

        # Find __init__ method
        pattern = r"(def __init__\(self.*?\):.*?super\(\).__init__\(\))"
        match = re.search(pattern, content, re.DOTALL)

        if match and "self.rs = get_responsive_scaling()" not in content:
            insert_pos = match.end()
            content = content[:insert_pos] + plot_config + content[insert_pos:]
            self.changes_made += 1
            print("✅ Added responsive plot configuration")
        else:
            print("ℹ️  Plot configuration already responsive or not found")

        return content

    def convert_file(self, file_path: Path) -> bool:
        """Convert a single file to responsive UI"""
        if not file_path.exists():
            print(f"❌ File not found: {file_path}")
            return False

        print(f"\n🔧 Processing: {file_path.name}")
        print("=" * 60)

        # Backup original
        if not self.dry_run:
            self.backup_file(file_path)

        # Read content
        content = file_path.read_text(encoding="utf-8")

        # Apply conversions
        content = self.add_imports(content)
        content = self.convert_window_size(content)
        content = self.convert_fixed_sizes(content)
        content = self.convert_fixed_widths(content)
        content = self.convert_spacing_and_margins(content)
        content = self.convert_splitter_sizes(content)
        content = self.add_resize_event(content)
        content = self.add_responsive_plot_config(content)

        # Write back
        if not self.dry_run:
            file_path.write_text(content, encoding="utf-8")
            print(f"\n✅ File updated: {file_path}")
        else:
            print(f"\n🔍 DRY RUN: Would update {file_path}")

        print(f"📝 Total changes: {self.changes_made}")
        return True


def main():
    parser = argparse.ArgumentParser(
        description="Apply responsive UI to AstroLink Mission Control"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show what would be changed without modifying files",
    )
    parser.add_argument(
        "--backup-dir",
        type=Path,
        default=Path("backups"),
        help="Directory for backup files (default: backups/)",
    )
    parser.add_argument(
        "--file",
        type=Path,
        help="Specific file to convert (default: src/ui/main_window.py)",
    )

    args = parser.parse_args()

    # Default file
    target_file = args.file or Path("src/ui/main_window.py")

    print("=" * 60)
    print("🚀 AstroLink Responsive UI Migration Tool")
    print("=" * 60)

    if args.dry_run:
        print("⚠️  DRY RUN MODE - No files will be modified")
        print()

    converter = ResponsiveUIConverter(
        dry_run=args.dry_run, backup_dir=args.backup_dir
    )

    success = converter.convert_file(target_file)

    print("\n" + "=" * 60)
    if success:
        print("✅ Migration complete!")
        if not args.dry_run:
            print(f"📁 Backups saved to: {args.backup_dir}/")
            print("\n📖 Next steps:")
            print("1. Review changes with: git diff")
            print("2. Test on different screen sizes")
            print("3. Check RESPONSIVE_UI_MIGRATION_GUIDE.md for manual fixes")
    else:
        print("❌ Migration failed")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
