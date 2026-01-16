# Hogyan Indítsd az AstroLink Mission Control-t

## 🚀 Gyors Start (iMac M1)

### 1. Első Indítás (Dependency Telepítés)

```bash
cd /Users/petersandor/Documents/PlatformIO/Projects/LORI_LORA/astrolink-desktop

# Első futtatás: telepíti a dependencies-eket
./run.sh
```

Ez automatikusan:
- ✅ Létrehozza a `venv` virtuális környezetet
- ✅ Telepíti a PyQt6, pyqtgraph, numpy, pandas, scipy, pyserial csomagokat
- ✅ Elindítja az alkalmazást

### 2. Következő Indítások (Gyorsabb)

```bash
# Egyszerűen futtasd:
./run.sh

# VAGY az új LAUNCH.sh-t (teszteli a responsive UI-t is):
./LAUNCH.sh
```

---

## 🧪 Responsive UI Tesztelés

### Standalone Teszt (UI nélkül)

```bash
cd astrolink-desktop
source venv/bin/activate
cd src

# Teszt 1: Responsive scaling detection
python3 -m ui.responsive

# Teszt 2: Plot optimizer
python3 -m ui.plot_optimizer
```

**Kimenet példa:**
```
Screen profile: xlarge
Scale factor: 1.5
Window size: (2160, 1350)
Button (42x42) scaled: QSize(42, 42)
Splitter [180, 720] scaled: [270, 1080]
Recommended plot points: 20000
Use OpenGL: True
Plot update interval: 33ms
```

---

## 📁 Fájlstruktúra

```
astrolink-desktop/
├── run.sh              ← HASZNÁLD EZT (hivatalos launcher)
├── LAUNCH.sh           ← ÚJ! Responsive UI teszttel
├── requirements.txt    ← Python dependencies
├── venv/              ← Virtual environment (auto-created)
├── src/
│   ├── main.py        ← Entry point
│   ├── config.py      ← Konfiguráció
│   ├── ui/
│   │   ├── responsive.py      ← ÚJ! Responsive scaling
│   │   ├── plot_optimizer.py  ← ÚJ! Plot performance
│   │   ├── main_window.py     ← Fő ablak
│   │   ├── mission_dashboard.py
│   │   └── ...
│   ├── core/          ← Serial handler, decoder, stb.
│   └── utils/         ← Helper functions
└── APPLY_RESPONSIVE_UI.py  ← Migration script
```

---

## 🖥️ Mi Fog Történni iMac M1-en?

### Auto-Detection

Az app indításakor automatikusan detektálja:

```
📊 Screen Profile: xlarge
📏 Scale Factor: 1.5×
🖱️  Button Scale: 1.0×
📈 Max Plot Points: 20,000
⏱️  Update Interval: 33ms
🎮 OpenGL: Enabled
🖥️  Window Size: 2160×1350
```

### Eredmény

- **Ablak méret**: ~2160×1350 px (90% a képernyőből, kényelmes margókkal)
- **Gombok**: Normál méret (nem túl nagyok)
- **Plot minőség**: 20,000 pont max (4× több mint kis laptopon!)
- **Update rate**: 30 FPS (simább mint kis képernyőn)
- **OpenGL**: Bekapcsolva (gyorsabb rendering)

---

## 🔧 Opcionális: Responsive UI Alkalmazása

Ha alkalmazni akarod a responsive UI-t a main_window.py-ra:

```bash
cd astrolink-desktop

# DRY RUN (csak mutatja, mit változtatna)
python3 APPLY_RESPONSIVE_UI.py --dry-run

# Alkalmaz (backup készül automatikusan)
python3 APPLY_RESPONSIVE_UI.py

# Ellenőrzés
git diff src/ui/main_window.py
```

**Backup helye**: `astrolink-desktop/backups/main_window.py.YYYYMMDD_HHMMSS.bak`

---

## 🐛 Hibaelhárítás

### "Python 3 not found"

```bash
# Ellenőrizd a Python verziót
python3 --version  # Legalább 3.10 kell

# Ha nincs telepítve:
brew install python@3.11
```

### "ModuleNotFoundError: No module named 'PyQt6'"

```bash
# Újratelepítés
rm -rf venv/
./run.sh
```

### "Cannot connect to display"

Ez SSH-n keresztül történik? A GUI alkalmazáshoz X11 forwarding kell:

```bash
ssh -X user@host
./run.sh
```

### "Qt platform plugin not found"

```bash
# macOS fix:
export QT_QPA_PLATFORM=cocoa
./run.sh
```

---

## 📊 Teljesítmény Monitoring

### Console Output

Az app indításakor látni fogod:

```
📊 Screen profile: xlarge
📊 Max plot points: 20000
📊 Update interval: 33ms
📊 Use OpenGL: True
```

### Runtime Stats

Az app futása közben a console-ban:

```python
# Ha később hozzáadod a main_window.py-hoz:
FPS: 29.8, Avg frame: 33.5ms
```

---

## 🎨 UI Módok

Az AstroLink több UI módot támogat:

1. **Main Window** (alapértelmezett) - Fájl alapú playback
2. **Mission Dashboard** - Professzionális dashboard gauge-okkal
3. **Open Mission Control** - NASA/ESA stílusú UI
4. **Professional Mission Control** - Fejlett multi-view

**Módváltás**: A felső menüben "Mode" dropdown

---

## ⚙️ Beállítások

### OpenGL ki/be kapcsolása

```bash
# Kézi override (config.py-ban):
USE_OPENGL = False  # Kikapcsol
USE_OPENGL = True   # Bekapcsol
```

### Plot pontok számának állítása

```bash
# config.py:
MAX_PLOT_POINTS = 5000   # Kevesebb (lassabb gépekhez)
MAX_PLOT_POINTS = 20000  # Több (gyors gépekhez)
```

### Ablak méret fix beállítása

```python
# main_window.py-ban:
# Responsive:
width, height = get_window_size(1440, 900)

# Fix (override):
width, height = 2560, 1440  # Full screen iMac-en
```

---

## 🚀 Következő Lépések

1. **Indítsd el először**: `./run.sh`
2. **Nézd meg a detektált beállításokat** (console output)
3. **Tesztelj különböző módokban** (Mode dropdown)
4. **Opcionális**: Alkalmazd a responsive UI-t (`APPLY_RESPONSIVE_UI.py`)

---

## 📞 Support

Ha valami nem működik:

1. Ellenőrizd a console output-ot (hibák láthatók)
2. Nézd meg a [RESPONSIVE_UI_SUMMARY.md](RESPONSIVE_UI_SUMMARY.md)-t
3. Debug mód:
   ```bash
   DEBUG=1 ./run.sh
   ```

**Élvezd! 🎉**
