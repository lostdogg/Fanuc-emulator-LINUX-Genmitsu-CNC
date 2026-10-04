# Funuc CNC Emulator – Linux Mint Edition

A Fanuc 0i/30i-style CNC G-code interpreter and tool-path visualiser, ported
to run natively on **Linux Mint** (and any Ubuntu/Debian-based system) using
Python 3 and tkinter – no Wine or virtual machine required.

---

## Features

| Feature | Details |
|---|---|
| **G-code editor** | Syntax-friendly text editor with undo/redo |
| **Tool-path viewer** | 2-D XY canvas with zoom (scroll wheel) and pan (drag) |
| **Machine status panel** | Live coordinates (X Y Z), spindle, feed, tool, mode |
| **MDI (Manual Data Input)** | Execute single G-code lines interactively |
| **Message log** | Execution messages, alarms, and status updates |
| **Open / Save programs** | Load `.nc`, `.gcode`, `.ngc`, `.prg`, or `.txt` files |

### Supported G/M codes

| Code | Description |
|---|---|
| G00 | Rapid positioning |
| G01 | Linear interpolation |
| G02 / G03 | Circular interpolation (CW / CCW) |
| G04 | Dwell |
| G17 / G18 / G19 | Plane selection (XY / XZ / YZ) |
| G20 / G21 | Inch / Metric units |
| G28 | Return to reference (home) |
| G40 / G41 / G42 | Tool radius compensation (parsed) |
| G43 / G44 / G49 | Tool length compensation (parsed) |
| G54–G59 | Work coordinate systems (parsed) |
| G80 | Cancel canned cycle |
| G90 / G91 | Absolute / Incremental mode |
| G94 / G95 | Feed per minute / Feed per revolution |
| M00 / M01 | Program stop / Optional stop |
| M02 / M30 | End of program |
| M03 / M04 | Spindle CW / CCW |
| M05 | Spindle stop |
| M06 | Tool change |
| M08 / M09 | Coolant on / off |

---

## Requirements

- Linux Mint 20+ (or Ubuntu 20.04+ / Debian 11+)
- Python 3.8+
- `python3-tk` system package (tkinter)

No third-party Python packages are required.

---

## Installation

```bash
git clone https://github.com/lostdogg/Funuc-emulator-LINUX.git
cd Funuc-emulator-LINUX
chmod +x install.sh
./install.sh
```

`install.sh` will:
1. Verify Python 3.8+ is present
2. Run `sudo apt-get install python3-tk` to install tkinter

---

## Running

```bash
python3 run_emulator.py
# or, after install.sh
./run_emulator.py
```

---

## Keyboard Shortcuts

| Key | Action |
|---|---|
| **F5** | Run program |
| **F6** | Stop program |
| **F** | Fit tool path in view |
| **Ctrl+O** | Open G-code file |
| **Ctrl+S** | Save G-code file |
| **Ctrl+N** | New program |
| **Ctrl+Q** | Quit |
| **Scroll wheel** | Zoom canvas |
| **Drag** | Pan canvas |
| **Enter** (MDI bar) | Execute MDI line |

---

## Project Layout

```
Funuc-emulator-LINUX/
├── funuc_emulator/
│   ├── __init__.py
│   ├── main.py          # Entry point
│   ├── parser.py        # G-code parser
│   ├── machine.py       # CNC machine simulation
│   └── ui/
│       ├── __init__.py
│       ├── app.py       # Main application window
│       ├── canvas.py    # Tool-path canvas widget
│       └── panels.py    # Coordinate & status panels
├── tests/
│   ├── test_parser.py
│   └── test_machine.py
├── run_emulator.py      # Top-level launcher
├── install.sh           # Linux Mint setup script
└── pytest.ini
```

---

## Running Tests

```bash
pip install pytest      # one-time
python -m pytest
```

---

## Differences from the Windows Version

The original emulator (`lostdogg/Funuc-emulator`) was built for Windows.
This port replaces any Windows-specific dependencies with Python-standard
equivalents:

- **GUI**: native `tkinter` (ships with CPython; `python3-tk` on Ubuntu/Mint)
- **No COM / DLL / .NET** dependencies
- **File paths**: POSIX-compatible throughout
- **Packaging**: plain Python package – no `.exe` or installer needed

## Conversational programming / A.G.E.

`funuc_emulator/conversational.py` provides an Auto Geometry Engine that solves
missing profile geometry (line/arc end points, tangents, arc centres,
intersections; `guess` points disambiguate solutions; unsolved elements raise
"Not Calculated") and generates G-code for Drill/Tap/Bore, Bolt Hole, Mill, Arc,
Pocket (rect/circular, with finish pass), Profile (G41/G42 cutter comp),
Conrad corner radiusing, and repeat/rotate/mirror/scale transforms.
