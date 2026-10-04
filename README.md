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

Adaptive milling helpers are also available from this module:
`radial_thinning_factor(ae, tool_dia)` calculates the radial chip-thinning
factor, `adaptive_feed_rate(chip_load, flutes, rpm, ae, tool_dia)` calculates
the compensated table feed, and `adaptive_rect_pocket(...)` generates a
rounded-loop rectangular pocket roughing path with a radial stepover below 50%
and optional axial depth-per-pass control. Review and verify generated G-code
against the actual machine, tooling, material, and workholding before machining.

The **A.G.E. > Solve Profile…** menu opens a solver dialog: enter only the known
values per element (`line angle=0`, `arc r=2 cw`, `guess=x,y`, `tangent`) and
fields are coloured live – white *Given*, green *Calculated*, orange *Guess*,
red *Not Calculated*. Supported: line/arc tangency, arc tangent to two lines,
line tangent to two arcs, line/arc intersections, and chamfer/radius corners.

## CAD import and stock simulation

The **CAD > Import DXF as G-code…** command reads ASCII DXF `LINE`, `ARC`,
`CIRCLE`, and `LWPOLYLINE` entities. Declared DXF units are converted to
millimeters; polyline bulges are tessellated, closed boundaries and near-
connected line chains become profile paths, and circles become drill positions.
The import asks for a cut depth and feed before placing generated G-code in the
editor. Inspect and simulate the program before use.
Profiles that meet at ambiguous branch junctions are skipped by the current
line-chain recognizer. Generated profile paths follow the CAD boundary at the
tool center and do not apply cutter-radius compensation; choose tooling/offsets
accordingly and verify the resulting program before machining.

`funuc_emulator/simulation.py` provides a deterministic CPU voxel-stock
reference simulator for sampled end-mill paths, reporting removed stock, rapid
contacts, and flute-length overflows. It is not a machine-verification system:
it does not model fixtures, toolholders, machine kinematics, surface scallops,
or interactive 3D rendering. DWG, Parasolid, NURBS, arbitrary 3D CAD/B-rep,
entity picking, and GPU simulation are not currently supported. DXF layer
semantics, G54/WCS or part-zero transformations, and bulged polylines'
tessellated geometry should be reviewed before generating machine code.

`funuc_emulator/surface.py` adds numerical multi-surface path mathematics:
parametric surface normals, ball/flat/toroidal cutter-center offsets, affine
WCS transforms, curvature-based scallop/stepover calculations, adaptive
chord-error subdivision, sampled surface-clearance queries, and sampled
vertical drop-cutter height. These operations approximate surfaces on finite
grids and are not certified gouge detection or production CNC verification.
The helpers are library APIs and are not yet connected to the DXF UI or machine
simulation.

### Production CAM development and assurance

Future production-oriented geometry and verification work should preserve a
usable CPU reference path and make accelerated or external backends optional.
For B-rep support, Open CASCADE through its Python bindings (`OCP`) is a
candidate geometry-kernel integration; GPU verification could use Vulkan
compute with GLSL/SPIR-V or OpenCL. These are architectural candidates, not
implemented or supported features. Backend results would need comparison
against defined CPU reference cases and documented numeric tolerances; the
supported geometry formats and target hardware must be stated explicitly.

Production readiness requires more than adding a geometry kernel or GPU
backend. Establish intended use and target jurisdictions, assess hazards and
limitations, trace verification requirements to tests and evidence, validate
each supported backend/configuration, and retain reproducible records of
inputs, toolpaths, software and dependency versions, settings, results, and
warnings. Failed, incomplete, or unsupported checks must not be reported as
verified. Changes and releases need review and regression evidence.

This project is not certified for production machining or as a machine safety
system. Offline toolpath generation or simulation does not replace machine
guarding, controller safety functions, or operator checks. Determine applicable
standards and conformity requirements for the product's actual role, market,
and use; standards such as ISO 12100, ISO 13849-1, or IEC 61508 may be relevant
in particular contexts, but mentioning them does not establish compliance.
