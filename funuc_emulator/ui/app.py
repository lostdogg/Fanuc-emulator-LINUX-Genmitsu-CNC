"""Main application window for the Fanuc CNC emulator (Linux Mint edition)."""

from __future__ import annotations

import threading
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk
from typing import Optional

from .. import cad, conversational
from ..machine import Machine
from .canvas import ToolPathCanvas
from .panels import CoordinatePanel, MessageLog, StatusPanel


_DARK_BG = "#1e1e1e"
_EDITOR_BG = "#1a1a2e"
_EDITOR_FG = "#e0e0e0"
_MONO_SM = ("Monospace", 9)

_SAMPLE_PROGRAM = """\
%
O0001 (SAMPLE SQUARE PROFILE)
N10 G21 G90 G94
N20 G28
N30 T01 M06
N40 S1000 M03
N50 G00 X0. Y0.
N60 G01 Z-1. F100.
N70 G01 X50. F200.
N80 G01 Y50.
N90 G01 X0.
N100 G01 Y0.
N110 G00 Z5.
N120 M05
N130 M30
%
"""


class App(tk.Tk):
    """Root window of the Fanuc emulator."""

    def __init__(self) -> None:
        super().__init__()

        self.title("Funuc CNC Emulator – Linux Mint")
        self.configure(bg=_DARK_BG)
        self.geometry("1200x750")
        self.minsize(900, 600)

        self._machine = Machine()
        self._run_thread: Optional[threading.Thread] = None

        self._build_ui()
        self._load_sample()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        self._build_menu()

        # Top toolbar
        toolbar = tk.Frame(self, bg="#2d2d2d", pady=4)
        toolbar.pack(fill="x", padx=4, pady=(4, 0))
        self._build_toolbar(toolbar)

        # Main pane: left editor | centre canvas | right panels
        pane = tk.PanedWindow(self, orient="horizontal", bg=_DARK_BG,
                              sashwidth=4, sashrelief="flat")
        pane.pack(fill="both", expand=True, padx=4, pady=4)

        # --- Left: G-code editor ---
        left = tk.Frame(pane, bg=_DARK_BG)
        pane.add(left, minsize=220, width=280)

        tk.Label(left, text="G-CODE PROGRAM", bg=_DARK_BG, fg="#aaaaaa",
                 font=_MONO_SM).pack(anchor="w", padx=4, pady=(2, 0))

        ed_frame = tk.Frame(left, bg="#111111")
        ed_frame.pack(fill="both", expand=True, padx=4, pady=4)

        ed_scroll = tk.Scrollbar(ed_frame)
        ed_scroll.pack(side="right", fill="y")

        self._editor = tk.Text(
            ed_frame,
            bg=_EDITOR_BG,
            fg=_EDITOR_FG,
            insertbackground="#ffffff",
            font=_MONO_SM,
            relief="flat",
            undo=True,
            yscrollcommand=ed_scroll.set,
        )
        self._editor.pack(fill="both", expand=True)
        ed_scroll.config(command=self._editor.yview)

        # --- Centre: tool path canvas ---
        centre = tk.Frame(pane, bg=_DARK_BG)
        pane.add(centre, minsize=300)

        tk.Label(centre, text="TOOL PATH (XY)", bg=_DARK_BG, fg="#aaaaaa",
                 font=_MONO_SM).pack(anchor="w", padx=4, pady=(2, 0))

        self._canvas = ToolPathCanvas(centre)
        self._canvas.pack(fill="both", expand=True, padx=4, pady=4)

        legend = tk.Label(
            centre,
            text="  ── Rapid   ── Feed   ── Arc CW   ── Arc CCW  "
                 "  Scroll: zoom   Drag: pan",
            bg=_DARK_BG, fg="#555555", font=("Monospace", 8),
        )
        legend.pack(anchor="w", padx=4)

        # --- Right: status panels ---
        right = tk.Frame(pane, bg=_DARK_BG)
        pane.add(right, minsize=200, width=240)

        self._coord_panel = CoordinatePanel(right)
        self._coord_panel.pack(fill="x", padx=4, pady=(4, 2))

        self._status_panel = StatusPanel(right)
        self._status_panel.pack(fill="x", padx=4, pady=2)

        self._log = MessageLog(right)
        self._log.pack(fill="both", expand=True, padx=4, pady=2)

        # --- MDI bar at the bottom ---
        self._build_mdi_bar()

    def _build_menu(self) -> None:
        menubar = tk.Menu(self, bg="#2d2d2d", fg="#ffffff",
                          activebackground="#3d3d3d",
                          activeforeground="#ffffff")

        file_menu = tk.Menu(menubar, tearoff=False, bg="#2d2d2d", fg="#ffffff",
                            activebackground="#3d3d3d",
                            activeforeground="#ffffff")
        file_menu.add_command(label="New", command=self._new_program,
                              accelerator="Ctrl+N")
        file_menu.add_command(label="Open…", command=self._open_file,
                              accelerator="Ctrl+O")
        file_menu.add_command(label="Save…", command=self._save_file,
                              accelerator="Ctrl+S")
        file_menu.add_separator()
        file_menu.add_command(label="Quit", command=self.quit,
                              accelerator="Ctrl+Q")
        menubar.add_cascade(label="File", menu=file_menu)

        machine_menu = tk.Menu(menubar, tearoff=False, bg="#2d2d2d",
                               fg="#ffffff", activebackground="#3d3d3d",
                               activeforeground="#ffffff")
        machine_menu.add_command(label="Run Program", command=self._run_program,
                                 accelerator="F5")
        machine_menu.add_command(label="Stop", command=self._stop_program,
                                 accelerator="F6")
        machine_menu.add_separator()
        machine_menu.add_command(label="Reset Machine", command=self._reset_machine)
        machine_menu.add_command(label="Fit View", command=self._canvas.fit_all,
                                 accelerator="F")
        menubar.add_cascade(label="Machine", menu=machine_menu)

        age_menu = tk.Menu(menubar, tearoff=False, bg="#2d2d2d", fg="#ffffff",
                           activebackground="#094771")
        age_menu.add_command(label="Solve Profile…", command=self._open_age)
        menubar.add_cascade(label="A.G.E.", menu=age_menu)

        cad_menu = tk.Menu(menubar, tearoff=False, bg="#2d2d2d",
                           fg="#ffffff", activebackground="#3d3d3d",
                           activeforeground="#ffffff")
        cad_menu.add_command(label="Import DXF as G-code…",
                             command=self._import_dxf)
        menubar.add_cascade(label="CAD", menu=cad_menu)

        help_menu = tk.Menu(menubar, tearoff=False, bg="#2d2d2d",
                            fg="#ffffff", activebackground="#3d3d3d",
                            activeforeground="#ffffff")
        help_menu.add_command(label="About", command=self._show_about)
        menubar.add_cascade(label="Help", menu=help_menu)

        self.config(menu=menubar)

        # Keyboard shortcuts
        self.bind("<Control-n>", lambda _: self._new_program())
        self.bind("<Control-o>", lambda _: self._open_file())
        self.bind("<Control-s>", lambda _: self._save_file())
        self.bind("<Control-q>", lambda _: self.quit())
        self.bind("<F5>", lambda _: self._run_program())
        self.bind("<F6>", lambda _: self._stop_program())
        self.bind("f", lambda _: self._canvas.fit_all())
        self.bind("F", lambda _: self._canvas.fit_all())

    def _build_toolbar(self, parent: tk.Frame) -> None:
        btn_opts = dict(
            bg="#3d3d3d", fg="#ffffff",
            activebackground="#4d4d4d", activeforeground="#ffffff",
            relief="flat", padx=10, pady=2,
            font=_MONO_SM, cursor="hand2",
        )

        tk.Button(parent, text="▶  Run (F5)", command=self._run_program,
                  **btn_opts).pack(side="left", padx=2)
        tk.Button(parent, text="■  Stop (F6)", command=self._stop_program,
                  **btn_opts).pack(side="left", padx=2)
        tk.Button(parent, text="⟳  Reset", command=self._reset_machine,
                  **btn_opts).pack(side="left", padx=2)
        tk.Button(parent, text="⊞  Fit (F)", command=self._canvas.fit_all,
                  **btn_opts).pack(side="left", padx=2)

        tk.Frame(parent, bg="#2d2d2d", width=2).pack(side="left", padx=6,
                                                       fill="y")

        tk.Button(parent, text="Open", command=self._open_file,
                  **btn_opts).pack(side="left", padx=2)
        tk.Button(parent, text="Save", command=self._save_file,
                  **btn_opts).pack(side="left", padx=2)

    def _build_mdi_bar(self) -> None:
        bar = tk.Frame(self, bg="#2d2d2d", pady=4)
        bar.pack(fill="x", padx=4, pady=(0, 4))

        tk.Label(bar, text="MDI:", bg="#2d2d2d", fg="#aaaaaa",
                 font=_MONO_SM).pack(side="left", padx=(4, 2))

        self._mdi_var = tk.StringVar()
        entry = tk.Entry(bar, textvariable=self._mdi_var,
                         bg="#111111", fg="#00ff88",
                         insertbackground="#ffffff",
                         font=_MONO_SM, relief="flat", width=60)
        entry.pack(side="left", padx=2)
        entry.bind("<Return>", self._execute_mdi)

        tk.Button(bar, text="Execute", command=self._execute_mdi,
                  bg="#3d3d3d", fg="#ffffff",
                  activebackground="#4d4d4d", activeforeground="#ffffff",
                  relief="flat", padx=8, pady=2,
                  font=_MONO_SM, cursor="hand2").pack(side="left", padx=4)

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------

    def _load_sample(self) -> None:
        self._editor.delete("1.0", "end")
        self._editor.insert("1.0", _SAMPLE_PROGRAM)

    def _new_program(self) -> None:
        if messagebox.askyesno("New Program",
                               "Clear current program?", parent=self):
            self._editor.delete("1.0", "end")
            self._reset_machine()

    def _open_file(self) -> None:
        path = filedialog.askopenfilename(
            title="Open G-code file",
            filetypes=[
                ("G-code files", "*.nc *.gcode *.ngc *.txt *.prg"),
                ("All files", "*.*"),
            ],
        )
        if path:
            try:
                with open(path, "r", encoding="utf-8", errors="replace") as fh:
                    content = fh.read()
                self._editor.delete("1.0", "end")
                self._editor.insert("1.0", content)
                self._reset_machine()
                self._log.log(f"Opened: {path}")
            except OSError as exc:
                messagebox.showerror("Error", str(exc), parent=self)

    def _save_file(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Save G-code file",
            defaultextension=".nc",
            filetypes=[
                ("G-code files", "*.nc *.gcode *.ngc *.txt *.prg"),
                ("All files", "*.*"),
            ],
        )
        if path:
            try:
                content = self._editor.get("1.0", "end-1c")
                with open(path, "w", encoding="utf-8") as fh:
                    fh.write(content)
                self._log.log(f"Saved: {path}")
            except OSError as exc:
                messagebox.showerror("Error", str(exc), parent=self)

    def _reset_machine(self) -> None:
        self._stop_program()
        self._machine.reset()
        self._canvas.set_tool_path([], (0.0, 0.0))
        self._log.clear()
        self._log.log("Machine reset.")
        self._refresh_panels()

    def _run_program(self) -> None:
        if self._run_thread and self._run_thread.is_alive():
            return  # Already running
        source = self._editor.get("1.0", "end-1c")
        self._machine.reset()
        self._log.clear()
        self._log.log("Executing program …")
        self._refresh_panels()

        def _worker():
            messages = self._machine.run_program(source)
            self.after(0, self._on_run_complete, messages)

        self._run_thread = threading.Thread(target=_worker, daemon=True)
        self._run_thread.start()

    def _on_run_complete(self, messages: list) -> None:
        for msg in messages:
            self._log.log(msg)
        self._log.log(f"Status: {self._machine.status}")
        pos = self._machine.position
        self._canvas.set_tool_path(
            self._machine.tool_path,
            current_pos=(pos.x, pos.y),
        )
        self._canvas.fit_all()
        self._refresh_panels()

    def _stop_program(self) -> None:
        self._machine.program_stopped = True

    def _execute_mdi(self, _event=None) -> None:
        line = self._mdi_var.get().strip()
        if not line:
            return
        result = self._machine.execute_mdi(line)
        self._log.log(f"MDI> {line}  →  {result}")
        pos = self._machine.position
        self._canvas.set_tool_path(
            self._machine.tool_path,
            current_pos=(pos.x, pos.y),
        )
        self._mdi_var.set("")
        self._refresh_panels()

    def _refresh_panels(self) -> None:
        self._coord_panel.update_from(self._machine)
        self._status_panel.update_from(self._machine)

    def _open_age(self) -> None:
        from .age_dialog import AGEDialog
        AGEDialog(self)

    def _import_dxf(self) -> None:
        path = filedialog.askopenfilename(
            title="Import ASCII DXF",
            filetypes=[("DXF files", "*.dxf"), ("All files", "*.*")],
            parent=self,
        )
        if not path:
            return
        try:
            try:
                with open(path, "r", encoding="utf-8-sig") as fh:
                    source = fh.read()
            except UnicodeDecodeError:
                with open(path, "r", encoding="cp1252") as fh:
                    source = fh.read()
            document = cad.parse_dxf(source)
            features = cad.extract_features(document)
        except (OSError, UnicodeError, cad.CadError) as exc:
            messagebox.showerror("DXF Import", str(exc), parent=self)
            return
        if not features.holes and not features.profiles:
            messagebox.showerror(
                "DXF Import",
                "No circular holes or closed planar profiles were recognized.",
                parent=self,
            )
            return
        summary = (
            f"Units: {document.units} "
            f"({document.unit_scale:g} mm per drawing unit)\n"
            f"Circular holes: {len(features.holes)}\n"
            f"Closed profiles: {len(features.profiles)}\n\n"
            "This import only recognizes circles and closed polylines/line "
            "chains. Source XY coordinates are preserved; no G54/WCS or part "
            "zero transform is applied. Confirm units and origin before "
            "generating code."
        )
        if not messagebox.askyesno("Review DXF Features", summary, parent=self):
            return
        depth = simpledialog.askfloat(
            "DXF Import", "Cut depth (mm; negative):", parent=self,
            minvalue=-100000.0, maxvalue=-0.000001,
        )
        if depth is None:
            return
        feed = simpledialog.askfloat(
            "DXF Import", "Feed rate (mm/min):", parent=self,
            minvalue=0.000001,
        )
        if feed is None:
            return
        try:
            program = "\n".join(cad.features_to_gcode(features, depth, feed)) + "\n"
        except (ValueError, conversational.AGEError) as exc:
            messagebox.showerror("DXF Import", str(exc), parent=self)
            return
        if messagebox.askyesno(
            "DXF Import", "Replace the current editor contents?\n"
            "Choose No to append the generated program.", parent=self,
        ):
            self._editor.edit_separator()
            self._editor.delete("1.0", "end")
            self._editor.insert("1.0", program)
        else:
            self._editor.edit_separator()
            existing = self._editor.get("1.0", "end-1c")
            combined = cad.append_gcode_program(existing, program)
            self._editor.delete("1.0", "end")
            self._editor.insert("1.0", combined)
        self._editor.edit_separator()
        self._log.log(
            f"Imported DXF features from {path}: "
            f"{len(features.holes)} holes, {len(features.profiles)} profiles."
        )

    def _show_about(self) -> None:
        messagebox.showinfo(
            "About Funuc CNC Emulator",
            "Funuc CNC Emulator – Linux Mint Edition\n"
            "Version 1.0.0\n\n"
            "A Fanuc 0i/30i-style G-code interpreter and tool-path\n"
            "visualiser for Linux Mint.\n\n"
            "Supports G00/G01/G02/G03, G90/G91, G20/G21,\n"
            "M03/M04/M05/M06/M30 and more.\n\n"
            "Controls:\n"
            "  F5 – Run program\n"
            "  F6 – Stop\n"
            "  F  – Fit view\n"
            "  Scroll wheel – Zoom\n"
            "  Drag – Pan\n"
            "  MDI bar – Execute single G-code line",
            parent=self,
        )
