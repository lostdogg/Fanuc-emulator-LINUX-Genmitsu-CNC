"""A.G.E. dialog: enter known profile data, see solved/guess/unsolved fields."""
from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from .. import conversational as conv

HELP = ("One element per line, e.g.\n"
        "line angle=0\narc r=2 cw\nline angle=90 x=10 y=10\n"
        "Keys: x y angle r cx cy guess=x,y; flags: cw ccw tangent")


class AGEDialog(tk.Toplevel):
    def __init__(self, master) -> None:
        super().__init__(master)
        self.title("A.G.E. – Auto Geometry Engine")
        self.configure(bg="#1e1e1e")
        top = tk.Frame(self, bg="#1e1e1e")
        top.pack(fill=tk.X, padx=6, pady=4)
        tk.Label(top, text="Start X", bg="#1e1e1e", fg="white").pack(side=tk.LEFT)
        self._sx = ttk.Entry(top, width=8)
        self._sx.insert(0, "0")
        self._sx.pack(side=tk.LEFT, padx=4)
        tk.Label(top, text="Y", bg="#1e1e1e", fg="white").pack(side=tk.LEFT)
        self._sy = ttk.Entry(top, width=8)
        self._sy.insert(0, "0")
        self._sy.pack(side=tk.LEFT, padx=4)
        self._input = tk.Text(self, width=60, height=8, bg="#252526", fg="white",
                              insertbackground="white")
        self._input.pack(fill=tk.X, padx=6)
        self._input.bind("<KeyRelease>", lambda _e: self.solve())
        tk.Label(self, text=HELP, justify=tk.LEFT, bg="#1e1e1e", fg="#888").pack(anchor=tk.W, padx=6)
        self._out = tk.Text(self, width=60, height=14, bg="#111", fg="white", state=tk.DISABLED)
        for name, colour in conv.STATUS_COLOURS.items():
            self._out.tag_configure(name, foreground={"white": "#ffffff", "green": "#00e000",
                                                      "orange": "#ffa500", "red": "#ff5555"}[colour])
        self._out.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)
        self._status = tk.Label(self, text="", bg="#1e1e1e", fg="white")
        self._status.pack(anchor=tk.W, padx=6, pady=(0, 4))

    def solve(self) -> None:
        self._out.configure(state=tk.NORMAL)
        self._out.delete("1.0", tk.END)
        try:
            start = (float(self._sx.get()), float(self._sy.get()))
            els = conv.parse_elements(self._input.get("1.0", tk.END))
            rows = conv.field_report(start, els)
            for r in rows:
                self._out.insert(tk.END, conv.format_report([r]) + "\n", r["status"])
            ok = bool(rows) and conv.fully_constrained(rows)
            self._status.configure(text="Fully constrained" if ok else "Not fully constrained",
                                   fg="#00e000" if ok else "#ffa500")
        except (ValueError, conv.AGEError) as exc:
            self._out.insert(tk.END, f"Error: {exc}\n", conv.NOT_CALCULATED)
            self._status.configure(text="", fg="white")
        self._out.configure(state=tk.DISABLED)
