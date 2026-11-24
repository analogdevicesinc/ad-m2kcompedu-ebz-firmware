#!/usr/bin/env python3
#
# ebz_demo.py
#
# Copyright (c) 2026 Analog Devices, Inc.
#
# SPDX-License-Identifier: Apache-2.0
#

import argparse
import math
import os
import random
import sys
import time
import tkinter as tk
from collections import deque
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

try:
    from serial.tools import list_ports
except Exception:
    list_ports = None

from adc_capture import (
    AdcCaptureWorker, ADC_BUF_SIZE, ADC_SAMPLE_RATE,
    FILTER_TYPES, biquad_coeffs, apply_biquad, compute_fft_db,
)


_HERE = os.path.dirname(os.path.abspath(__file__))
for _sub in ("motor_control", "iio"):
    _p = os.path.join(_HERE, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)
import ffb
import siggen

WAVE_NAMES = siggen.WAVE_NAMES


ASSET_DIR = os.path.dirname(os.path.abspath(__file__))
LOGO_FILE = os.path.join(ASSET_DIR, "adi_logo.png")
AMP_FILE = os.path.join(ASSET_DIR, "adi_amp.png")

DEFAULT_EDS = os.path.join(ASSET_DIR, "motor_control", "adrd3161.eds")


BG      = "#eef1f5"
CARD    = "#ffffff"
FIELD   = "#ffffff"
TROUGH  = "#dfe3e9"
BORDER  = "#d3d9e0"
FG      = "#1f2a37"
MUTED   = "#66707e"
ACCENT  = "#0067b9"
ACCENT2 = "#00568f"
GREEN   = "#1b8a3e"
RED     = "#c5221f"
AMBER   = "#d98a08"


WARN_BG     = "#fff4e5"
WARN_FG     = "#8a5a00"
WARN_BORDER = "#f0d9a8"


PANEL   = "#0d1522"
GRID    = "#22303f"
ZERO    = "#5a6675"
T_VEL   = "#4a9eff"
T_TGT   = "#9aa4b2"
T_CUR   = "#ff9e40"
T_UQ    = "#3ddc84"


T_PRE   = "#4a9eff"
T_POST  = "#ff9e40"
T_FILT  = "#3ddc84"


def int_flexible(x):
    return int(x, 0)


class Scene(tk.Canvas):

    W, H = 440, 232
    TS = (0.0, 0.18, 0.40, 0.68, 1.0)
    CRUISE = 0.85

    def __init__(self, master):
        super().__init__(master, width=self.W, height=self.H,
                         bg="#6ec6f0", highlightthickness=0)
        self.horizon = int(self.H * 0.34)
        self._scroll = 0.0

        self.create_oval(self.W - 96, self.horizon - 58, self.W - 30,
                         self.horizon + 8, fill="#fff2b8", outline="")
        self.create_oval(-40, self.horizon - 34, 150, self.horizon + 30,
                         fill="#3f9e51", outline="")
        self.create_oval(self.W - 170, self.horizon - 26, self.W + 40,
                         self.horizon + 30, fill="#3f9e51", outline="")
        self.create_rectangle(0, self.horizon, self.W, self.H,
                              fill="#48a85a", outline="")

        self._road = self.create_polygon(*([0] * (len(self.TS) * 4)),
                                         fill="#3c424b", outline="")
        self._seams = [self.create_line(0, 0, 0, 0, fill="#4a515a")
                       for _ in range(7)]
        self._edge_l = self.create_line(*([0] * (len(self.TS) * 2)),
                                        fill="#eef2f4", width=3)
        self._edge_r = self.create_line(*([0] * (len(self.TS) * 2)),
                                        fill="#eef2f4", width=3)
        self._dashes = [self.create_rectangle(0, 0, 0, 0, fill="#f5c542",
                                              outline="") for _ in range(6)]

        self._car = {
            "shadow": self.create_oval(0, 0, 0, 0, fill="#2a2e34", outline=""),
            "wl": self.create_rectangle(0, 0, 0, 0, fill="#16181c", outline=""),
            "wr": self.create_rectangle(0, 0, 0, 0, fill="#16181c", outline=""),
            "body": self.create_polygon(0, 0, 0, 0, 0, 0, 0, 0, fill="#2f6fd0",
                                        outline="#1c4a8f"),
            "bumper": self.create_rectangle(0, 0, 0, 0, fill="#3a3f45", outline=""),
            "cabin": self.create_polygon(0, 0, 0, 0, 0, 0, 0, 0, fill="#2559a8",
                                         outline=""),
            "glass": self.create_polygon(0, 0, 0, 0, 0, 0, 0, 0, fill="#263038",
                                         outline=""),
            "tl": self.create_rectangle(0, 0, 0, 0, fill="#ff6b4a", outline=""),
            "tr": self.create_rectangle(0, 0, 0, 0, fill="#ff6b4a", outline=""),
        }

    def _cx(self, tt, top_c, bot_c):
        w = (1.0 - tt) ** 1.6
        return bot_c + (top_c - bot_c) * w

    def _half(self, tt):
        return 8.0 + 167.0 * tt

    def _edge(self, tt, side, top_c, bot_c):
        x = self._cx(tt, top_c, bot_c) + side * self._half(tt)
        y = self.horizon + tt * (self.H - self.horizon)
        return x, y

    def _place_car(self, cx, cy, s):
        lean = max(-10.0, min(10.0, s * 10.0))
        self.coords(self._car["shadow"], cx - 48, cy + 18, cx + 48, cy + 34)
        self.coords(self._car["wl"], cx - 52, cy + 8, cx - 42, cy + 26)
        self.coords(self._car["wr"], cx + 42, cy + 8, cx + 52, cy + 26)
        self.coords(self._car["body"], cx - 38, cy - 6, cx + 38, cy - 6,
                    cx + 46, cy + 22, cx - 46, cy + 22)
        self.coords(self._car["bumper"], cx - 42, cy + 16, cx + 42, cy + 26)
        self.coords(self._car["cabin"], cx - 30 + lean, cy - 30,
                    cx + 30 + lean, cy - 30, cx + 34, cy - 6, cx - 34, cy - 6)
        self.coords(self._car["glass"], cx - 24 + lean, cy - 27,
                    cx + 24 + lean, cy - 27, cx + 26, cy - 10, cx - 26, cy - 10)
        self.coords(self._car["tl"], cx - 44, cy + 2, cx - 30, cy + 13)
        self.coords(self._car["tr"], cx + 30, cy + 2, cx + 44, cy + 13)

    def update_view(self, angle_deg, lock_deg, rumble, t, dt):
        s = max(-1.0, min(1.0, angle_deg / max(1.0, lock_deg)))
        top_c = self.W / 2 + s * 150.0
        bot_c = self.W / 2 + s * 22.5

        poly = []
        for tt in self.TS:
            poly += self._edge(tt, -1, top_c, bot_c)
        for tt in reversed(self.TS):
            poly += self._edge(tt, +1, top_c, bot_c)
        self.coords(self._road, *poly)
        left, right = [], []
        for tt in self.TS:
            left += self._edge(tt, -1, top_c, bot_c)
            right += self._edge(tt, +1, top_c, bot_c)
        self.coords(self._edge_l, *left)
        self.coords(self._edge_r, *right)

        self._scroll = (self._scroll + self.CRUISE * min(dt, 0.1)) % 1.0
        n = len(self._seams)
        for i, seam in enumerate(self._seams):
            tt = (((i + self._scroll) / n)) ** 2
            lx, ly = self._edge(tt, -1, top_c, bot_c)
            rx, ry = self._edge(tt, +1, top_c, bot_c)
            self.coords(seam, lx, ly, rx, ry)
            self.itemconfigure(seam, width=max(1, int(1 + 3 * tt)))
        m = len(self._dashes)
        for i, dash in enumerate(self._dashes):
            tt = (((i + self._scroll) / m)) ** 2
            cxx = self._cx(tt, top_c, bot_c)
            yy = self.horizon + tt * (self.H - self.horizon)
            hw, hh = max(1.0, 3.0 * tt + 1.0), max(1.0, 11.0 * tt)
            self.coords(dash, cxx - hw, yy - hh, cxx + hw, yy + hh)

        jx = jy = 0.0
        if rumble > 0.0:
            amp = min(11.0, rumble / 12.0)
            jx, jy = random.uniform(-amp, amp), random.uniform(-amp, amp)
        self._place_car(self.W / 2 + s * 70.0 + jx, self.H - 44 + jy, s)


class Dial(tk.Canvas):

    SIZE = 150

    def __init__(self, master):
        super().__init__(master, width=self.SIZE, height=self.SIZE,
                         bg=PANEL, highlightthickness=0)
        c = self.SIZE / 2
        self._c = c
        self._r = c - 12
        self.create_oval(c - self._r, c - self._r, c + self._r, c + self._r,
                         outline="#2b3240", width=13)
        self.create_oval(c - self._r, c - self._r, c + self._r, c + self._r,
                         outline="#3a4150", width=2)
        self._spokes = [self.create_line(c, c, c, c, fill="#5a6474", width=6,
                                         capstyle="round") for _ in range(3)]
        self._lo = self.create_line(c, c, c, c, fill="#c5221f", width=3)
        self._hi = self.create_line(c, c, c, c, fill="#c5221f", width=3)
        self._hub = self.create_oval(c - 13, c - 13, c + 13, c + 13,
                                     fill="#4fc3f7", outline="")

    def _point(self, deg, r):

        a = math.radians(deg - 90.0)
        return self._c + r * math.cos(a), self._c + r * math.sin(a)

    def update_view(self, angle_deg, lock_deg):
        for i, spoke in enumerate(self._spokes):
            self.coords(spoke, self._c, self._c,
                        *self._point(angle_deg + i * 120.0, self._r - 6))
        self.coords(self._lo, *self._point(-lock_deg, self._r * 0.66),
                    *self._point(-lock_deg, self._r))
        self.coords(self._hi, *self._point(lock_deg, self._r * 0.66),
                    *self._point(lock_deg, self._r))


class MplPlot:
    ANN_RED = "#d62728"

    def __init__(self, parent, title, width=440, height=180, fixed_range=None,
                 xlabel="Sample", ylabel="Volts"):
        self.fixed_range = fixed_range
        self.xlabel = xlabel
        self.ylabel = ylabel

        wrap = tk.Frame(parent, bg=CARD)
        wrap.pack(fill="x", pady=(6, 0))

        tk.Label(wrap, text=title, bg=CARD, fg=FG,
                 font=("Segoe UI", 9, "bold")).pack(anchor="w", pady=(0, 4))

        dpi = 100
        self.fig = Figure(figsize=(width / dpi, height / dpi), dpi=dpi,
                          facecolor="#ffffff")
        self.ax = self.fig.add_subplot(111)
        self.fig.subplots_adjust(left=0.12, right=0.97, top=0.92, bottom=0.24)

        self.mpl_canvas = FigureCanvasTkAgg(self.fig, master=wrap)
        self.mpl_canvas.get_tk_widget().pack(fill="x")

        self._style_ax()

    def _style_ax(self):
        ax = self.ax
        ax.set_facecolor("#ffffff")
        ax.grid(True, color="#d0d0d0", linewidth=0.5)
        ax.tick_params(axis="both", labelsize=8, colors="#333333")
        ax.set_xlabel(self.xlabel, fontsize=9, color="#555555")
        ax.set_ylabel(self.ylabel, fontsize=9, color="#555555")
        for spine in ax.spines.values():
            spine.set_color("#333333")
            spine.set_linewidth(0.8)

    def set_traces(self, data_dict, annotations=None):
        ax = self.ax
        ax.clear()
        self._style_ax()

        if not data_dict:
            self.mpl_canvas.draw_idle()
            return

        is_fft = self.xlabel == "Frequency [Hz]"
        for label, (samples, color) in data_dict.items():
            is_raw = label == "Raw"
            ax.plot(samples, color=color,
                    linewidth=1.2 if (is_raw and not is_fft) else 2.0,
                    alpha=0.6 if (is_raw and not is_fft) else 1.0,
                    label=label)

        n_samples = max(len(s) for s, _ in data_dict.values())
        ax.set_xlim(0, n_samples - 1)

        if is_fft:
            n_ticks = 6
            positions = [int(i * (n_samples - 1) / (n_ticks - 1))
                         for i in range(n_ticks)]
            labels = [f"{p * ADC_SAMPLE_RATE / 2 / (n_samples - 1):.0f}"
                      for p in positions]
            ax.set_xticks(positions)
            ax.set_xticklabels(labels)

        if self.fixed_range is not None:
            ax.set_ylim(self.fixed_range)

        ax.legend(fontsize=8, loc="upper right", framealpha=0.8,
                  edgecolor="#d0d0d0")

        if annotations:
            for ann in annotations:
                if ann[0] == "vline":
                    _, sample_idx, label = ann
                    ax.axvline(x=sample_idx, color=self.ANN_RED,
                               linestyle="--", linewidth=1.5)
                    yhi = ax.get_ylim()[1]
                    ylo = ax.get_ylim()[0]
                    ax.text(sample_idx + 2, yhi - (yhi - ylo) * 0.05, label,
                            color=self.ANN_RED, fontsize=7,
                            fontweight="bold", va="top",
                            family="monospace")
                elif ann[0] == "text":
                    _, x_frac, y_frac, label = ann
                    ax.text(x_frac, 1.0 - y_frac, label,
                            transform=ax.transAxes,
                            color=self.ANN_RED, fontsize=8,
                            fontweight="bold", va="top",
                            family="monospace")

        self.mpl_canvas.draw_idle()


class StripChart:

    def __init__(self, parent, title, traces, fmt,
                 width=440, height=120, maxlen=240, symmetric=False):
        self.traces = traces
        self.fmt = fmt
        self.width = width
        self.height = height
        self.maxlen = maxlen
        self.symmetric = symmetric
        self.data = [deque(maxlen=maxlen) for _ in traces]

        wrap = tk.Frame(parent, bg=CARD)
        wrap.pack(fill="x", pady=(10, 0))

        header = tk.Frame(wrap, bg=CARD)
        header.pack(fill="x", pady=(0, 4))
        tk.Label(header, text=title.upper(), bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8, "bold")).pack(side="left")

        self.val_vars = []
        for label, color, _dash in traces:
            var = tk.StringVar(value="--")
            self.val_vars.append(var)
            tk.Label(header, textvariable=var, bg=CARD, fg=FG,
                     font=("Consolas", 8)).pack(side="right", padx=(2, 8))
            tk.Label(header, text=label, bg=CARD, fg=MUTED,
                     font=("Segoe UI", 8)).pack(side="right", padx=(6, 2))
            tk.Label(header, text="●", bg=CARD, fg=color,
                     font=("Segoe UI", 9)).pack(side="right")

        self.canvas = tk.Canvas(
            wrap, width=width, height=height, bg=PANEL,
            highlightthickness=1, highlightbackground=BORDER,
        )
        self.canvas.pack(fill="x")

    def clear(self):
        for d in self.data:
            d.clear()
        for var in self.val_vars:
            var.set("--")
        self.canvas.delete("all")

    def push(self, values):
        for i, val in enumerate(values):
            self.data[i].append(val)
            self.val_vars[i].set(self.fmt(val))
        self._redraw()

    def _redraw(self):
        c = self.canvas
        c.delete("all")
        w, h = self.width, self.height
        pad = 6

        allvals = [v for d in self.data for v in d]
        if not allvals:
            return

        lo, hi = min(allvals), max(allvals)
        if self.symmetric:
            m = max(abs(lo), abs(hi), 1)
            lo, hi = -m, m
        if hi == lo:
            hi = lo + 1
        span = float(hi - lo)

        def y_of(val):
            return pad + (hi - val) / span * (h - 2 * pad)

        step = w / (self.maxlen - 1)

        for frac in (0.25, 0.5, 0.75):
            gy = pad + frac * (h - 2 * pad)
            c.create_line(0, gy, w, gy, fill=GRID)

        if lo < 0 < hi:
            yz = y_of(0)
            c.create_line(0, yz, w, yz, fill=ZERO, dash=(2, 2))

        for i, (_label, color, dash) in enumerate(self.traces):
            d = self.data[i]
            if len(d) < 2:
                continue
            count = len(d)
            pts = []
            for j, val in enumerate(d):
                x = w - (count - 1 - j) * step
                pts.extend((x, y_of(val)))
            kwargs = {"fill": color}
            if dash:
                kwargs["dash"] = (5, 3)
                kwargs["width"] = 1
            else:
                kwargs["width"] = 2
            c.create_line(*pts, **kwargs)


class App:
    def __init__(self, root, args):
        self.root = root
        self.eds = args.eds
        self.default_port = args.iface
        self.node = args.node
        self.cfg = ffb.Config()
        self.worker = None
        self.iio = None
        self.adc = None
        self._t0 = time.perf_counter()
        self._last_frame = self._t0

        root.title("AD-M2KCOMPEDU-EBZ Demo  by Analog Devices")
        root.configure(bg=BG, padx=18, pady=16)
        root.resizable(False, False)
        self._init_theme(root)
        self._load_brand(root)

        header = tk.Frame(root, bg=BG)
        header.pack(fill="x")
        tk.Label(header, text="AD-M2KCOMPEDU-EBZ", bg=BG, fg=FG,
                 font=("Segoe UI", 16, "bold")).pack(side="left")
        tk.Label(header, text="Evaluation platform demo", bg=BG, fg=MUTED,
                 font=("Segoe UI", 9)).pack(side="left", padx=(12, 0), pady=(8, 0))

        if self.logo_img is not None:
            tk.Label(header, image=self.logo_img, bg=BG).pack(
                side="right", padx=(24, 0)
            )
        tk.Frame(root, bg=ACCENT, height=2).pack(fill="x", pady=(8, 12))

        nb = ttk.Notebook(root)
        nb.pack(fill="both", expand=True)
        self.motor_tab = tk.Frame(nb, bg=BG)
        self.iio_tab = tk.Frame(nb, bg=BG)
        self.adc_tab = tk.Frame(nb, bg=BG)
        nb.add(self.motor_tab, text="  Serial CAN Motor Control  ")
        nb.add(self.iio_tab, text="  Signal Generator · IIO  ")
        nb.add(self.adc_tab, text="  ADC Capture  ")

        self._build_ffb_tab(self.motor_tab, args)
        self._build_iio_tab(self.iio_tab)
        self._build_adc_tab(self.adc_tab)

        tk.Frame(root, bg=BORDER, height=1).pack(fill="x", pady=(10, 8))
        tk.Label(root, text="© 2026 Analog Devices, Inc. · AD-M2KCOMPEDU-EBZ evaluation demo",
                 bg=BG, fg=MUTED, font=("Segoe UI", 8)).pack(anchor="w")

        self._set_ffb_controls(False)
        self.disc_btn.state(["disabled"])
        self._set_iio_controls(False)
        self.iio_disc_btn.state(["disabled"])
        self.adc_disc_btn.state(["disabled"])
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._refresh()
        self._animate()

    def _card(self, parent, title=None):
        outer = tk.Frame(parent, bg=CARD, highlightbackground=BORDER,
                         highlightthickness=1, bd=0)
        outer.pack(fill="x", pady=(0, 12))
        inner = tk.Frame(outer, bg=CARD)
        inner.pack(fill="both", expand=True, padx=14, pady=12)
        if title:
            tk.Label(inner, text=title.upper(), bg=CARD, fg=MUTED,
                     font=("Segoe UI", 8, "bold")).pack(anchor="w", pady=(0, 8))
        return inner

    @staticmethod
    def _banner(parent, text):
        bar = tk.Frame(parent, bg=WARN_BG, highlightbackground=WARN_BORDER,
                       highlightthickness=1, bd=0)
        bar.pack(fill="x", pady=(0, 12))
        tk.Label(bar, text="⚠", bg=WARN_BG, fg=WARN_FG,
                 font=("Segoe UI", 11, "bold")).pack(side="left", padx=(10, 6), pady=8)
        tk.Label(bar, text=text, bg=WARN_BG, fg=WARN_FG, justify="left",
                 wraplength=760, font=("Segoe UI", 8)).pack(
            side="left", padx=(0, 10), pady=8)

    def _build_ffb_tab(self, tab, args):
        self._banner(
            tab,
            "CAN mode required. Install the CAN jumper, reboot, then open "
            "\"CAN Messenger\" on the device menu so the port speaks slcan. "
            "In SigGen mode (jumper removed) this tab will not connect.",
        )

        body = tk.Frame(tab, bg=BG)
        body.pack(fill="both", expand=True)
        left = tk.Frame(body, bg=BG)
        left.pack(side="left", fill="y", anchor="n")
        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True, padx=(14, 0))

        self.node_var = tk.StringVar(value=hex(args.node))
        self.port_var = tk.StringVar(value=args.iface)

        conn = self._card(left, "Connection")
        form = tk.Frame(conn, bg=CARD)
        form.pack(fill="x")
        self.form_widgets = []

        def field(row, col, label, var, width, values=None):
            tk.Label(form, text=label, bg=CARD, fg=MUTED, font=("Segoe UI", 8)).grid(
                row=row * 2, column=col, sticky="w", padx=(0, 12)
            )
            if values is not None:
                w = ttk.Combobox(form, textvariable=var, width=width, values=values)
            else:
                w = ttk.Entry(form, textvariable=var, width=width)
            w.grid(row=row * 2 + 1, column=col, sticky="w", padx=(0, 12), pady=(0, 8))
            self.form_widgets.append(w)
            return w

        field(0, 0, "Node ID", self.node_var, 8)
        field(0, 1, "COM port", self.port_var, 10, values=self._list_ports())

        stat_row = tk.Frame(conn, bg=CARD)
        stat_row.pack(fill="x", pady=(2, 8))
        self.status_dot = tk.Label(stat_row, text="●", bg=CARD, fg=MUTED,
                                   font=("Segoe UI", 10))
        self.status_dot.pack(side="left", padx=(0, 6))
        self.status_var = tk.StringVar(value="Idle")
        tk.Label(stat_row, textvariable=self.status_var, bg=CARD, fg=FG,
                 font=("Segoe UI", 9)).pack(side="left")

        conn_btns = tk.Frame(conn, bg=CARD)
        conn_btns.pack(fill="x", pady=(6, 0))
        self.conn_btn = ttk.Button(conn_btns, text="Connect", style="Accent.TButton",
                                   command=self._ffb_connect)
        self.conn_btn.pack(side="left", expand=True, fill="x", padx=(0, 3))
        self.disc_btn = ttk.Button(conn_btns, text="Disconnect", style="Stop.TButton",
                                   command=self._ffb_disconnect)
        self.disc_btn.pack(side="left", expand=True, fill="x", padx=(3, 0))

        fx = self._card(left, "Force-feedback effects")
        specs = [
            ("spring",    "Spring (centering)",    0, 20, 0.5),
            ("damping",   "Damping",               0, 2, 0.05),
            ("lock_deg",  "Lock-to-lock  ± deg",   90, 900, 10),
            ("endstop",   "End-stop wall",         0, 120, 5),
            ("rumble",    "Bumpy road",            0, 150, 5),
            ("rumble_hz", "Road frequency [Hz]",   3, 30, 1),
            ("max_force", "Max force (clamp)",     50, ffb.TORQUE_HARD_CLAMP, 10),
        ]
        for key, label, lo, hi, res in specs:
            var = tk.DoubleVar(value=getattr(self.cfg, key))
            self._slider(fx, label, "", lo, hi, var,
                         lambda v, k=key: self.cfg.set(**{k: float(v)}), res=res)

        fx_btns = tk.Frame(fx, bg=CARD)
        fx_btns.pack(fill="x", pady=(6, 0))
        self.recenter_btn = ttk.Button(fx_btns, text="Recenter", style="Accent.TButton",
                                       command=self._recenter)
        self.stop_btn = ttk.Button(fx_btns, text="■ STOP (de-energize)",
                                   style="Stop.TButton", command=self._ffb_stop)
        self.recenter_btn.pack(side="left", expand=True, fill="x", padx=(0, 3))
        self.stop_btn.pack(side="left", expand=True, fill="x", padx=(3, 0))

        view = self._card(right, "Drive view")
        self.scene = Scene(view)
        self.scene.pack()

        steer = self._card(right, "Steering")
        wheel_row = tk.Frame(steer, bg=CARD)
        wheel_row.pack(fill="x")
        dial_wrap = tk.Frame(wheel_row, bg=PANEL, highlightbackground=BORDER,
                             highlightthickness=1, bd=0)
        dial_wrap.pack(side="left")
        self.dial = Dial(dial_wrap)
        self.dial.pack(padx=6, pady=6)

        read = tk.Frame(wheel_row, bg=CARD)
        read.pack(side="left", padx=(16, 0), anchor="n")
        self.stat_vars = {}

        def stat(key, label, font):
            tk.Label(read, text=label.upper(), bg=CARD, fg=MUTED,
                     font=("Segoe UI", 7)).pack(anchor="w")
            var = tk.StringVar(value="--")
            self.stat_vars[key] = var
            tk.Label(read, textvariable=var, bg=CARD, fg=FG,
                     font=font).pack(anchor="w", pady=(0, 6))

        stat("angle", "Angle", ("Segoe UI", 22, "bold"))
        stat("torque", "Torque", ("Segoe UI", 14))
        stat("loop", "Loop rate", ("Consolas", 10))
        stat("raw", "Raw (cal)", ("Consolas", 10))

        self.tchart = StripChart(
            steer, "Commanded torque",
            [("torque", T_UQ, False)],
            fmt=lambda v: f"{v:+d}", width=440, height=90, symmetric=True,
        )

    def _build_iio_tab(self, tab):
        self._banner(
            tab,
            "SigGen mode required. Remove the CAN jumper and reboot so the "
            "iiod server owns the USB serial port. If \"CAN Messenger\" is "
            "running on the device, long-press to exit it first.",
        )

        body = tk.Frame(tab, bg=BG)
        body.pack(fill="both", expand=True)
        left = tk.Frame(body, bg=BG)
        left.pack(side="left", fill="y", anchor="n")
        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True, padx=(14, 0))

        self.iio_port_var = tk.StringVar(value=self.default_port)
        conn = self._card(left, "Connection")
        row = tk.Frame(conn, bg=CARD)
        row.pack(fill="x")
        tk.Label(row, text="COM port", bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8)).pack(anchor="w")
        self.iio_port_cbo = ttk.Combobox(row, textvariable=self.iio_port_var,
                                         width=12, values=self._list_ports())
        self.iio_port_cbo.pack(anchor="w", pady=(2, 8))

        stat_row = tk.Frame(conn, bg=CARD)
        stat_row.pack(fill="x", pady=(0, 8))
        self.iio_status_dot = tk.Label(stat_row, text="●", bg=CARD, fg=MUTED,
                                       font=("Segoe UI", 10))
        self.iio_status_dot.pack(side="left", padx=(0, 6))
        self.iio_status_var = tk.StringVar(value="Idle")
        tk.Label(stat_row, textvariable=self.iio_status_var, bg=CARD, fg=FG,
                 font=("Segoe UI", 9)).pack(side="left")

        iio_conn_btns = tk.Frame(conn, bg=CARD)
        iio_conn_btns.pack(fill="x")
        self.iio_conn_btn = ttk.Button(iio_conn_btns, text="Connect",
                                       style="Accent.TButton",
                                       command=self._iio_connect)
        self.iio_conn_btn.pack(side="left", expand=True, fill="x", padx=(0, 3))
        self.iio_disc_btn = ttk.Button(iio_conn_btns, text="Disconnect",
                                       style="Stop.TButton",
                                       command=self._iio_disconnect)
        self.iio_disc_btn.pack(side="left", expand=True, fill="x", padx=(3, 0))

        wave = self._card(left, "Analog output — altvoltage0")

        wt_row = tk.Frame(wave, bg=CARD)
        wt_row.pack(fill="x", pady=(0, 8))
        tk.Label(wt_row, text="Wave type", bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8)).pack(side="left")
        self.wave_var = tk.StringVar(value=WAVE_NAMES[2])
        self.wave_cbo = ttk.Combobox(wt_row, textvariable=self.wave_var, width=12,
                                     state="readonly", values=WAVE_NAMES)
        self.wave_cbo.pack(side="right")
        self.wave_cbo.bind("<<ComboboxSelected>>", self._on_wave)

        self.ampl_val = tk.IntVar(value=1000)
        self.ampl_scale = self._slider(wave, "Amplitude", "mV", 0, 5000,
                                       self.ampl_val, self._on_ampl, res=10)
        self.off_val = tk.IntVar(value=0)
        self.off_scale = self._slider(wave, "DC offset", "mV", -2500, 2500,
                                      self.off_val, self._on_off, res=10)

        self.freq_var = tk.StringVar(value="1000")
        self.freq_entry = self._entry_row(wave, "Frequency [Hz]", self.freq_var,
                                          self._on_freq)

        pwm = self._card(right, "PWM output — altvoltage1")
        en_row = tk.Frame(pwm, bg=CARD)
        en_row.pack(fill="x", pady=(0, 8))
        self.pwm_en_var = tk.BooleanVar(value=True)
        self.pwm_en_chk = ttk.Checkbutton(en_row, text="Enable PWM",
                                          variable=self.pwm_en_var,
                                          style="Card.TCheckbutton",
                                          command=self._on_pwm_en)
        self.pwm_en_chk.pack(side="left")

        self.duty_val = tk.IntVar(value=50)
        self.duty_scale = self._slider(pwm, "Duty cycle", "%", 0, 100,
                                       self.duty_val, self._on_duty, res=1)
        self.pwm_freq_var = tk.StringVar(value="1000")
        self.pwm_freq_entry = self._entry_row(pwm, "PWM frequency [Hz]",
                                              self.pwm_freq_var, self._on_pwm_freq)

        demo = self._card(right, "Demo sequence")
        tk.Label(demo, text="Replays the demos/iio wave sweep: cycles wave "
                            "types while sweeping frequency, amplitude, offset "
                            "and fading the PWM duty.",
                 bg=CARD, fg=MUTED, justify="left", wraplength=300,
                 font=("Segoe UI", 8)).pack(anchor="w", pady=(0, 8))
        self.sweep_var = tk.StringVar(value="▶ Run demo sweep")
        self.sweep_btn = ttk.Button(demo, textvariable=self.sweep_var,
                                    style="Accent.TButton",
                                    command=self._toggle_sweep)
        self.sweep_btn.pack(fill="x")

        self.iio_form = [self.iio_port_cbo]
        self.iio_controls = [
            self.wave_cbo, self.ampl_scale, self.off_scale, self.freq_entry,
            self.pwm_en_chk, self.duty_scale, self.pwm_freq_entry,
        ]

    def _build_adc_tab(self, tab):
        self._banner(
            tab,
            "ADC capture mode. Connect the M2K signal generator (or mic) to "
            "channel 1 (A2) and channel 2 (A3). On the OLED, open ADC Monitor "
            "so the firmware streams the dual-channel capture at 8 kHz.",
        )

        body = tk.Frame(tab, bg=BG)
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=1, uniform="adc")
        body.columnconfigure(1, weight=1, uniform="adc")
        left = tk.Frame(body, bg=BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 7))
        right = tk.Frame(body, bg=BG)
        right.grid(row=0, column=1, sticky="nsew", padx=(7, 0))

        self.adc_port_var = tk.StringVar(value=self.default_port)
        conn = self._card(left, "Connection")
        row = tk.Frame(conn, bg=CARD)
        row.pack(fill="x")
        tk.Label(row, text="COM port", bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8)).pack(anchor="w")
        self.adc_port_cbo = ttk.Combobox(row, textvariable=self.adc_port_var,
                                         width=12, values=self._list_ports())
        self.adc_port_cbo.pack(anchor="w", pady=(2, 8))

        stat_row = tk.Frame(conn, bg=CARD)
        stat_row.pack(fill="x", pady=(0, 8))
        self.adc_status_dot = tk.Label(stat_row, text="●", bg=CARD, fg=MUTED,
                                       font=("Segoe UI", 10))
        self.adc_status_dot.pack(side="left", padx=(0, 6))
        self.adc_status_var = tk.StringVar(value="Idle")
        tk.Label(stat_row, textvariable=self.adc_status_var, bg=CARD, fg=FG,
                 font=("Segoe UI", 9)).pack(side="left")

        adc_conn_btns = tk.Frame(conn, bg=CARD)
        adc_conn_btns.pack(fill="x")
        self.adc_conn_btn = ttk.Button(adc_conn_btns, text="Connect",
                                       style="Accent.TButton",
                                       command=self._adc_connect)
        self.adc_conn_btn.pack(side="left", expand=True, fill="x", padx=(0, 3))
        self.adc_disc_btn = ttk.Button(adc_conn_btns, text="Disconnect",
                                       style="Stop.TButton",
                                       command=self._adc_disconnect)
        self.adc_disc_btn.pack(side="left", expand=True, fill="x", padx=(3, 0))

        wave_card = self._card(left, "ADC Waveform")
        desc_row = tk.Frame(wave_card, bg=CARD)
        desc_row.pack(fill="x")
        tk.Label(desc_row, text="Raw ADC data: blue = channel 1 (A2), "
                 "orange = channel 2 (A3)",
                 bg=CARD, fg=MUTED, font=("Segoe UI", 7),
                 wraplength=340, justify="left").pack(side="left")
        self.show_post_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(desc_row, text="Show channel 2",
                        variable=self.show_post_var,
                        style="Card.TCheckbutton").pack(side="right")

        self.timebase_var = tk.IntVar(value=ADC_BUF_SIZE)
        tb_wrap = tk.Frame(wave_card, bg=CARD)
        tb_wrap.pack(fill="x", pady=(2, 0))
        tb_head = tk.Frame(tb_wrap, bg=CARD)
        tb_head.pack(fill="x")
        tk.Label(tb_head, text="Time base", bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8)).pack(side="left")
        self._tb_val = tk.StringVar(
            value=f"{ADC_BUF_SIZE} samples · "
                  f"{ADC_BUF_SIZE / ADC_SAMPLE_RATE * 1000:.1f} ms")
        tk.Label(tb_head, textvariable=self._tb_val, bg=CARD, fg=RED,
                 font=("Consolas", 9, "bold")).pack(side="right")

        def _tb_moved(v):
            n = int(float(v))
            ms = n / ADC_SAMPLE_RATE * 1000
            self._tb_val.set(f"{n} samples · {ms:.1f} ms")

        tk.Scale(tb_wrap, from_=16, to=ADC_BUF_SIZE, orient="horizontal",
                 length=300, resolution=8, showvalue=False,
                 variable=self.timebase_var, command=_tb_moved,
                 bg=CARD, fg=FG, troughcolor=TROUGH,
                 activebackground=ACCENT, highlightthickness=0, bd=1,
                 sliderrelief="raised").pack(fill="x")

        self.raw_plot = MplPlot(wave_card, "Channel 1 / Channel 2",
                                     width=440, height=180, fixed_range=(-0.3, 5.3))

        self.filter_slots = []

        fc = self._card(right, "Biquad Filter")
        tk.Label(fc, text="Apply a digital biquad filter (Direct Form II "
                 "Transposed) to the raw ADC signal in real time.",
                 bg=CARD, fg=MUTED, font=("Segoe UI", 7),
                 wraplength=420, justify="left").pack(anchor="w", pady=(0, 6))
        slot = {}

        top_row = tk.Frame(fc, bg=CARD)
        top_row.pack(fill="x", pady=(0, 6))

        tk.Label(top_row, text="Type", bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8)).pack(side="left")
        slot["type_var"] = tk.StringVar(value="Off")
        slot["type_cbo"] = ttk.Combobox(
            top_row, textvariable=slot["type_var"], width=12,
            state="readonly", values=FILTER_TYPES)
        slot["type_cbo"].pack(side="left", padx=(6, 14))
        slot["type_cbo"].bind("<<ComboboxSelected>>",
                              lambda _e: self._on_filter_change(0))

        tk.Label(top_row, text="Source", bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8)).pack(side="left")
        slot["src_var"] = tk.StringVar(value="Channel 1")
        slot["src_cbo"] = ttk.Combobox(
            top_row, textvariable=slot["src_var"], width=9,
            state="readonly", values=["Channel 1", "Channel 2"])
        slot["src_cbo"].pack(side="left", padx=(6, 0))

        slot["fc_var"] = tk.DoubleVar(value=1000.0)
        slot["fc_scale"] = self._adc_slider(
            fc, "Cutoff frequency (Fc) — the -3 dB point of the filter",
            "Hz", 10, 4000, slot["fc_var"],
            lambda _v: self._on_filter_change(0))

        slot["q_var"] = tk.DoubleVar(value=0.707)
        slot["q_scale"] = self._adc_slider(
            fc, "Q factor — sharpness of the filter (0.707 = Butterworth)",
            "", 0.1, 10.0, slot["q_var"],
            lambda _v: self._on_filter_change(0), res=0.01)

        slot["gain_var"] = tk.DoubleVar(value=0.0)
        slot["gain_frame"] = tk.Frame(fc, bg=CARD)
        slot["gain_scale"] = self._adc_slider(
            slot["gain_frame"], "Gain — boost/cut at Fc (Peak & Shelf only)",
            "dB", -12.0, 12.0,
            slot["gain_var"],
            lambda _v: self._on_filter_change(0), res=0.1)

        slot["plot"] = MplPlot(fc,
            "Time domain — grey: raw input, green: filter applied from midpoint",
            width=440, height=160, fixed_range=(-0.3, 5.3))
        slot["fft_plot"] = MplPlot(fc,
            "FFT — grey: raw spectrum, green: filtered spectrum",
            width=440, height=160, fixed_range=(-80, 3),
            xlabel="Frequency [Hz]", ylabel="dB")
        slot["state"] = None
        self.filter_slots.append(slot)

    def _adc_slider(self, parent, label, unit, lo, hi, var, cmd, res=1):
        wrap = tk.Frame(parent, bg=CARD)
        wrap.pack(fill="x", pady=(0, 8))
        head = tk.Frame(wrap, bg=CARD)
        head.pack(fill="x")
        tk.Label(head, text=label, bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8)).pack(side="left")
        valvar = tk.StringVar(value=f"{var.get():.4g} {unit}")
        tk.Label(head, textvariable=valvar, bg=CARD, fg=RED,
                 font=("Consolas", 9, "bold")).pack(side="right")

        def on_move(v):
            valvar.set(f"{float(v):.4g} {unit}")
            cmd(v)

        s = tk.Scale(wrap, from_=lo, to=hi, orient="horizontal", length=300,
                     resolution=res, showvalue=False, variable=var,
                     command=on_move, bg=CARD, fg=FG, troughcolor=TROUGH,
                     activebackground=ACCENT, highlightthickness=0, bd=1,
                     sliderrelief="raised")
        s.pack(fill="x")
        return s

    def _on_filter_change(self, idx):
        slot = self.filter_slots[idx]
        ftype = slot["type_var"].get()
        needs_gain = ftype in ("Peak", "Low Shelf", "High Shelf")
        if needs_gain:
            slot["gain_frame"].pack(fill="x")
        else:
            slot["gain_frame"].pack_forget()
        slot["state"] = None

    def _adc_connect(self):
        if self.adc and self.adc.is_alive():
            return
        self._stop_ffb_worker()
        self._stop_iio()
        port = self.adc_port_var.get().strip()
        if not port:
            self.adc_status_var.set("Bad input: port empty")
            return
        self.adc_conn_btn.state(["disabled"])
        self.adc_disc_btn.state(["!disabled"])
        self.adc_status_var.set("Connecting...")
        self.adc = AdcCaptureWorker(port)
        self.adc.start()

    def _adc_disconnect(self):
        if not (self.adc and self.adc.is_alive()):
            return
        self.adc_disc_btn.state(["disabled"])
        self.adc_status_var.set("Disconnecting...")
        self.root.update_idletasks()
        self._stop_adc()
        self.adc_status_var.set("Disconnected")
        self.adc_conn_btn.state(["!disabled"])
        self.adc_conn_btn.configure(text="Connect")

    def _stop_adc(self):
        if self.adc and self.adc.is_alive():
            self.adc.request_stop()
            self.adc.join(timeout=3.0)
        self.adc = None

    def _slider(self, parent, label, unit, lo, hi, var, cmd, res=1):
        wrap = tk.Frame(parent, bg=CARD)
        wrap.pack(fill="x", pady=(0, 8))
        head = tk.Frame(wrap, bg=CARD)
        head.pack(fill="x")
        tk.Label(head, text=label, bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8)).pack(side="left")
        suffix = f" {unit}" if unit else ""

        def _fmt(v):

            return f"{float(v):g}{suffix}"

        valvar = tk.StringVar(value=_fmt(var.get()))
        tk.Label(head, textvariable=valvar, bg=CARD, fg=FG,
                 font=("Consolas", 9)).pack(side="right")

        def on_move(v):
            valvar.set(_fmt(v))
            cmd(v)

        s = tk.Scale(wrap, from_=lo, to=hi, orient="horizontal", length=300,
                     resolution=res, showvalue=False, variable=var,
                     command=on_move, bg=CARD, fg=FG, troughcolor=TROUGH,
                     activebackground=ACCENT, highlightthickness=0, bd=1,
                     sliderrelief="raised")
        s.pack(fill="x")
        return s

    def _entry_row(self, parent, label, var, apply_cb):
        wrap = tk.Frame(parent, bg=CARD)
        wrap.pack(fill="x", pady=(0, 8))
        tk.Label(wrap, text=label, bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8)).pack(side="left")
        e = ttk.Entry(wrap, textvariable=var, width=12)
        e.pack(side="left", padx=(10, 6))

        e.bind("<Return>", lambda _e: apply_cb())
        e.bind("<FocusOut>", lambda _e: apply_cb())
        ttk.Button(wrap, text="Set", style="Accent.TButton",
                   command=apply_cb).pack(side="left")
        return e

    def _init_theme(self, root):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure(".", background=BG, foreground=FG)
        style.configure("TEntry", fieldbackground=FIELD, foreground=FG,
                        bordercolor=BORDER, lightcolor=BORDER, darkcolor=BORDER,
                        insertcolor=FG, padding=5)
        style.map("TEntry", fieldbackground=[("disabled", CARD)],
                  foreground=[("disabled", MUTED)])
        style.configure("TCombobox", fieldbackground=FIELD, background=FIELD,
                        foreground=FG, bordercolor=BORDER, arrowcolor=FG, padding=4)
        style.map("TCombobox",
                  fieldbackground=[("readonly", FIELD), ("disabled", CARD)],
                  foreground=[("disabled", MUTED)],
                  arrowcolor=[("disabled", MUTED)])
        root.option_add("*TCombobox*Listbox.background", FIELD)
        root.option_add("*TCombobox*Listbox.foreground", FG)
        root.option_add("*TCombobox*Listbox.selectBackground", ACCENT)
        root.option_add("*TCombobox*Listbox.selectForeground", "#ffffff")

        style.configure("Card.TCheckbutton", background=CARD, foreground=FG,
                        font=("Segoe UI", 9))
        style.map("Card.TCheckbutton", background=[("active", CARD)],
                  foreground=[("disabled", MUTED)])

        style.configure("TNotebook", background=BG, borderwidth=0, tabmargins=(0, 0, 0, 0))
        style.configure("TNotebook.Tab", background=BORDER, foreground=MUTED,
                        font=("Segoe UI", 9, "bold"), padding=(14, 8), borderwidth=0)
        style.map("TNotebook.Tab",
                  background=[("selected", CARD)],
                  foreground=[("selected", ACCENT)])

        def mkbtn(name, base, active):
            style.configure(name, background=base, foreground="#ffffff",
                            font=("Segoe UI", 10, "bold"), relief="flat",
                            borderwidth=0, padding=9, focuscolor=base)
            style.map(name,
                      background=[("active", active), ("disabled", BORDER)],
                      foreground=[("disabled", MUTED)])

        mkbtn("Accent.TButton", ACCENT, ACCENT2)
        mkbtn("Fwd.TButton", GREEN, "#156e31")
        mkbtn("Rev.TButton", AMBER, "#b87207")
        mkbtn("Stop.TButton", RED, "#a51b18")

    def _load_brand(self, root):
        self.logo_img = None
        self.amp_img = None
        try:
            if os.path.exists(LOGO_FILE):
                self.logo_img = tk.PhotoImage(file=LOGO_FILE)
            if os.path.exists(AMP_FILE):
                self.amp_img = tk.PhotoImage(file=AMP_FILE)
                root.iconphoto(True, self.amp_img)
        except tk.TclError:

            self.logo_img = None
            self.amp_img = None

    @staticmethod
    def _list_ports():
        ports = []
        if list_ports is not None:
            ports = [p.device for p in list_ports.comports()]
        return ports

    def _set_ffb_controls(self, armed):

        for w in (self.recenter_btn, self.stop_btn):
            w.state(["!disabled" if armed else "disabled"])
        for w in self.form_widgets:
            w.state(["disabled" if armed else "!disabled"])

    def _ffb_connect(self):
        if self.worker and self.worker.is_alive():
            return

        self._stop_iio()
        self._stop_adc()

        try:
            node = int_flexible(self.node_var.get().strip())
            port = self.port_var.get().strip()
            if not port:
                raise ValueError("port empty")
        except (ValueError, TypeError) as exc:
            self.status_var.set(f"Bad input: {exc}")
            return

        self.tchart.clear()
        self.conn_btn.state(["disabled"])
        self.disc_btn.state(["!disabled"])
        self.status_var.set("Connecting...")
        self.worker = ffb.FfbWorker(node, port, self.eds, self.cfg)
        self.worker.start()

    def _ffb_disconnect(self):

        if not (self.worker and self.worker.is_alive()):
            return
        self.disc_btn.state(["disabled"])
        self.status_var.set("Disconnecting...")
        self.root.update_idletasks()
        self._stop_ffb_worker()
        self.status_var.set("Disconnected")
        self._set_ffb_controls(False)
        self.conn_btn.state(["!disabled"])
        self.conn_btn.configure(text="Connect")

    def _recenter(self):

        if self.worker and self.worker.connected:
            self.worker.recenter()

    def _ffb_stop(self):

        self._ffb_disconnect()

    def _set_iio_controls(self, connected):
        for w in self.iio_controls:
            try:
                w.configure(state=("normal" if connected else "disabled"))
            except tk.TclError:
                w.state(["!disabled" if connected else "disabled"])
        self.sweep_btn.state(["!disabled" if connected else "disabled"])
        for w in self.iio_form:
            w.state(["disabled" if connected else "!disabled"])

    def _push_all_iio(self):
        if not (self.iio and self.iio.connected):
            return
        self.iio.set_param("wave", WAVE_NAMES.index(self.wave_var.get()))
        self.iio.set_param("ampl", self.ampl_val.get())
        self.iio.set_param("off", self.off_val.get())
        self.iio.set_param("pwm_en", int(self.pwm_en_var.get()))
        self.iio.set_param("pwm_duty", self.duty_val.get())
        for key, var in (("freq", self.freq_var), ("pwm_freq", self.pwm_freq_var)):
            try:
                self.iio.set_param(key, float(var.get()))
            except (ValueError, TypeError):
                pass

    def _iio_connect(self):
        if self.iio and self.iio.is_alive():
            return

        self._stop_ffb_worker()
        self._stop_adc()
        port = self.iio_port_var.get().strip()
        if not port:
            self.iio_status_var.set("Bad input: port empty")
            return
        self.iio_conn_btn.state(["disabled"])
        self.iio_disc_btn.state(["!disabled"])
        self.iio_status_var.set("Connecting...")
        self.iio = siggen.IioWorker(port)
        self.iio.start()

    def _iio_disconnect(self):

        if not (self.iio and self.iio.is_alive()):
            return
        self.iio_disc_btn.state(["disabled"])
        self.iio_status_var.set("Disconnecting...")
        self.root.update_idletasks()
        self._stop_iio()
        self.iio_status_var.set("Disconnected")
        self.sweep_var.set("▶ Run demo sweep")
        self._set_iio_controls(False)
        self.iio_conn_btn.state(["!disabled"])
        self.iio_conn_btn.configure(text="Connect")

    def _param(self, key, value):
        if self.iio and self.iio.connected:
            self.iio.set_param(key, value)

    def _on_wave(self, _e=None):
        self._param("wave", WAVE_NAMES.index(self.wave_var.get()))

    def _on_ampl(self, _v=None):
        self._param("ampl", self.ampl_val.get())

    def _on_off(self, _v=None):
        self._param("off", self.off_val.get())

    def _on_freq(self):
        try:
            self._param("freq", float(self.freq_var.get()))
        except (ValueError, TypeError):
            pass

    def _on_pwm_en(self):
        self._param("pwm_en", int(self.pwm_en_var.get()))

    def _on_duty(self, _v=None):
        self._param("pwm_duty", self.duty_val.get())

    def _on_pwm_freq(self):
        try:
            self._param("pwm_freq", float(self.pwm_freq_var.get()))
        except (ValueError, TypeError):
            pass

    def _toggle_sweep(self):
        if not (self.iio and self.iio.connected):
            return
        running = not self.iio.sweeping
        self.iio.set_sweep(running)
        if running:
            self.sweep_var.set("■ Stop demo sweep")
            for w in self.iio_controls:
                try:
                    w.configure(state="disabled")
                except tk.TclError:
                    w.state(["disabled"])
        else:
            self.sweep_var.set("▶ Run demo sweep")
            self._set_iio_controls(True)
            self._push_all_iio()

    def _stop_ffb_worker(self):
        if self.worker and self.worker.is_alive():
            self.worker.request_stop()
            self.worker.join(timeout=3.0)
        self.worker = None

    def _stop_iio(self):
        if self.iio and self.iio.is_alive():
            self.iio.set_sweep(False)
            self.iio.request_stop()
            self.iio.join(timeout=3.0)
        self.iio = None

    @staticmethod
    def _status_color(connected, busy_ok, status):
        s = (status or "").lower()
        if "fail" in s or "error" in s or "missing" in s or "disconnected" in s:
            return RED
        if not connected or "connecting" in s:
            return AMBER
        return busy_ok

    def _refresh(self):

        w = self.worker
        if w:

            if getattr(w, "runaway", False):
                self.status_var.set("Runaway cutout -- coasting (lower spring)")
                self.status_dot.configure(fg=AMBER)
            else:
                self.status_var.set(w.status)
                self.status_dot.configure(
                    fg=self._status_color(w.connected, GREEN if w.armed else ACCENT, w.status))
            self._set_ffb_controls(w.connected)
            if w.connected:

                self.tchart.push((w.torque,))
            if w.is_alive():

                self.conn_btn.state(["disabled"])
                self.disc_btn.state(["!disabled"])
            else:

                self.conn_btn.state(["!disabled"])
                self.conn_btn.configure(text="Reconnect")
                self.disc_btn.state(["disabled"])

        iw = self.iio
        if iw:
            self.iio_status_var.set(iw.status)
            self.iio_status_dot.configure(
                fg=self._status_color(iw.connected, ACCENT, iw.status))
            if iw.is_alive():
                if iw.connected and not iw.sweeping:
                    self._set_iio_controls(True)
                self.iio_conn_btn.state(["disabled"])
                self.iio_disc_btn.state(["!disabled"])
            else:
                self._set_iio_controls(False)
                self.iio_conn_btn.state(["!disabled"])
                self.iio_conn_btn.configure(text="Reconnect")
                self.iio_disc_btn.state(["disabled"])
                self.sweep_var.set("▶ Run demo sweep")

        aw = self.adc
        if aw:
            self.adc_status_var.set(aw.status)
            self.adc_status_dot.configure(
                fg=self._status_color(aw.connected, ACCENT, aw.status))
            if aw.is_alive():
                self.adc_conn_btn.state(["disabled"])
                self.adc_disc_btn.state(["!disabled"])
                self.adc_port_cbo.state(["disabled"])
            else:
                self.adc_conn_btn.state(["!disabled"])
                self.adc_conn_btn.configure(text="Reconnect")
                self.adc_disc_btn.state(["disabled"])
                self.adc_port_cbo.state(["!disabled"])
            if aw.has_new_block():
                pre, post = aw.get_buffers()
                tb = self.timebase_var.get()
                traces = {"Channel 1": (pre[:tb], T_PRE)}
                if self.show_post_var.get() and post:
                    traces["Channel 2"] = (post[:tb], T_POST)
                self.raw_plot.set_traces(traces)

                for slot in self.filter_slots:
                    ftype = slot["type_var"].get()
                    if ftype == "Off":
                        slot["plot"].set_traces({})
                        slot["fft_plot"].set_traces({})
                        slot["state"] = None
                        continue
                    src = pre if slot["src_var"].get() == "Channel 1" else post
                    fc = slot["fc_var"].get()
                    q = slot["q_var"].get()
                    gain = slot["gain_var"].get()
                    coeffs = biquad_coeffs(ftype, fc, max(q, 0.1),
                                           ADC_SAMPLE_RATE, gain)
                    if slot["state"] is None:
                        _, warmup = apply_biquad(src, *coeffs, state=None)
                        slot["state"] = warmup
                    filtered, new_state = apply_biquad(
                        src, *coeffs, state=slot["state"])
                    slot["state"] = new_state
                    mid_v = tb // 2
                    src_v = src[:tb]
                    combined_v = list(src[:mid_v]) + list(filtered[mid_v:tb])
                    ann = [
                        ("vline", mid_v, "filter on"),
                        ("text", 0.02, 0.02,
                         f"Fc={fc:.0f}Hz  Q={q:.2f}"
                         + (f"  G={gain:+.1f}dB" if ftype in
                            ("Peak", "Low Shelf", "High Shelf") else "")),
                    ]
                    slot["plot"].set_traces(
                        {"Raw": (src_v, MUTED), "Output": (combined_v, T_FILT)},
                        annotations=ann)

                    freqs_r, db_r = compute_fft_db(src, ADC_SAMPLE_RATE)
                    freqs_f, db_f = compute_fft_db(filtered, ADC_SAMPLE_RATE)
                    fc_bin = min(range(len(freqs_r)),
                                key=lambda i: abs(freqs_r[i] - fc))
                    fft_ann = [("vline", fc_bin, f"Fc={fc:.0f}Hz")]
                    slot["fft_plot"].set_traces(
                        {"Raw": (db_r, MUTED), "Filtered": (db_f, T_FILT)},
                        annotations=fft_ann)

        self.root.after(150, self._refresh)

    def _animate(self):

        now = time.perf_counter()
        dt = now - self._last_frame
        self._last_frame = now

        snap = self.cfg.snapshot()
        lock_deg, rumble = snap[2], snap[4]

        w = self.worker
        if w and w.connected:
            angle = w.angle_deg
            self.stat_vars["angle"].set(f"{angle:+.1f}\N{DEGREE SIGN}")
            self.stat_vars["torque"].set(f"{w.torque:+d}")
            self.stat_vars["loop"].set(f"{w.loop_hz:.1f} Hz")
            self.stat_vars["raw"].set(f"{w.raw}")
        else:
            angle, rumble = 0.0, 0.0
            for var in self.stat_vars.values():
                var.set("--")

        self.scene.update_view(angle, lock_deg, rumble, now - self._t0, dt)
        self.dial.update_view(angle, lock_deg)
        self.root.after(30, self._animate)

    def _on_close(self):
        busy = ((self.worker and self.worker.is_alive()) or
                (self.iio and self.iio.is_alive()) or
                (self.adc and self.adc.is_alive()))
        if busy:
            self.status_var.set("Stopping...")
            self.iio_status_var.set("Stopping...")
            self.adc_status_var.set("Stopping...")
            self.root.update_idletasks()
        self._stop_ffb_worker()
        self._stop_iio()
        self._stop_adc()
        self.root.destroy()


def main():
    p = argparse.ArgumentParser(description="AD-M2KCOMPEDU-EBZ demo panel")
    p.add_argument("node", type=int_flexible, nargs="?", default=0x16, help="Target node ID (motor)")
    p.add_argument("-i", "--iface", default="COM4", help="slcan / IIO serial COM port")
    p.add_argument("-e", "--eds", default=DEFAULT_EDS, help="EDS file (motor)")
    args = p.parse_args()

    root = tk.Tk()
    App(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
