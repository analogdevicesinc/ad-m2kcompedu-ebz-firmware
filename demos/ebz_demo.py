#!/usr/bin/env python3
"""AD-M2KCOMPEDU-EBZ demo panel.

A single Tkinter panel for the two things the board can do over its USB serial
port, one tab each:

  * Motor Drive (CAN) -- drives a TMC9660 (ADRD3161) in FOC open-loop current
    mode over CANopen/slcan. Same arming sequence as open_loop_spin.py.
  * Signal Generator (IIO) -- drives the on-board signal generator + PWM through
    the firmware's iiod server (libiio), the interactive form of demos/iio.

The two are MUTUALLY EXCLUSIVE in hardware: a jumper selects CAN mode vs. the
signal-generator (IIO) mode at power-on, and both share the one USB serial port.
Each tab shows a banner reminding you which jumper/mode it needs, and connecting
one tab disconnects the other.

All bus traffic runs on a worker thread so the controls stay smooth, and every
output is driven to a safe state when a tab disconnects or the window closes.

canopen (CAN tab) and iio (IIO tab) are imported lazily -- the panel launches
with neither installed; each tab just reports what it needs when you connect.

Usage:
    python ebz_demo.py                 # node 0x16, COM4
    python ebz_demo.py 0x16 -i COM4 --max 2000000 --current 2500
"""

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

from audio import (
    AudioWorker, AUDIO_BUF_SIZE, AUDIO_SAMPLE_RATE,
    FILTER_TYPES, biquad_coeffs, apply_biquad, compute_fft_db,
)


# The worker/backend logic lives in the motor_control/ and iio/ folders; the UI
# stays here in the main window. Bootstrap sys.path with both so we can import
# them by BARE module name. Deliberately NOT `import iio.siggen`: a package
# named `iio` on sys.path would shadow the real libiio module that siggen.py
# imports. libiio (a regular module in site-packages) still wins over the
# demos/iio namespace-portion for `import iio`, so nothing is shadowed here.
_HERE = os.path.dirname(os.path.abspath(__file__))
for _sub in ("motor_control", "iio"):
    _p = os.path.join(_HERE, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)
import ffb        # noqa: E402  demos/motor_control/ffb.py -- FFB torque engine
import siggen     # noqa: E402  demos/iio/siggen.py -- signal-gen / PWM worker

WAVE_NAMES = siggen.WAVE_NAMES

# Brand assets ship next to this script. adi_logo.png is the ADI blue logo
# (wordmark, no tagline -- the variant the ADI logo guidelines prefer for
# digital UIs); adi_amp.png is the amp-in-square mark used as the window icon.
# Both are box-filtered crops of the official kit artwork -- unaltered colour
# and proportions, per the guidelines. Loading is best-effort.
ASSET_DIR = os.path.dirname(os.path.abspath(__file__))
LOGO_FILE = os.path.join(ASSET_DIR, "adi_logo.png")
AMP_FILE = os.path.join(ASSET_DIR, "adi_amp.png")
# The EDS lives with the other motor-tuning files in motor_control/.
DEFAULT_EDS = os.path.join(ASSET_DIR, "motor_control", "adrd3161.eds")


# --- palette (light "ADI" theme; charts stay dark) -----------------------
BG      = "#eef1f5"   # window background (light grey)
CARD    = "#ffffff"   # card surface (white)
FIELD   = "#ffffff"   # input background (white, outlined)
TROUGH  = "#dfe3e9"   # slider trough
BORDER  = "#d3d9e0"   # hairline borders
FG      = "#1f2a37"   # primary text (near-black)
MUTED   = "#66707e"   # secondary text (grey)
ACCENT  = "#0067b9"   # ADI blue -- primary / connect / active
ACCENT2 = "#00568f"   # ADI blue, pressed/hover
GREEN   = "#1b8a3e"   # forward / on
RED     = "#c5221f"   # stop
AMBER   = "#d98a08"   # reverse

# Warning banner (jumper reminders).
WARN_BG     = "#fff4e5"
WARN_FG     = "#8a5a00"
WARN_BORDER = "#f0d9a8"

# Chart panels stay dark, like the reference tool.
PANEL   = "#0d1522"   # plot background
GRID    = "#22303f"   # plot gridlines
ZERO    = "#5a6675"   # zero reference line
T_VEL   = "#4a9eff"   # velocity: actual trace
T_TGT   = "#9aa4b2"   # velocity: target trace (dashed)
T_CUR   = "#ff9e40"   # motor-current trace
T_UQ    = "#3ddc84"   # FOC UQ trace

# Audio tab traces.
T_PRE   = "#4a9eff"   # pre-amp (blue, matches velocity)
T_POST  = "#ff9e40"   # post-amp (orange, matches current)
T_FILT  = "#3ddc84"   # filtered output (green)



def int_flexible(x):
    return int(x, 0)


class Scene(tk.Canvas):
    """Arcade 'behind the wheel' view: a scrolling perspective road with a car.

    Pure eye-candy, but driven entirely by real telemetry -- it renders whatever
    the shaft is actually doing. Steering angle bends the road ahead and slides
    the car across it (leaning into the turn); the Bumpy-road amplitude shakes
    the car and rattles the ride. The road always scrolls, so even idle it looks
    like a cruise. The car is ADI blue in this panel."""

    W, H = 440, 232
    TS = (0.0, 0.18, 0.40, 0.68, 1.0)   # perspective samples, horizon(0)->near(1)
    CRUISE = 0.85                        # road scroll speed (laps/second)

    def __init__(self, master):
        super().__init__(master, width=self.W, height=self.H,
                         bg="#6ec6f0", highlightthickness=0)   # sky
        self.horizon = int(self.H * 0.34)
        self._scroll = 0.0

        # Static backdrop (drawn once, behind everything).
        self.create_oval(self.W - 96, self.horizon - 58, self.W - 30,
                         self.horizon + 8, fill="#fff2b8", outline="")   # sun
        self.create_oval(-40, self.horizon - 34, 150, self.horizon + 30,
                         fill="#3f9e51", outline="")                     # hill L
        self.create_oval(self.W - 170, self.horizon - 26, self.W + 40,
                         self.horizon + 30, fill="#3f9e51", outline="")  # hill R
        self.create_rectangle(0, self.horizon, self.W, self.H,
                              fill="#48a85a", outline="")                # grass

        # Dynamic road (repositioned every frame by coords()).
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

        # Car (rear view), built once and moved as a group by _place_car().
        # Body is ADI blue; taillights stay red (correct for a rear view).
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

    # -- perspective helpers: map (tt in 0..1 horizon->near, side) to screen ---
    def _cx(self, tt, top_c, bot_c):
        w = (1.0 - tt) ** 1.6            # curvature bunched toward the horizon
        return bot_c + (top_c - bot_c) * w

    def _half(self, tt):
        return 8.0 + 167.0 * tt          # road half-width, narrow far -> wide near

    def _edge(self, tt, side, top_c, bot_c):
        x = self._cx(tt, top_c, bot_c) + side * self._half(tt)
        y = self.horizon + tt * (self.H - self.horizon)
        return x, y

    def _place_car(self, cx, cy, s):
        lean = max(-10.0, min(10.0, s * 10.0))   # body roll into the turn
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
        s = max(-1.0, min(1.0, angle_deg / max(1.0, lock_deg)))   # -1..+1 steer
        top_c = self.W / 2 + s * 150.0          # far end swings hard with steer
        bot_c = self.W / 2 + s * 22.5           # near end barely moves

        # Road surface + painted edges.
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

        # Scroll the seams and centre dashes toward the viewer (speed sensation).
        self._scroll = (self._scroll + self.CRUISE * min(dt, 0.1)) % 1.0
        n = len(self._seams)
        for i, seam in enumerate(self._seams):
            tt = (((i + self._scroll) / n)) ** 2      # bunch near the horizon
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

        # Car: slide toward the steer, jitter with the bumpy-road amplitude.
        jx = jy = 0.0
        if rumble > 0.0:
            amp = min(11.0, rumble / 12.0)
            jx, jy = random.uniform(-amp, amp), random.uniform(-amp, amp)
        self._place_car(self.W / 2 + s * 70.0 + jx, self.H - 44 + jy, s)


class Dial(tk.Canvas):
    """A steering wheel that actually turns: a thick rim, three spokes rotating
    with the shaft angle, and red markers at the +/- lock-to-lock limits. Sits on
    a dark card (PANEL) beside the scene, matching the panel's dark plots."""

    SIZE = 150

    def __init__(self, master):
        super().__init__(master, width=self.SIZE, height=self.SIZE,
                         bg=PANEL, highlightthickness=0)
        c = self.SIZE / 2
        self._c = c
        self._r = c - 12
        self.create_oval(c - self._r, c - self._r, c + self._r, c + self._r,
                         outline="#2b3240", width=13)          # tyre/grip
        self.create_oval(c - self._r, c - self._r, c + self._r, c + self._r,
                         outline="#3a4150", width=2)           # rim highlight
        self._spokes = [self.create_line(c, c, c, c, fill="#5a6474", width=6,
                                         capstyle="round") for _ in range(3)]
        self._lo = self.create_line(c, c, c, c, fill="#c5221f", width=3)
        self._hi = self.create_line(c, c, c, c, fill="#c5221f", width=3)
        self._hub = self.create_oval(c - 13, c - 13, c + 13, c + 13,
                                     fill="#4fc3f7", outline="")

    def _point(self, deg, r):
        # 0 deg = straight up; positive = clockwise, matching a wheel.
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


def fmt_hz(v):
    """Compact frequency, e.g. 1.50 MHz, 2.4 kHz, 750 Hz."""
    v = float(v)
    if v >= 1_000_000:
        return f"{v / 1_000_000:.2f} MHz"
    if v >= 1_000:
        return f"{v / 1_000:.2f} kHz"
    return f"{v:.0f} Hz"






# ---------------------------------------------------------------------------
# MplPlot — embedded matplotlib figure for waveform / FFT display
# ---------------------------------------------------------------------------

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

        if self.xlabel == "Frequency [Hz]":
            ax.set_xlim(0, n_samples - 1)
            n_ticks = 6
            positions = [int(i * (n_samples - 1) / (n_ticks - 1))
                         for i in range(n_ticks)]
            labels = [f"{p * AUDIO_SAMPLE_RATE / 2 / (n_samples - 1):.0f}"
                      for p in positions]
            ax.set_xticks(positions)
            ax.set_xticklabels(labels)
        else:
            ax.set_xlim(0, n_samples - 1)

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
    """A tiny scrolling line chart hand-drawn on a tk.Canvas -- no matplotlib.

    Newest sample sits at the right edge; older samples scroll off the left.
    The y-axis autoscales to the data window (optionally forced symmetric about
    zero so forward/reverse look balanced). One or more traces share the axis.
    """

    # trace = (label, color, dashed)
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
        # Legend + live value per trace, laid out right-to-left.
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

        # Faint horizontal gridlines for a scope-like feel.
        for frac in (0.25, 0.5, 0.75):
            gy = pad + frac * (h - 2 * pad)
            c.create_line(0, gy, w, gy, fill=GRID)

        # Zero reference line when the window straddles zero.
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
        self.cfg = ffb.Config()  # live FFB effect params (worker reads snapshots)
        self.worker = None       # ffb.FfbWorker
        self.iio = None          # siggen.IioWorker
        self.audio = None        # AudioWorker
        self._t0 = time.perf_counter()
        self._last_frame = self._t0

        root.title("AD-M2KCOMPEDU-EBZ Demo  by Analog Devices")
        root.configure(bg=BG, padx=18, pady=16)
        root.resizable(False, False)
        self._init_theme(root)
        self._load_brand(root)

        # --- header bar (ADI house style) -----------------------------------
        header = tk.Frame(root, bg=BG)
        header.pack(fill="x")
        tk.Label(header, text="AD-M2KCOMPEDU-EBZ", bg=BG, fg=FG,
                 font=("Segoe UI", 16, "bold")).pack(side="left")
        tk.Label(header, text="Evaluation platform demo", bg=BG, fg=MUTED,
                 font=("Segoe UI", 9)).pack(side="left", padx=(12, 0), pady=(8, 0))
        # ADI logo, right-aligned. The left padding is the brand "clear space"
        # (>= half the amp width) so nothing crowds the mark.
        if self.logo_img is not None:
            tk.Label(header, image=self.logo_img, bg=BG).pack(
                side="right", padx=(24, 0)
            )
        tk.Frame(root, bg=ACCENT, height=2).pack(fill="x", pady=(8, 12))

        # --- tabs -----------------------------------------------------------
        nb = ttk.Notebook(root)
        nb.pack(fill="both", expand=True)
        self.motor_tab = tk.Frame(nb, bg=BG)
        self.iio_tab = tk.Frame(nb, bg=BG)
        self.audio_tab = tk.Frame(nb, bg=BG)
        nb.add(self.motor_tab, text="  Serial CAN Motor Control  ")
        nb.add(self.iio_tab, text="  Signal Generator · IIO  ")
        nb.add(self.audio_tab, text="  Audio Amplifier · ADC  ")

        self._build_ffb_tab(self.motor_tab, args)
        self._build_iio_tab(self.iio_tab)
        self._build_audio_tab(self.audio_tab)

        # --- footer credit --------------------------------------------------
        tk.Frame(root, bg=BORDER, height=1).pack(fill="x", pady=(10, 8))
        tk.Label(root, text="© 2026 Analog Devices, Inc. · AD-M2KCOMPEDU-EBZ evaluation demo",
                 bg=BG, fg=MUTED, font=("Segoe UI", 8)).pack(anchor="w")

        self._set_ffb_controls(False)
        self.disc_btn.state(["disabled"])       # nothing to disconnect yet
        self._set_iio_controls(False)
        self.iio_disc_btn.state(["disabled"])
        self.aud_disc_btn.state(["disabled"])
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._refresh()
        self._animate()

    # -- shared UI helpers ----------------------------------------------------
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
        """Amber jumper/mode reminder pinned to the top of a tab."""
        bar = tk.Frame(parent, bg=WARN_BG, highlightbackground=WARN_BORDER,
                       highlightthickness=1, bd=0)
        bar.pack(fill="x", pady=(0, 12))
        tk.Label(bar, text="⚠", bg=WARN_BG, fg=WARN_FG,
                 font=("Segoe UI", 11, "bold")).pack(side="left", padx=(10, 6), pady=8)
        tk.Label(bar, text=text, bg=WARN_BG, fg=WARN_FG, justify="left",
                 wraplength=760, font=("Segoe UI", 8)).pack(
            side="left", padx=(0, 10), pady=8)

    # -- FFB steering-wheel tab ----------------------------------------------
    def _build_ffb_tab(self, tab, args):
        self._banner(
            tab,
            "CAN mode required. Install the CAN jumper, reboot, then open "
            "\"CAN Messenger\" on the device menu so the port speaks slcan. "
            "In SigGen mode (jumper removed) this tab will not connect.",
        )

        # Two columns: connection + effect controls on the left, the live
        # driving view + steering wheel on the right.
        body = tk.Frame(tab, bg=BG)
        body.pack(fill="both", expand=True)
        left = tk.Frame(body, bg=BG)
        left.pack(side="left", fill="y", anchor="n")
        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True, padx=(14, 0))

        # === connection card ================================================
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

        # Status dot + text, shared with _refresh / _status_color.
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

        # === effects card ===================================================
        # Each slider edits one ffb.Config field live; the worker reads a
        # snapshot every tick. Sliders stay enabled even while disconnected so
        # the feel can be dialled in before arming.
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

        # === driving view (right) ===========================================
        view = self._card(right, "Drive view")
        self.scene = Scene(view)
        self.scene.pack()

        # === steering wheel + readouts ======================================
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

        # Torque history strip (reuses the panel's scope-style StripChart).
        self.tchart = StripChart(
            steer, "Commanded torque",
            [("torque", T_UQ, False)],
            fmt=lambda v: f"{v:+d}", width=440, height=90, symmetric=True,
        )

    # -- IIO (signal generator) tab -------------------------------------------
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

        # === connection card ================================================
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

        # === analog output card (altvoltage0) ===============================
        wave = self._card(left, "Analog output — altvoltage0")

        wt_row = tk.Frame(wave, bg=CARD)
        wt_row.pack(fill="x", pady=(0, 8))
        tk.Label(wt_row, text="Wave type", bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8)).pack(side="left")
        self.wave_var = tk.StringVar(value=WAVE_NAMES[2])   # Sine
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

        # === PWM card (altvoltage1) + demo ==================================
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

        # Widgets toggled by connection state / sweep.
        self.iio_form = [self.iio_port_cbo]
        self.iio_controls = [
            self.wave_cbo, self.ampl_scale, self.off_scale, self.freq_entry,
            self.pwm_en_chk, self.duty_scale, self.pwm_freq_entry,
        ]

    # -- audio tab ---------------------------------------------------------------
    def _build_audio_tab(self, tab):
        self._banner(
            tab,
            "Audio demo mode. Connect the M2K signal generator (or mic) to "
            "A2 (pre-amp) and A3 (post-amp). The firmware must be running the "
            "audio sketch (dual-channel ADC capture at 8 kHz).",
        )

        body = tk.Frame(tab, bg=BG)
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=1, uniform="aud")
        body.columnconfigure(1, weight=1, uniform="aud")
        left = tk.Frame(body, bg=BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 7))
        right = tk.Frame(body, bg=BG)
        right.grid(row=0, column=1, sticky="nsew", padx=(7, 0))

        # === connection card ================================================
        self.aud_port_var = tk.StringVar(value=self.default_port)
        conn = self._card(left, "Connection")
        row = tk.Frame(conn, bg=CARD)
        row.pack(fill="x")
        tk.Label(row, text="COM port", bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8)).pack(anchor="w")
        self.aud_port_cbo = ttk.Combobox(row, textvariable=self.aud_port_var,
                                         width=12, values=self._list_ports())
        self.aud_port_cbo.pack(anchor="w", pady=(2, 8))

        stat_row = tk.Frame(conn, bg=CARD)
        stat_row.pack(fill="x", pady=(0, 8))
        self.aud_status_dot = tk.Label(stat_row, text="●", bg=CARD, fg=MUTED,
                                       font=("Segoe UI", 10))
        self.aud_status_dot.pack(side="left", padx=(0, 6))
        self.aud_status_var = tk.StringVar(value="Idle")
        tk.Label(stat_row, textvariable=self.aud_status_var, bg=CARD, fg=FG,
                 font=("Segoe UI", 9)).pack(side="left")

        aud_conn_btns = tk.Frame(conn, bg=CARD)
        aud_conn_btns.pack(fill="x")
        self.aud_conn_btn = ttk.Button(aud_conn_btns, text="Connect",
                                       style="Accent.TButton",
                                       command=self._aud_connect)
        self.aud_conn_btn.pack(side="left", expand=True, fill="x", padx=(0, 3))
        self.aud_disc_btn = ttk.Button(aud_conn_btns, text="Disconnect",
                                       style="Stop.TButton",
                                       command=self._aud_disconnect)
        self.aud_disc_btn.pack(side="left", expand=True, fill="x", padx=(3, 0))

        # === waveform card (raw pre/post) ===================================
        wave_card = self._card(left, "ADC Waveform")
        desc_row = tk.Frame(wave_card, bg=CARD)
        desc_row.pack(fill="x")
        tk.Label(desc_row, text="Raw ADC data: blue = pre-amplifier (A2), "
                 "orange = post-amplifier (A3)",
                 bg=CARD, fg=MUTED, font=("Segoe UI", 7),
                 wraplength=340, justify="left").pack(side="left")
        self.show_post_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(desc_row, text="Show post-amplifier output",
                        variable=self.show_post_var,
                        style="Card.TCheckbutton").pack(side="right")

        self.timebase_var = tk.IntVar(value=AUDIO_BUF_SIZE)
        tb_wrap = tk.Frame(wave_card, bg=CARD)
        tb_wrap.pack(fill="x", pady=(2, 0))
        tb_head = tk.Frame(tb_wrap, bg=CARD)
        tb_head.pack(fill="x")
        tk.Label(tb_head, text="Time base", bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8)).pack(side="left")
        self._tb_val = tk.StringVar(
            value=f"{AUDIO_BUF_SIZE} samples · "
                  f"{AUDIO_BUF_SIZE / AUDIO_SAMPLE_RATE * 1000:.1f} ms")
        tk.Label(tb_head, textvariable=self._tb_val, bg=CARD, fg=RED,
                 font=("Consolas", 9, "bold")).pack(side="right")

        def _tb_moved(v):
            n = int(float(v))
            ms = n / AUDIO_SAMPLE_RATE * 1000
            self._tb_val.set(f"{n} samples · {ms:.1f} ms")

        tk.Scale(tb_wrap, from_=16, to=AUDIO_BUF_SIZE, orient="horizontal",
                 length=300, resolution=8, showvalue=False,
                 variable=self.timebase_var, command=_tb_moved,
                 bg=CARD, fg=FG, troughcolor=TROUGH,
                 activebackground=ACCENT, highlightthickness=0, bd=1,
                 sliderrelief="raised").pack(fill="x")

        self.raw_plot = MplPlot(wave_card, "Pre-amp / Post-amp",
                                     width=440, height=180, fixed_range=(-0.3, 5.3))

        # === filter card (right column) =====================================
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
        slot["src_var"] = tk.StringVar(value="Pre-amp")
        slot["src_cbo"] = ttk.Combobox(
            top_row, textvariable=slot["src_var"], width=9,
            state="readonly", values=["Pre-amp", "Post-amp"])
        slot["src_cbo"].pack(side="left", padx=(6, 0))

        slot["fc_var"] = tk.DoubleVar(value=1000.0)
        slot["fc_scale"] = self._audio_log_slider(
            fc, "Cutoff frequency (Fc) — the -3 dB point of the filter",
            "Hz", 10, 4000, slot["fc_var"],
            lambda _v: self._on_filter_change(0))

        slot["q_var"] = tk.DoubleVar(value=0.707)
        slot["q_scale"] = self._audio_slider(
            fc, "Q factor — sharpness of the filter (0.707 = Butterworth)",
            "", 0.1, 10.0, slot["q_var"],
            lambda _v: self._on_filter_change(0), res=0.01)

        slot["gain_var"] = tk.DoubleVar(value=0.0)
        slot["gain_frame"] = tk.Frame(fc, bg=CARD)
        slot["gain_scale"] = self._audio_slider(
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

    def _audio_log_slider(self, parent, label, unit, lo, hi, var, cmd):
        """Slider with red value readout for the audio filter controls."""
        wrap = tk.Frame(parent, bg=CARD)
        wrap.pack(fill="x", pady=(0, 8))
        head = tk.Frame(wrap, bg=CARD)
        head.pack(fill="x")
        tk.Label(head, text=label, bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8)).pack(side="left")
        valvar = tk.StringVar(value=f"{var.get():.0f} {unit}")
        tk.Label(head, textvariable=valvar, bg=CARD, fg=RED,
                 font=("Consolas", 9, "bold")).pack(side="right")

        def on_move(v):
            valvar.set(f"{float(v):.0f} {unit}")
            cmd(v)

        s = tk.Scale(wrap, from_=lo, to=hi, orient="horizontal", length=300,
                     resolution=1, showvalue=False, variable=var,
                     command=on_move, bg=CARD, fg=FG, troughcolor=TROUGH,
                     activebackground=ACCENT, highlightthickness=0, bd=1,
                     sliderrelief="raised")
        s.pack(fill="x")
        return s

    def _audio_slider(self, parent, label, unit, lo, hi, var, cmd, res=1):
        """Slider with red value readout for the audio filter controls."""
        wrap = tk.Frame(parent, bg=CARD)
        wrap.pack(fill="x", pady=(0, 8))
        head = tk.Frame(wrap, bg=CARD)
        head.pack(fill="x")
        tk.Label(head, text=label, bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8)).pack(side="left")
        valvar = tk.StringVar(value=f"{var.get()} {unit}")
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

    # -- audio control helpers ------------------------------------------------
    def _aud_connect(self):
        if self.audio and self.audio.is_alive():
            return
        self._stop_ffb_worker()
        self._stop_iio()
        port = self.aud_port_var.get().strip()
        if not port:
            self.aud_status_var.set("Bad input: port empty")
            return
        self.aud_conn_btn.state(["disabled"])
        self.aud_disc_btn.state(["!disabled"])
        self.aud_status_var.set("Connecting...")
        self.audio = AudioWorker(port)
        self.audio.start()

    def _aud_disconnect(self):
        if not (self.audio and self.audio.is_alive()):
            return
        self.aud_disc_btn.state(["disabled"])
        self.aud_status_var.set("Disconnecting...")
        self.root.update_idletasks()
        self._stop_audio()
        self.aud_status_var.set("Disconnected")
        self.aud_conn_btn.state(["!disabled"])
        self.aud_conn_btn.configure(text="Connect")

    def _stop_audio(self):
        if self.audio and self.audio.is_alive():
            self.audio.request_stop()
            self.audio.join(timeout=3.0)
        self.audio = None

    def _slider(self, parent, label, unit, lo, hi, var, cmd, res=1):
        wrap = tk.Frame(parent, bg=CARD)
        wrap.pack(fill="x", pady=(0, 8))
        head = tk.Frame(wrap, bg=CARD)
        head.pack(fill="x")
        tk.Label(head, text=label, bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8)).pack(side="left")
        suffix = f" {unit}" if unit else ""

        def _fmt(v):
            # %g keeps fractional resolutions (0.5, 0.05) legible while showing
            # whole numbers without a trailing ".0".
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
        # Apply on Enter or when focus leaves the field.
        e.bind("<Return>", lambda _e: apply_cb())
        e.bind("<FocusOut>", lambda _e: apply_cb())
        ttk.Button(wrap, text="Set", style="Accent.TButton",
                   command=apply_cb).pack(side="left")
        return e

    # -- theme ----------------------------------------------------------------
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

        # Notebook: flat light tabs with an ADI-blue selected tab.
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

    # -- branding -------------------------------------------------------------
    def _load_brand(self, root):
        """Load the ADI logo (header) and amp mark (window icon). References are
        kept on self so Tk doesn't garbage-collect the images to blank."""
        self.logo_img = None
        self.amp_img = None
        try:
            if os.path.exists(LOGO_FILE):
                self.logo_img = tk.PhotoImage(file=LOGO_FILE)
            if os.path.exists(AMP_FILE):
                self.amp_img = tk.PhotoImage(file=AMP_FILE)
                root.iconphoto(True, self.amp_img)
        except tk.TclError:
            # Tk without PNG support (< 8.6): carry on with no imagery.
            self.logo_img = None
            self.amp_img = None

    @staticmethod
    def _list_ports():
        ports = []
        if list_ports is not None:
            ports = [p.device for p in list_ports.comports()]
        return ports

    # -- FFB steering-wheel helpers -------------------------------------------
    def _set_ffb_controls(self, armed):
        # Recenter/STOP live only while armed; the connection form is editable
        # only while disconnected. Effect sliders stay enabled either way.
        for w in (self.recenter_btn, self.stop_btn):
            w.state(["!disabled" if armed else "disabled"])
        for w in self.form_widgets:
            w.state(["disabled" if armed else "!disabled"])

    def _ffb_connect(self):
        if self.worker and self.worker.is_alive():
            return
        # Only one mode can own the port -- drop any other session first.
        self._stop_iio()
        self._stop_audio()
        # Read the form fields; bail with a clear message on bad input.
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
        # De-energize the drive and release the port. The worker's finally-block
        # disarms; _stop_ffb_worker waits for it to finish.
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
        # Zero the steering angle at the current shaft position.
        if self.worker and self.worker.connected:
            self.worker.recenter()

    def _ffb_stop(self):
        # Emergency de-energize: drop the FFB link entirely (worker disarms in
        # its finally-block). Reconnect to arm again.
        self._ffb_disconnect()

    # -- IIO control helpers --------------------------------------------------
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
        """Send every current control value to the worker (used after connect
        and when leaving the demo sweep, to restore the manual settings)."""
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
        # Only one mode can own the port -- drop any other session first.
        self._stop_ffb_worker()
        self._stop_audio()
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
        # Quiet the outputs (worker's finally-block) and release the port.
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
            self._push_all_iio()      # restore the manual settings

    # -- teardown helpers -----------------------------------------------------
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
        # --- Serial CAN motor control (FFB) tab ---
        w = self.worker
        if w:
            # A tripped runaway cutout is the one transient worth hoisting over
            # the steady "Armed" line, so the user can see why the wheel went
            # limp (torque suppressed while it coasts down); amber, not red --
            # it's self-recovering, not a fault.
            if getattr(w, "runaway", False):
                self.status_var.set("Runaway cutout -- coasting (lower spring)")
                self.status_dot.configure(fg=AMBER)
            else:
                self.status_var.set(w.status)
                self.status_dot.configure(
                    fg=self._status_color(w.connected, GREEN if w.armed else ACCENT, w.status))
            self._set_ffb_controls(w.connected)
            if w.connected:
                # Sample the worker's latest telemetry into the torque strip.
                self.tchart.push((w.torque,))
            if w.is_alive():
                # Connected or still connecting: connect locked, disconnect live.
                self.conn_btn.state(["disabled"])
                self.disc_btn.state(["!disabled"])
            else:
                # Worker thread has ended (link dropped or connect failed).
                self.conn_btn.state(["!disabled"])
                self.conn_btn.configure(text="Reconnect")
                self.disc_btn.state(["disabled"])

        # --- IIO tab ---
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

        # --- audio tab ---
        aw = self.audio
        if aw:
            self.aud_status_var.set(aw.status)
            self.aud_status_dot.configure(
                fg=self._status_color(aw.connected, ACCENT, aw.status))
            if aw.is_alive():
                self.aud_conn_btn.state(["disabled"])
                self.aud_disc_btn.state(["!disabled"])
                self.aud_port_cbo.state(["disabled"])
            else:
                self.aud_conn_btn.state(["!disabled"])
                self.aud_conn_btn.configure(text="Reconnect")
                self.aud_disc_btn.state(["disabled"])
                self.aud_port_cbo.state(["!disabled"])
            if aw.has_new_block():
                pre, post = aw.get_buffers()
                tb = self.timebase_var.get()
                traces = {"Pre-amp": (pre[:tb], T_PRE)}
                if self.show_post_var.get() and post:
                    traces["Post-amp"] = (post[:tb], T_POST)
                self.raw_plot.set_traces(traces)

                for slot in self.filter_slots:
                    ftype = slot["type_var"].get()
                    if ftype == "Off":
                        slot["plot"].set_traces({})
                        slot["fft_plot"].set_traces({})
                        slot["state"] = None
                        continue
                    src = pre if slot["src_var"].get() == "Pre-amp" else post
                    fc = slot["fc_var"].get()
                    q = slot["q_var"].get()
                    gain = slot["gain_var"].get()
                    coeffs = biquad_coeffs(ftype, fc, max(q, 0.1),
                                           AUDIO_SAMPLE_RATE, gain)
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

                    freqs_r, db_r = compute_fft_db(src, AUDIO_SAMPLE_RATE)
                    freqs_f, db_f = compute_fft_db(filtered, AUDIO_SAMPLE_RATE)
                    fc_bin = min(range(len(freqs_r)),
                                key=lambda i: abs(freqs_r[i] - fc))
                    fft_ann = [("vline", fc_bin, f"Fc={fc:.0f}Hz")]
                    slot["fft_plot"].set_traces(
                        {"Raw": (db_r, MUTED), "Filtered": (db_f, T_FILT)},
                        annotations=fft_ann)

        self.root.after(150, self._refresh)

    def _animate(self):
        # ~33 Hz redraw of the driving Scene + steering Dial, decoupled from the
        # 150 ms status _refresh so the road scrolls smoothly. Reads the worker's
        # latest telemetry; idle-cruises at 0 deg (no jitter) when not armed.
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
            angle, rumble = 0.0, 0.0   # no bumpy-road jitter until armed
            for var in self.stat_vars.values():
                var.set("--")

        self.scene.update_view(angle, lock_deg, rumble, now - self._t0, dt)
        self.dial.update_view(angle, lock_deg)
        self.root.after(30, self._animate)

    def _on_close(self):
        busy = ((self.worker and self.worker.is_alive()) or
                (self.iio and self.iio.is_alive()) or
                (self.audio and self.audio.is_alive()))
        if busy:
            self.status_var.set("Stopping...")
            self.iio_status_var.set("Stopping...")
            self.aud_status_var.set("Stopping...")
            self.root.update_idletasks()
        self._stop_ffb_worker()
        self._stop_iio()
        self._stop_audio()
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
