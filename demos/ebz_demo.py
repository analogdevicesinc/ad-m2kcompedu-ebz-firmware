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
import os
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import ttk

try:
    from serial.tools import list_ports
except Exception:  # pyserial should be present, but be safe
    list_ports = None


# COMMUTATION_MODE (0x2004) enum -- APP_SHADOW, resets to 0 every boot.
SYSTEM_OFF = 0
FOC_OPENLOOP_CURRENT = 4

# IIO signal-generator wave types (altvoltage0 "wavetype"), index == value.
WAVE_NAMES = ["Off", "DC", "Sine", "Square", "Triangle"]

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


def int_flexible(x):
    return int(x, 0)


class MotorWorker(threading.Thread):
    """Owns the canopen network/node. Connects, arms open loop, then pushes the
    latest target velocity and polls actual velocity until told to stop."""

    def __init__(self, node_id, port, eds, current):
        super().__init__(daemon=True)
        self._node_id = node_id
        self._port = port
        self._eds = eds
        self._current = current

        self._lock = threading.Lock()
        self._target = 0
        self._target_dirty = False
        self._disarm_req = False
        self._stop = threading.Event()

        # Snapshot read by the UI thread (plain attribute stores are atomic in
        # CPython, so the UI can read these without holding the lock).
        self.status = "Connecting..."
        self.connected = False
        self.armed = False
        self.actual = 0            # ACTUAL_VELOCITY
        self.current_a = 0         # ACTUAL_TOTAL_MOTOR_CURRENT
        self.uq = 0                # FOC_VOLTAGE_UQ (q-axis voltage, rises w/ speed)
        self.cmd_vel = 0           # velocity currently commanded (for plotting)

    def set_target(self, velocity):
        with self._lock:
            if velocity != self._target:
                self._target = int(velocity)
                self._target_dirty = True

    def disarm(self):
        """STOP: zero the velocity and de-energize the motor (COMMUTATION off),
        but stay connected. The next non-zero target re-arms automatically."""
        with self._lock:
            self._target = 0
            self._disarm_req = True

    def request_stop(self):
        self._stop.set()

    def _arm(self, node):
        # FOC open-loop current commutation (see open_loop_spin.py). Must be
        # redone every time: COMMUTATION_MODE is APP_SHADOW and drops to 0.
        node.sdo["MOTOR_TYPE"].raw = 2            # stepper
        node.sdo["MOTOR_POLE_PAIRS"].raw = 50
        if self._current is not None:
            node.sdo["OPENLOOP_CURRENT"].raw = self._current
        node.sdo["DIRECT_VELOCITY_MODE"].raw = 1  # apply target directly, no ramp
        node.sdo["COMMUTATION_MODE"].raw = FOC_OPENLOOP_CURRENT
        self.armed = True

    def _disarm(self, node):
        node.sdo["TARGET_VELOCITY"].raw = 0
        time.sleep(0.3)
        node.sdo["COMMUTATION_MODE"].raw = SYSTEM_OFF
        self.armed = False

    def run(self):
        try:
            import canopen
        except Exception as exc:  # noqa: BLE001
            self.status = f"canopen library missing: {exc}"
            return

        net = canopen.Network()
        try:
            net.connect(
                interface="slcan",
                channel=self._port,
                bitrate=500000,
                ttyBaudrate=115200,
            )
            node = net.add_node(self._node_id, self._eds)
            node.nmt.wait_for_heartbeat()
            node.nmt.state = "OPERATIONAL"
            self._arm(node)

            self.connected = True
            self.status = "Armed"
        except Exception as exc:  # noqa: BLE001 - surface any bring-up failure to the UI
            self.status = f"Connect failed: {exc}"
            try:
                net.disconnect()
            except Exception:
                pass
            return

        try:
            while not self._stop.is_set():
                with self._lock:
                    target = self._target
                    dirty = self._target_dirty
                    self._target_dirty = False
                    disarm_req = self._disarm_req
                    self._disarm_req = False

                try:
                    if disarm_req:
                        self._disarm(node)          # full stop + de-energize
                        self.status = "Stopped (disarmed)"
                    elif dirty:
                        if target != 0 and not self.armed:
                            self._arm(node)          # re-arm on demand
                            self.status = "Armed"
                        node.sdo["TARGET_VELOCITY"].raw = target
                    self.cmd_vel = target
                    # Fast telemetry every loop (feeds the charts).
                    self.actual = node.sdo["ACTUAL_VELOCITY"].raw
                    self.current_a = node.sdo["ACTUAL_TOTAL_MOTOR_CURRENT"].raw
                    self.uq = node.sdo["FOC_VOLTAGE_UQ"].raw
                except Exception as exc:  # noqa: BLE001
                    self.status = f"Link error: {exc}"

                time.sleep(0.12)
        finally:
            # Always stop and de-energize, whatever got us here.
            try:
                self._disarm(node)
            except Exception:
                pass
            try:
                net.disconnect()
            except Exception:
                pass
            self.connected = False
            self.status = "Disconnected"


class IioWorker(threading.Thread):
    """Owns the libiio context to the board's iiod server. Applies signal-gen /
    PWM attribute changes, or replays the demos/iio wave sweep, until stopped.

    Parameter keys (UI -> worker): wave, ampl, freq, off (altvoltage0) and
    pwm_en, pwm_duty, pwm_freq (altvoltage1)."""

    def __init__(self, port):
        super().__init__(daemon=True)
        self._port = port
        self._lock = threading.Lock()
        self._desired = {}       # key -> value, pending writes
        self._dirty = False
        self._sweep = False
        self._stop = threading.Event()

        self.status = "Connecting..."
        self.connected = False
        self.sweeping = False
        self.applied = {}        # last-applied value per key (for read-out)

    def set_param(self, key, value):
        with self._lock:
            self._desired[key] = value
            self._dirty = True

    def set_sweep(self, on):
        with self._lock:
            self._sweep = bool(on)

    def request_stop(self):
        self._stop.set()

    @staticmethod
    def _build_sweep_frames():
        """Flatten the demos/iio/waves.py animation into (key, value, delay)
        frames so it can be stepped one frame per loop and stopped instantly."""
        frames = []

        def add(key, val, delay=0.02):
            frames.append((key, val, delay))

        add("ampl", 1000); add("freq", 1000); add("off", 0)
        add("pwm_freq", 1000); add("pwm_duty", 0); add("pwm_en", 1)
        for wt in (2, 3, 4):                      # Sine, Square, Triangle
            add("wave", wt, 0.1)
            for duty in range(0, 101, 5):         # fade PWM in
                add("pwm_duty", duty, 0.01)
            for duty in range(100, -1, -5):       # ...and out
                add("pwm_duty", duty, 0.01)
            for f in range(1000, 2550, 50):       # sweep frequency up
                add("freq", f)
            for f in range(2500, 950, -50):       # ...and down
                add("freq", f)
            for a in range(1000, 2550, 50):       # sweep amplitude up
                add("ampl", a)
            for a in range(2500, 950, -50):       # ...and down
                add("ampl", a)
            for o in range(0, 300, 50):           # sweep offset
                add("off", o)
            for o in range(250, -300, -50):
                add("off", o)
            for o in range(-250, 50, 50):
                add("off", o)
            add("wave", wt, 0.5)                  # brief pause between waves
        return frames

    @staticmethod
    def _apply(ch0, ch1, key, val):
        if key == "wave":
            ch0.attrs["wavetype"].value = str(int(val))
        elif key == "ampl":
            ch0.attrs["amplitude"].value = str(int(val))
        elif key == "freq":
            ch0.attrs["frequency"].value = str(float(val))
        elif key == "off":
            ch0.attrs["offset"].value = str(int(val))
        elif key == "pwm_en":
            ch1.attrs["enable"].value = str(int(val))
        elif key == "pwm_duty":
            ch1.attrs["duty_cycle"].value = str(int(val))
        elif key == "pwm_freq":
            ch1.attrs["frequency"].value = str(float(val))

    def run(self):
        try:
            import iio
        except Exception as exc:  # noqa: BLE001
            self.status = f"iio library missing: {exc}"
            return

        uri = f"serial:{self._port},115200,8n1"
        try:
            ctx = iio.Context(uri)
            dev = ctx.find_device("m2kcompedu-ebz")
            if dev is None:
                raise RuntimeError(
                    "device 'm2kcompedu-ebz' not found -- is the board in "
                    "SigGen mode (CAN jumper removed)?"
                )
            ch0 = dev.find_channel("altvoltage0", True)   # True = output
            ch1 = dev.find_channel("altvoltage1", True)
            if ch0 is None or ch1 is None:
                raise RuntimeError("altvoltage0/1 output channels not found")
            self.connected = True
            self.status = "Connected"
        except Exception as exc:  # noqa: BLE001
            self.status = f"Connect failed: {exc}"
            return

        frames = self._build_sweep_frames()
        idx = 0
        try:
            while not self._stop.is_set():
                with self._lock:
                    sweep = self._sweep
                    pending = self._desired
                    self._desired = {}
                    dirty = self._dirty
                    self._dirty = False
                self.sweeping = sweep

                try:
                    if sweep:
                        key, val, delay = frames[idx]
                        self._apply(ch0, ch1, key, val)
                        self.applied[key] = val
                        idx = (idx + 1) % len(frames)
                        self.status = "Running demo sweep"
                        time.sleep(delay)
                        continue
                    if dirty:
                        for k, v in pending.items():
                            self._apply(ch0, ch1, k, v)
                            self.applied[k] = v
                        self.status = "Connected"
                except Exception as exc:  # noqa: BLE001
                    self.status = f"Link error: {exc}"

                time.sleep(0.05)
        finally:
            # Leave the outputs quiet: wave off, PWM disabled.
            try:
                ch0.attrs["wavetype"].value = "0"
                ch1.attrs["enable"].value = "0"
            except Exception:
                pass
            try:
                del ctx
            except Exception:
                pass
            self.connected = False
            self.status = "Disconnected"


def fmt_vel(v):
    """Compact signed velocity, e.g. -1.20 M, 850 k, 0."""
    a = abs(v)
    if a >= 1_000_000:
        return f"{v / 1_000_000:+.2f} M"
    if a >= 1_000:
        return f"{v / 1_000:+.0f} k"
    return f"{v:+d}"


def fmt_hz(v):
    """Compact frequency, e.g. 1.50 MHz, 2.4 kHz, 750 Hz."""
    v = float(v)
    if v >= 1_000_000:
        return f"{v / 1_000_000:.2f} MHz"
    if v >= 1_000:
        return f"{v / 1_000:.2f} kHz"
    return f"{v:.0f} Hz"


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
        self.max_vel = args.max
        self.eds = args.eds
        self.default_port = args.iface
        self.worker = None       # MotorWorker
        self.iio = None          # IioWorker

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
        nb.add(self.motor_tab, text="  Motor Drive · CAN  ")
        nb.add(self.iio_tab, text="  Signal Generator · IIO  ")

        self._build_motor_tab(self.motor_tab, args)
        self._build_iio_tab(self.iio_tab)

        # --- footer credit --------------------------------------------------
        tk.Frame(root, bg=BORDER, height=1).pack(fill="x", pady=(10, 8))
        tk.Label(root, text="© 2026 Analog Devices, Inc. · AD-M2KCOMPEDU-EBZ evaluation demo",
                 bg=BG, fg=MUTED, font=("Segoe UI", 8)).pack(anchor="w")

        self._set_controls(False)
        self.disc_btn.state(["disabled"])       # nothing to disconnect yet
        self._set_iio_controls(False)
        self.iio_disc_btn.state(["disabled"])
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._refresh()

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

    # -- motor tab ------------------------------------------------------------
    def _build_motor_tab(self, tab, args):
        self._banner(
            tab,
            "CAN mode required. Install the CAN jumper, reboot, then open "
            "\"CAN Messenger\" on the device menu so the port speaks slcan. "
            "In SigGen mode (jumper removed) this tab will not connect.",
        )

        # Two columns: controls on the left, telemetry on the right, so the
        # three charts sit beside the controls instead of running off-screen.
        body = tk.Frame(tab, bg=BG)
        body.pack(fill="both", expand=True)
        left = tk.Frame(body, bg=BG)
        left.pack(side="left", fill="y", anchor="n")
        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True, padx=(14, 0))

        # === connection card ================================================
        self.node_var = tk.StringVar(value=hex(args.node))
        self.port_var = tk.StringVar(value=args.iface)
        self.curr_var = tk.StringVar(value=str(args.current))
        self.speed_var = tk.StringVar(value=str(args.max))

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

        ports = self._list_ports()

        field(0, 0, "Node ID", self.node_var, 8)
        field(0, 1, "COM port", self.port_var, 10, values=ports)
        field(1, 0, "Current [mA]", self.curr_var, 8)
        field(1, 1, "Speed (slider ±)", self.speed_var, 10)

        conn_btns = tk.Frame(conn, bg=CARD)
        conn_btns.pack(fill="x", pady=(6, 0))
        self.conn_btn = ttk.Button(conn_btns, text="Connect", style="Accent.TButton",
                                   command=self._connect)
        self.conn_btn.pack(side="left", expand=True, fill="x", padx=(0, 3))
        self.disc_btn = ttk.Button(conn_btns, text="Disconnect", style="Stop.TButton",
                                   command=self._disconnect)
        self.disc_btn.pack(side="left", expand=True, fill="x", padx=(3, 0))

        # === drive card =====================================================
        drive = self._card(left, "Drive")

        self.dir_var = tk.StringVar(value="=")
        self.big_var = tk.StringVar(value="0")
        head = tk.Frame(drive, bg=CARD)
        head.pack(fill="x")
        self.dir_lbl = tk.Label(head, textvariable=self.dir_var, bg=CARD, fg=MUTED,
                                font=("Segoe UI", 36, "bold"))
        self.dir_lbl.pack(side="left", padx=(0, 14))
        vcol = tk.Frame(head, bg=CARD)
        vcol.pack(side="left")
        tk.Label(vcol, text="TARGET VELOCITY", bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8)).pack(anchor="w")
        self.big_lbl = tk.Label(vcol, textvariable=self.big_var, bg=CARD, fg=FG,
                                font=("Segoe UI", 30, "bold"))
        self.big_lbl.pack(anchor="w")

        self.vel = tk.IntVar(value=0)
        self.scale = tk.Scale(
            drive, from_=-self.max_vel, to=self.max_vel, orient="horizontal",
            length=440, resolution=max(1, self.max_vel // 200), showvalue=False,
            variable=self.vel, command=self._on_slide,
            bg=CARD, fg=FG, troughcolor=TROUGH, activebackground=ACCENT,
            highlightthickness=0, bd=1, sliderrelief="raised",
        )
        self.scale.pack(fill="x", pady=(14, 2))

        ends = tk.Frame(drive, bg=CARD)
        ends.pack(fill="x")
        self.end_lo = tk.Label(ends, text=f"◀ {fmt_vel(-self.max_vel)}", bg=CARD,
                               fg=MUTED, font=("Segoe UI", 8))
        self.end_hi = tk.Label(ends, text=f"{fmt_vel(self.max_vel)} ▶", bg=CARD,
                               fg=MUTED, font=("Segoe UI", 8))
        self.end_lo.pack(side="left")
        self.end_hi.pack(side="right")

        btns = tk.Frame(drive, bg=CARD)
        btns.pack(fill="x", pady=(14, 0))
        self.rev_btn = ttk.Button(btns, text="◀ REVERSE", style="Rev.TButton",
                                  command=lambda: self._jam(-self.max_vel))
        self.stop_btn = ttk.Button(btns, text="■ STOP", style="Stop.TButton",
                                   command=self._stop_motor)
        self.fwd_btn = ttk.Button(btns, text="FORWARD ▶", style="Fwd.TButton",
                                  command=lambda: self._jam(self.max_vel))
        self.rev_btn.pack(side="left", expand=True, fill="x", padx=(0, 3))
        self.stop_btn.pack(side="left", expand=True, fill="x", padx=3)
        self.fwd_btn.pack(side="left", expand=True, fill="x", padx=(3, 0))

        # === telemetry card =================================================
        tele = self._card(right, "Telemetry")

        stat_row = tk.Frame(tele, bg=CARD)
        stat_row.pack(fill="x", pady=(0, 4))
        self.status_dot = tk.Label(stat_row, text="●", bg=CARD, fg=MUTED,
                                   font=("Segoe UI", 10))
        self.status_dot.pack(side="left", padx=(0, 6))
        self.status_var = tk.StringVar(value="Idle")
        tk.Label(stat_row, textvariable=self.status_var, bg=CARD, fg=FG,
                 font=("Segoe UI", 9)).pack(side="left")

        # Live numeric read-outs pulled straight from the drive.
        grid = tk.Frame(tele, bg=CARD)
        grid.pack(fill="x", pady=(8, 4))
        for c in range(3):
            grid.columnconfigure(c, weight=1, uniform="stat")
        self.stat_vars = {}

        def stat(idx, key, label):
            cell = tk.Frame(grid, bg=CARD)
            cell.grid(row=idx // 3, column=idx % 3, sticky="w", pady=(0, 8))
            tk.Label(cell, text=label.upper(), bg=CARD, fg=MUTED,
                     font=("Segoe UI", 7)).pack(anchor="w")
            var = tk.StringVar(value="--")
            self.stat_vars[key] = var
            tk.Label(cell, textvariable=var, bg=CARD, fg=FG,
                     font=("Consolas", 12, "bold")).pack(anchor="w")

        stat(0, "target", "Target vel")
        stat(1, "actual", "Actual vel")
        stat(2, "current", "Current")
        stat(3, "uq", "FOC UQ")

        self.vchart = StripChart(
            tele, "Velocity",
            [("target", T_TGT, True), ("actual", T_VEL, False)],
            fmt=fmt_vel, width=440, height=120, symmetric=True,
        )
        self.ichart = StripChart(
            tele, "Motor current",
            [("actual", T_CUR, False)],
            fmt=lambda v: f"{v:+d} mA", width=440, height=90,
        )
        self.uqchart = StripChart(
            tele, "FOC q-axis voltage (UQ)",
            [("UQ", T_UQ, False)],
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

    def _slider(self, parent, label, unit, lo, hi, var, cmd, res=1):
        wrap = tk.Frame(parent, bg=CARD)
        wrap.pack(fill="x", pady=(0, 8))
        head = tk.Frame(wrap, bg=CARD)
        head.pack(fill="x")
        tk.Label(head, text=label, bg=CARD, fg=MUTED,
                 font=("Segoe UI", 8)).pack(side="left")
        valvar = tk.StringVar(value=f"{var.get()} {unit}")
        tk.Label(head, textvariable=valvar, bg=CARD, fg=FG,
                 font=("Consolas", 9)).pack(side="right")

        def on_move(v):
            valvar.set(f"{int(float(v))} {unit}")
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

    # -- motor control helpers ------------------------------------------------
    def _set_controls(self, armed):
        # Drive controls live only while armed; the connection form is editable
        # only while disconnected.
        for w in (self.rev_btn, self.stop_btn, self.fwd_btn):
            w.state(["!disabled" if armed else "disabled"])
        self.scale.configure(state=("normal" if armed else "disabled"))
        for w in self.form_widgets:
            w.state(["disabled" if armed else "!disabled"])

    def _apply_range(self, max_vel):
        self.max_vel = max_vel
        self.scale.configure(
            from_=-max_vel, to=max_vel, resolution=max(1, max_vel // 200)
        )
        self.vel.set(0)
        self.end_lo.configure(text=f"◀ {fmt_vel(-max_vel)}")
        self.end_hi.configure(text=f"{fmt_vel(max_vel)} ▶")
        self._on_slide(None)

    def _on_slide(self, _val):
        v = self.vel.get()
        self.big_var.set(fmt_vel(v))
        self.dir_var.set("▶" if v > 0 else "◀" if v < 0 else "=")
        color = GREEN if v > 0 else RED if v < 0 else MUTED
        self.dir_lbl.configure(fg=color)
        self.big_lbl.configure(fg=(FG if v == 0 else color))
        if self.worker and self.worker.connected:
            self.worker.set_target(v)

    def _jam(self, v):
        self.vel.set(v)
        self._on_slide(None)

    def _stop_motor(self):
        # Full stop like open_loop_spin.py: zero velocity + de-energize. Stays
        # connected; moving the slider off zero re-arms the drive.
        self.vel.set(0)
        self._on_slide(None)
        if self.worker and self.worker.connected:
            self.worker.disarm()

    def _connect(self):
        if self.worker and self.worker.is_alive():
            return
        # Only one mode can own the port -- drop any IIO session first.
        self._stop_iio()
        # Read the form fields; bail with a clear message on bad input.
        try:
            node = int_flexible(self.node_var.get().strip())
            port = self.port_var.get().strip()
            current = int(self.curr_var.get().strip())
            speed = int(self.speed_var.get().strip())
            if not port or speed <= 0:
                raise ValueError("port empty or speed <= 0")
        except (ValueError, TypeError) as exc:
            self.status_var.set(f"Bad input: {exc}")
            return

        self._apply_range(speed)
        self.vchart.clear()
        self.ichart.clear()
        self.uqchart.clear()
        self.conn_btn.state(["disabled"])
        self.disc_btn.state(["!disabled"])
        self.status_var.set("Connecting...")
        self.worker = MotorWorker(node, port, self.eds, current)
        self.worker.start()

    def _disconnect(self):
        # Stop + de-energize the motor and release the port. The worker's
        # finally-block disarms; _stop_motor_worker waits for it to finish.
        if not (self.worker and self.worker.is_alive()):
            return
        self.vel.set(0)
        self._on_slide(None)
        self.disc_btn.state(["disabled"])
        self.status_var.set("Disconnecting...")
        self.root.update_idletasks()
        self._stop_motor_worker()
        self.status_var.set("Disconnected")
        self._set_controls(False)
        self.conn_btn.state(["!disabled"])
        self.conn_btn.configure(text="Connect")

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
        # Only one mode can own the port -- drop any motor session first.
        self._stop_motor_worker()
        port = self.iio_port_var.get().strip()
        if not port:
            self.iio_status_var.set("Bad input: port empty")
            return
        self.iio_conn_btn.state(["disabled"])
        self.iio_disc_btn.state(["!disabled"])
        self.iio_status_var.set("Connecting...")
        self.iio = IioWorker(port)
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
    def _stop_motor_worker(self):
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
        # --- motor tab ---
        w = self.worker
        if w:
            self.status_var.set(w.status)
            self.status_dot.configure(
                fg=self._status_color(w.connected, GREEN if w.armed else ACCENT, w.status))
            armed = w.connected
            self._set_controls(armed)
            if w.connected:
                # Sample the worker's latest telemetry into the charts + grid.
                self.vchart.push((w.cmd_vel, w.actual))
                self.ichart.push((w.current_a,))
                self.uqchart.push((w.uq,))
                self.stat_vars["target"].set(fmt_vel(w.cmd_vel))
                self.stat_vars["actual"].set(fmt_vel(w.actual))
                self.stat_vars["current"].set(f"{w.current_a:+d} mA")
                self.stat_vars["uq"].set(f"{w.uq:+d}")
            else:
                for var in self.stat_vars.values():
                    var.set("--")
            if w.is_alive():
                # Connected or still connecting: connect locked, disconnect live.
                self.conn_btn.state(["disabled"])
                self.disc_btn.state(["!disabled"])
                if w.armed:
                    self.worker.set_target(self.vel.get())
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

        self.root.after(150, self._refresh)

    def _on_close(self):
        busy = (self.worker and self.worker.is_alive()) or (self.iio and self.iio.is_alive())
        if busy:
            self.status_var.set("Stopping...")
            self.iio_status_var.set("Stopping...")
            self.root.update_idletasks()
        self._stop_motor_worker()
        self._stop_iio()
        self.root.destroy()


def main():
    p = argparse.ArgumentParser(description="AD-M2KCOMPEDU-EBZ demo panel")
    p.add_argument("node", type=int_flexible, nargs="?", default=0x16, help="Target node ID (motor)")
    p.add_argument("-i", "--iface", default="COM4", help="slcan / IIO serial COM port")
    p.add_argument("-e", "--eds", default=DEFAULT_EDS, help="EDS file (motor)")
    p.add_argument("--max", type=int, default=1_000_000, help="Slider range (+/-)")
    p.add_argument("--current", type=int, default=1000,
                   help="Open-loop current [mA]")
    args = p.parse_args()

    root = tk.Tk()
    App(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
