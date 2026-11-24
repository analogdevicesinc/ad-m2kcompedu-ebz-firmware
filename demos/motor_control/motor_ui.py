#!/usr/bin/env python3
"""ADRD3161 open-loop spin UI.

A tiny Tkinter panel that drives the TMC9660 in FOC open-loop current mode and
lets you slide the target velocity from full reverse to full forward. Same
arming sequence as open_loop_spin.py (manufacturer objects, no SYNC / no
CiA-402 state machine), so it needs nothing on the bus but the board's CAN
Messenger (slcan) on the COM port.

All CAN traffic runs on a worker thread so the slider stays smooth, and the
drive is always commanded to zero and de-energized when the window closes.

Usage:
    python motor_ui.py                 # node 0x17, COM4
    python motor_ui.py 0x17 -i COM4 --max 2000000 --current 2500
"""

import argparse
import threading
import time
import tkinter as tk
from tkinter import ttk

import canopen

try:
    from serial.tools import list_ports
except Exception:  # pyserial should be present (canopen slcan needs it), but be safe
    list_ports = None


# COMMUTATION_MODE (0x2004) enum -- APP_SHADOW, resets to 0 every boot.
SYSTEM_OFF = 0
FOC_OPENLOOP_CURRENT = 4


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

        # Snapshot read by the UI thread.
        self.status = "Connecting..."
        self.connected = False
        self.armed = False
        self.actual = 0

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
                    self.actual = node.sdo["ACTUAL_VELOCITY"].raw
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


def fmt_vel(v):
    """Compact signed velocity, e.g. -1.20 M, 850 k, 0."""
    a = abs(v)
    if a >= 1_000_000:
        return f"{v / 1_000_000:+.2f} M"
    if a >= 1_000:
        return f"{v / 1_000:+.0f} k"
    return f"{v:+d}"


class App:
    def __init__(self, root, args):
        self.root = root
        self.max_vel = args.max
        self.eds = args.eds
        self.worker = None

        root.title("ADRD3161 Motor Control")
        root.configure(padx=16, pady=14)
        root.resizable(False, False)

        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        # --- connection form -------------------------------------------------
        self.node_var = tk.StringVar(value=hex(args.node))
        self.port_var = tk.StringVar(value=args.iface)
        self.curr_var = tk.StringVar(value=str(args.current))
        self.speed_var = tk.StringVar(value=str(args.max))

        form = ttk.Frame(root)
        form.pack(fill="x")
        self.form_widgets = []

        def field(row, col, label, var, width, values=None):
            ttk.Label(form, text=label, font=("Segoe UI", 8)).grid(
                row=row * 2, column=col, sticky="w", padx=(0, 10)
            )
            if values is not None:
                w = ttk.Combobox(form, textvariable=var, width=width, values=values)
            else:
                w = ttk.Entry(form, textvariable=var, width=width)
            w.grid(row=row * 2 + 1, column=col, sticky="w", padx=(0, 10), pady=(0, 6))
            self.form_widgets.append(w)
            return w

        ports = []
        if list_ports is not None:
            ports = [p.device for p in list_ports.comports()]
        if self.port_var.get() not in ports:
            ports.insert(0, self.port_var.get())

        field(0, 0, "Node ID", self.node_var, 8)
        field(0, 1, "COM port", self.port_var, 10, values=ports)
        field(1, 0, "Current [mA]", self.curr_var, 8)
        field(1, 1, "Speed (slider ±)", self.speed_var, 10)

        self.conn_btn = ttk.Button(root, text="Connect", command=self._connect)
        self.conn_btn.pack(fill="x", pady=(4, 12))

        ttk.Separator(root, orient="horizontal").pack(fill="x", pady=(0, 12))

        # --- direction / velocity readout -----------------------------------
        self.dir_var = tk.StringVar(value="=")
        self.big_var = tk.StringVar(value="0")
        head = ttk.Frame(root)
        head.pack(fill="x")
        self.dir_lbl = ttk.Label(head, textvariable=self.dir_var, font=("Segoe UI", 34))
        self.dir_lbl.pack(side="left", padx=(0, 12))
        col = ttk.Frame(head)
        col.pack(side="left")
        ttk.Label(col, text="TARGET VELOCITY", font=("Segoe UI", 8)).pack(anchor="w")
        ttk.Label(col, textvariable=self.big_var, font=("Segoe UI", 28, "bold")).pack(anchor="w")

        # --- slider ----------------------------------------------------------
        self.vel = tk.IntVar(value=0)
        self.scale = tk.Scale(
            root,
            from_=-self.max_vel,
            to=self.max_vel,
            orient="horizontal",
            length=440,
            resolution=max(1, self.max_vel // 200),
            showvalue=False,
            variable=self.vel,
            command=self._on_slide,
            troughcolor="#d9dde3",
            highlightthickness=0,
        )
        self.scale.pack(pady=(14, 2))

        ends = ttk.Frame(root)
        ends.pack(fill="x")
        self.end_lo = ttk.Label(ends, text=f"◀ {fmt_vel(-self.max_vel)}", font=("Segoe UI", 8))
        self.end_hi = ttk.Label(ends, text=f"{fmt_vel(self.max_vel)} ▶", font=("Segoe UI", 8))
        self.end_lo.pack(side="left")
        self.end_hi.pack(side="right")

        # --- buttons ---------------------------------------------------------
        btns = ttk.Frame(root)
        btns.pack(fill="x", pady=(12, 6))
        self.rev_btn = ttk.Button(btns, text="◀ Reverse", command=lambda: self._jam(-self.max_vel))
        self.stop_btn = ttk.Button(btns, text="■ STOP", command=self._stop_motor)
        self.fwd_btn = ttk.Button(btns, text="Forward ▶", command=lambda: self._jam(self.max_vel))
        self.rev_btn.pack(side="left", expand=True, fill="x", padx=2)
        self.stop_btn.pack(side="left", expand=True, fill="x", padx=2)
        self.fwd_btn.pack(side="left", expand=True, fill="x", padx=2)

        # --- status / actual -------------------------------------------------
        foot = ttk.Frame(root)
        foot.pack(fill="x", pady=(8, 0))
        self.status_var = tk.StringVar(value="Idle")
        self.actual_var = tk.StringVar(value="actual: --")
        ttk.Label(foot, textvariable=self.status_var, font=("Segoe UI", 9)).pack(side="left")
        ttk.Label(foot, textvariable=self.actual_var, font=("Consolas", 9)).pack(side="right")

        self._set_controls(False)
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._refresh()

    # -- control helpers ------------------------------------------------------
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
        self.dir_lbl.configure(
            foreground=("#137333" if v > 0 else "#c5221f" if v < 0 else "#5f6368")
        )
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
        self.conn_btn.state(["disabled"])
        self.status_var.set("Connecting...")
        self.worker = MotorWorker(node, port, self.eds, current)
        self.worker.start()

    def _refresh(self):
        w = self.worker
        if w:
            self.status_var.set(w.status)
            self.actual_var.set(f"actual: {w.actual:+d}" if w.connected else "actual: --")
            armed = w.connected
            self._set_controls(armed)
            if armed:
                self.conn_btn.state(["disabled"])
                # Keep the drive tracking the slider in case a write got dropped,
                # but only while actually armed -- don't re-arm after a STOP.
                if w.armed:
                    self.worker.set_target(self.vel.get())
            elif not w.is_alive():
                self.conn_btn.state(["!disabled"])
                self.conn_btn.configure(text="Reconnect")
        self.root.after(150, self._refresh)

    def _on_close(self):
        if self.worker and self.worker.is_alive():
            self.status_var.set("Stopping...")
            self.root.update_idletasks()
            self.worker.request_stop()
            self.worker.join(timeout=3.0)
        self.root.destroy()


def main():
    p = argparse.ArgumentParser(description="ADRD3161 open-loop spin UI")
    p.add_argument("node", type=int_flexible, nargs="?", default=0x16, help="Target node ID")
    p.add_argument("-i", "--iface", default="COM4", help="slcan COM port")
    p.add_argument("-e", "--eds", default="adrd3161.eds", help="EDS file")
    p.add_argument("--max", type=int, default=1_000_000, help="Slider range (+/-)")
    p.add_argument("--current", type=int, default=1000,
                   help="Open-loop current [mA]")
    args = p.parse_args()

    root = tk.Tk()
    App(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
