#!/usr/bin/env python3

import math
import random
import threading
import time


MODE_PT = 4

MODE_CST = 10

TORQUE_MODE = MODE_PT


CPR = 3278993


TORQUE_SIGN = 1


TORQUE_HARD_CLAMP = 1000


SPRING_HEADROOM = 0.7


DAMPING_PER_SPRING = 0.05


VEL_SMOOTH = 0.5


RUNAWAY_DPS = 1500.0


class Config:

    def __init__(self):
        self.lock = threading.Lock()
        self.spring = 2.0
        self.damping = 0.1
        self.lock_deg = 360.0
        self.endstop = 30.0
        self.rumble = 0.0
        self.rumble_hz = 10.0
        self.max_force = 700.0

    def snapshot(self):
        with self.lock:
            return (self.spring, self.damping, self.lock_deg, self.endstop,
                    self.rumble, self.rumble_hz, self.max_force)

    def set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)


def compute_torque(angle_deg, vel_dps, t, snap):
    spring, damping, lock_deg, endstop, rumble, rumble_hz, max_force = snap
    clamp = min(max_force, TORQUE_HARD_CLAMP)

    spring_cap = SPRING_HEADROOM * clamp
    tau = max(-spring_cap, min(spring_cap, -spring * angle_deg))

    damping = max(damping, DAMPING_PER_SPRING * spring)
    tau -= damping * vel_dps

    if rumble > 0.0:
        buzz = math.sin(2.0 * math.pi * rumble_hz * t)
        tau += rumble * buzz

    over = abs(angle_deg) - lock_deg
    if over > 0.0:
        tau -= math.copysign(endstop * over, angle_deg)

    return int(max(-clamp, min(clamp, tau)))


class FfbWorker(threading.Thread):

    LOOP_DT = 0.008

    def __init__(self, node_id, port, eds, cfg):
        super().__init__(daemon=True)
        self._node_id = node_id
        self._port = port
        self._eds = eds
        self._cfg = cfg

        self._stop = threading.Event()
        self._recenter = threading.Event()
        self._center_counts = 0

        self.status = "Connecting..."
        self.connected = False
        self.armed = False
        self.raw = 0
        self.angle_deg = 0.0
        self.vel_dps = 0.0
        self.torque = 0
        self.loop_hz = 0.0
        self.runaway = False
        self.err_total = 0
        self.err_streak = 0

    def request_stop(self):
        self._stop.set()

    def recenter(self):
        self._recenter.set()

    def _disarm(self, node):

        try:
            node.sdo["Target torque"].raw = 0
            node.sdo["Controlword"].raw = 0x0000
            node.nmt.state = "PRE-OPERATIONAL"
        finally:
            self.armed = False

    def run(self):
        try:
            import canopen
        except Exception as exc:
            self.status = f"canopen library missing: {exc}"
            return

        net = canopen.Network()
        try:
            net.connect(interface="slcan", channel=self._port,
                        bitrate=500000, ttyBaudrate=921600)

            node = net.add_node(self._node_id, self._eds)

            node.sdo.RESPONSE_TIMEOUT = 1.0
            node.nmt.wait_for_heartbeat()

            node.nmt.state = "PRE-OPERATIONAL"
            time.sleep(0.5)

            node.sdo["Controlword"].raw = 0x0080
            time.sleep(0.2)
            node.sdo["Modes of operation"].raw = TORQUE_MODE
            time.sleep(0.2)
            node.sdo["Controlword"].raw = 0x0006
            time.sleep(0.2)
            node.sdo["Controlword"].raw = 0x0007
            time.sleep(0.2)
            node.sdo["Controlword"].raw = 0x000F

            deadline = time.perf_counter() + 5.0
            while True:
                sw = node.sdo["Statusword"].raw
                if (sw & 0x6F) == 0x27:
                    break
                if sw & 0x08:
                    raise RuntimeError(f"drive faulted during enable (Statusword=0x{sw:04X})")
                if time.perf_counter() > deadline:
                    raise RuntimeError(
                        f"drive did not reach OPERATION ENABLED (Statusword=0x{sw:04X})")
                node.sdo["Controlword"].raw = 0x000F
                time.sleep(0.2)

            time.sleep(0.3)
            self._center_counts = node.sdo["ACTUAL_POSITION"].raw
            self.armed = True
            self.connected = True
            self.status = f"Armed (CiA 402 torque mode {TORQUE_MODE})"
        except Exception as exc:
            self.status = f"Connect failed: {exc}"
            try:
                net.disconnect()
            except Exception:
                pass
            return

        prev_angle = self.angle_deg
        prev_t = time.perf_counter()
        t0 = prev_t
        try:
            while not self._stop.is_set():
                now = time.perf_counter()
                dt = now - prev_t
                if dt < self.LOOP_DT:
                    time.sleep(self.LOOP_DT - dt)
                    now = time.perf_counter()
                    dt = now - prev_t
                prev_t = now
                if dt > 0:
                    self.loop_hz = 0.9 * self.loop_hz + 0.1 * (1.0 / dt)

                try:
                    if self._recenter.is_set():
                        self._center_counts = node.sdo["ACTUAL_POSITION"].raw
                        self._recenter.clear()

                    raw = node.sdo["ACTUAL_POSITION"].raw
                    angle = 360.0 * (raw - self._center_counts) / CPR

                    vel = (angle - prev_angle) / dt if dt > 0 else 0.0
                    self.vel_dps = (1.0 - VEL_SMOOTH) * self.vel_dps + VEL_SMOOTH * vel
                    prev_angle = angle

                    tau = compute_torque(angle, self.vel_dps, now - t0,
                                         self._cfg.snapshot())

                    if abs(self.vel_dps) > RUNAWAY_DPS:
                        tau = 0
                        self.runaway = True
                    else:
                        self.runaway = False

                    node.sdo["Target torque"].raw = TORQUE_SIGN * tau

                    self.raw = raw
                    self.angle_deg = angle
                    self.torque = tau
                    if self.err_streak:

                        self.err_streak = 0
                        self.status = f"Armed (recovered, {self.err_total} drops)"
                except Exception as exc:

                    self.err_total += 1
                    self.err_streak += 1
                    try:
                        net.bus.flush()
                    except Exception:
                        pass
                    self.status = (f"Link error (#{self.err_total}, "
                                   f"streak {self.err_streak}): {exc}")
        finally:
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
