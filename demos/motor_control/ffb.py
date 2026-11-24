#!/usr/bin/env python3
"""Force-feedback torque engine for the ADRD3161 / QSH5718 (UI-free backend).

This is the brains of the steering-wheel demo, with no Tkinter in sight: the
drive constants, the live effect Config, the pure torque function, and the
worker thread that arms closed-loop torque and runs the fast host loop. Both
the standalone ffb_wheel.py window and the big ebz_demo.py panel import from
here so there is a single source of truth for how the wheel *feels*.

Transport is canopen over slcan through the board's CAN Messenger, same as
motor_ui.py / adrd_params.py. canopen is imported lazily inside FfbWorker.run
so a UI can launch (and report the missing library) without it installed.
"""

import math
import random
import threading
import time


# ============================ DRIVE CONFIG (CiA 402) =========================
# Modes of operation (0x6060) values. 0x6502 advertises both, so no guessing.
MODE_PT = 4     # Profile Torque -- applies Target torque directly (this EDS has
                # no Torque slope object), holds over pure SDO. Works on this bus.
MODE_CST = 10   # Cyclic Synchronous Torque -- needs a SYNC producer to latch each
                # target; over slcan (no SYNC) it only pulses -> buzz, no hold.
TORQUE_MODE = MODE_PT

# Signed encoder counts per mechanical revolution for ACTUAL_POSITION (0x2090),
# the manufacturer feedback object -- on this firmware the CiA 402 Position
# actual value (0x6064) stays dead at 0, while 0x2090/ACTUAL_VELOCITY track the
# ABN encoder (same objects motor_ui.py uses). Measured with calibrate_angle.py
# (~9108 counts/deg). The sign orients the dial/car readout; flip it to reverse
# which way the car steers for a given wheel turn -- but it is NOT independent of
# TORQUE_SIGN (the torque is computed from this angle), so flip the two TOGETHER
# or the centering force inverts into a runaway. Re-run calibrate_angle.py if you
# re-home or swap the coupling.
CPR = 3278993
# =============================================================================

# Torque direction: +1 or -1. Aligns the motor's Target torque sign with the
# (signed) encoder direction so every effect restores instead of driving away.
# If the wheel RUNS AWAY / spins continuously instead of centering, flip this --
# that's positive feedback from a torque/encoder sign mismatch. Coupled to CPR's
# sign: to reverse the car's steering direction WITHOUT changing the force feel,
# flip this AND CPR together (the two flips cancel in the torque path).
TORQUE_SIGN = 1

# Safety clamp on Target torque (0x6071), in per-mille where +/-1000 == 100% of
# the drive's configured torque-current limit (MAX_TORQUE in params_qsh5718.ini,
# currently 1000 mA ~= 1.0 A -> ~0.67 Nm on the QSH5718, only ~36% of its 2.8 A
# rating). So 1000 here just fully USES the configured budget; to get MORE grunt
# you must raise MAX_TORQUE/MAX_FLUX in the .ini and re-provision to NVM
# (adrd_params.py write). This is the ceiling the Max force slider can reach.
TORQUE_HARD_CLAMP = 1000

# Fraction of the torque budget the centering spring is allowed to use. Capping
# the spring below the clamp reserves the top of the range for the end-stop, so
# hitting the lock feels like a distinct wall instead of "the spring, saturated"
# -- without this the spring pins to the clamp long before the wall engages and
# the end-stop can't add anything you'd feel.
SPRING_HEADROOM = 0.7

# Stiffness-tracking damping floor. Spring and damping are separate sliders, so
# nothing stops a stiff spring being run nearly undamped -- and in this slow,
# delayed loop (two SDO round-trips per tick over slcan, ~45 Hz) a stiff UNDER-damped spring
# doesn't merely overshoot, it sustains a LIMIT CYCLE: the restoring torque
# saturates and lands a tick late, pumping energy back in every swing, so the
# wheel "oscillates hard and never settles" when you let go at max spring. We
# hold a constant damping ratio by flooring damping proportional to spring,
# anchored so the floor is EXACTLY the default damping at the default spring
# (DAMPING_PER_SPRING = default_damping/default_spring = 0.1/2.0 = 0.05). The
# slider still wins wherever it's set higher, so the low-force feel is untouched;
# the floor only ramps in as the spring is turned up (at the max spring of 20 it
# pulls in ~1.0, half the damping slider's own range). Linear, not the textbook
# sqrt(k) critical law, on purpose -- slightly OVER-damping the stiff end is what
# kills the limit cycle. Raise it if max spring still oscillates; lower it if the
# stiff end feels sluggish.
DAMPING_PER_SPRING = 0.05

# Weight on the newest sample in the velocity EMA that feeds the damping term.
# Velocity is a position delta, so heavier smoothing means less quantization
# noise but more LAG -- and lag on the damping term is exactly what lets a fast
# oscillation slip past it. Kept high enough that damping stays roughly in phase
# with a few-Hz wheel oscillation; lower it only if the damping feels buzzy.
VEL_SMOOTH = 0.5

# Runaway cutout. No hand turns a wheel this fast, so a velocity past this is by
# definition the instability above driving the shaft, not the user. When tripped
# we command ZERO torque (not a fight-back), which immediately breaks the
# positive-feedback loop and lets the shaft coast down on its own friction;
# normal torque resumes automatically once it's back under the threshold.
RUNAWAY_DPS = 1500.0


class Config:
    """Live effect parameters, edited by the GUI thread, read by the worker.

    Guarded by its own lock; the worker takes a cheap snapshot each tick.
    Gains are expressed against angle in DEGREES and velocity in DEG/S so the
    sliders read in intuitive units.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.spring = 2.0        # per-mille torque per degree of offset
        self.damping = 0.1       # per-mille torque per (deg/s)
        self.lock_deg = 360.0    # +/- end-stop angle (lock-to-lock = 2x this)
        self.endstop = 30.0      # wall stiffness past the lock (per degree over)
        self.rumble = 0.0        # bumpy-road amplitude (per-mille torque)
        self.rumble_hz = 10.0    # road buzz frequency (keep well below loop Hz)
        self.max_force = 700.0   # user force clamp (<= TORQUE_HARD_CLAMP)

    def snapshot(self):
        with self.lock:
            return (self.spring, self.damping, self.lock_deg, self.endstop,
                    self.rumble, self.rumble_hz, self.max_force)

    def set(self, **kw):
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)


def compute_torque(angle_deg, vel_dps, t, snap):
    """The whole FFB feel in one pure function: torque as f(angle, velocity, t).

    Additive primitives -- adding a new effect is adding a term. Returns a
    signed integer suitable for Target torque, clamped to the user force limit
    and then the hard safety clamp.
    """
    spring, damping, lock_deg, endstop, rumble, rumble_hz, max_force = snap
    clamp = min(max_force, TORQUE_HARD_CLAMP)

    # Centering spring, soft-capped below the clamp. Small-angle feel is
    # unchanged (linear); it just stops climbing once it reaches the cap, which
    # leaves the top of the torque budget free for the end-stop wall.
    spring_cap = SPRING_HEADROOM * clamp
    tau = max(-spring_cap, min(spring_cap, -spring * angle_deg))

    # Velocity damping (weight, kills oscillation). Floor it proportional to the
    # spring so a stiff spring can't be run undamped into the delay-driven limit
    # cycle; the user's slider wins whenever it's already above the floor, so the
    # low-force feel is unchanged.
    damping = max(damping, DAMPING_PER_SPRING * spring)
    tau -= damping * vel_dps

    if rumble > 0.0:                   # bumpy road: a FIXED-amplitude buzz, so it's
        buzz = math.sin(2.0 * math.pi * rumble_hz * t)   # felt even at a standstill.
        tau += rumble * buzz           # No speed gate on purpose -- gating the buzz
                                       # by the wheel's own velocity is self-exciting
                                       # (buzz -> motion -> wider gate -> more buzz),
                                       # which ramps into a growing wobble/spin.

    # Hard end-stop past lock-to-lock: a steep wall stacked ON TOP of the
    # (capped) spring, so it slams from the spring cap up to the full clamp
    # within a couple of degrees -- a crisp wall you can distinctly feel.
    over = abs(angle_deg) - lock_deg
    if over > 0.0:
        tau -= math.copysign(endstop * over, angle_deg)

    return int(max(-clamp, min(clamp, tau)))


class FfbWorker(threading.Thread):
    """Owns the canopen node. Arms closed-loop torque, then every tick reads the
    shaft angle, computes a torque from the effect stack, and commands it.
    Velocity is derived from position deltas so each tick is one read + one
    write rather than two reads."""

    LOOP_DT = 0.008  # 125 Hz target; real rate is bounded by slcan SDO latency

    def __init__(self, node_id, port, eds, cfg):
        super().__init__(daemon=True)
        self._node_id = node_id
        self._port = port
        self._eds = eds
        self._cfg = cfg

        self._stop = threading.Event()
        self._recenter = threading.Event()
        self._center_counts = 0

        # Snapshot read by the UI thread.
        self.status = "Connecting..."
        self.connected = False
        self.armed = False
        self.raw = 0            # raw Position actual value (for CPR calibration)
        self.angle_deg = 0.0
        self.vel_dps = 0.0
        self.torque = 0
        self.loop_hz = 0.0      # measured loop rate; caps renderable rumble_hz
        self.runaway = False    # True while the velocity cutout is suppressing torque
        self.err_total = 0      # cumulative dropped-SDO/link errors this session
        self.err_streak = 0     # consecutive failing ticks (0 == currently healthy)

    def request_stop(self):
        self._stop.set()

    def recenter(self):
        self._recenter.set()

    def _disarm(self, node):
        # Zero torque, then drop OPERATION ENABLED via the controlword and fall
        # back to a safe, de-energized state. Order matters: kill torque first.
        try:
            node.sdo["Target torque"].raw = 0
            node.sdo["Controlword"].raw = 0x0000   # disable voltage -> SWITCH ON DISABLED
            node.nmt.state = "PRE-OPERATIONAL"
        finally:
            self.armed = False

    def run(self):
        try:
            import canopen
        except Exception as exc:  # noqa: BLE001
            self.status = f"canopen library missing: {exc}"
            return

        net = canopen.Network()
        try:
            net.connect(interface="slcan", channel=self._port,
                        bitrate=500000, ttyBaudrate=921600)
            # Plain SDO node -- deliberately NOT BaseNode402. The library drives
            # the 402 state machine through a controlword RPDO that never actuates
            # over this slcan bridge, so its enable walk times out ("Timeout when
            # trying to change state"). The manual controlword sequence below --
            # proven by enable_probe.py to reach OPERATION ENABLED -- talks pure
            # SDO, the same transport style as motor_ui.py / adrd_params.py.
            node = net.add_node(self._node_id, self._eds)
            # Give slow/late SDO responses room to arrive before the client
            # aborts the transfer (abort 0x05040000, "SDO protocol timed out").
            # Default is 0.3 s -- too tight for the slcan/USB-CDC bridge under
            # the sustained fast loop, especially on slower Linux hosts.
            node.sdo.RESPONSE_TIMEOUT = 1.0
            node.nmt.wait_for_heartbeat()
            # Everything here is driven over SDO; we never read a PDO. Going
            # OPERATIONAL only makes the drive stream its TPDOs, which the slcan
            # bridge forwards to the host as a constant background flood -- enough
            # to overflow the serial buffer on a slower Linux host (e.g. the Pi)
            # and drop SDO response frames (abort 0x05040000, "No SDO response").
            # The identical flood is harmless on a faster host that drains the
            # serial stream quicker than it fills. PRE-OPERATIONAL keeps SDO fully
            # available while silencing the PDO traffic, so the buffer never backs
            # up. The CiA 402 drive state machine (controlword/torque below) is
            # independent of NMT state and still arms/holds torque here.
            node.nmt.state = "PRE-OPERATIONAL"
            time.sleep(0.5)

            node.sdo["Controlword"].raw = 0x0080          # clear any latched fault
            time.sleep(0.2)
            node.sdo["Modes of operation"].raw = TORQUE_MODE
            time.sleep(0.2)
            node.sdo["Controlword"].raw = 0x0006          # shutdown  -> READY TO SWITCH ON
            time.sleep(0.2)
            node.sdo["Controlword"].raw = 0x0007          # switch on -> SWITCHED ON
            time.sleep(0.2)
            node.sdo["Controlword"].raw = 0x000F          # enable op -> OPERATION ENABLED

            # A cold enable triggers ABN encoder/commutation init (ABN_1_INIT_DELAY
            # ~1s in the .ini plus a small alignment move); the drive holds at
            # SWITCHED ON until that finishes, then steps to OPERATION ENABLED. Poll
            # rather than read once, and hold 0x0F in case the first frame landed
            # mid-init.
            deadline = time.perf_counter() + 5.0
            while True:
                sw = node.sdo["Statusword"].raw
                if (sw & 0x6F) == 0x27:                    # 0x27 == OPERATION ENABLED
                    break
                if sw & 0x08:                              # fault bit -> stop waiting
                    raise RuntimeError(f"drive faulted during enable (Statusword=0x{sw:04X})")
                if time.perf_counter() > deadline:
                    raise RuntimeError(
                        f"drive did not reach OPERATION ENABLED (Statusword=0x{sw:04X})")
                node.sdo["Controlword"].raw = 0x000F
                time.sleep(0.2)

            time.sleep(0.3)   # commutation align / feedback settle
            self._center_counts = node.sdo["ACTUAL_POSITION"].raw
            self.armed = True
            self.connected = True
            self.status = f"Armed (CiA 402 torque mode {TORQUE_MODE})"
        except Exception as exc:  # noqa: BLE001 - surface bring-up failures to the UI
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
                    # Velocity from position deltas, lightly EMA-filtered so the
                    # damping term isn't riding on encoder quantization noise.
                    vel = (angle - prev_angle) / dt if dt > 0 else 0.0
                    self.vel_dps = (1.0 - VEL_SMOOTH) * self.vel_dps + VEL_SMOOTH * vel
                    prev_angle = angle

                    tau = compute_torque(angle, self.vel_dps, now - t0,
                                         self._cfg.snapshot())

                    # Runaway cutout: if the shaft is moving faster than a hand
                    # ever could, the loop is driving it (not the user). Command
                    # zero torque to break the positive feedback and let it coast
                    # down; normal torque resumes once it's back under threshold.
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
                        # Recovered on this tick; report the clean streak reset.
                        self.err_streak = 0
                        self.status = f"Armed (recovered, {self.err_total} drops)"
                except Exception as exc:  # noqa: BLE001
                    # A dropped SDO response (abort 0x05040000 / "No SDO response")
                    # leaves the slcan parser possibly mid-frame; flush both the
                    # parser buffer and the OS serial buffer so the NEXT tick
                    # starts byte-aligned instead of limping on a desynced stream.
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
