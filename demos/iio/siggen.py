#!/usr/bin/env python3
#
# siggen.py
#
# Copyright (c) 2026 Analog Devices, Inc.
#
# SPDX-License-Identifier: Apache-2.0
#

import threading
import time


WAVE_NAMES = ["Off", "DC", "Sine", "Square", "Triangle"]


class IioWorker(threading.Thread):

    def __init__(self, port):
        super().__init__(daemon=True)
        self._port = port
        self._lock = threading.Lock()
        self._desired = {}
        self._dirty = False
        self._sweep = False
        self._stop = threading.Event()

        self.status = "Connecting..."
        self.connected = False
        self.sweeping = False
        self.applied = {}

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
        frames = []

        def add(key, val, delay=0.02):
            frames.append((key, val, delay))

        add("ampl", 1000); add("freq", 1000); add("off", 0)
        add("pwm_freq", 1000); add("pwm_duty", 0); add("pwm_en", 1)
        for wt in (2, 3, 4):
            add("wave", wt, 0.1)
            for duty in range(0, 101, 5):
                add("pwm_duty", duty, 0.01)
            for duty in range(100, -1, -5):
                add("pwm_duty", duty, 0.01)
            for f in range(1000, 2550, 50):
                add("freq", f)
            for f in range(2500, 950, -50):
                add("freq", f)
            for a in range(1000, 2550, 50):
                add("ampl", a)
            for a in range(2500, 950, -50):
                add("ampl", a)
            for o in range(0, 300, 50):
                add("off", o)
            for o in range(250, -300, -50):
                add("off", o)
            for o in range(-250, 50, 50):
                add("off", o)
            add("wave", wt, 0.5)
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
        except Exception as exc:
            self.status = f"iio library missing: {exc}"
            return

        uri = f"serial:{self._port},921600,8n1"
        try:
            ctx = iio.Context(uri)
            dev = ctx.find_device("m2kcompedu-ebz")
            if dev is None:
                raise RuntimeError(
                    "device 'm2kcompedu-ebz' not found -- is the board in "
                    "SigGen mode (CAN jumper removed)?"
                )
            ch0 = dev.find_channel("altvoltage0", True)
            ch1 = dev.find_channel("altvoltage1", True)
            if ch0 is None or ch1 is None:
                raise RuntimeError("altvoltage0/1 output channels not found")
            self.connected = True
            self.status = "Connected"
        except Exception as exc:
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
                except Exception as exc:
                    self.status = f"Link error: {exc}"

                time.sleep(0.05)
        finally:

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
