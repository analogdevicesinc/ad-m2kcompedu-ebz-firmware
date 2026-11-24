
import struct
import threading

try:
    import serial as _serial
except Exception:
    _serial = None

ADC_BUF_SIZE = 256
ADC_SAMPLE_RATE = 8000


class AdcCaptureWorker(threading.Thread):

    _ADC_TO_VOLTS = 5.0 / 16383.0

    def __init__(self, port):
        super().__init__(daemon=True)
        self._port = port
        self._lock = threading.Lock()
        self._stop = threading.Event()

        self.status = "Connecting..."
        self.connected = False
        self.pre_buf = []
        self.post_buf = []
        self._new_block = False

    def request_stop(self):
        self._stop.set()

    def has_new_block(self):
        with self._lock:
            if self._new_block:
                self._new_block = False
                return True
            return False

    def get_buffers(self):
        with self._lock:
            return list(self.pre_buf), list(self.post_buf)

    def _read_exact(self, ser, n):
        buf = b""
        while len(buf) < n and not self._stop.is_set():
            chunk = ser.read(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf if len(buf) == n else None

    def run(self):
        if _serial is None:
            self.status = "pyserial library missing"
            return

        try:
            ser = _serial.Serial(self._port, 921600, timeout=0.5)
            ser.reset_input_buffer()
            self.connected = True
            self.status = "Connected"
        except Exception as exc:
            self.status = f"Connect failed: {exc}"
            return

        try:
            while not self._stop.is_set():
                b = ser.read(1)
                if not b:
                    continue
                if b[0] != 0xAA:
                    continue
                b2 = ser.read(1)
                if not b2 or b2[0] != 0x55:
                    continue

                ch_byte = ser.read(1)
                if not ch_byte:
                    continue
                channels = ch_byte[0]

                pre_raw = self._read_exact(ser, ADC_BUF_SIZE * 2)
                if pre_raw is None:
                    continue
                pre = [v * self._ADC_TO_VOLTS
                       for v in struct.unpack(f"<{ADC_BUF_SIZE}h", pre_raw)]

                post = []
                if channels >= 2:
                    post_raw = self._read_exact(ser, ADC_BUF_SIZE * 2)
                    if post_raw is None:
                        continue
                    post = [v * self._ADC_TO_VOLTS
                            for v in struct.unpack(f"<{ADC_BUF_SIZE}h", post_raw)]

                with self._lock:
                    self.pre_buf = pre
                    self.post_buf = post
                    self._new_block = True
        finally:
            try:
                ser.close()
            except Exception:
                pass
            self.connected = False
            self.status = "Disconnected"
