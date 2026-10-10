"""Fast rt/lowstate writer: builds the unitree_hg LowState_ CDR payload with numpy instead of cyclonedds'
per-field Python serializer (~0.65 ms + 0.35 ms CRC per sample -> ~0.05 ms), then hands the bytes to the
same CycloneDDS writer. Verified byte-for-byte against the slow path at start-up (falls back if it differs).

The numpy buffer mirrors Unitree's C struct (CRC pack format '<2I2B2xI' + '13fh2x' + 'B3x4f2hf7I' * 35 + '40B5I',
2092 bytes), which the CRC is computed over. The XCDR1 wire payload (2088 bytes) differs only in that CDR has no
struct-end padding: motor_state[0].mode follows imu_state.temperature directly (offset 70, not 72).
"""

from __future__ import annotations

import ctypes
import os

import numpy as np

_MOTOR = np.dtype([("mode", "u1"), ("_p", "u1", 3), ("q", "<f4"), ("dq", "<f4"), ("ddq", "<f4"), ("tau_est", "<f4"),
                   ("temperature", "<i2", 2), ("vol", "<f4"), ("sensor", "<u4", 2), ("motorstate", "<u4"),
                   ("reserve", "<u4", 4)])
_IMU = np.dtype([("quaternion", "<f4", 4), ("gyroscope", "<f4", 3), ("accelerometer", "<f4", 3), ("rpy", "<f4", 3),
                 ("temperature", "<i2"), ("_p", "u1", 2)])
LOWSTATE = np.dtype([("version", "<u4", 2), ("mode_pr", "u1"), ("mode_machine", "u1"), ("_p", "u1", 2),
                     ("tick", "<u4"), ("imu", _IMU), ("motor", _MOTOR, 35), ("wireless_remote", "u1", 40),
                     ("reserve", "<u4", 4), ("crc", "<u4")])
assert LOWSTATE.itemsize == 2092, LOWSTATE.itemsize


class FastLowState:
    def __init__(self, writer, slow_sample, crc_obj, sdk_dir):
        from cyclonedds._clayer import ddspy_write

        self._write = ddspy_write
        self._writer = writer
        self.buf = np.zeros(1, dtype=LOWSTATE)
        self.m = self.buf["motor"][0]
        self.imu = self.buf["imu"][0]
        lib = ctypes.CDLL(os.path.join(sdk_dir, "utils/lib/crc_amd64.so"))
        lib.crc32_core.argtypes = (ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32)
        lib.crc32_core.restype = ctypes.c_uint32
        self._crc = lib.crc32_core
        self._words = self.buf.view(np.uint32)  # 523 words; crc = over the first 522
        self._ptr = self._words.ctypes.data_as(ctypes.POINTER(ctypes.c_uint32))
        self.ok = self._selftest(slow_sample, crc_obj)

    def _fill_from(self, s):
        b = self.buf[0]
        b["version"] = list(s.version)
        b["mode_pr"], b["mode_machine"], b["tick"] = s.mode_pr, s.mode_machine, s.tick
        i = s.imu_state
        b["imu"]["quaternion"], b["imu"]["gyroscope"] = i.quaternion, i.gyroscope
        b["imu"]["accelerometer"], b["imu"]["rpy"], b["imu"]["temperature"] = i.accelerometer, i.rpy, i.temperature
        for k, ms in enumerate(s.motor_state):
            m = self.m[k]
            m["mode"], m["q"], m["dq"], m["ddq"], m["tau_est"] = ms.mode, ms.q, ms.dq, ms.ddq, ms.tau_est
            m["temperature"], m["vol"], m["sensor"], m["motorstate"] = ms.temperature, ms.vol, ms.sensor, ms.motorstate
        b["wireless_remote"] = list(s.wireless_remote)

    def _selftest(self, s, crc_obj):
        s.tick = 1234
        s.motor_state[3].q = 0.5
        s.imu_state.quaternion = [1.0, 0.0, 0.0, 0.0]
        s.crc = crc_obj.Crc(s)
        ref = s.serialize()
        self._fill_from(s)
        c = self.crc_value()
        self.buf["crc"] = c
        mine = self.header(ref) + self._cdr()
        ok = c == s.crc and mine == ref
        self._hdr = ref[:4]
        return ok

    @staticmethod
    def header(ref):
        return ref[:4]

    def crc_value(self):
        return self._crc(self._ptr, 522)

    def _cdr(self):
        raw = self.buf.tobytes()
        return raw[:70] + raw[72:73] + b"\0" + raw[76:]

    def write(self):
        self.buf["crc"] = self.crc_value()
        self._write(self._writer._ref, self._hdr + self._cdr())
