"""Tiny ROS 2 test client for the Isaac (or MuJoCo) backend: zero-velocity PD hold at the default pose.

    bash -c 'source env.sh; python g1-isaac-sim/tools/lowcmd_hold_test.py --seconds 8'

Publishes /lowcmd (unitree_hg/LowCmd, CRC filled with Unitree's crc32 routine) at 500 Hz and listens to
/lowstate: reports its receive rate, tick progression, and the IMU roll/pitch, and fails if the robot tips over.
"""

import argparse
import ctypes
import math
import os
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_hg.msg import LowCmd, LowState

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import g1_config as C  # noqa: E402

_CRC_LIB = os.path.join(os.path.dirname(HERE), "pydeps/unitree_sdk2py/utils/lib/crc_amd64.so")
_FMT = "<2B2x" + "B3x5fI" * 35 + "5I"  # unitree_hg LowCmd wire layout (1004 bytes)


class Crc:
    def __init__(self):
        self.lib = ctypes.CDLL(_CRC_LIB)
        self.lib.crc32_core.argtypes = (ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32)
        self.lib.crc32_core.restype = ctypes.c_uint32

    def lowcmd(self, m):
        vals = [m.mode_pr, m.mode_machine]
        for c in m.motor_cmd:
            vals += [c.mode, c.q, c.dq, c.tau, c.kp, c.kd, c.reserve]
        vals += list(m.reserve) + [0]
        raw = struct.pack(_FMT, *vals)
        n = len(raw) // 4 - 1
        buf = (ctypes.c_uint32 * n).from_buffer_copy(raw[: 4 * n])
        return self.lib.crc32_core(buf, n)


class HoldTest(Node):
    def __init__(self, a):
        super().__init__("g1_lowcmd_hold_test")
        self.a = a
        self.crc = Crc()
        default_q, src = C.load_default_qpos()
        self.q0 = default_q
        self.kp, self.kd = (C.STAND_KP, C.STAND_KD) if a.gains == "stand" else (C.DEFAULT_KP, C.DEFAULT_KD)
        self.get_logger().info(f"gains: {a.gains} (legs kp {self.kp[0]:.0f} kd {self.kd[0]:.1f})")
        self.get_logger().info(f"default pose from {src}")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT) if a.best_effort else 10
        self.pub = self.create_publisher(LowCmd, "/lowcmd", qos)
        self.sub = self.create_subscription(LowState, "/lowstate", self.on_state, qos)
        self.n_state = 0
        self.first_tick = None
        self.last_tick = None
        self.mode_machine = 5
        self.max_tilt = 0.0
        self.last_rpy = (0.0, 0.0, 0.0)
        self.t_first = None
        self.sent = 0
        self.timer = self.create_timer(1.0 / a.rate, self.tick)

    def on_state(self, m):
        now = time.monotonic()
        if self.t_first is None:
            self.t_first = now
            self.get_logger().info(f"first /lowstate: tick={m.tick} mode_machine={m.mode_machine}")
        self.n_state += 1
        self.first_tick = m.tick if self.first_tick is None else self.first_tick
        self.last_tick = m.tick
        self.mode_machine = m.mode_machine
        r, p, y = m.imu_state.rpy
        self.last_rpy = (r, p, y)
        self.max_tilt = max(self.max_tilt, abs(r), abs(p))

    def tick(self):
        m = LowCmd()
        m.mode_pr = 0
        m.mode_machine = self.mode_machine
        for i in range(29):
            c = m.motor_cmd[i]
            c.mode = 1
            c.q = float(self.q0[i])
            c.dq = 0.0
            c.tau = 0.0
            c.kp = float(self.kp[i])
            c.kd = float(self.kd[i])
        m.crc = self.crc.lowcmd(m)
        self.pub.publish(m)
        self.sent += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--rate", type=float, default=500.0)
    ap.add_argument("--best-effort", action="store_true")
    ap.add_argument("--gains", choices=["stand", "ultra"], default="stand",
                    help="stand: stiff test-stand PD (legs 500/10, waist 300/6, arms ULTRA); ultra: ULTRA policy gains")
    ap.add_argument("--keep-harness", action="store_true", help="do not release the sim harness first")
    ap.add_argument("--wait", type=float, default=60.0, help="max seconds to wait for the first /lowstate")
    a = ap.parse_args()
    rclpy.init()
    n = HoldTest(a)
    t0 = time.monotonic()
    while rclpy.ok() and n.t_first is None and time.monotonic() - t0 < a.wait:
        rclpy.spin_once(n, timeout_sec=0.05)
    if n.t_first is None:
        print("FAIL: no /lowstate received")
        sys.exit(2)
    if not a.keep_harness:
        from std_srvs.srv import SetBool

        cli = n.create_client(SetBool, "/sim/harness")
        if cli.wait_for_service(timeout_sec=3.0):
            fut = cli.call_async(SetBool.Request(data=False))
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 3.0:
                rclpy.spin_once(n, timeout_sec=0.05)
            print(f"harness: {fut.result().message if fut.done() else 'no reply'}; settling 2 s under PD only")
            t0 = time.monotonic()
            while time.monotonic() - t0 < 2.0:
                rclpy.spin_once(n, timeout_sec=0.01)
            n.max_tilt = 0.0
        else:
            print("no /sim/harness service (no harness in this sim?)")
    n0, s0, tk0, t1 = n.n_state, n.sent, n.last_tick, time.monotonic()
    while rclpy.ok() and time.monotonic() - t1 < a.seconds:
        rclpy.spin_once(n, timeout_sec=0.01)
    el = time.monotonic() - t1
    rate = (n.n_state - n0) / el
    sim_s = ((n.last_tick - tk0) & 0xFFFFFFFF) / 1000.0
    r, p, y = n.last_rpy
    print(f"/lowstate rx rate {rate:.1f} Hz over {el:.1f} s wall; sim advanced {sim_s:.2f} s (RTF {sim_s / el:.2f}); "
          f"/lowcmd sent {n.sent - s0} ({(n.sent - s0) / el:.1f} Hz)")
    print(f"IMU rpy at end ({math.degrees(r):.2f}, {math.degrees(p):.2f}, {math.degrees(y):.2f}) deg; "
          f"max |roll|/|pitch| {math.degrees(n.max_tilt):.2f} deg")
    ok = n.max_tilt < math.radians(20)
    print("PASS: upright" if ok else "FAIL: tipped over")
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
