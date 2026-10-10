#!/usr/bin/env python3
"""Relay the Isaac sim's standard topics from a prefix (run_sim.py --topic-prefix /isaac) to their normal names,
from a system-ROS (/opt/ros/humble) process.

Why: during the 2026-10-09 hospital SLAM survey, Nav2's costmap nodes (controller_server, planner_server) stopped
updating within minutes: their TF buffers stopped advancing (published footprints froze, "Transform data too old",
the planner planned from a stale pose, the processes sat at 0 % CPU and ignored SIGINT). Other subscribers (rclpy
nodes, tf2_echo) were fine and the same Nav2 stack on the MuJoCo sim did not do it. What this relay changes:
  * Nav2 only ever talks to system-ROS writers;
  * sensor data is held back until the TF it needs has been published (/scan: odom->pelvis and pelvis->laser_level
    at the scan's stamp; /livox/lidar: also robot_state_publisher's torso TF at that stamp), so the costmaps'
    tf2 MessageFilters never have to queue a message and wait for its transform (suspected deadlock: in Humble the
    MessageFilter callbacks run on the TF listener thread);
  * --cloud-stride N keeps every Nth MID-360 point (smaller DDS samples than the ~260 kB clouds; loopback socket
    buffers are 208 kB on this machine and cannot be raised without root).

    source env.sh && python3 g1-isaac-sim/tools/isaac_relay.py [--prefix /isaac] [--cloud-stride 4]
"""
import argparse
import array
import collections

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage

RAW = [("/clock", Clock, 10), ("/odom", Odometry, 10), ("/joint_states", JointState, 10)]


def st(h):
    return h.stamp.sec + 1e-9 * h.stamp.nanosec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefix", default="/isaac")
    ap.add_argument("--cloud-stride", type=int, default=1)
    ap.add_argument("--max-hold", type=float, default=0.5, help="publish a held sensor message anyway after N s wall")
    a, _ = ap.parse_known_args()
    rclpy.init()
    n = Node("isaac_relay")
    keep = []
    for name, typ, depth in RAW:
        pub = n.create_publisher(typ, name, depth)
        keep.append(n.create_subscription(typ, a.prefix + name, lambda m, p=pub: p.publish(m), depth, raw=True))
        keep.append(pub)
    tf_pub = n.create_publisher(TFMessage, "/tf", 100)
    scan_pub = n.create_publisher(LaserScan, "/scan", 5)  # reliable: slam_toolbox subscribes reliable
    pc_pub = n.create_publisher(PointCloud2, "/livox/lidar", qos_profile_sensor_data)
    seen = {"pelvis": -1.0, "laser_level": -1.0, "rsp": -1.0}
    held = collections.deque()  # (kind, stamp, wall, msg)
    import time as _time

    def flush():
        now = _time.monotonic()
        while held:
            kind, t, w, m = held[0]
            ok = seen["pelvis"] >= t and seen["laser_level"] >= t
            if kind == "cloud":
                ok = ok and seen["rsp"] >= t
            if not ok and now - w < a.max_hold:
                break
            held.popleft()
            (scan_pub if kind == "scan" else pc_pub).publish(m)

    def on_tf(m):
        tf_pub.publish(m)
        for t in m.transforms:
            if t.child_frame_id in ("pelvis", "laser_level"):
                seen[t.child_frame_id] = max(seen[t.child_frame_id], st(t.header))
        flush()

    def on_rsp_tf(m):  # robot_state_publisher's link TF (from /joint_states): needed for the cloud's mid360_link
        for t in m.transforms:
            if t.child_frame_id == "torso_link":
                seen["rsp"] = max(seen["rsp"], st(t.header))
                flush()
                return

    def on_scan(m):
        held.append(("scan", st(m.header), _time.monotonic(), m))
        flush()

    def on_cloud(m):
        if a.cloud_stride > 1 and m.point_step:
            d = np.frombuffer(bytes(m.data), np.uint8).reshape(-1, m.point_step)[::a.cloud_stride]
            m.width, m.height = d.shape[0], 1
            m.row_step = m.point_step * d.shape[0]
            m.data = array.array("B", d.tobytes())
        held.append(("cloud", st(m.header), _time.monotonic(), m))
        flush()

    keep.append(n.create_subscription(TFMessage, a.prefix + "/tf", on_tf, 100))
    keep.append(n.create_subscription(TFMessage, "/tf", on_rsp_tf, 100))
    keep.append(n.create_subscription(LaserScan, a.prefix + "/scan", on_scan, qos_profile_sensor_data))
    keep.append(n.create_subscription(PointCloud2, a.prefix + "/livox/lidar", on_cloud, qos_profile_sensor_data))
    keep.append(n.create_timer(0.05, flush))
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    ps = n.create_publisher(TFMessage, "/tf_static", latched)
    keep.append(n.create_subscription(TFMessage, a.prefix + "/tf_static", lambda m: ps.publish(m), latched, raw=True))
    n.get_logger().info(f"relaying {a.prefix}/{{clock,tf,tf_static,odom,joint_states,scan,livox/lidar}} -> /..., "
                        f"sensor data held until its TF is out, cloud stride {a.cloud_stride}")
    rclpy.spin(n)


if __name__ == "__main__":
    main()
