#!/usr/bin/env python3
"""ROS 2 check of the theatre / perception topics of a running sim (.venv, after `source env.sh`).

    python g1-isaac-sim/tools/theatre_topics_check.py --duration 15 --out /tmp/check
Measures the rates of /sim/objects and /g1/head_camera/{image_raw,depth,camera_info}, saves the last RGB frame
(PNG), depth (NPY + false-colour PNG) and /sim/objects (JSON) to --out, prints a summary JSON.
"""
import argparse
import json
import os
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--out", default="/tmp/theatre_check")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    n = rclpy.create_node("theatre_topics_check")
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    st = {k: {"n": 0, "t0": None, "t1": None} for k in ("objects", "rgb", "depth", "info")}
    last = {}

    def cb(key):
        def f(msg):
            s = st[key]
            now = time.monotonic()
            s["n"] += 1
            s["t0"] = s["t0"] or now
            s["t1"] = now
            last[key] = msg
        return f

    n.create_subscription(String, "/sim/objects", cb("objects"), 10)
    n.create_subscription(Image, "/g1/head_camera/image_raw", cb("rgb"), qos)
    n.create_subscription(Image, "/g1/head_camera/depth", cb("depth"), qos)
    n.create_subscription(CameraInfo, "/g1/head_camera/camera_info", cb("info"), qos)
    t_end = time.monotonic() + a.duration
    while time.monotonic() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
    res = {k: dict(count=v["n"], hz=round((v["n"] - 1) / (v["t1"] - v["t0"]), 2) if v["n"] > 1 else 0.0) for k, v in st.items()}
    if "rgb" in last:
        m = last["rgb"]
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        from PIL import Image as PI
        PI.fromarray(img).save(os.path.join(a.out, "rgb.png"))
        res["rgb_shape"] = [m.height, m.width]
    if "depth" in last:
        m = last["depth"]
        d = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
        np.save(os.path.join(a.out, "depth.npy"), d)
        fin = d[np.isfinite(d)]
        res["depth_m"] = dict(min=round(float(fin.min()), 3), median=round(float(np.median(fin)), 3), valid=round(fin.size / d.size, 3)) if fin.size else None
        from PIL import Image as PI
        v = np.nan_to_num(np.clip((d - 0.2) / 2.5, 0, 1), nan=1.0)
        PI.fromarray((255 * (1 - v)).astype(np.uint8)).save(os.path.join(a.out, "depth.png"))
    if "info" in last:
        res["K"] = list(last["info"].k)
        res["frame"] = last["info"].header.frame_id
    if "objects" in last:
        o = json.loads(last["objects"].data)
        json.dump(o, open(os.path.join(a.out, "objects.json"), "w"), indent=1)
        res["objects"] = sorted(o)
    print(json.dumps(res))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
