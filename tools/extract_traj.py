#!/usr/bin/env python3
"""Extract the robot trajectory of a recorded run for tools/replay_camera.py.

    source env.sh && python3 g1-isaac-sim/tools/extract_traj.py <run_dir> out.npz [--hz 3] [--bags bag bag2]
Reads /sim/ground_truth (pelvis pose, world frame) and /joint_states from the run's rosbag2 directories and samples
them at --hz (sim time) between the survey's walk_start and finish. Where no bag covers a stretch of the walk (the
recorders were restarted), the pose comes from survey_runner's trail_map.csv (map frame = world here, 2 Hz, x/y
only; yaw from the direction of travel, pelvis at standing height, joints held) and the samples are flagged
(filled = True) so the caller can say so.
"""
import argparse
import json
import math
import os

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


def read_bag(path):
    r = SequentialReader()
    r.open(StorageOptions(uri=path, storage_id="sqlite3"), ConverterOptions("cdr", "cdr"))
    r.set_filter(StorageFilter(topics=["/sim/ground_truth", "/joint_states"]))
    gt_t, gt, js_t, js, names = [], [], [], [], None
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == "/sim/ground_truth":
            m = deserialize_message(data, Odometry)
            p, o = m.pose.pose.position, m.pose.pose.orientation
            gt_t.append(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
            gt.append((p.x, p.y, p.z, o.x, o.y, o.z, o.w))
        else:
            m = deserialize_message(data, JointState)
            if names is None:
                names = list(m.name)
            if list(m.name) != names:
                continue
            js_t.append(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
            js.append(m.position)
    return np.array(gt_t), np.array(gt), np.array(js_t), np.array(js), names


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("out")
    ap.add_argument("--hz", type=float, default=3.0)
    ap.add_argument("--bags", nargs="*", default=None)
    a = ap.parse_args()
    ev = [json.loads(ln) for ln in open(os.path.join(a.run_dir, "survey_events.jsonl"))]
    s0 = next(e["sim"] for e in ev if e["event"] == "walk_start")
    fin = [e["sim"] for e in ev if e["event"] == "finish"]
    s1 = fin[-1] if fin else ev[-1]["sim"]
    bags = a.bags or sorted(d for d in os.listdir(a.run_dir) if d.startswith("bag"))
    parts = [read_bag(os.path.join(a.run_dir, b)) for b in bags]
    names = next(p[4] for p in parts if p[4])
    gt_t = np.concatenate([p[0] for p in parts if len(p[0])])
    gt = np.concatenate([p[1] for p in parts if len(p[1])])
    js_t = np.concatenate([p[2] for p in parts if len(p[2])])
    js = np.concatenate([p[3] for p in parts if len(p[3])])
    o = np.argsort(gt_t)
    gt_t, gt = gt_t[o], gt[o]
    o = np.argsort(js_t)
    js_t, js = js_t[o], js[o]
    trail = np.loadtxt(os.path.join(a.run_dir, "trail_map.csv"), delimiter=",", ndmin=2)
    T = np.arange(s0, s1, 1.0 / a.hz)
    root = np.zeros((len(T), 7))
    q = np.zeros((len(T), len(names)))
    filled = np.zeros(len(T), bool)
    for i, t in enumerate(T):
        k = np.searchsorted(gt_t, t)
        near = [j for j in (k - 1, k) if 0 <= j < len(gt_t)]
        j = min(near, key=lambda j: abs(gt_t[j] - t)) if near else None
        if j is not None and abs(gt_t[j] - t) < 0.2:
            root[i] = gt[j]
            kj = min(max(np.searchsorted(js_t, t), 0), len(js_t) - 1)
            q[i] = js[kj]
        else:  # gap in the bags: trail x, y (2 Hz) + heading
            filled[i] = True
            kt = min(max(np.searchsorted(trail[:, 0], t), 1), len(trail) - 1)
            x, y = np.interp(t, trail[:, 0], trail[:, 1]), np.interp(t, trail[:, 0], trail[:, 2])
            a0 = max(0, kt - 3)
            dx, dy = trail[min(kt + 2, len(trail) - 1), 1] - trail[a0, 1], trail[min(kt + 2, len(trail) - 1), 2] - trail[a0, 2]
            yaw = math.atan2(dy, dx)
            root[i] = (x, y, 0.77, 0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
            kj = min(max(np.searchsorted(js_t, t), 0), len(js_t) - 1)
            q[i] = js[kj]
    np.savez_compressed(a.out, t=T, root=root, names=np.array(names), q=q, filled=filled)
    print(f"{len(T)} samples {s0:.1f}..{s1:.1f} s, {filled.sum()} filled from the trail ({filled.mean() * 100:.1f} %)")


if __name__ == "__main__":
    main()
