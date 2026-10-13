"""ROS 2 check of the fixture interface of a running simulator (Isaac run_sim.py or g1_mujoco_sim), whole hospital:

    source env.sh && python3 g1-isaac-sim/tools/fixtures_ros_check.py

Start the simulator first with the robot in the wing corridor, west of the CrossCorridor fire door, facing east,
for example (Isaac): ./g1-isaac-sim/run.sh --headless --spawn 19.43,-12.45,0 --idle hold
(MuJoCo): ros2 run g1_mujoco_sim sim_node --ros-args -p spawn:=19.43,-12.45,0
Use the same HG1_FIXTURES for the simulator and this check (default tier2): the expected doors, held doors, lifts
and buttons come from hospital-sim-env/fixtures/hospital_fixtures.py at that tier.
Steps: initial door state (every door of the tier; held open or shut), /scan range ahead (door open),
/sim/fire_alarm true -> every door closes, /scan ahead again (door closed), /sim/lift/call and /sim/lift/call_main
refused during the alarm, alarm false, /sim/lift/call -> a wing car opens, dwells, closes, /sim/lift/call_main -> a
main-bank car opens, dwells, closes, /sim/fixtures/press "MainCall" -> a main-bank car opens, /sim/fixtures/press
"<lift>_Open" for every lift, an unknown button name is ignored, /sim/reset -> held doors open again.
Times are sim time from the /sim/fixtures JSON ("t").
"""

import importlib.util
import json
import math
import os
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_s = importlib.util.spec_from_file_location("hospital_fixtures",
                                            os.path.join(REPO, "hospital-sim-env", "fixtures", "hospital_fixtures.py"))
F = importlib.util.module_from_spec(_s)
_s.loader.exec_module(F)
SPEC = F.build_spec(F.load_layout(os.path.join(REPO, "hospital-sim-env", "isaac", "scene", "layout.py")))
DOORS = [d["name"] for d in SPEC["doors"]]
HELD = [d["name"] for d in SPEC["doors"] if d["hold"] is not None]
SHUT = [n for n in DOORS if n not in HELD]
LIFTS = [l["name"] for l in SPEC["lifts"]]
BANK = {l["name"]: l["bank"] for l in SPEC["lifts"]}

rclpy.init()
node = rclpy.create_node("fixtures_ros_check")
last = {"fx": None, "scan": None}
node.create_subscription(String, "/sim/fixtures", lambda m: last.__setitem__("fx", json.loads(m.data)), 10)
node.create_subscription(LaserScan, "/scan", lambda m: last.__setitem__("scan", m),
                         QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
pub_alarm = node.create_publisher(Bool, "/sim/fire_alarm", 10)
pub_press = node.create_publisher(String, "/sim/fixtures/press", 10)
cli_call = node.create_client(Trigger, "/sim/lift/call")
cli_call_main = node.create_client(Trigger, "/sim/lift/call_main")
cli_reset = node.create_client(Trigger, "/sim/reset")
results = []


def spin(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def wait(pred, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if last["fx"] is not None and pred(last["fx"]):
            return last["fx"]
    return None


def check(name, ok, info=""):
    results.append(bool(ok))
    print(f"{'PASS' if ok else 'FAIL'} {name} {info}", flush=True)


def ahead():
    """Shortest /scan range within +-3 deg of straight ahead."""
    spin(0.5)
    s = last["scan"]
    if s is None:
        return float("inf")
    n = len(s.ranges)
    i0 = int(round((0.0 - s.angle_min) / s.angle_increment))
    k = int(math.radians(3) / s.angle_increment)
    vals = [s.ranges[(i0 + d) % n] for d in range(-k, k + 1)]
    return min(v for v in vals if math.isfinite(v)) if any(math.isfinite(v) for v in vals) else float("inf")


def call(cli, timeout=10.0):
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    fut = cli.call_async(Trigger.Request())
    end = time.monotonic() + timeout
    while not fut.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
    return fut.result()


def all_closed(f):
    return all(l["state"] == "closed" for l in f["lifts"].values())


def lift_cycle(cli, bank, label):
    """Service call -> a car of `bank` opens, dwells ~7 s, closes."""
    wait(all_closed, 30.0)
    t0 = last["fx"]["t"]
    res = call(cli)
    check(f"{label} accepted", res is not None and res.success, res.message if res else "no response")
    f = wait(lambda f: any(l["state"] == "open" for l in f["lifts"].values()), 15.0)
    lift = next((n for n, l in f["lifts"].items() if l["state"] == "open"), None) if f else None
    check(f"{label}: a {bank}-bank car opens", lift is not None and BANK.get(lift) == bank,
          f"{lift} open after {f['t'] - t0:.1f} s sim, gap {f['lifts'][lift]['door_gap_m']} m" if lift else "")
    if lift is None:
        return
    t1 = last["fx"]["t"]
    f = wait(lambda f: f["lifts"][lift]["state"] == "closing", 15.0)
    check(f"{label}: it dwells ~7 s then closes", f is not None and 6.0 < f["t"] - t1 < 8.5,
          f"closing after {f['t'] - t1:.1f} s sim" if f else "")
    f = wait(lambda f: f["lifts"][lift]["state"] == "closed", 10.0)
    check(f"{label}: doors closed", f is not None)


fx = wait(lambda f: True, 120.0)
if fx is None:
    print("FAIL no /sim/fixtures")
    sys.exit(1)
spin(1.0)
fx = last["fx"]
print(f"tier {SPEC['tier']}: {len(DOORS)} doors, {len(LIFTS)} lifts")
print("doors at start:", {d: v["leaves_deg"] for d, v in fx["doors"].items()})
check(f"/sim/fixtures lists the {len(DOORS)} fire doors of tier {SPEC['tier']}", sorted(fx["doors"]) == sorted(DOORS),
      sorted(set(fx["doors"]) ^ set(DOORS)))
check(f"/sim/fixtures lists the {len(LIFTS)} lifts", sorted(fx["lifts"]) == sorted(LIFTS), sorted(fx["lifts"]))
check("held doors open at start, kept-shut doors shut",
      all(fx["doors"][d]["held_open"] for d in HELD if d in fx["doors"])
      and all(fx["doors"][d]["closed"] for d in SHUT if d in fx["doors"]))
r_open = ahead()

t0 = last["fx"]["t"]
pub_alarm.publish(Bool(data=True))
f = wait(lambda f: f["fire_alarm"] and all(d["closed"] for d in f["doors"].values()), 30.0)
check("/sim/fire_alarm true closes every fire door", f is not None, f"in {f['t'] - t0:.1f} s sim" if f else "")
r_closed = ahead()
check("/scan sees the closed CrossCorridor door ahead", r_closed < r_open - 3.0,
      f"ahead: open {r_open:.2f} m, closed {r_closed:.2f} m")
for cli, name in ((cli_call, "/sim/lift/call"), (cli_call_main, "/sim/lift/call_main")):
    res = call(cli)
    check(f"{name} refused while the alarm is on (fire recall)", res is not None and not res.success,
          res.message if res else "no response")
pub_alarm.publish(Bool(data=False))
wait(lambda f: not f["fire_alarm"] and all_closed(f), 30.0)

lift_cycle(cli_call, "Wing", "/sim/lift/call (LobbyCall)")
lift_cycle(cli_call_main, "Main", "/sim/lift/call_main (MainCall)")

wait(all_closed, 30.0)
pub_press.publish(String(data="MainCall"))
f = wait(lambda f: any(l["state"] in ("opening", "open") and BANK.get(n) == "Main" for n, l in f["lifts"].items()), 5.0)
check("/sim/fixtures/press MainCall opens a main-bank car", f is not None)
for lname in LIFTS:
    pub_press.publish(String(data=f"{lname}_Open"))
    f = wait(lambda f, n=lname: f["lifts"][n]["state"] in ("opening", "open"), 5.0)
    check(f"/sim/fixtures/press {lname}_Open opens {lname}", f is not None)
pub_press.publish(String(data="NoSuchButton"))
spin(1.0)
t_a = last["fx"]["t"]
spin(1.0)
check("an unknown button name is ignored (state still published)", last["fx"]["t"] > t_a)

pub_alarm.publish(Bool(data=True))
wait(lambda f: all(d["closed"] for d in f["doors"].values()), 30.0)
res = call(cli_reset)
f = wait(lambda f: not f["fire_alarm"] and all(f["doors"][d]["held_open"] for d in HELD), 10.0)
check("/sim/reset puts the held doors back open and clears the alarm", f is not None)
print("last events:", last["fx"]["events"])
print("ALL PASS" if all(results) else "FAILURES", f"({sum(results)}/{len(results)})")
sys.exit(0 if all(results) else 1)
