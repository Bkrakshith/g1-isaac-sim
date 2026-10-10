#!/usr/bin/env python3
"""Hardware e-stop stub for the Isaac backend (docs/interfaces.md 3.4).

g1_mujoco_sim publishes /g1/estop/heartbeat itself (g1_mujoco_sim/stubs.py); run_sim.py does not, so motion modes
(LOCO/MANIP/NAV/AUTO) stay locked on Isaac without this. It reuses the same RobotStubs class: /g1/estop/heartbeat
at 10 Hz (true = released), /sim/estop (SetBool, true = press) and /sim/estop_link (SetBool, false = dead receiver).

    source env.sh && python3 g1-isaac-sim/tools/estop_stub.py
"""
import rclpy

from g1_mujoco_sim.stubs import RobotStubs


def main():
    rclpy.init()
    node = rclpy.create_node("isaac_robot_stubs")
    RobotStubs(node)
    node.get_logger().info("e-stop heartbeat stub up (/g1/estop/heartbeat 10 Hz, /sim/estop, /sim/estop_link)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
