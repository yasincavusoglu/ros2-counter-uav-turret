#!/usr/bin/env python3
"""lead_solver — intercept / lead aim-point for a MOVING target (the differentiator).

Static aim fails on a moving target. Given the estimated target state (position + velocity) and the
effector time-of-flight, iterate to a self-consistent intercept:

    t = 0
    repeat:  aim   = target_pos + target_vel * t
             t_new = |aim| / v_effector          # time of flight to that aim point
    until |t_new - t| < tol
    aim-point = target_pos + target_vel * t

The output is a boresight pan/tilt pointing AHEAD of the target, so the round and the target arrive
at the same place at the same time.

Subscribes: /target/state (nav_msgs/Odometry)
Publishes:  /aim/setpoint (geometry_msgs/Vector3Stamped) — x = pan (rad), y = tilt (rad)

The solver core is pure Python and unit-tested standalone.
"""
from __future__ import annotations

import numpy as np

from counter_uav_turret.geometry import aim_angles


class LeadSolver:
    def __init__(self, v_effector=250.0, max_iters=12, tol=1e-4):
        self.v_effector = v_effector
        self.max_iters = max_iters
        self.tol = tol

    def solve(self, pos, vel):
        """pos, vel: length-3 world vectors. Returns (aim_point, pan, tilt, t_impact)."""
        pos = np.asarray(pos, float)
        vel = np.asarray(vel, float)
        t = float(np.linalg.norm(pos) / self.v_effector)     # first guess: TOF to current pos
        for _ in range(self.max_iters):
            aim = pos + vel * t
            t_new = float(np.linalg.norm(aim) / self.v_effector)
            if abs(t_new - t) < self.tol:
                t = t_new
                break
            t = t_new
        aim = pos + vel * t
        pan, tilt = aim_angles(aim)
        return aim, pan, tilt, t


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Vector3Stamped

    class LeadSolverNode(Node):
        def __init__(self):
            super().__init__('lead_solver')
            self.declare_parameter('v_effector', 250.0)
            self.solver = LeadSolver(self.get_parameter('v_effector').value)
            self.pub = self.create_publisher(Vector3Stamped, '/aim/setpoint', 10)
            self.create_subscription(Odometry, '/target/state', self._on_state, 10)
            self.get_logger().info('lead_solver running.')

        def _on_state(self, msg):
            p = msg.pose.pose.position
            v = msg.twist.twist.linear
            _aim, pan, tilt, _t = self.solver.solve([p.x, p.y, p.z], [v.x, v.y, v.z])
            out = Vector3Stamped()
            out.header = msg.header
            out.vector.x, out.vector.y = float(pan), float(tilt)
            self.pub.publish(out)

    rclpy.init(args=args)
    node = LeadSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
