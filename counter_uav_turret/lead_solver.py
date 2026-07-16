#!/usr/bin/env python3
"""lead_solver — intercept / lead aim-point for a MOVING target (the differentiator).

Static aim fails on a moving target. Given the estimated target state and the effector
time-of-flight, iterate:  t_impact = range(aim) / v_effector ; aim = predict_target(t_impact) ;
repeat until convergence. Output is an aim-point AHEAD of the target.

Subscribes: /target/state
Publishes:  /aim/setpoint (…) — desired pan/tilt (or 3D aim point) for the gimbal

TODO(M2): iterative intercept solver; effector-speed param; convergence guard; publish setpoint.
"""
import rclpy
from rclpy.node import Node


class LeadSolver(Node):
    def __init__(self):
        super().__init__('lead_solver')
        self.get_logger().info('lead_solver started (stub — see TODO).')
        # TODO: params (v_effector, max_iters, tol); sub /target/state; pub /aim/setpoint.


def main(args=None):
    rclpy.init(args=args)
    node = LeadSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
