#!/usr/bin/env python3
"""turret_plant — 2nd-order pan/tilt gimbal dynamics (the simulated actuator).

Subscribes: /gimbal/cmd
Publishes:  /gimbal/state (…) — current pan/tilt angle + rate

A second-order model (inertia + damping + rate/angle limits) so the control loop faces realistic
lag, and stabilization can be demonstrated. Feeds /gimbal/state back to scenario_sim (camera pointing)
and gimbal_control.

TODO(M2): integrate 2nd-order dynamics at fixed rate; apply limits; publish state.
"""
import rclpy
from rclpy.node import Node


class TurretPlant(Node):
    def __init__(self):
        super().__init__('turret_plant')
        self.get_logger().info('turret_plant started (stub — see TODO).')
        # TODO: dynamics params (inertia, damping, limits); sub /gimbal/cmd; integrate; pub /gimbal/state.


def main(args=None):
    rclpy.init(args=args)
    node = TurretPlant()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
