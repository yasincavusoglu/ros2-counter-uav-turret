#!/usr/bin/env python3
"""gimbal_control — pan/tilt PID + stabilization, driving the gimbal to the aim setpoint.

Subscribes: /aim/setpoint, /gimbal/state
Publishes:  /gimbal/cmd (…) — pan/tilt rate/torque command via a ros2_control-style interface

PID on the angular error with rate limits and a stabilization term. The original hardware system
commanded a Fatek PLC over Modbus TCP; here the interface is standardized to ros2_control.

TODO(M2): PID (per axis) + rate limit + stabilization; publish /gimbal/cmd.
"""
import rclpy
from rclpy.node import Node


class GimbalControl(Node):
    def __init__(self):
        super().__init__('gimbal_control')
        self.get_logger().info('gimbal_control started (stub — see TODO).')
        # TODO: PID gains params; subs (setpoint, state); pub /gimbal/cmd.


def main(args=None):
    rclpy.init(args=args)
    node = GimbalControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
