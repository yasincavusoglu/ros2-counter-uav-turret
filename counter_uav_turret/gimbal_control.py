#!/usr/bin/env python3
"""gimbal_control — pan/tilt PID that drives the gimbal onto the aim setpoint.

A PID per axis on the (wrapped) angular error produces a rate command, clamped to a maximum slew
rate. The original hardware system commanded a Fatek PLC over Modbus TCP; here the interface is a
standard ros2_control-style rate command.

Subscribes: /aim/setpoint (geometry_msgs/Vector3Stamped, x=pan, y=tilt),
            /gimbal/state  (geometry_msgs/Vector3Stamped, x=pan, y=tilt)
Publishes:  /gimbal/cmd    (geometry_msgs/Vector3Stamped, x=pan_rate, y=tilt_rate)

The controller core is pure Python and unit-tested standalone.
"""
from __future__ import annotations

import math


def wrap(a):
    """Wrap an angle to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class PID:
    def __init__(self, kp, ki, kd, out_limit):
        self.kp, self.ki, self.kd, self.lim = kp, ki, kd, out_limit
        self.i = 0.0
        self.prev = None

    def step(self, err, dt):
        self.i += err * dt
        d = 0.0 if self.prev is None or dt <= 0 else (err - self.prev) / dt
        self.prev = err
        out = self.kp * err + self.ki * self.i + self.kd * d
        return max(-self.lim, min(self.lim, out))


class PIDGimbalController:
    def __init__(self, kp=6.0, ki=0.5, kd=0.3, max_rate=3.0):
        self.pan = PID(kp, ki, kd, max_rate)
        self.tilt = PID(kp, ki, kd, max_rate)

    def compute(self, pan_sp, tilt_sp, pan, tilt, dt):
        """Return (pan_rate_cmd, tilt_rate_cmd)."""
        return (self.pan.step(wrap(pan_sp - pan), dt),
                self.tilt.step(wrap(tilt_sp - tilt), dt))


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Vector3Stamped

    class GimbalControl(Node):
        def __init__(self):
            super().__init__('gimbal_control')
            self.declare_parameter('max_rate', 3.0)
            self.ctrl = PIDGimbalController(max_rate=self.get_parameter('max_rate').value)
            self.pan_sp = self.tilt_sp = 0.0
            self.last_t = None
            self.pub = self.create_publisher(Vector3Stamped, '/gimbal/cmd', 10)
            self.create_subscription(Vector3Stamped, '/aim/setpoint', self._on_sp, 10)
            self.create_subscription(Vector3Stamped, '/gimbal/state', self._on_state, 10)
            self.get_logger().info('gimbal_control running.')

        def _on_sp(self, m):
            self.pan_sp, self.tilt_sp = m.vector.x, m.vector.y

        def _on_state(self, m):
            now = self.get_clock().now().nanoseconds * 1e-9
            dt = 1 / 30.0 if self.last_t is None else max(1e-3, now - self.last_t)
            self.last_t = now
            pr, tr = self.ctrl.compute(self.pan_sp, self.tilt_sp, m.vector.x, m.vector.y, dt)
            out = Vector3Stamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.vector.x, out.vector.y = pr, tr
            self.pub.publish(out)

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
