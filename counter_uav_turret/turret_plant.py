#!/usr/bin/env python3
"""turret_plant — 2nd-order pan/tilt gimbal dynamics (the simulated actuator).

A rate command does not take effect instantly: the actual slew rate approaches the commanded rate
with a time constant (inertia), and the angle integrates the actual rate. Rate and (tilt) angle
limits are enforced. This gives the control loop realistic lag so stabilization can be shown.

Subscribes: /gimbal/cmd   (geometry_msgs/Vector3Stamped, x=pan_rate, y=tilt_rate)
Publishes:  /gimbal/state (geometry_msgs/Vector3Stamped, x=pan, y=tilt)

The dynamics core is pure Python and unit-tested standalone.
"""
from __future__ import annotations

import math


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class GimbalPlant:
    def __init__(self, tau=0.12, max_rate=3.0, tilt_min=-0.35, tilt_max=1.4):
        self.tau = tau              # actuator time constant [s]
        self.max_rate = max_rate
        self.tilt_min, self.tilt_max = tilt_min, tilt_max
        self.pan = self.tilt = 0.0
        self.pan_rate = self.tilt_rate = 0.0

    def step(self, cmd_pan_rate, cmd_tilt_rate, dt):
        # first-order lag toward commanded rate (the "2nd order" is rate + angle integration)
        self.pan_rate += (cmd_pan_rate - self.pan_rate) * min(1.0, dt / self.tau)
        self.tilt_rate += (cmd_tilt_rate - self.tilt_rate) * min(1.0, dt / self.tau)
        self.pan_rate = max(-self.max_rate, min(self.max_rate, self.pan_rate))
        self.tilt_rate = max(-self.max_rate, min(self.max_rate, self.tilt_rate))
        self.pan = _wrap(self.pan + self.pan_rate * dt)
        self.tilt += self.tilt_rate * dt
        if self.tilt <= self.tilt_min:
            self.tilt, self.tilt_rate = self.tilt_min, 0.0
        elif self.tilt >= self.tilt_max:
            self.tilt, self.tilt_rate = self.tilt_max, 0.0
        return self.pan, self.tilt


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Vector3Stamped

    class TurretPlant(Node):
        def __init__(self):
            super().__init__('turret_plant')
            self.declare_parameter('rate', 60.0)
            self.plant = GimbalPlant()
            self.cmd = (0.0, 0.0)
            self.dt = 1.0 / float(self.get_parameter('rate').value)
            self.pub = self.create_publisher(Vector3Stamped, '/gimbal/state', 10)
            self.create_subscription(Vector3Stamped, '/gimbal/cmd', self._on_cmd, 10)
            self.create_timer(self.dt, self._tick)
            self.get_logger().info('turret_plant running.')

        def _on_cmd(self, m):
            self.cmd = (m.vector.x, m.vector.y)

        def _tick(self):
            pan, tilt = self.plant.step(self.cmd[0], self.cmd[1], self.dt)
            out = Vector3Stamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'gimbal'
            out.vector.x, out.vector.y = pan, tilt
            self.pub.publish(out)

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
