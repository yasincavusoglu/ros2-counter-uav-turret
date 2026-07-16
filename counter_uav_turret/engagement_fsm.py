#!/usr/bin/env python3
"""engagement_fsm — SEARCH -> TRACK_LOCK -> ENGAGE with safety interlocks.

Fire is permitted ONLY when every interlock holds simultaneously:
  * a target track is present and fresh (perception watchdog),
  * the aim error is within tolerance (the gimbal is actually on target),
  * the target is inside the allowed engagement zone,
  * safe mode is not asserted,
and only after the lock has been stable for a few frames. A safe-mode flag forbids fire entirely.

Subscribes: /tracks, /aim/setpoint, /gimbal/state, /target/state
Publishes:  /engagement/status (std_msgs/String), /engagement/fire_permit (std_msgs/Bool)

The FSM core is pure Python and unit-tested standalone.
"""
from __future__ import annotations

SEARCH, TRACK_LOCK, ENGAGE = 'SEARCH', 'TRACK_LOCK', 'ENGAGE'


class EngagementFSM:
    def __init__(self, aim_tol=0.01, lock_frames=5):
        self.aim_tol = aim_tol
        self.lock_frames = lock_frames
        self.mode = SEARCH
        self.lock_count = 0

    def update(self, has_track, aim_error, in_zone, watchdog_fresh, safe_mode):
        """Return (mode, fire_permit)."""
        if not has_track or not watchdog_fresh:
            self.mode = SEARCH
            self.lock_count = 0
            return self.mode, False

        engage_ready = (aim_error < self.aim_tol) and in_zone and (not safe_mode)
        if engage_ready:
            self.lock_count += 1
            self.mode = ENGAGE if self.lock_count >= self.lock_frames else TRACK_LOCK
        else:
            self.lock_count = 0
            self.mode = TRACK_LOCK

        fire_permit = (self.mode == ENGAGE) and engage_ready and watchdog_fresh
        return self.mode, fire_permit


def main(args=None):
    import math
    import rclpy
    from rclpy.node import Node
    from vision_msgs.msg import Detection2DArray
    from geometry_msgs.msg import Vector3Stamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, Bool

    def wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    class EngagementNode(Node):
        def __init__(self):
            super().__init__('engagement_fsm')
            self.declare_parameter('safe_mode', False)
            self.declare_parameter('min_range', 15.0)
            self.declare_parameter('max_range', 150.0)
            self.declare_parameter('watchdog_s', 0.5)
            self.fsm = EngagementFSM()
            self.pan_sp = self.tilt_sp = self.pan = self.tilt = 0.0
            self.in_zone = False
            self.last_track_t = None
            self.pub_status = self.create_publisher(String, '/engagement/status', 10)
            self.pub_fire = self.create_publisher(Bool, '/engagement/fire_permit', 10)
            self.create_subscription(Detection2DArray, '/tracks', self._on_tracks, 10)
            self.create_subscription(Vector3Stamped, '/aim/setpoint', self._on_sp, 10)
            self.create_subscription(Vector3Stamped, '/gimbal/state', self._on_state, 10)
            self.create_subscription(Odometry, '/target/state', self._on_target, 10)
            self.create_timer(1 / 30.0, self._tick)
            self.get_logger().info('engagement_fsm running.')

        def _now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def _on_tracks(self, m):
            if m.detections:
                self.last_track_t = self._now()

        def _on_sp(self, m):
            self.pan_sp, self.tilt_sp = m.vector.x, m.vector.y

        def _on_state(self, m):
            self.pan, self.tilt = m.vector.x, m.vector.y

        def _on_target(self, m):
            p = m.pose.pose.position
            rng = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
            lo = self.get_parameter('min_range').value
            hi = self.get_parameter('max_range').value
            self.in_zone = lo <= rng <= hi

        def _tick(self):
            fresh = (self.last_track_t is not None and
                     (self._now() - self.last_track_t) < self.get_parameter('watchdog_s').value)
            aim_err = math.hypot(wrap(self.pan_sp - self.pan), wrap(self.tilt_sp - self.tilt))
            mode, fire = self.fsm.update(fresh, aim_err, self.in_zone, fresh,
                                         self.get_parameter('safe_mode').value)
            self.pub_status.publish(String(data=mode))
            self.pub_fire.publish(Bool(data=fire))

    rclpy.init(args=args)
    node = EngagementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
