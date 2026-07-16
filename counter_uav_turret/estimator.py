#!/usr/bin/env python3
"""estimator — 6-state Kalman filter on the locked track (position + velocity in the world frame).

The camera is monocular, so range is recovered from the target's apparent size: for a known target
span S, range = f_px * S / bbox_height (the monocular fallback used in the original system). The
pixel bearing is back-projected to a world ray; range * ray gives a 3D position measurement, which
drives a constant-velocity Kalman filter with:
  * adaptive process noise (Q grows when the innovation says the target is maneuvering),
  * chi-square innovation gating (rejects outliers/jumps),
  * NIS telemetry (the filter reports its own consistency).

If the sibling package `perception_accel` is importable, its C++ predict/update core is used;
otherwise a pure-NumPy fallback runs (same result). See github.com/yasincavusoglu/perception-accel.

Subscribes: /tracks (vision_msgs/Detection2DArray), /gimbal/state (geometry_msgs/Vector3Stamped)
Publishes:  /target/state (nav_msgs/Odometry) — pose.position + twist.linear ; /estimator/nis (Float64)
"""
from __future__ import annotations

import numpy as np

from counter_uav_turret.geometry import pixel_to_ray

try:
    import perception_accel as _accel          # optional C++ acceleration
    USING_CPP = True
except Exception:                              # pragma: no cover
    _accel = None
    USING_CPP = False


class TargetEstimator:
    """6D constant-velocity Kalman filter fed by monocular (bearing + size→range) measurements."""

    def __init__(self, f_px, cx, cy, drone_size=1.2, chi2=11.34, r_pos=4.0):
        self.f_px, self.cx, self.cy = f_px, cx, cy
        self.drone_size = drone_size
        self.chi2 = chi2
        self.R = np.eye(3) * r_pos
        self.H = np.zeros((3, 6))
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = 1.0
        self.x = None
        self.P = None
        self.last_nis = 0.0
        self.gate_rejects = 0

    # -- monocular measurement -------------------------------------------------
    def measure(self, pan, tilt, u, v, bbox_h):
        bbox_h = max(1.0, float(bbox_h))
        rng = self.f_px * self.drone_size / bbox_h
        ray = pixel_to_ray(pan, tilt, u, v, self.f_px, self.cx, self.cy)
        return rng * ray

    # -- KF core (uses perception_accel if present) ----------------------------
    def _predict(self, dt, maneuver):
        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        q_pos = 0.5 * dt * dt
        q_vel = dt * (1.0 + 4.0 * maneuver)         # adaptive: grow velocity noise under maneuver
        Q = np.diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel])
        if _accel is not None:
            self.x, self.P = _accel.kf_predict(self.x, self.P, F, Q)
        else:
            self.x = F @ self.x
            self.P = F @ self.P @ F.T + Q

    def _update(self, z):
        if _accel is not None:
            status, nis, _, xn, Pn = _accel.kf_update(self.x, self.P, self.H, self.R, z,
                                                       self.chi2, True)
            self.last_nis = nis
            if status == 0:
                self.x, self.P = np.asarray(xn), np.asarray(Pn)
            elif status == 1:
                self.gate_rejects += 1
            return
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        nis = float(y @ np.linalg.solve(S, y))
        self.last_nis = nis
        if nis > self.chi2:                          # chi-square gating
            self.gate_rejects += 1
            return
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def step(self, pan, tilt, u, v, bbox_h, dt=1 / 30.0):
        z = self.measure(pan, tilt, u, v, bbox_h)
        if self.x is None:                           # initialize on first measurement
            self.x = np.array([z[0], z[1], z[2], 0.0, 0.0, 0.0])
            self.P = np.diag([25.0, 25.0, 25.0, 100.0, 100.0, 100.0])
            return self.x.copy(), 0.0
        maneuver = float(np.clip(self.last_nis / self.chi2, 0.0, 1.0))
        self._predict(dt, maneuver)
        self._update(z)
        return self.x.copy(), self.last_nis


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from vision_msgs.msg import Detection2DArray
    from geometry_msgs.msg import Vector3Stamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float64

    class Estimator(Node):
        def __init__(self):
            super().__init__('estimator')
            self.declare_parameter('f_px', 1194.3)
            self.declare_parameter('cx', 320.0)
            self.declare_parameter('cy', 240.0)
            self.declare_parameter('drone_size', 1.2)
            self.est = TargetEstimator(self.get_parameter('f_px').value,
                                       self.get_parameter('cx').value,
                                       self.get_parameter('cy').value,
                                       self.get_parameter('drone_size').value)
            self.pan = self.tilt = 0.0
            self.pub = self.create_publisher(Odometry, '/target/state', 10)
            self.pub_nis = self.create_publisher(Float64, '/estimator/nis', 10)
            self.create_subscription(Vector3Stamped, '/gimbal/state', self._on_gimbal, 10)
            self.create_subscription(Detection2DArray, '/tracks', self._on_tracks, 10)
            self.get_logger().info(f'estimator running (C++ accel={USING_CPP}).')

        def _on_gimbal(self, m):
            self.pan, self.tilt = m.vector.x, m.vector.y

        def _on_tracks(self, msg):
            if not msg.detections:
                return
            t = max(msg.detections, key=lambda d: d.bbox.size_x * d.bbox.size_y)  # locked = largest
            x, nis = self.est.step(self.pan, self.tilt,
                                   t.bbox.center.position.x, t.bbox.center.position.y,
                                   t.bbox.size_y)
            od = Odometry()
            od.header = msg.header
            od.header.frame_id = 'world'
            od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = \
                float(x[0]), float(x[1]), float(x[2])
            od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = \
                float(x[3]), float(x[4]), float(x[5])
            self.pub.publish(od)
            self.pub_nis.publish(Float64(data=float(nis)))

    rclpy.init(args=args)
    node = Estimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
