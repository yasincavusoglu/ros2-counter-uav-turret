#!/usr/bin/env python3
"""scenario_sim — maneuvering aerial target + synthetic camera (the test world).

The real system ran on real cameras/hardware that cannot be shared, and anyone cloning this repo
has no drone and no camera. This node generates a maneuvering aerial target and renders the view
from the turret camera given the current gimbal pointing, so the whole pipeline runs end-to-end
with zero hardware.

World frame (ROS REP-103): X forward, Y left, Z up; turret/camera at the origin.

Publishes:  /camera/image  (sensor_msgs/Image, bgr8) — synthetic view along the current boresight
            /target/truth  (geometry_msgs/PointStamped) — ground-truth target position (scoring only)
Subscribes: /gimbal/state  (geometry_msgs/Vector3Stamped) — x = pan (rad), y = tilt (rad)

The core (TargetTrajectory, SyntheticCamera) is pure NumPy/OpenCV and has no ROS dependency, so it
can be unit-tested standalone.
"""
from __future__ import annotations

import math
import numpy as np
import cv2


# ─────────────────────────────────────────────────────────────────────────────
# Pure-Python core (no ROS) — testable standalone
# ─────────────────────────────────────────────────────────────────────────────
class TargetTrajectory:
    """A small quadrotor that approaches the turret while weaving (a maneuvering target)."""

    def __init__(self, start_range=120.0, closing_speed=8.0, altitude=15.0,
                 weave_amp=14.0, weave_freq=0.15, climb_amp=4.0):
        self.start_range = start_range      # initial forward distance [m]
        self.closing_speed = closing_speed  # approach speed toward turret [m/s]
        self.altitude = altitude            # nominal height [m]
        self.weave_amp = weave_amp          # lateral weave amplitude [m]
        self.weave_freq = weave_freq        # weave frequency [Hz]
        self.climb_amp = climb_amp          # vertical bob amplitude [m]

    def position(self, t: float) -> np.ndarray:
        """World position [x, y, z] at time t. Loops after the target overflies the turret."""
        # loop period: fly from start_range in to a small standoff, then respawn far again
        span = max(self.start_range - 10.0, 1.0)
        period = span / self.closing_speed
        tau = t % period
        x = self.start_range - self.closing_speed * tau            # forward, decreasing
        w = 2.0 * math.pi * self.weave_freq
        y = self.weave_amp * math.sin(w * t)                       # lateral weave
        z = self.altitude + self.climb_amp * math.sin(0.5 * w * t)  # gentle bob
        return np.array([x, y, z], dtype=np.float64)


class SyntheticCamera:
    """Pinhole camera at the origin; renders a drone sprite against sky along a pan/tilt boresight."""

    def __init__(self, width=640, height=480, hfov_deg=30.0, drone_size=1.2):
        self.w, self.h = int(width), int(height)
        self.cx, self.cy = self.w / 2.0, self.h / 2.0
        self.f = (self.w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)  # focal length [px]
        self.drone_size = drone_size  # physical span [m], for perspective sizing
        self._rng = np.random.default_rng(0)

    @staticmethod
    def _boresight_frame(pan: float, tilt: float):
        """Return (forward, right, down) unit vectors for a given pan (yaw) / tilt (pitch)."""
        f = np.array([math.cos(tilt) * math.cos(pan),
                      math.cos(tilt) * math.sin(pan),
                      math.sin(tilt)])
        world_up = np.array([0.0, 0.0, 1.0])
        right = np.cross(f, world_up)
        n = np.linalg.norm(right)
        right = np.array([0.0, -1.0, 0.0]) if n < 1e-6 else right / n
        down = np.cross(f, right)
        down /= (np.linalg.norm(down) + 1e-12)
        return f, right, down

    def project(self, pan: float, tilt: float, target: np.ndarray):
        """Project a world target to pixel coords. Returns (visible, u, v, depth)."""
        fwd, right, down = self._boresight_frame(pan, tilt)
        depth = float(np.dot(target, fwd))
        if depth <= 0.5:                       # behind camera / too close
            return False, -1.0, -1.0, depth
        u = self.cx + self.f * float(np.dot(target, right)) / depth
        v = self.cy + self.f * float(np.dot(target, down)) / depth
        visible = (0 <= u < self.w) and (0 <= v < self.h)
        return visible, u, v, depth

    def render(self, pan: float, tilt: float, target: np.ndarray):
        """Render a frame. Returns (image_bgr, visible, (u, v), bbox_xywh_or_None)."""
        img = self._sky()
        visible, u, v, depth = self.project(pan, tilt, target)
        bbox = None
        if visible:
            size = self.f * self.drone_size / depth                   # true projected size [px] (float)
            r = max(2, int(round(size / 2.0)))                        # int radius for the drawn sprite
            self._draw_drone(img, int(round(u)), int(round(v)), r)
            bbox = (u - size / 2.0, v - size / 2.0, size, size)       # accurate ground-truth bbox
        self._add_noise(img)
        return img, visible, (u, v), bbox

    def _sky(self) -> np.ndarray:
        """Vertical sky gradient (lighter near the top)."""
        top = np.array([235, 206, 135], np.float32)     # BGR light blue
        bot = np.array([180, 150, 90], np.float32)
        ramp = np.linspace(0.0, 1.0, self.h, dtype=np.float32)[:, None]
        rows = (top[None, :] * (1 - ramp) + bot[None, :] * ramp)      # (h,3)
        return np.repeat(rows[:, None, :], self.w, axis=1).astype(np.uint8)

    def _draw_drone(self, img, u, v, r):
        """Draw a CONNECTED dark quadrotor (one blob) whose bbox is ~2r x 2r (matches true size)."""
        body = (40, 40, 45)
        rr = max(1, r // 3)
        arm = max(1, (2 * r) // 3)          # rotor offset so total extent stays within +/- r
        for dx, dy in ((-arm, -arm), (arm, -arm), (-arm, arm), (arm, arm)):
            cv2.line(img, (u, v), (u + dx, v + dy), body, max(1, r // 3))  # arms connect it
            cv2.circle(img, (u + dx, v + dy), rr, body, -1)               # 4 rotors
        cv2.circle(img, (u, v), max(2, r // 2), body, -1)                 # hub
        return (u - r, v - r, 2 * r, 2 * r)

    def _add_noise(self, img):
        noise = self._rng.normal(0, 3.0, img.shape).astype(np.float32)
        img[:] = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)


# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 node wrapper
# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import PointStamped, Vector3Stamped
    from cv_bridge import CvBridge

    class ScenarioSim(Node):
        def __init__(self):
            super().__init__('scenario_sim')
            self.declare_parameter('fps', 30.0)
            self.declare_parameter('hfov_deg', 30.0)
            self.declare_parameter('width', 640)
            self.declare_parameter('height', 480)
            fps = self.get_parameter('fps').value
            self.cam = SyntheticCamera(
                width=self.get_parameter('width').value,
                height=self.get_parameter('height').value,
                hfov_deg=self.get_parameter('hfov_deg').value)
            self.traj = TargetTrajectory()
            self.bridge = CvBridge()
            self.pan = 0.0
            self.tilt = 0.0
            self.t = 0.0
            self.dt = 1.0 / float(fps)

            self.pub_img = self.create_publisher(Image, '/camera/image', 10)
            self.pub_truth = self.create_publisher(PointStamped, '/target/truth', 10)
            self.create_subscription(Vector3Stamped, '/gimbal/state', self._on_gimbal, 10)
            self.create_timer(self.dt, self._tick)
            self.get_logger().info(f'scenario_sim running @ {fps:.0f} fps.')

        def _on_gimbal(self, msg: 'Vector3Stamped'):
            self.pan = msg.vector.x
            self.tilt = msg.vector.y

        def _tick(self):
            self.t += self.dt
            p = self.traj.position(self.t)
            img, _visible, _uv, _bbox = self.cam.render(self.pan, self.tilt, p)

            stamp = self.get_clock().now().to_msg()
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = 'camera'
            self.pub_img.publish(img_msg)

            truth = PointStamped()
            truth.header.stamp = stamp
            truth.header.frame_id = 'world'
            truth.point.x, truth.point.y, truth.point.z = float(p[0]), float(p[1]), float(p[2])
            self.pub_truth.publish(truth)

    rclpy.init(args=args)
    node = ScenarioSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
