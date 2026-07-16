#!/usr/bin/env python3
"""estimator — 6-state Kalman filter on the locked track.

State [x, y, z, vx, vy, vz]. Features: adaptive process noise (Q grows under maneuver),
chi-square innovation gating (rejects outliers/jumps) and NIS telemetry (reports filter
consistency). The predict/update hot path uses the C++ core from the sibling repo
`perception-accel` (with a pure-Python fallback), proving real-time, low-latency estimation.

Subscribes: /tracks
Publishes:  /target/state (…) — position + velocity; also /estimator/nis for telemetry

TODO(M2): wire perception-accel kf_predict/kf_update; adaptive-Q; chi2 gating; publish state + NIS.
"""
import rclpy
from rclpy.node import Node


class Estimator(Node):
    def __init__(self):
        super().__init__('estimator')
        self.get_logger().info('estimator started (stub — see TODO).')
        # TODO: import perception_accel (fallback if missing); KF state; subs/pubs.


def main(args=None):
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
