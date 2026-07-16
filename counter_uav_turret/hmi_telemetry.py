#!/usr/bin/env python3
"""hmi_telemetry — run logging and summary metrics (and, under ROS, live plots + recording).

The core RunLogger accumulates per-frame records and computes the summary metrics reported in the
README (acquisition/lock time, aim-error RMSE, target-in-frame fraction, estimator error). It is
pure Python and used both by the ROS node and by scripts/run_demo.py.

Subscribes (ROS): /tracks, /target/state, /aim/setpoint, /gimbal/state, /engagement/status
Publishes:        (writes a JSONL run log; optional matplotlib dashboard)
"""
from __future__ import annotations

import json
import math


class RunLogger:
    def __init__(self):
        self.records = []

    def add(self, **rec):
        self.records.append(rec)

    def summary(self):
        n = len(self.records)
        if n == 0:
            return {}
        vis = [r for r in self.records if r.get('visible')]
        aim = [r['aim_error'] for r in self.records if r.get('aim_error') is not None]
        pos = [r['pos_error'] for r in self.records if r.get('pos_error') is not None]
        lead = [abs(r['lead_angle']) for r in self.records if r.get('lead_angle') is not None]
        lock_frame = next((i for i, r in enumerate(self.records) if r.get('locked')), None)
        rms = lambda xs: math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float('nan')
        return {
            'frames': n,
            'lock_frame': lock_frame,
            'target_in_frame_pct': round(100.0 * len(vis) / n, 1),
            'aim_error_rmse_mrad': round(1000.0 * rms(aim), 2),
            'pos_error_rmse_m': round(rms(pos), 2),
            'mean_lead_mrad': round(1000.0 * (sum(lead) / len(lead)) if lead else 0.0, 1),
        }

    def save_jsonl(self, path):
        with open(path, 'w') as f:
            for r in self.records:
                f.write(json.dumps(r) + '\n')


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from vision_msgs.msg import Detection2DArray
    from geometry_msgs.msg import Vector3Stamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    class HmiTelemetry(Node):
        def __init__(self):
            super().__init__('hmi_telemetry')
            self.declare_parameter('record', False)
            self.log = RunLogger()
            self.mode = 'SEARCH'
            self.create_subscription(String, '/engagement/status', self._on_mode, 10)
            self.create_subscription(Detection2DArray, '/tracks', self._on_tracks, 10)
            self.create_subscription(Odometry, '/target/state', self._on_state, 10)
            self.create_subscription(Vector3Stamped, '/gimbal/state', self._on_gimbal, 10)
            self.get_logger().info('hmi_telemetry running (mode + tracks logged).')

        def _on_mode(self, m):
            self.mode = m.data

        def _on_tracks(self, m):
            self.log.add(locked=bool(m.detections), mode=self.mode)

        def _on_state(self, m):
            pass  # extend: log estimated state for offline plots

        def _on_gimbal(self, m):
            pass

    rclpy.init(args=args)
    node = HmiTelemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
