#!/usr/bin/env python3
"""hmi_telemetry — live plots, JSONL logging and demo recording.

Subscribes to the key topics (tracks, target state, aim setpoint, gimbal state, engagement status)
and produces: a live dashboard (track-lock, aim error, NIS, stabilization), a JSONL run log for
offline analysis/metrics, and an optional MP4 recording for the README demo.

TODO(M3): matplotlib live dashboard; JSONL logger; optional frame recorder (record:=true).
"""
import rclpy
from rclpy.node import Node


class HmiTelemetry(Node):
    def __init__(self):
        super().__init__('hmi_telemetry')
        self.declare_parameter('record', False)
        self.get_logger().info('hmi_telemetry started (stub — see TODO).')
        # TODO: subscribers; live plot; JSONL logger; optional recorder.


def main(args=None):
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
