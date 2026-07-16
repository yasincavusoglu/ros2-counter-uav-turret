#!/usr/bin/env python3
"""tracker — ByteTrack multi-object tracking + ID continuity.

Subscribes: /detections (vision_msgs/Detection2DArray)
Publishes:  /tracks     (…) — stable track IDs; the engagement FSM picks the locked track

TODO(M1): ByteTrack association over detections; publish tracks with persistent IDs.
"""
import rclpy
from rclpy.node import Node


class Tracker(Node):
    def __init__(self):
        super().__init__('tracker')
        self.get_logger().info('tracker started (stub — see TODO).')
        # TODO: ByteTrack state; subscriber /detections; publisher /tracks.


def main(args=None):
    rclpy.init(args=args)
    node = Tracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
