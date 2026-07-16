#!/usr/bin/env python3
"""detector — YOLOv8/v11 aerial-target detection.

Subscribes: /camera/image (sensor_msgs/Image)
Publishes:  /detections   (vision_msgs/Detection2DArray) — 2D boxes + scores

The detector is pluggable: any pretrained/fine-tuned YOLO model. Weights are not shipped in the repo.
CLAHE contrast enhancement is applied before inference (small aerial targets against bright sky).

TODO(M1): load YOLO model (param: model path); cv_bridge decode; CLAHE; inference; publish detections.
"""
import rclpy
from rclpy.node import Node


class Detector(Node):
    def __init__(self):
        super().__init__('detector')
        self.get_logger().info('detector started (stub — see TODO).')
        # TODO: model param, subscriber /camera/image, publisher /detections.


def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
