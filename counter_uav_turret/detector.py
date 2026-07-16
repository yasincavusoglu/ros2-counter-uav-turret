#!/usr/bin/env python3
"""detector — aerial-target detection with a pluggable backend.

Two backends:
  * `blob`  (default) — a lightweight dark-object-on-bright-sky detector. Requires no weights and
                        works out of the box on the synthetic camera, so the pipeline runs end-to-end.
  * `yolo`            — YOLOv8/v11 (ultralytics) for real footage / a drone-trained model. Weights
                        are supplied via the `model` parameter and are not shipped in the repo.

CLAHE contrast enhancement is applied before detection (small targets against bright sky).

Subscribes: /camera/image (sensor_msgs/Image, bgr8)
Publishes:  /detections   (vision_msgs/Detection2DArray)

The detection cores are pure NumPy/OpenCV (no ROS) and are unit-tested standalone.
"""
from __future__ import annotations

import numpy as np
import cv2


# ─────────────────────────────────────────────────────────────────────────────
# Detection cores (no ROS) — return a list of (cx, cy, w, h, score)
# ─────────────────────────────────────────────────────────────────────────────
class BlobDetector:
    """Detects dark, compact objects (the target) against a bright sky."""

    def __init__(self, dark_thresh=110, min_area=4, max_area=8000, use_clahe=True):
        self.dark_thresh = dark_thresh
        self.min_area = min_area
        self.max_area = max_area
        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)) if use_clahe else None

    def detect(self, bgr: np.ndarray):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if self._clahe is not None:
            gray = self._clahe.apply(gray)
        # target is darker than sky → invert threshold
        mask = (gray < self.dark_thresh).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in contours:
            area = cv2.contourArea(c)
            if not (self.min_area <= area <= self.max_area):
                continue
            x, y, w, h = cv2.boundingRect(c)
            fill = area / max(1.0, w * h)          # compactness → pseudo-confidence
            score = float(np.clip(0.5 + 0.5 * fill, 0.0, 0.99))
            out.append((x + w / 2.0, y + h / 2.0, float(w), float(h), score))
        out.sort(key=lambda d: d[2] * d[3], reverse=True)   # largest first
        return out


class YoloDetector:
    """Wraps an ultralytics YOLOv8/v11 model. Import/model load is lazy."""

    def __init__(self, model_path: str, conf=0.25, classes=None):
        from ultralytics import YOLO           # lazy: only when this backend is used
        self.model = YOLO(model_path)
        self.conf = conf
        self.classes = classes

    def detect(self, bgr: np.ndarray):
        res = self.model.predict(bgr, conf=self.conf, classes=self.classes, verbose=False)[0]
        out = []
        for b in res.boxes:
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            out.append(((x1 + x2) / 2.0, (y1 + y2) / 2.0, x2 - x1, y2 - y1, float(b.conf[0])))
        out.sort(key=lambda d: d[2] * d[3], reverse=True)
        return out


def make_detector(backend='blob', model_path='', conf=0.25):
    if backend == 'yolo':
        if not model_path:
            raise ValueError("yolo backend needs a 'model' parameter (path to weights).")
        return YoloDetector(model_path, conf=conf)
    return BlobDetector()


# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 node wrapper
# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
    from cv_bridge import CvBridge

    class Detector(Node):
        def __init__(self):
            super().__init__('detector')
            self.declare_parameter('backend', 'blob')
            self.declare_parameter('model', '')
            self.declare_parameter('conf', 0.25)
            backend = self.get_parameter('backend').value
            self.det = make_detector(backend,
                                     self.get_parameter('model').value,
                                     self.get_parameter('conf').value)
            self.bridge = CvBridge()
            self.pub = self.create_publisher(Detection2DArray, '/detections', 10)
            self.create_subscription(Image, '/camera/image', self._on_image, 10)
            self.get_logger().info(f"detector running (backend={backend}).")

        def _on_image(self, msg: 'Image'):
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            dets = self.det.detect(bgr)

            arr = Detection2DArray()
            arr.header = msg.header
            for cx, cy, w, h, score in dets:
                d = Detection2D()
                d.header = msg.header
                d.bbox.center.position.x = cx
                d.bbox.center.position.y = cy
                d.bbox.size_x = w
                d.bbox.size_y = h
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = 'uav'
                hyp.hypothesis.score = score
                d.results.append(hyp)
                arr.detections.append(d)
            self.pub.publish(arr)

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
