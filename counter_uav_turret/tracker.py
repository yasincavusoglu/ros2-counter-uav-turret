#!/usr/bin/env python3
"""tracker — lightweight ByteTrack-style multi-object tracking + ID continuity.

Associates detections to tracks by IoU (high-confidence detections first, then low-confidence
to the remaining tracks — the two-stage idea of ByteTrack), predicts track motion with a constant
-velocity model so a track survives a short miss, ages out lost tracks, and only reports tracks
that have been confirmed over a few frames.

Subscribes: /detections (vision_msgs/Detection2DArray)
Publishes:  /tracks     (vision_msgs/Detection2DArray, with detection.id = track id)

The tracker core (ByteTrackLite) is pure Python and unit-tested standalone.
"""
from __future__ import annotations


def _iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ax1, ay1, ax2, ay2 = ax - aw / 2, ay - ah / 2, ax + aw / 2, ay + ah / 2
    bx1, by1, bx2, by2 = bx - bw / 2, by - bh / 2, bx + bw / 2, by + bh / 2
    ix1, iy1, ix2, iy2 = max(ax1, bx1), max(ay1, by1), min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    union = aw * ah + bw * bh - inter
    return inter / union if union > 0 else 0.0


class Track:
    _next_id = 1

    def __init__(self, det):
        self.id = Track._next_id
        Track._next_id += 1
        self.x, self.y, self.w, self.h, self.score = det
        self.vx = self.vy = 0.0
        self.hits = 1
        self.tsu = 0            # time since update
        self.confirmed = False

    @property
    def box(self):
        return (self.x, self.y, self.w, self.h)

    def predict(self):
        self.x += self.vx
        self.y += self.vy
        self.tsu += 1

    def update(self, det):
        nx, ny, nw, nh, nscore = det
        self.vx = 0.6 * self.vx + 0.4 * (nx - self.x)     # EMA velocity
        self.vy = 0.6 * self.vy + 0.4 * (ny - self.y)
        self.x, self.y, self.w, self.h, self.score = nx, ny, nw, nh, nscore
        self.hits += 1
        self.tsu = 0


class ByteTrackLite:
    def __init__(self, iou_thresh=0.2, high_conf=0.5, new_conf=0.6,
                 min_hits=3, max_age=15):
        self.iou_thresh = iou_thresh
        self.high_conf = high_conf
        self.new_conf = new_conf
        self.min_hits = min_hits
        self.max_age = max_age
        self.tracks: list[Track] = []

    def _associate(self, tracks, dets):
        """Greedy IoU association. Returns (matches, unmatched_track_idx, unmatched_det_idx)."""
        pairs = []
        for ti, t in enumerate(tracks):
            for di, d in enumerate(dets):
                iou = _iou(t.box, d[:4])
                if iou >= self.iou_thresh:
                    pairs.append((iou, ti, di))
        pairs.sort(reverse=True)
        used_t, used_d, matches = set(), set(), []
        for iou, ti, di in pairs:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti); used_d.add(di); matches.append((ti, di))
        um_t = [i for i in range(len(tracks)) if i not in used_t]
        um_d = [i for i in range(len(dets)) if i not in used_d]
        return matches, um_t, um_d

    def update(self, detections):
        """detections: list of (cx, cy, w, h, score). Returns confirmed tracks."""
        for t in self.tracks:
            t.predict()

        high = [d for d in detections if d[4] >= self.high_conf]
        low = [d for d in detections if d[4] < self.high_conf]

        # stage 1: high-confidence detections
        m1, um_t, um_d = self._associate(self.tracks, high)
        for ti, di in m1:
            self.tracks[ti].update(high[di])
        # stage 2: low-confidence detections to still-unmatched tracks
        rem_tracks = [self.tracks[i] for i in um_t]
        m2, um_t2_local, _ = self._associate(rem_tracks, low)
        for ti_local, di in m2:
            rem_tracks[ti_local].update(low[di])

        # new tracks from unmatched high-confidence detections
        for di in um_d:
            if high[di][4] >= self.new_conf:
                self.tracks.append(Track(high[di]))

        # confirm / age out
        alive = []
        for t in self.tracks:
            if t.hits >= self.min_hits:
                t.confirmed = True
            if t.tsu <= self.max_age:
                alive.append(t)
        self.tracks = alive
        return [t for t in self.tracks if t.confirmed]


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

    class Tracker(Node):
        def __init__(self):
            super().__init__('tracker')
            self.tr = ByteTrackLite()
            self.pub = self.create_publisher(Detection2DArray, '/tracks', 10)
            self.create_subscription(Detection2DArray, '/detections', self._on_det, 10)
            self.get_logger().info('tracker running.')

        def _on_det(self, msg: 'Detection2DArray'):
            dets = []
            for d in msg.detections:
                score = d.results[0].hypothesis.score if d.results else 1.0
                dets.append((d.bbox.center.position.x, d.bbox.center.position.y,
                             d.bbox.size_x, d.bbox.size_y, score))
            tracks = self.tr.update(dets)

            out = Detection2DArray()
            out.header = msg.header
            for t in tracks:
                d = Detection2D()
                d.header = msg.header
                d.id = str(t.id)
                d.bbox.center.position.x, d.bbox.center.position.y = t.x, t.y
                d.bbox.size_x, d.bbox.size_y = t.w, t.h
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = 'uav'
                hyp.hypothesis.score = t.score
                d.results.append(hyp)
                out.detections.append(d)
            self.pub.publish(out)

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
