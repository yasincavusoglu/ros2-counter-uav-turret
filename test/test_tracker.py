#!/usr/bin/env python3
"""Unit tests for the ByteTrackLite core (pure Python, no ROS)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from counter_uav_turret.tracker import ByteTrackLite, _iou  # noqa: E402


def test_iou_basics():
    assert _iou((0, 0, 10, 10), (0, 0, 10, 10)) == 1.0
    assert _iou((0, 0, 10, 10), (100, 100, 10, 10)) == 0.0


def test_confirms_and_keeps_stable_id():
    tr = ByteTrackLite(min_hits=3)
    ids = set()
    conf = []
    for i in range(10):
        det = (100 + 2 * i, 100 + i, 20, 20, 0.9)   # smooth path
        conf = tr.update([det])
        ids.update(t.id for t in conf)
    assert len(conf) == 1                            # exactly one target
    assert len(ids) == 1                             # its ID never changed


def test_survives_short_miss():
    tr = ByteTrackLite(min_hits=3, max_age=15)
    for i in range(5):
        tr.update([(100 + 2 * i, 100, 20, 20, 0.9)])
    tid = tr.update([(110, 100, 20, 20, 0.9)])[0].id
    tr.update([])                                    # one frame with no detection
    conf = tr.update([(114, 100, 20, 20, 0.9)])      # target reappears near predicted spot
    assert len(conf) == 1 and conf[0].id == tid      # same track survived the miss


def test_low_confidence_alone_creates_no_track():
    tr = ByteTrackLite(new_conf=0.6)
    conf = []
    for _ in range(5):
        conf = tr.update([(100, 100, 20, 20, 0.2)])  # only weak detections
    assert conf == []


if __name__ == '__main__':
    fns = [f for n, f in sorted(globals().items()) if n.startswith('test_')]
    for f in fns:
        f(); print(f'  ok  {f.__name__}')
    print(f'\n=== {len(fns)}/{len(fns)} passed ===')
