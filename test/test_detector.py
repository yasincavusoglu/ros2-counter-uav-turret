#!/usr/bin/env python3
"""Unit tests for the detector cores (pure NumPy/OpenCV, no ROS).

Run:  python3 test/test_detector.py   (or: pytest)
"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from counter_uav_turret.scenario_sim import TargetTrajectory, SyntheticCamera  # noqa: E402
from counter_uav_turret.detector import BlobDetector, make_detector  # noqa: E402


def test_blob_detects_visible_target():
    """The blob detector must find the target near its true pixel across the acquire window."""
    cam, traj, det = SyntheticCamera(), TargetTrajectory(), BlobDetector()
    hits, total = 0, 0
    for i in range(450):
        p = traj.position(i / 30.0)
        vis, u, v, _ = cam.project(0.0, 0.0, p)
        if not vis:
            continue
        total += 1
        dets = det.detect(cam.render(0.0, 0.0, p)[0])
        if dets:
            d = min(dets, key=lambda d: (d[0] - u) ** 2 + (d[1] - v) ** 2)
            if math.hypot(d[0] - u, d[1] - v) < 15.0:
                hits += 1
    assert total > 60
    assert hits >= 0.95 * total, f"only {hits}/{total} correct detections"


def test_no_false_positive_on_empty_sky():
    cam, det = SyntheticCamera(), BlobDetector()
    behind = np.array([-50.0, 0.0, 10.0])          # target behind the boresight → empty frame
    assert det.detect(cam.render(0.0, 0.0, behind)[0]) == []


def test_factory_defaults_to_blob():
    assert isinstance(make_detector('blob'), BlobDetector)


def test_yolo_backend_requires_model():
    with pytest.raises(ValueError):
        make_detector('yolo', model_path='')


if __name__ == '__main__':
    fns = [f for name, f in sorted(globals().items()) if name.startswith('test_')]
    passed = 0
    for f in fns:
        try:
            f()
        except Exception as e:                     # emulate pytest.raises path for __main__
            if f.__name__ == 'test_yolo_backend_requires_model' and isinstance(e, ValueError):
                pass
            else:
                raise
        print(f"  ok  {f.__name__}")
        passed += 1
    print(f"\n=== {passed}/{len(fns)} passed ===")
