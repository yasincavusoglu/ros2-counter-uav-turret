#!/usr/bin/env python3
"""run_demo.py — full closed-loop counter-UAS turret demo, headless, no ROS required.

Wires every core together — scenario_sim -> detector -> tracker -> estimator -> lead_solver ->
gimbal_control -> turret_plant (feedback) + engagement_fsm — and reports the metrics used in the
README, plus a montage image (docs/demo.png).

Run:  python3 scripts/run_demo.py [--frames 240] [--no-image]
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from counter_uav_turret.scenario_sim import TargetTrajectory, SyntheticCamera
from counter_uav_turret.detector import BlobDetector
from counter_uav_turret.tracker import ByteTrackLite
from counter_uav_turret.estimator import TargetEstimator
from counter_uav_turret.lead_solver import LeadSolver
from counter_uav_turret.gimbal_control import PIDGimbalController
from counter_uav_turret.turret_plant import GimbalPlant
from counter_uav_turret.engagement_fsm import EngagementFSM
from counter_uav_turret.geometry import aim_angles, boresight_frame
from counter_uav_turret.hmi_telemetry import RunLogger


def _ang_dist(pan_a, tilt_a, pan_b, tilt_b):
    """Angle between two boresight directions."""
    da = boresight_frame(pan_a, tilt_a)[0]
    db = boresight_frame(pan_b, tilt_b)[0]
    return math.acos(float(np.clip(np.dot(da, db), -1.0, 1.0)))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def run(frames=240, dt=1 / 30.0, make_image=True):
    cam, traj = SyntheticCamera(), TargetTrajectory()
    detector, tracker = BlobDetector(), ByteTrackLite()
    # Monocular sensor: range is noisy, so R is set to that uncertainty (~8 m std) to keep the
    # filter from latching a spurious velocity; drone_size is calibrated to the detector's bbox.
    est = TargetEstimator(cam.f, cam.cx, cam.cy, drone_size=1.05, chi2=30.0, r_pos=64.0)
    lead, ctrl, plant, fsm = LeadSolver(), PIDGimbalController(), GimbalPlant(), EngagementFSM()
    log = RunLogger()

    pan_sp = tilt_sp = 0.0
    shots = []
    shot_at = {int(frames * f) for f in (0.15, 0.45, 0.70, 0.95)}

    for i in range(frames):
        t = i / 30.0
        p = traj.position(t)
        img, visible, (u, v), bbox = cam.render(plant.pan, plant.tilt, p)

        dets = detector.detect(img)
        tracks = tracker.update(dets)
        locked = max(tracks, key=lambda tr: tr.w * tr.h) if tracks else None

        pos_err = lead_ang = None
        if locked is not None:
            state, _nis = est.step(plant.pan, plant.tilt, locked.x, locked.y, locked.h)
            pos, vel = state[:3], state[3:]
            pan_sp, tilt_sp = aim_angles(pos)               # TRACK the target → keep it centered
            _aim, lead_pan, lead_tilt, _tof = lead.solve(pos, vel)   # separate fire solution
            lead_ang = _ang_dist(pan_sp, tilt_sp, lead_pan, lead_tilt)  # lead offset magnitude
            pos_err = float(np.linalg.norm(pos - p))

        pan_rate, tilt_rate = ctrl.compute(pan_sp, tilt_sp, plant.pan, plant.tilt, dt)
        plant.step(pan_rate, tilt_rate, dt)

        aim_err = _ang_dist(plant.pan, plant.tilt, pan_sp, tilt_sp)   # gimbal vs target (tracking)
        rng = float(np.linalg.norm(p))
        mode, fire = fsm.update(locked is not None, aim_err, 15.0 <= rng <= 150.0,
                                locked is not None, False)
        log.add(visible=bool(visible), locked=locked is not None,
                aim_error=aim_err, pos_error=pos_err, lead_angle=lead_ang, mode=mode, fire=fire)

        if make_image and i in shot_at:
            shots.append(_annotate(img.copy(), bbox, mode, fire, (u, v)))

    summ = log.summary()
    if make_image and shots:
        out = Path(__file__).resolve().parents[1] / 'docs' / 'demo.png'
        cv2.imwrite(str(out), cv2.hconcat([cv2.resize(s, (320, 240)) for s in shots]))
    return summ


def _annotate(img, bbox, mode, fire, truth_uv):
    h, w = img.shape[:2]
    cv2.drawMarker(img, (w // 2, h // 2), (0, 255, 255), cv2.MARKER_CROSS, 24, 1)  # boresight
    if bbox is not None:
        x, y, bw, bh = [int(round(c)) for c in bbox]
        m = 6
        cv2.rectangle(img, (x - m, y - m), (x + bw + m, y + bh + m), (0, 220, 0), 1)
    color = (0, 0, 255) if fire else (0, 220, 0)
    cv2.putText(img, f'{mode}{"  FIRE" if fire else ""}', (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return img


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=240)
    ap.add_argument('--no-image', action='store_true')
    a = ap.parse_args()
    s = run(frames=a.frames, make_image=not a.no_image)
    print('\n=== counter-UAS turret demo — summary ===')
    for k, v in s.items():
        print(f'  {k:24s} {v}')
