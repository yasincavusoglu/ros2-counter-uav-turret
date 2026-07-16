#!/usr/bin/env python3
"""End-to-end integration test: the full closed loop must acquire and track the target with lead."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.run_demo import run  # noqa: E402


def test_closed_loop_acquires_tracks_and_leads():
    s = run(frames=240, make_image=False)
    assert s['lock_frame'] is not None and s['lock_frame'] < 15    # acquires quickly
    assert s['target_in_frame_pct'] > 90.0                         # gimbal keeps the target in frame
    assert s['aim_error_rmse_mrad'] < 50.0                         # gimbal tracks the target
    assert s['pos_error_rmse_m'] < 20.0                            # monocular range accuracy
    assert s['mean_lead_mrad'] > 10.0                              # a real intercept lead is applied


if __name__ == '__main__':
    test_closed_loop_acquires_tracks_and_leads()
    print('  ok  test_closed_loop_acquires_tracks_and_leads')
    print('\n=== 1/1 passed ===')
