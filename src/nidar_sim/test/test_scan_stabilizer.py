#!/usr/bin/env python3
"""Ground-truth regression tests for scan_stabilizer.

Builds a synthetic world (a square room of 4 m vertical walls over a floor, the
same geometry as maze.sdf), puts the lidar at 2 m with a known roll/pitch/yaw,
ray-casts the RAW scan the way Gazebo's gpu_lidar would, feeds it through the
real node, and checks the corrected output against the analytically-known
horizontal distance to the walls.

Two properties are being guarded, and they are the whole reason the node
exists:

  ACCURACY   a return that IS reported must be at the right horizontal bearing
             and the right horizontal range, at any lean. Getting this wrong is
             what smears a wall across a band of cells so it never accumulates
             enough hits to go black.
  SAFETY     a return that CANNOT be vouched for as horizontal - a floor strike
             above all - must come out as NaN, never as a range and never as
             inf. NaN is ignored outright by Karto and by Nav2's obstacle
             layer, so it marks nothing AND traces nothing. A leaked floor
             strike would do both: paint a phantom obstacle on open floor, and
             ray-trace every real wall between the drone and it out of the map.

Run with:  colcon test --packages-select nidar_sim
     or:   python3 -m pytest src/nidar_sim/test/test_scan_stabilizer.py -v
"""
import math

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from sensor_msgs.msg import LaserScan                       # noqa: E402
from nidar_sim.scan_stabilizer import ScanStabilizer        # noqa: E402

ALT = 2.0          # lidar altitude
HALF = 5.0         # room half-width: walls at x,y = +-5
WALL_BOTTOM, WALL_TOP = -0.75, 3.25     # as in maze.sdf
RANGE_MIN, RANGE_MAX = 0.08, 10.0
N = 1081           # matches lidar_2d/model.sdf


def _rot(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def _raycast(origin, d):
    """Nearest hit against the four walls and the floor, or inf."""
    best = math.inf
    for axis, lim in ((0, HALF), (0, -HALF), (1, HALF), (1, -HALF)):
        if abs(d[axis]) < 1e-12:
            continue
        t = (lim - origin[axis]) / d[axis]
        if t <= 0:
            continue
        hz = origin[2] + d[2] * t
        other = origin[1 - axis] + d[1 - axis] * t
        if WALL_BOTTOM <= hz <= WALL_TOP and abs(other) <= HALF + 1e-9:
            best = min(best, t)
    if d[2] < -1e-12:                                        # floor at z = 0
        t = -origin[2] / d[2]
        if (abs(origin[0] + d[0] * t) <= HALF
                and abs(origin[1] + d[1] * t) <= HALF):
            best = min(best, t)
    return best if best <= RANGE_MAX else math.inf


def _true_horizontal(bearing):
    """Analytic horizontal distance from the room centre along `bearing`."""
    dx, dy = math.cos(bearing), math.sin(bearing)
    best = math.inf
    for axis, lim in ((0, HALF), (0, -HALF), (1, HALF), (1, -HALF)):
        d = dx if axis == 0 else dy
        if abs(d) < 1e-12:
            continue
        t = lim / d
        if t > 0 and abs((dy if axis == 0 else dx) * t) <= HALF + 1e-9:
            best = min(best, t)
    return best


def _run(roll_deg, pitch_deg, yaw_deg):
    """Push one synthetic scan through the real node; return (out, raw, R)."""
    rclpy.init()
    try:
        node = ScanStabilizer()
        captured = {}
        node.pub.publish = lambda m: captured.setdefault('msg', m)

        R = _rot(*map(math.radians, (roll_deg, pitch_deg, yaw_deg)))
        # Bypass TF: hand the node the attitude directly. Everything after
        # this point is the production code path.
        node._pose = lambda frame, stamp: (R, np.zeros(3), R, ALT)

        angles = -math.pi + np.arange(N) * (2 * math.pi / (N - 1))
        world = R @ np.vstack((np.cos(angles), np.sin(angles), np.zeros(N)))

        msg = LaserScan()
        msg.header.frame_id = 'base_scan'
        msg.angle_min, msg.angle_max = -math.pi, math.pi
        msg.angle_increment = 2 * math.pi / (N - 1)
        msg.range_min, msg.range_max = RANGE_MIN, RANGE_MAX
        msg.ranges = [float(_raycast((0.0, 0.0, ALT), world[:, i]))
                      for i in range(N)]

        node._scan_cb(msg)
        out = np.asarray(captured['msg'].ranges, dtype=np.float64)
        raw = np.asarray(msg.ranges, dtype=np.float64)
        node.destroy_node()
        return out, raw, angles + math.radians(yaw_deg)
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize('roll,pitch,yaw', [
    (0, 0, 0),          # level: must be exact
    (0, 5, 0),          # gentle acceleration
    (0, 10, 0),
    (0, 15, 0),
    (12, 0, 0),         # pure roll (a turn)
    (8, 12, 0),         # combined lean
    (8, 12, 40),        # combined lean with the airframe yawed
    (0, 20, 0),         # hard braking
])
def test_reported_ranges_are_true_horizontal_distances(roll, pitch, yaw):
    """ACCURACY. Whatever survives must be geometrically right."""
    out, _, bearings = _run(roll, pitch, yaw)
    finite = np.isfinite(out)
    errs = np.array([abs(out[i] - _true_horizontal(bearings[i]))
                     for i in np.nonzero(finite)[0]])

    assert finite.sum() > 100, 'almost nothing survived; the node is too strict'
    assert np.percentile(errs, 95) < 0.06, f'p95 error {np.percentile(errs, 95):.3f} m'
    assert errs.max() < 0.20, f'max error {errs.max():.3f} m'


def test_level_scan_is_untouched():
    """A level scan must come through bit-for-bit: no correction to apply."""
    out, raw, _ = _run(0, 0, 0)
    both = np.isfinite(out) & np.isfinite(raw)
    assert np.allclose(out[both], raw[both], atol=1e-6)


def test_floor_strikes_never_reach_the_map():
    """SAFETY. This is the property the whole node is for.

    At 2 m altitude a beam depressed past ~12 deg reaches the floor inside the
    lidar's range and comes back as a confident finite return. Forwarded, it
    would mark a phantom obstacle on open floor AND ray-trace every real wall
    between the drone and it out of the map. Every one must be NaN.
    """
    out, raw, _ = _run(0, 20, 0)
    # In this room the walls are at most 7.07 m away horizontally, and at 20 deg
    # a genuine wall return stays well inside the node's off-plane budget; the
    # long returns are the floor.
    floor_like = np.isfinite(raw) & (raw > 6.0)
    assert floor_like.sum() > 50, 'test world produced no floor strikes to reject'
    assert np.isnan(out[floor_like]).all(), (
        f'{np.count_nonzero(~np.isnan(out[floor_like]))} floor strikes leaked '
        f'through as ranges or as inf')


def test_untrustworthy_rays_are_nan_not_inf():
    """Withheld must mean NaN, never inf.

    The distinction is not cosmetic. Karto ignores NaN, but treats a reading
    between max_laser_range and range_max as a FREE ray and traces it - so
    emitting inf for "I could not tell" would clear exactly the cells the node
    is trying to protect.
    """
    out, raw, _ = _run(0, 18, 0)
    # Rays that had a real return but were withheld must not have become inf.
    had_return = np.isfinite(raw)
    withheld = had_return & ~np.isfinite(out)
    assert withheld.sum() > 0
    assert np.isnan(out[withheld]).all(), 'a withheld return became inf (clears space)'
