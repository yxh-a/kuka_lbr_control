"""Unit tests for the trajectory maths, no ROS graph required."""

import math

import pytest

from kuka_control_tutorial.trajectory import (
    TrapezoidalProfile,
    distance,
    interpolate,
    quat_normalize,
    quat_slerp,
)


def test_trapezoid_reaches_cruise_speed():
    p = TrapezoidalProfile(distance=0.5, speed=0.05, acceleration=0.1)
    assert p.peak_speed == pytest.approx(0.05)
    assert p.t_cruise > 0.0
    # 2 * 0.5 s of ramp plus the cruise stretch.
    assert p.duration == pytest.approx(2 * 0.5 + (0.5 - 0.025) / 0.05)


def test_short_move_degenerates_to_triangle():
    p = TrapezoidalProfile(distance=0.001, speed=0.05, acceleration=0.1)
    assert p.t_cruise == pytest.approx(0.0)
    assert p.peak_speed < 0.05
    assert p.peak_speed == pytest.approx(math.sqrt(0.1 * 0.001))


def test_profile_endpoints_and_monotonicity():
    p = TrapezoidalProfile(distance=0.4, speed=0.06, acceleration=0.15)
    assert p.distance_at(-1.0) == 0.0
    assert p.distance_at(0.0) == 0.0
    assert p.distance_at(p.duration) == pytest.approx(0.4)
    assert p.distance_at(p.duration + 10.0) == pytest.approx(0.4)

    previous = -1.0
    for i in range(1001):
        d = p.distance_at(p.duration * i / 1000.0)
        assert d >= previous - 1e-12
        previous = d


def test_profile_starts_and_ends_at_rest():
    p = TrapezoidalProfile(distance=0.4, speed=0.06, acceleration=0.15)
    dt = 1e-4
    v_start = (p.distance_at(dt) - p.distance_at(0.0)) / dt
    v_end = (p.distance_at(p.duration) - p.distance_at(p.duration - dt)) / dt
    assert v_start < 0.01 * p.peak_speed
    assert v_end < 0.01 * p.peak_speed


def test_profile_is_symmetric():
    p = TrapezoidalProfile(distance=0.4, speed=0.06, acceleration=0.15)
    for frac in (0.1, 0.25, 0.4):
        t = p.duration * frac
        covered = p.distance_at(t)
        remaining = p.distance - p.distance_at(p.duration - t)
        assert covered == pytest.approx(remaining)


def test_profile_rejects_bad_arguments():
    with pytest.raises(ValueError):
        TrapezoidalProfile(distance=-0.1, speed=0.05, acceleration=0.1)
    with pytest.raises(ValueError):
        TrapezoidalProfile(distance=0.1, speed=0.0, acceleration=0.1)
    with pytest.raises(ValueError):
        TrapezoidalProfile(distance=0.1, speed=0.05, acceleration=0.0)


def test_zero_distance_reports_complete():
    p = TrapezoidalProfile(distance=0.0, speed=0.05, acceleration=0.1)
    assert p.progress_at(0.0) == 1.0


def test_slerp_endpoints_and_midpoint():
    q0 = (0.0, 0.0, 0.0, 1.0)
    q1 = (0.0, 0.0, 1.0, 0.0)  # 180 deg about z
    assert quat_slerp(q0, q1, 0.0) == pytest.approx(q0)
    assert quat_slerp(q0, q1, 1.0) == pytest.approx(q1)
    mid = quat_slerp(q0, q1, 0.5)  # 90 deg about z
    assert mid == pytest.approx((0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)))


def test_slerp_takes_the_short_way_round():
    q0 = (0.0, 0.0, 0.0, 1.0)
    q1 = (0.0, 0.0, -0.0, -1.0)  # same rotation, opposite sign
    mid = quat_slerp(q0, q1, 0.5)
    # Must stay at the identity rather than sweeping a full turn.
    assert abs(mid[3]) == pytest.approx(1.0)


def test_slerp_output_is_unit_length():
    q0 = quat_normalize((0.1, 0.2, 0.3, 0.4))
    q1 = quat_normalize((-0.5, 0.1, 0.8, 0.2))
    for i in range(11):
        q = quat_slerp(q0, q1, i / 10.0)
        assert sum(c * c for c in q) == pytest.approx(1.0)


def test_quat_normalize_handles_degenerate_input():
    assert quat_normalize((0.0, 0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0, 1.0)


def test_distance_and_interpolate():
    a, b = (0.0, 0.0, 0.0), (3.0, 4.0, 0.0)
    assert distance(a, b) == pytest.approx(5.0)
    assert interpolate(a, b, 0.0) == pytest.approx(a)
    assert interpolate(a, b, 1.0) == pytest.approx(b)
    assert interpolate(a, b, 0.5) == pytest.approx((1.5, 2.0, 0.0))
