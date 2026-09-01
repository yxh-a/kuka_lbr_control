"""Unit tests for the drawn-path shaping, no ROS graph required."""

import math

import pytest

from kuka_control_tutorial.path_planning import (
    drop_dense_points,
    enforce_min_dwell,
    limit_speed,
    path_length,
    peak_speed,
    prepare_drawn_path,
    reachable_radius_xy,
    sample_path,
    scale_times,
    smooth,
    strip_leading_pause,
)


def line(n, spacing=0.01, dt=0.05):
    """A straight stroke of ``n`` samples at constant speed."""
    points = [(i * spacing, 0.0, 0.4) for i in range(n)]
    times = [i * dt for i in range(n)]
    return points, times


def test_drop_dense_points_thins_but_keeps_ends():
    points = [(0.0, 0.0, 0.0), (0.0001, 0.0, 0.0), (0.05, 0.0, 0.0)]
    times = [0.0, 0.01, 0.5]
    kept, kept_times = drop_dense_points(points, times, min_spacing=0.002)
    assert kept[0] == points[0]
    assert kept[-1] == points[-1]
    assert kept_times[-1] == times[-1]
    assert len(kept) == 2


def test_drop_dense_points_passes_through_sparse_input():
    points, times = line(5, spacing=0.01)
    kept, kept_times = drop_dense_points(points, times, min_spacing=0.002)
    assert kept == points
    assert kept_times == times


def test_smooth_pins_the_endpoints():
    points = [(0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
    out = smooth(points, window=1)
    assert out[0] == points[0]
    assert out[-1] == points[-1]
    # The spike at index 1 is pulled towards its neighbours.
    assert out[1][1] < points[1][1]


def test_smooth_is_a_noop_for_tiny_paths():
    points = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    assert smooth(points, window=3) == points
    assert smooth(points, window=0) == points


def test_strip_leading_pause_removes_the_hesitation():
    points = [(0.0, 0.0, 0.0)] * 4 + [(0.01, 0.0, 0.0), (0.02, 0.0, 0.0)]
    times = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
    out_points, out_times = strip_leading_pause(points, times, min_spacing=0.002)
    assert out_times[0] == 0.0
    assert len(out_points) < len(points)
    # The stroke now starts just before it began moving.
    assert out_points[-1] == points[-1]


def test_strip_leading_pause_keeps_a_stroke_that_moves_immediately():
    points, times = line(4)
    out_points, out_times = strip_leading_pause(points, times, 0.002)
    assert out_points == points
    assert out_times == times


def test_scale_times_slows_by_the_ratio():
    assert scale_times([0.0, 1.0, 2.0], 0.2) == [0.0, 5.0, 10.0]
    assert scale_times([0.0, 1.0], 2.0) == [0.0, 0.5]
    with pytest.raises(ValueError):
        scale_times([0.0, 1.0], 0.0)


def test_peak_speed_and_path_length():
    points = [(0.0, 0.0, 0.0), (0.3, 0.4, 0.0), (0.3, 0.4, 1.0)]
    times = [0.0, 1.0, 4.0]
    assert path_length(points) == pytest.approx(1.5)
    assert peak_speed(points, times) == pytest.approx(0.5)


def test_enforce_min_dwell_separates_equal_timestamps():
    out = enforce_min_dwell([0.0, 0.0, 0.0], 0.01)
    assert out == [0.0, 0.01, 0.02]
    assert all(out[i] > out[i - 1] for i in range(1, len(out)))


def test_limit_speed_stretches_only_when_needed():
    points, times = line(5, spacing=0.1, dt=0.1)  # 1.0 m/s
    slowed, factor = limit_speed(times, points, max_speed=0.25)
    assert factor == pytest.approx(4.0)
    assert peak_speed(points, slowed) == pytest.approx(0.25)

    slow_points, slow_times = line(5, spacing=0.001, dt=1.0)
    unchanged, factor = limit_speed(slow_times, slow_points, max_speed=0.25)
    assert factor == 1.0
    assert unchanged == slow_times


def test_prepare_drawn_path_applies_the_ratio():
    points, times = line(20, spacing=0.005, dt=0.05)  # 0.1 m/s drawn
    path, path_times, info = prepare_drawn_path(
        points, times, speed_ratio=0.2, max_speed=0.25
    )
    assert path_times[0] == 0.0
    assert all(path_times[i] > path_times[i - 1] for i in range(1, len(path_times)))
    assert info["drawn_duration"] == pytest.approx(0.95)
    # A fifth of the speed means five times the duration.
    assert info["duration"] == pytest.approx(0.95 * 5)
    assert info["extra_slowdown"] == 1.0
    assert peak_speed(path, path_times) <= 0.25 + 1e-9


def test_prepare_drawn_path_slows_a_fast_stroke_further():
    points, times = line(20, spacing=0.05, dt=0.01)  # 5 m/s drawn
    path, path_times, info = prepare_drawn_path(
        points, times, speed_ratio=0.2, max_speed=0.25
    )
    assert info["extra_slowdown"] > 1.0
    assert peak_speed(path, path_times) <= 0.25 + 1e-9


def test_prepare_drawn_path_rejects_a_click():
    with pytest.raises(ValueError):
        prepare_drawn_path([(0.0, 0.0, 0.0)], [0.0], 0.2, 0.25)


def test_sample_path_interpolates_and_clamps():
    points = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 2.0, 0.0)]
    times = [0.0, 1.0, 3.0]
    assert sample_path(points, times, -1.0) == points[0]
    assert sample_path(points, times, 0.0) == points[0]
    assert sample_path(points, times, 0.5) == pytest.approx((0.5, 0.0, 0.0))
    assert sample_path(points, times, 1.0) == pytest.approx((1.0, 0.0, 0.0))
    assert sample_path(points, times, 2.0) == pytest.approx((1.0, 1.0, 0.0))
    assert sample_path(points, times, 3.0) == points[-1]
    assert sample_path(points, times, 99.0) == points[-1]


def test_sample_path_is_continuous_across_segments():
    points = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 2.0, 0.0)]
    times = [0.0, 1.0, 3.0]
    before = sample_path(points, times, 1.0 - 1e-6)
    after = sample_path(points, times, 1.0 + 1e-6)
    assert before == pytest.approx(after, abs=1e-4)


def test_reachable_radius_slices_the_sphere():
    assert reachable_radius_xy(0.85, 0.0) == pytest.approx(0.85)
    assert reachable_radius_xy(0.85, 0.405) == pytest.approx(
        math.sqrt(0.85**2 - 0.405**2)
    )
    assert reachable_radius_xy(0.85, 0.9) == 0.0


def test_the_far_corner_of_the_default_square_is_out_of_reach():
    # Guards the assumption the GUI's shading is built on: at the documented
    # home pose one corner of the 30 cm square really is outside the envelope.
    home = (-0.396, 0.472, 0.405)
    radius = reachable_radius_xy(0.85, home[2])
    corner = (home[0] - 0.15, home[1] + 0.15)
    assert math.hypot(*corner) > radius
    assert math.hypot(home[0], home[1]) < radius
