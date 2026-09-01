"""Shaping a hand-drawn path into something an arm can follow.

A path picked up from mouse motion is noisy, unevenly sampled and timed by
whatever rate the windowing system delivered events at. These helpers turn that
into a clean, time-parameterised path. Everything here is ROS-free so it can be
tested directly.

Points are (x, y, z) tuples and times are seconds from the start of the drag.
"""

import math

from kuka_control_tutorial.trajectory import distance


def drop_dense_points(points, times, min_spacing):
    """Thin out samples closer together than ``min_spacing`` metres.

    Mouse motion arrives in bursts; consecutive events can land on the same
    pixel. Those add nothing to the shape and make the speed estimate noisy.
    The first and last samples are always kept.
    """
    if len(points) < 2:
        return list(points), list(times)

    kept_points = [points[0]]
    kept_times = [times[0]]
    for point, t in zip(points[1:], times[1:]):
        if distance(point, kept_points[-1]) >= min_spacing:
            kept_points.append(point)
            kept_times.append(t)

    # Always finish on the real last sample, so the path ends where the user
    # released the mouse rather than at the last point that cleared the filter.
    if kept_points[-1] != points[-1]:
        if distance(points[-1], kept_points[-1]) < min_spacing and len(kept_points) > 1:
            kept_points[-1] = points[-1]
            kept_times[-1] = times[-1]
        else:
            kept_points.append(points[-1])
            kept_times.append(times[-1])

    return kept_points, kept_times


def smooth(points, window):
    """Moving-average smoothing that leaves the endpoints untouched.

    ``window`` is the number of samples on each side. Smoothing the endpoints
    would pull the path away from where the user pressed and released, so they
    are pinned.
    """
    if window <= 0 or len(points) < 3:
        return list(points)

    smoothed = []
    for i, point in enumerate(points):
        if i == 0 or i == len(points) - 1:
            smoothed.append(point)
            continue
        # Shrink the window near the ends so it stays symmetric.
        half = min(window, i, len(points) - 1 - i)
        chunk = points[i - half : i + half + 1]
        smoothed.append(tuple(sum(c[axis] for c in chunk) / len(chunk) for axis in range(3)))
    return smoothed


def strip_leading_pause(points, times, min_spacing):
    """Drop samples where the pointer had not started moving yet.

    Users often press, hesitate, then draw. Those leading samples would make
    the arm sit still while the clock runs.
    """
    start = 0
    for i in range(1, len(points)):
        if distance(points[i], points[0]) >= min_spacing:
            start = i - 1
            break
    else:
        return list(points), list(times)

    trimmed_times = [t - times[start] for t in times[start:]]
    return list(points[start:]), trimmed_times


def peak_speed(points, times):
    """Highest speed [m/s] over any segment of the path."""
    fastest = 0.0
    for i in range(1, len(points)):
        dt = times[i] - times[i - 1]
        if dt > 0.0:
            fastest = max(fastest, distance(points[i], points[i - 1]) / dt)
    return fastest


def path_length(points):
    """Total arc length [m]."""
    return sum(distance(points[i], points[i - 1]) for i in range(1, len(points)))


def scale_times(times, speed_ratio):
    """Stretch the timeline so the path is traversed at ``speed_ratio`` of the
    speed it was drawn at.

    A ratio of 0.2 means "one fifth the drawn speed", so every timestamp is
    multiplied by five.
    """
    if speed_ratio <= 0.0:
        raise ValueError("speed_ratio must be positive")
    return [t / speed_ratio for t in times]


def enforce_min_dwell(times, min_dt):
    """Push timestamps apart so no two are closer than ``min_dt``.

    Two samples at the same timestamp would be a division by zero when the
    server interpolates, and a near-zero gap is an unreachable speed spike.
    """
    fixed = [times[0]]
    for t in times[1:]:
        fixed.append(max(t, fixed[-1] + min_dt))
    return fixed


def limit_speed(times, points, max_speed):
    """Uniformly stretch the timeline until no segment exceeds ``max_speed``.

    Returns ``(times, slowdown)`` where ``slowdown`` is the extra factor
    applied, 1.0 when the path was already slow enough. Stretching uniformly
    preserves the shape of the drawn speed profile, which keeps the replay
    recognisable as the drawing.
    """
    fastest = peak_speed(points, times)
    if fastest <= max_speed or fastest == 0.0:
        return list(times), 1.0
    slowdown = fastest / max_speed
    return [t * slowdown for t in times], slowdown


def prepare_drawn_path(
    points,
    times,
    speed_ratio,
    max_speed,
    min_spacing=0.002,
    smooth_window=2,
    min_dt=0.01,
):
    """Turn raw pointer samples into a path the FollowPath action will accept.

    Returns ``(points, times, info)``. ``info`` carries the numbers worth
    showing the user: drawn duration, path length, the peak speed asked for and
    any extra slowdown needed to respect ``max_speed``.
    """
    if len(points) < 2:
        raise ValueError("need at least two points to make a path")

    points, times = strip_leading_pause(points, times, min_spacing)
    points, times = drop_dense_points(points, times, min_spacing)
    if len(points) < 2:
        raise ValueError("the drawn path is too short")

    points = smooth(points, smooth_window)

    drawn_duration = times[-1] - times[0]
    times = [t - times[0] for t in times]
    times = scale_times(times, speed_ratio)
    times = enforce_min_dwell(times, min_dt)
    times, slowdown = limit_speed(times, points, max_speed)

    info = {
        "waypoints": len(points),
        "length": path_length(points),
        "drawn_duration": drawn_duration,
        "duration": times[-1],
        "peak_speed": peak_speed(points, times),
        "extra_slowdown": slowdown,
    }
    return points, times, info


def sample_path(points, times, t):
    """Position on the path at time ``t``, linearly interpolated.

    Clamps to the endpoints outside the path's time range.
    """
    if t <= times[0]:
        return points[0]
    if t >= times[-1]:
        return points[-1]

    # Paths here are short enough that a linear scan is not worth replacing.
    for i in range(1, len(times)):
        if t <= times[i]:
            span = times[i] - times[i - 1]
            frac = 0.0 if span <= 0.0 else (t - times[i - 1]) / span
            a, b = points[i - 1], points[i]
            return tuple(a[axis] + frac * (b[axis] - a[axis]) for axis in range(3))
    return points[-1]


def reachable_radius_xy(max_reach, z):
    """Radius [m] of the reachable disc in a horizontal plane at height ``z``.

    The envelope check is a sphere of radius ``max_reach`` about the base, so
    slicing it at constant z gives a circle. Returns 0.0 when the plane misses
    the sphere entirely.
    """
    if abs(z) >= max_reach:
        return 0.0
    return math.sqrt(max_reach * max_reach - z * z)
