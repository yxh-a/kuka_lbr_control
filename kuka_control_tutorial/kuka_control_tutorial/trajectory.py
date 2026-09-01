"""Straight-line Cartesian trajectory generation.

Kept free of ROS types so the profile can be exercised on its own.
"""

import math


class TrapezoidalProfile:
    """Trapezoidal velocity profile along a scalar path length.

    Accelerates at ``acceleration`` up to ``speed``, cruises, then decelerates
    symmetrically. When the path is too short to ever reach ``speed`` the
    profile degenerates into a triangular one and the peak speed is lowered
    accordingly.
    """

    def __init__(self, distance: float, speed: float, acceleration: float):
        if distance < 0.0:
            raise ValueError("distance must be non-negative")
        if speed <= 0.0:
            raise ValueError("speed must be positive")
        if acceleration <= 0.0:
            raise ValueError("acceleration must be positive")

        self.distance = distance
        self.acceleration = acceleration

        # Distance needed to reach `speed` and come back to rest again.
        ramp_distance = speed * speed / acceleration

        if ramp_distance > distance:
            # Triangular: never reaches the requested cruise speed.
            self.peak_speed = math.sqrt(acceleration * distance)
            self.t_accel = self.peak_speed / acceleration
            self.t_cruise = 0.0
        else:
            self.peak_speed = speed
            self.t_accel = speed / acceleration
            self.t_cruise = (distance - ramp_distance) / speed

        self.t_decel_start = self.t_accel + self.t_cruise
        self.duration = 2.0 * self.t_accel + self.t_cruise

    def distance_at(self, t: float) -> float:
        """Arc length covered at time ``t`` since the start of the motion."""
        if t <= 0.0:
            return 0.0
        if t >= self.duration:
            return self.distance

        if t < self.t_accel:
            return 0.5 * self.acceleration * t * t

        if t < self.t_decel_start:
            d_accel = 0.5 * self.acceleration * self.t_accel * self.t_accel
            return d_accel + self.peak_speed * (t - self.t_accel)

        # Decelerating: mirror the acceleration phase around the end.
        t_remaining = self.duration - t
        return self.distance - 0.5 * self.acceleration * t_remaining * t_remaining

    def progress_at(self, t: float) -> float:
        """Normalised path parameter in [0, 1]."""
        if self.distance <= 0.0:
            return 1.0
        return self.distance_at(t) / self.distance


def quat_normalize(q):
    """Normalise an (x, y, z, w) tuple. Falls back to identity if degenerate."""
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def quat_slerp(q0, q1, t: float):
    """Spherical linear interpolation between two (x, y, z, w) quaternions."""
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)

    dot = sum(a * b for a, b in zip(q0, q1))

    # q and -q describe the same rotation; pick the shorter arc.
    if dot < 0.0:
        q1 = tuple(-c for c in q1)
        dot = -dot

    if dot > 0.9995:
        # Nearly parallel: lerp and renormalise to avoid dividing by ~zero.
        return quat_normalize(tuple(a + t * (b - a) for a, b in zip(q0, q1)))

    theta_0 = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * t
    s0 = math.sin(theta_0 - theta) / sin_theta_0
    s1 = math.sin(theta) / sin_theta_0
    return quat_normalize(tuple(s0 * a + s1 * b for a, b in zip(q0, q1)))


def distance(a, b) -> float:
    """Euclidean distance between two (x, y, z) tuples."""
    return math.sqrt(sum((p - q) ** 2 for p, q in zip(a, b)))


def interpolate(a, b, t: float):
    """Linear interpolation between two (x, y, z) tuples."""
    return tuple(p + t * (q - p) for p, q in zip(a, b))
