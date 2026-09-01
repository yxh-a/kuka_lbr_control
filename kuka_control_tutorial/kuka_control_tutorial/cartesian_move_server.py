"""Action servers that drive the LBR end effector via the impedance controller.

The Cartesian impedance controller takes a single setpoint pose on
``<controller>/target_frame`` and pulls the end effector towards it with a
spring. Publishing a distant goal in one shot would stretch that spring by the
full travel distance, which is both a jolt and a large force. Everything here
instead streams closely spaced setpoints so the spring stays short.

Two actions share that machinery:

``move_to_pose`` (:class:`MoveToPose`)
    Straight line to a point, timed by a trapezoidal velocity profile. The
    server picks the timing.

``follow_path`` (:class:`FollowPath`)
    An arbitrary path with timing supplied by the caller, for replaying a
    recorded or drawn trajectory. The caller picks the timing.

The measured pose comes from TF (``base_link`` -> ``ee_link``), since the
controller publishes no end effector pose of its own.
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

from kuka_control_tutorial.action import FollowPath, MoveToPose
from kuka_control_tutorial.path_planning import path_length, sample_path
from kuka_control_tutorial.trajectory import (
    TrapezoidalProfile,
    distance,
    interpolate,
    quat_normalize,
    quat_slerp,
)

# Outcomes of the setpoint streaming loop.
DONE = "done"
CANCELED = "canceled"
ABORTED = "aborted"


class CartesianMoveServer(Node):
    def __init__(self):
        super().__init__("cartesian_move_server")

        # --- Frames and topics -------------------------------------------------
        self.declare_parameter("base_link", "lbr_link_0")
        self.declare_parameter("ee_link", "lbr_link_ee")
        self.declare_parameter(
            "target_frame_topic", "/lbr/cartesian_impedance_controller/target_frame"
        )

        # --- Motion shaping ----------------------------------------------------
        self.declare_parameter("control_rate", 100.0)  # setpoint stream rate [Hz]
        self.declare_parameter("default_speed", 0.05)  # [m/s]
        self.declare_parameter("max_speed", 0.25)  # [m/s]
        self.declare_parameter("acceleration", 0.1)  # [m/s^2]

        # --- Limits and safety -------------------------------------------------
        self.declare_parameter("goal_tolerance", 0.02)  # [m]
        self.declare_parameter("settle_timeout", 5.0)  # [s]
        self.declare_parameter("max_travel", 1.0)  # [m] per move_to_pose goal
        self.declare_parameter("max_path_length", 5.0)  # [m] per follow_path goal
        self.declare_parameter("max_reach", 0.85)  # [m] from base_link origin
        self.declare_parameter("min_z", 0.05)  # [m] floor guard in base_link
        self.declare_parameter("max_lag", 0.15)  # [m] setpoint vs. measured
        self.declare_parameter("path_start_tolerance", 0.05)  # [m]
        self.declare_parameter("tf_timeout", 2.0)  # [s] wait for first transform
        self.declare_parameter("clock_stall_timeout", 5.0)  # [s] wall, clock frozen
        self.declare_parameter("require_controller", True)

        self.base_link = self.get_parameter("base_link").value
        self.ee_link = self.get_parameter("ee_link").value
        topic = self.get_parameter("target_frame_topic").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # The controller keeps a depth-1 queue and only ever uses the newest
        # setpoint, so a shallow reliable queue matches it.
        self.target_pub = self.create_publisher(
            PoseStamped,
            topic,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )

        # Last setpoint we published, so a follow-up goal starts from where the
        # previous one left the spring rather than from the sagged pose.
        self._last_setpoint = None
        self._goal_lock = threading.Lock()
        self._busy = False

        group = ReentrantCallbackGroup()
        self._move_server = ActionServer(
            self,
            MoveToPose,
            "move_to_pose",
            execute_callback=self.execute_move,
            goal_callback=self.move_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=group,
        )
        self._path_server = ActionServer(
            self,
            FollowPath,
            "follow_path",
            execute_callback=self.execute_path,
            goal_callback=self.path_goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=group,
        )

        self.get_logger().info(
            f"cartesian_move_server ready: {self.base_link} -> {self.ee_link}, "
            f"publishing setpoints on {topic}"
        )

    # ------------------------------------------------------------------ helpers

    def lookup_ee_pose(self):
        """Return the current end effector pose as ((x, y, z), (qx, qy, qz, qw)).

        Returns None when TF has no recent transform.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_link, self.ee_link, rclpy.time.Time()
            )
        except Exception as exc:  # tf2 raises several unrelated exception types
            self.get_logger().warn(
                f"TF lookup {self.base_link} -> {self.ee_link} failed: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

        t = tf.transform.translation
        r = tf.transform.rotation
        return ((t.x, t.y, t.z), (r.x, r.y, r.z, r.w))

    def wait_for_ee_pose(self, timeout: float):
        """Poll TF until the transform shows up or ``timeout`` seconds elapse."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pose = self.lookup_ee_pose()
            if pose is not None:
                return pose
            time.sleep(0.05)
        return None

    def publish_setpoint(self, position, orientation):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # The controller rejects any target that is not in robot_base_link.
        msg.header.frame_id = self.base_link
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = orientation
        self.target_pub.publish(msg)
        self._last_setpoint = (position, orientation)

    @staticmethod
    def to_pose_msg(position, orientation) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = orientation
        return pose

    def start_pose(self, measured):
        """Where the next motion should start its setpoint from.

        Prefers the last setpoint we published, so back-to-back goals do not
        step the spring backwards, but falls back to the measured pose when the
        two have drifted apart.
        """
        measured_xyz, measured_quat = measured
        if self._last_setpoint is None:
            return measured_xyz, measured_quat

        lag = distance(self._last_setpoint[0], measured_xyz)
        if lag <= self.get_parameter("max_lag").value:
            return self._last_setpoint

        self.get_logger().warn(
            f"Previous setpoint is {lag:.3f} m from the measured pose; "
            "restarting from the measured pose."
        )
        return measured_xyz, measured_quat

    def check_point(self, xyz):
        """Return a rejection reason for a target point, or None if it is fine."""
        if not all(math.isfinite(v) for v in xyz):
            return "position is not finite"

        reach = math.sqrt(sum(v * v for v in xyz))
        max_reach = self.get_parameter("max_reach").value
        if reach > max_reach:
            return (
                f"({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) is {reach:.3f} m from "
                f"{self.base_link}, beyond max_reach {max_reach:.3f} m"
            )

        min_z = self.get_parameter("min_z").value
        if xyz[2] < min_z:
            return f"z {xyz[2]:.3f} m is below min_z {min_z:.3f} m"

        return None

    def claim(self, what):
        """Take the single-motion lock, or return False if one is running.

        Two motions at once would fight over the same setpoint topic, so a
        second goal is rejected rather than queued.
        """
        with self._goal_lock:
            if self._busy:
                self.get_logger().error(
                    f"Rejecting {what}: a motion is already running."
                )
                return False
            self._busy = True
        return True

    def release(self):
        with self._goal_lock:
            self._busy = False

    def controller_missing(self):
        return (
            self.get_parameter("require_controller").value
            and self.target_pub.get_subscription_count() == 0
        )

    # ---------------------------------------------------------- streaming core

    def stream(self, goal_handle, sampler, duration, publish_feedback):
        """Stream setpoints from a sampler until ``duration`` of ROS time passes.

        ``sampler(elapsed)`` returns ``(progress, position, orientation)``, and
        ``publish_feedback(measured, progress, elapsed)`` is called at about
        10 Hz. Returns ``(outcome, measured, message)``.

        This is the part both actions share: cancellation, the clock-stall
        watchdog, the tracking-lag watchdog and the actual publishing.
        """
        rate = self.get_parameter("control_rate").value
        period = 1.0 / rate
        max_lag = self.get_parameter("max_lag").value
        stall_timeout = self.get_parameter("clock_stall_timeout").value
        feedback_every = max(1, int(rate / 10.0))  # ~10 Hz feedback

        measured = self.lookup_ee_pose()
        start_time = self.get_clock().now()
        # The profile is timed off the ROS clock while the loop sleeps on the
        # wall clock, so a sim running faster or slower than real time only
        # changes how many setpoints we emit per simulated second -- the motion
        # itself stays correct. The one thing that must not happen is spinning
        # forever on a clock that has stopped, so watch for that specifically
        # instead of putting a wall-clock cap on the motion duration.
        last_ros_time = start_time
        last_advance_wall = time.monotonic()
        tick = 0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                # Freeze at the setpoint we last commanded: the arm stops where
                # it is instead of springing anywhere new.
                if self._last_setpoint is not None:
                    self.publish_setpoint(*self._last_setpoint)
                return (
                    CANCELED,
                    self.lookup_ee_pose() or measured,
                    "Canceled; holding the setpoint reached so far.",
                )

            now = self.get_clock().now()
            if now > last_ros_time:
                last_ros_time = now
                last_advance_wall = time.monotonic()
            elif time.monotonic() - last_advance_wall > stall_timeout:
                return (
                    ABORTED,
                    self.lookup_ee_pose() or measured,
                    f"The ROS clock has not advanced for {stall_timeout:.1f} s; "
                    "is the simulation paused or /clock not being published?",
                )

            elapsed = (now - start_time).nanoseconds * 1e-9
            progress, setpoint_xyz, setpoint_quat = sampler(elapsed)
            self.publish_setpoint(setpoint_xyz, setpoint_quat)

            measured = self.lookup_ee_pose() or measured
            lag = distance(setpoint_xyz, measured[0])
            if lag > max_lag:
                # The arm is not following: collision, joint limit, e-stop or a
                # controller that is no longer active. Drop the setpoint onto
                # the measured pose so the spring force decays to ~zero.
                self.publish_setpoint(measured[0], measured[1])
                return (
                    ABORTED,
                    measured,
                    f"Aborted: end effector lags the setpoint by {lag:.3f} m "
                    f"(max_lag {max_lag:.3f} m). Setpoint reset to the measured pose.",
                )

            if tick % feedback_every == 0:
                publish_feedback(measured, progress, elapsed)
            tick += 1

            if elapsed >= duration:
                return DONE, measured, ""

            # Sleeping on the wall clock while timing the motion from the ROS
            # clock keeps the stream rate steady without needing a Rate object
            # inside an executor callback.
            time.sleep(period)

        return ABORTED, measured, "Shutting down."

    def settle(self, goal_handle, target_xyz, target_quat, measured):
        """Hold the final setpoint until the arm catches up.

        Under a soft stiffness the arm stops short of the setpoint: gravity and
        the spring balance at a non-zero offset. That is expected, so we wait
        rather than declaring the motion done the instant streaming ends.
        Returns ``(outcome, measured, error)``.
        """
        period = 1.0 / self.get_parameter("control_rate").value
        tolerance = self.get_parameter("goal_tolerance").value
        settle_timeout = self.get_parameter("settle_timeout").value
        stall_timeout = self.get_parameter("clock_stall_timeout").value

        settle_start = self.get_clock().now()
        last_ros_time = settle_start
        last_advance_wall = time.monotonic()
        error = float("inf")

        while rclpy.ok():
            now = self.get_clock().now()
            if now > last_ros_time:
                last_ros_time = now
                last_advance_wall = time.monotonic()
            elif time.monotonic() - last_advance_wall > stall_timeout:
                break
            if (now - settle_start).nanoseconds * 1e-9 >= settle_timeout:
                break

            self.publish_setpoint(target_xyz, target_quat)
            measured = self.lookup_ee_pose() or measured
            error = distance(measured[0], target_xyz)
            if error <= tolerance:
                return DONE, measured, error
            if goal_handle.is_cancel_requested:
                return CANCELED, measured, error
            time.sleep(period)

        return (DONE if error <= tolerance else ABORTED), measured, error

    # ------------------------------------------------------- move_to_pose action

    def move_goal_callback(self, goal_request):
        """Reject anything unsafe before the arm starts moving."""
        target = goal_request.target_position
        target_xyz = (target.x, target.y, target.z)

        reason = self.check_point(target_xyz)
        if reason is not None:
            self.get_logger().error(f"Rejecting goal: target {reason}.")
            return GoalResponse.REJECT

        max_speed = self.get_parameter("max_speed").value
        if goal_request.speed > max_speed:
            self.get_logger().error(
                f"Rejecting goal: speed {goal_request.speed:.3f} m/s exceeds "
                f"max_speed {max_speed:.3f} m/s."
            )
            return GoalResponse.REJECT

        if self.controller_missing():
            self.get_logger().error(
                "Rejecting goal: nothing is subscribed to the target_frame topic. "
                "Is the cartesian_impedance_controller running and active?"
            )
            return GoalResponse.REJECT

        if not self.claim("goal"):
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested, holding the current setpoint.")
        return CancelResponse.ACCEPT

    def execute_move(self, goal_handle):
        try:
            return self.run_move(goal_handle)
        finally:
            self.release()

    def move_result(self, success, message, measured, target):
        result = MoveToPose.Result()
        result.success = success
        result.message = message
        if measured is not None:
            result.final_pose = self.to_pose_msg(*measured)
            result.position_error = distance(measured[0], target)
        else:
            result.position_error = float("nan")
        return result

    def run_move(self, goal_handle):
        request = goal_handle.request
        target_xyz = (
            request.target_position.x,
            request.target_position.y,
            request.target_position.z,
        )

        measured = self.wait_for_ee_pose(self.get_parameter("tf_timeout").value)
        if measured is None:
            goal_handle.abort()
            return self.move_result(
                False,
                f"No TF transform {self.base_link} -> {self.ee_link}; "
                "is robot_state_publisher running?",
                None,
                target_xyz,
            )

        start_xyz, start_quat = self.start_pose(measured)

        if request.hold_orientation:
            target_quat = start_quat
        else:
            q = request.target_orientation
            target_quat = quat_normalize((q.x, q.y, q.z, q.w))

        travel = distance(start_xyz, target_xyz)
        max_travel = self.get_parameter("max_travel").value
        if travel > max_travel:
            goal_handle.abort()
            return self.move_result(
                False,
                f"Travel {travel:.3f} m exceeds max_travel {max_travel:.3f} m.",
                measured,
                target_xyz,
            )

        if travel < 1e-6:
            self.publish_setpoint(target_xyz, target_quat)
            goal_handle.succeed()
            return self.move_result(True, "Already at the target.", measured, target_xyz)

        speed = request.speed
        if speed <= 0.0:
            speed = self.get_parameter("default_speed").value
        profile = TrapezoidalProfile(
            travel, speed, self.get_parameter("acceleration").value
        )

        self.get_logger().info(
            f"Moving {travel:.3f} m to "
            f"({target_xyz[0]:.3f}, {target_xyz[1]:.3f}, {target_xyz[2]:.3f}) "
            f"at {profile.peak_speed:.3f} m/s over {profile.duration:.2f} s."
        )

        def sampler(elapsed):
            progress = profile.progress_at(elapsed)
            return (
                progress,
                interpolate(start_xyz, target_xyz, progress),
                quat_slerp(start_quat, target_quat, progress),
            )

        feedback = MoveToPose.Feedback()

        def publish_feedback(measured_pose, progress, _elapsed):
            feedback.current_pose = self.to_pose_msg(*measured_pose)
            feedback.distance_remaining = distance(measured_pose[0], target_xyz)
            feedback.percent_complete = 100.0 * progress
            goal_handle.publish_feedback(feedback)

        outcome, measured, message = self.stream(
            goal_handle, sampler, profile.duration, publish_feedback
        )
        if outcome == CANCELED:
            goal_handle.canceled()
            return self.move_result(False, message, measured, target_xyz)
        if outcome == ABORTED:
            goal_handle.abort()
            return self.move_result(False, message, measured, target_xyz)

        outcome, measured, error = self.settle(
            goal_handle, target_xyz, target_quat, measured
        )
        if outcome == CANCELED:
            goal_handle.canceled()
            return self.move_result(
                False, "Canceled while settling.", measured, target_xyz
            )
        if outcome == ABORTED:
            goal_handle.abort()
            return self.move_result(
                False,
                f"Setpoint delivered but the end effector settled {error:.4f} m away "
                f"(tolerance {self.get_parameter('goal_tolerance').value:.4f} m). "
                "With a soft stiffness this is the expected steady-state offset -- "
                "raise the stiffness or the goal_tolerance parameter.",
                measured,
                target_xyz,
            )

        goal_handle.succeed()
        return self.move_result(
            True, f"Reached the target within {error:.4f} m.", measured, target_xyz
        )

    # -------------------------------------------------------- follow_path action

    def path_goal_callback(self, goal_request):
        """Validate the whole path before any of it is streamed."""
        points = [(p.x, p.y, p.z) for p in goal_request.waypoints]
        times = list(goal_request.time_from_start)

        if len(points) < 2:
            self.get_logger().error("Rejecting path: need at least two waypoints.")
            return GoalResponse.REJECT

        if len(times) != len(points):
            self.get_logger().error(
                f"Rejecting path: {len(times)} timestamps for {len(points)} waypoints."
            )
            return GoalResponse.REJECT

        if not all(math.isfinite(t) for t in times) or times[0] < 0.0:
            self.get_logger().error("Rejecting path: timestamps are not usable.")
            return GoalResponse.REJECT

        if any(times[i] <= times[i - 1] for i in range(1, len(times))):
            self.get_logger().error(
                "Rejecting path: time_from_start must strictly increase."
            )
            return GoalResponse.REJECT

        for i, xyz in enumerate(points):
            reason = self.check_point(xyz)
            if reason is not None:
                self.get_logger().error(f"Rejecting path: waypoint {i} {reason}.")
                return GoalResponse.REJECT

        length = path_length(points)
        max_length = self.get_parameter("max_path_length").value
        if length > max_length:
            self.get_logger().error(
                f"Rejecting path: length {length:.3f} m exceeds max_path_length "
                f"{max_length:.3f} m."
            )
            return GoalResponse.REJECT

        max_speed = self.get_parameter("max_speed").value
        for i in range(1, len(points)):
            segment_speed = distance(points[i], points[i - 1]) / (times[i] - times[i - 1])
            if segment_speed > max_speed:
                self.get_logger().error(
                    f"Rejecting path: segment {i} asks for {segment_speed:.3f} m/s, "
                    f"above max_speed {max_speed:.3f} m/s."
                )
                return GoalResponse.REJECT

        if self.controller_missing():
            self.get_logger().error(
                "Rejecting path: nothing is subscribed to the target_frame topic. "
                "Is the cartesian_impedance_controller running and active?"
            )
            return GoalResponse.REJECT

        if not self.claim("path"):
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def execute_path(self, goal_handle):
        try:
            return self.run_path(goal_handle)
        finally:
            self.release()

    def path_result(self, success, message, measured, target):
        result = FollowPath.Result()
        result.success = success
        result.message = message
        if measured is not None:
            result.final_pose = self.to_pose_msg(*measured)
            result.position_error = distance(measured[0], target)
        else:
            result.position_error = float("nan")
        return result

    def run_path(self, goal_handle):
        request = goal_handle.request
        points = [(p.x, p.y, p.z) for p in request.waypoints]
        times = list(request.time_from_start)
        target_xyz = points[-1]

        measured = self.wait_for_ee_pose(self.get_parameter("tf_timeout").value)
        if measured is None:
            goal_handle.abort()
            return self.path_result(
                False,
                f"No TF transform {self.base_link} -> {self.ee_link}; "
                "is robot_state_publisher running?",
                None,
                target_xyz,
            )

        start_xyz, start_quat = self.start_pose(measured)

        # The path carries its own timing, so there is no room to travel to its
        # start: the arm has to already be there or the first setpoint is a jump.
        start_gap = distance(start_xyz, points[0])
        start_tolerance = self.get_parameter("path_start_tolerance").value
        if start_gap > start_tolerance:
            goal_handle.abort()
            return self.path_result(
                False,
                f"The path starts {start_gap:.3f} m away (tolerance "
                f"{start_tolerance:.3f} m). Send a move_to_pose goal to "
                f"({points[0][0]:.3f}, {points[0][1]:.3f}, {points[0][2]:.3f}) first.",
                measured,
                target_xyz,
            )

        if request.hold_orientation:
            path_quat = start_quat
        else:
            q = request.orientation
            path_quat = quat_normalize((q.x, q.y, q.z, q.w))

        duration = times[-1]
        self.get_logger().info(
            f"Following a {path_length(points):.3f} m path of {len(points)} "
            f"waypoints over {duration:.2f} s."
        )

        def sampler(elapsed):
            progress = 0.0 if duration <= 0.0 else min(1.0, elapsed / duration)
            return progress, sample_path(points, times, elapsed), path_quat

        feedback = FollowPath.Feedback()

        def publish_feedback(measured_pose, progress, elapsed):
            feedback.current_pose = self.to_pose_msg(*measured_pose)
            feedback.percent_complete = 100.0 * progress
            feedback.time_remaining = max(0.0, duration - elapsed)
            goal_handle.publish_feedback(feedback)

        outcome, measured, message = self.stream(
            goal_handle, sampler, duration, publish_feedback
        )
        if outcome == CANCELED:
            goal_handle.canceled()
            return self.path_result(False, message, measured, target_xyz)
        if outcome == ABORTED:
            goal_handle.abort()
            return self.path_result(False, message, measured, target_xyz)

        outcome, measured, error = self.settle(
            goal_handle, target_xyz, path_quat, measured
        )
        if outcome == CANCELED:
            goal_handle.canceled()
            return self.path_result(
                False, "Canceled while settling.", measured, target_xyz
            )

        # A path is about the shape it traced, not about nailing the last point,
        # so a wide steady-state offset at the end is reported rather than fatal.
        goal_handle.succeed()
        return self.path_result(
            True,
            f"Path complete, ending {error:.4f} m from the last waypoint.",
            measured,
            target_xyz,
        )


def main(args=None):
    rclpy.init(args=args)
    node = CartesianMoveServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
