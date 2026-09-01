"""A blocking wrapper around the move_to_pose and follow_path actions.

The action API is asynchronous, which is awkward for a caller that just wants
to run a sequence of motions in order. This wraps both actions in blocking
calls built on futures and events, so a worker thread can write

    client.move_to(point)
    client.follow_path(points, times)

while the node itself is spun by an executor on another thread.

Call these from a thread that is *not* spinning the node -- they block waiting
on the executor to deliver results.
"""

import threading

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from rclpy.action import ActionClient

from kuka_control_tutorial.action import FollowPath, MoveToPose


class Outcome:
    """What a motion did. Falsy when the motion did not succeed."""

    def __init__(self, success, message, position_error=float("nan"), status=None):
        self.success = success
        self.message = message
        self.position_error = position_error
        self.status = status

    def __bool__(self):
        return self.success

    def __repr__(self):
        return f"Outcome(success={self.success}, message={self.message!r})"


class MotionClient:
    """Blocking client for the cartesian_move_server actions."""

    def __init__(self, node, move_action="move_to_pose", path_action="follow_path"):
        self.node = node
        self._move = ActionClient(node, MoveToPose, move_action)
        self._path = ActionClient(node, FollowPath, path_action)
        self._active_handle = None
        self._handle_lock = threading.Lock()

    def wait_for_servers(self, timeout_sec=10.0):
        """True once both action servers are up."""
        if not self._move.wait_for_server(timeout_sec=timeout_sec):
            return False
        return self._path.wait_for_server(timeout_sec=timeout_sec)

    # ------------------------------------------------------------------ motions

    def move_to(self, position, speed=0.0, orientation=None, feedback=None):
        """Straight line to ``position``. Blocks until the motion finishes."""
        goal = MoveToPose.Goal()
        goal.target_position.x, goal.target_position.y, goal.target_position.z = position
        goal.speed = float(speed)
        if orientation is None:
            goal.hold_orientation = True
        else:
            goal.hold_orientation = False
            (
                goal.target_orientation.x,
                goal.target_orientation.y,
                goal.target_orientation.z,
                goal.target_orientation.w,
            ) = orientation
        return self._run(self._move, goal, feedback)

    def follow_path(self, points, times, orientation=None, feedback=None):
        """Replay a time-parameterised path. Blocks until it finishes."""
        goal = FollowPath.Goal()
        goal.waypoints = [
            Point(x=float(x), y=float(y), z=float(z)) for x, y, z in points
        ]
        goal.time_from_start = [float(t) for t in times]
        if orientation is None:
            goal.hold_orientation = True
        else:
            goal.hold_orientation = False
            (
                goal.orientation.x,
                goal.orientation.y,
                goal.orientation.z,
                goal.orientation.w,
            ) = orientation
        return self._run(self._path, goal, feedback)

    def cancel(self):
        """Ask the running motion to stop. Safe to call when nothing is running."""
        with self._handle_lock:
            handle = self._active_handle
        if handle is not None:
            handle.cancel_goal_async()
            return True
        return False

    # ------------------------------------------------------------------ plumbing

    def _run(self, client, goal, feedback):
        goal_future = client.send_goal_async(
            goal, feedback_callback=self._wrap_feedback(feedback)
        )
        handle = self._wait(goal_future)
        if handle is None:
            return Outcome(False, "Timed out waiting for the goal to be accepted.")
        if not handle.accepted:
            return Outcome(False, "Goal rejected; see the server log for why.")

        with self._handle_lock:
            self._active_handle = handle
        try:
            wrapped = self._wait(handle.get_result_async())
        finally:
            with self._handle_lock:
                self._active_handle = None

        if wrapped is None:
            return Outcome(False, "Timed out waiting for the result.")

        result = wrapped.result
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            return Outcome(False, result.message or "Canceled.", status=wrapped.status)
        return Outcome(
            result.success, result.message, result.position_error, wrapped.status
        )

    @staticmethod
    def _wrap_feedback(feedback):
        if feedback is None:
            return None

        def on_feedback(msg):
            feedback(msg.feedback)

        return on_feedback

    @staticmethod
    def _wait(future, timeout=None):
        """Block until ``future`` resolves, without spinning the node ourselves."""
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            return None
        return future.result()
