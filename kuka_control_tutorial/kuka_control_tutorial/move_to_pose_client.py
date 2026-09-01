"""Command line client for the ``move_to_pose`` action.

    ros2 run kuka_control_tutorial move_to_pose_client -- 0.5 0.0 0.4
    ros2 run kuka_control_tutorial move_to_pose_client -- 0.5 0.0 0.4 --speed 0.03

Ctrl-C sends a cancel request, so the arm stops at the setpoint it has already
reached rather than continuing to the goal.
"""

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from kuka_control_tutorial.action import MoveToPose


class MoveToPoseClient(Node):
    def __init__(self, action_name: str):
        super().__init__("move_to_pose_client")
        self._client = ActionClient(self, MoveToPose, action_name)
        self.action_name = action_name

    def send(self, x, y, z, speed, orientation):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"Action server '{self.action_name}' is not available. "
                "Is cartesian_move_server running?"
            )
            return 1

        goal = MoveToPose.Goal()
        goal.target_position.x = x
        goal.target_position.y = y
        goal.target_position.z = z
        goal.speed = speed

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

        self.get_logger().info(f"Sending goal: ({x:.3f}, {y:.3f}, {z:.3f})")
        goal_future = self._client.send_goal_async(
            goal, feedback_callback=self.on_feedback
        )
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected; see the server log for why.")
            return 1

        result_future = goal_handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
        except KeyboardInterrupt:
            self.get_logger().warn("Interrupted, canceling the goal.")
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            return 1

        result = result_future.result().result
        p = result.final_pose.position
        self.get_logger().info(
            f"{'Succeeded' if result.success else 'Failed'}: {result.message}\n"
            f"  final pose:     ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})\n"
            f"  position error: {result.position_error:.4f} m"
        )
        return 0 if result.success else 1

    def on_feedback(self, msg):
        fb = msg.feedback
        p = fb.current_pose.position
        self.get_logger().info(
            f"{fb.percent_complete:5.1f}%  at ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})  "
            f"remaining {fb.distance_remaining:.4f} m"
        )


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("x", type=float, help="target x in the robot base frame [m]")
    parser.add_argument("y", type=float, help="target y in the robot base frame [m]")
    parser.add_argument("z", type=float, help="target z in the robot base frame [m]")
    parser.add_argument(
        "--speed",
        type=float,
        default=0.0,
        help="cruise speed [m/s]; 0 uses the server's default_speed",
    )
    parser.add_argument(
        "--orientation",
        type=float,
        nargs=4,
        metavar=("QX", "QY", "QZ", "QW"),
        default=None,
        help="target orientation quaternion; omitted means hold the current one",
    )
    parser.add_argument(
        "--action-name",
        default="move_to_pose",
        help="action name, useful when the server runs in a namespace",
    )
    parsed = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    node = MoveToPoseClient(parsed.action_name)
    try:
        code = node.send(
            parsed.x, parsed.y, parsed.z, parsed.speed, parsed.orientation
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
