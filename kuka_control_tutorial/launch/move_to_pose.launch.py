from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            name="robot_name",
            default_value="lbr",
            description="Namespace the robot and its controllers run in.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
            description="Set true when running against Gazebo.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="cfg",
            default_value="config/move_to_pose.yaml",
            description="Parameter file, relative to the kuka_control_tutorial share.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="gui",
            default_value="false",
            description="Also start the draw_trajectory_gui drawing demo.",
        )
    )

    params = [
        PathJoinSubstitution(
            [FindPackageShare("kuka_control_tutorial"), LaunchConfiguration("cfg")]
        ),
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]

    ld.add_action(
        Node(
            package="kuka_control_tutorial",
            executable="cartesian_move_server",
            name="cartesian_move_server",
            namespace=LaunchConfiguration("robot_name"),
            output="screen",
            parameters=params,
        )
    )

    ld.add_action(
        Node(
            package="kuka_control_tutorial",
            executable="draw_trajectory_gui",
            name="draw_trajectory_gui",
            namespace=LaunchConfiguration("robot_name"),
            output="screen",
            parameters=params,
            condition=IfCondition(LaunchConfiguration("gui")),
        )
    )

    return ld
