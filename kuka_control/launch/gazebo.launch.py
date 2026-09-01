from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch.substitutions import PathJoinSubstitution
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin
from lbr_bringup.gazebo import GazeboMixin


def launch_setup(context, *args, **kwargs):
    ctrl = LaunchConfiguration("ctrl").perform(context)

    # Dynamically set sys_cfg based on ctrl value
    sys_cfg_default = "config/lbr_system_config_position.yaml"
    if (
        ctrl == "cartesian_impedance_controller"
        or ctrl == "joint_impedance_controller"
        or ctrl == "gravity_compensation"
    ):
        sys_cfg_default = "config/lbr_system_config_torque.yaml"

    print(f"Using system config: {sys_cfg_default}")
    # Declare sys_cfg argument now that we know the correct default
    sys_cfg_arg = DeclareLaunchArgument(
        "sys_cfg",
        default_value=sys_cfg_default,
        description="Path to the system config YAML file",
    )

    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("lbr_description"),
                        "urdf",
                        LaunchConfiguration("model"),
                        LaunchConfiguration("model"),
                    ]
                ),
                ".xacro",
                " robot_name:=",
                LaunchConfiguration("robot_name"),
                " mode:=gazebo",
                " system_config_path:=",
                PathJoinSubstitution(
                    [FindPackageShare("kuka_control"), LaunchConfiguration("sys_cfg")]
                ),
                " initial_joint_positions_path:=",
                PathJoinSubstitution(
                    [
                        FindPackageShare("kuka_control"),
                        "config/initial_joint_positions.yaml",
                    ]
                ),
                " gazebo_controllers_path:=",
                PathJoinSubstitution(
                    [FindPackageShare("kuka_control"), "config/gazebo_controllers.yaml"]
                ),
            ]
        )
    }

    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=False
    )
    return [
        sys_cfg_arg,
        robot_state_publisher,
    ]


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Basic args
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(
        DeclareLaunchArgument(
            name="world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("kuka_control"), "config/world.sdf"]
            ),
            description="Path to the Gazebo world SDF file.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="ctrl",
            default_value="kuka_clik_controller",
            description="Desired default controller.",
            choices=[
                "admittance_controller",
                "joint_trajectory_controller",
                "forward_position_controller",
                "lbr_joint_position_command_controller",
                "lbr_torque_command_controller",
                "lbr_wrench_command_controller",
                "twist_controller",
                "gravity_compensation",
                "cartesian_impedance_controller",
                "joint_impedance_controller",
                "kuka_clik_controller",
            ],
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                ),
            ),
            launch_arguments={
                "gz_args": [
                    TextSubstitution(text="-r "),
                    LaunchConfiguration("world"),
                ]
            }.items(),
        )
    )  # Gazebo has its own controller manager
    ld.add_action(GazeboMixin.node_clock_bridge())
    ld.add_action(GazeboMixin.node_create())
    # Opaque function to evaluate 'ctrl' and configure dependent args/nodes
    ld.add_action(OpaqueFunction(function=launch_setup))

    joint_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="joint_state_broadcaster"
    )
    ld.add_action(joint_state_broadcaster)
    ld.add_action(
        LBRROS2ControlMixin.node_controller_spawner(
            controller=LaunchConfiguration("ctrl")
        )
    )

    return ld
