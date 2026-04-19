from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "bearing_friction",
            default_value="0.4",
            description="Bearing friction coefficient for the Chrono simulation.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "control_mode",
            default_value="cascade",
            description="Control mode for the velocity PID node.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "position_setpoint",
            default_value="0.5",
            description="Position setpoint (rad) for the velocity PID node in position_only mode.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_visualization",
            default_value="false",
            description="Enable Chrono 3D visualization (requires working Vulkan/GPU).",
        )
    )

    bearing_friction = LaunchConfiguration("bearing_friction")
    control_mode = LaunchConfiguration("control_mode")
    position_setpoint = LaunchConfiguration("position_setpoint")
    enable_visualization = LaunchConfiguration("enable_visualization")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("hil_odrive_ros2_control"), "description", "urdf", "motor.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    chrono_flap_node = Node(
        package="chrono_flap_sim",
        executable="chrono_flap_node",
        name="chrono_flap_node",
        parameters=[{
            "sil_mode": True,
            "bearing_friction": bearing_friction,
            "enable_visualization": enable_visualization,
        }],
        output="both",
    )

    velocity_pid_node = Node(
        package="odrive_velocity_pid",
        executable="velocity_pid_node",
        name="velocity_pid_node",
        parameters=[{
            "control_mode": control_mode,
            "position_setpoint": position_setpoint,
        }],
        output="both",
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_pub_node,
            chrono_flap_node,
            velocity_pid_node,
        ]
    )
