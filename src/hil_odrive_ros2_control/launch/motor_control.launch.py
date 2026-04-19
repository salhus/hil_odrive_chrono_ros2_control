from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = []

    # Allow overriding controllers yaml path
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("hil_odrive_ros2_control"), "config", "controllers.yaml"]
            ),
        )
    )

    controllers_file = LaunchConfiguration("controllers_file")

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

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="both",
    )

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motor_effort_controller", "--controller-manager", "/controller_manager"],
        output="both",
    )

    pto_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pto_effort_controller", "--controller-manager", "/controller_manager"],
        output="both",
    )

    chrono_flap_node = Node(
        package="chrono_flap_sim",
        executable="chrono_flap_node",
        name="chrono_flap_node",
        output="both",
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            effort_controller_spawner,
            pto_controller_spawner,
            chrono_flap_node,
        ]
    )
