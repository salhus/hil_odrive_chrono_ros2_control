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

    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_visualization",
            default_value="false",
            description="Enable Chrono 3D visualization (requires working Vulkan/GPU).",
        )
    )

    controllers_file = LaunchConfiguration("controllers_file")
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

    sim_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("hil_odrive_ros2_control"), "description", "urdf", "motor_sim.urdf.xacro"]
            ),
        ]
    )
    sim_robot_description = {"robot_description": ParameterValue(sim_robot_description_content, value_type=str)}

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

    sim_robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="sim_robot_state_publisher",
        parameters=[sim_robot_description, {"frame_prefix": "sim/"}],
        remappings=[("joint_states", "/sim_joint_states")],
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

    velocity_pid_node = Node(
        package="odrive_velocity_pid",
        executable="velocity_pid_node",
        name="velocity_pid_node",
        output="both",
    )

    chrono_flap_node = Node(
        package="chrono_flap_sim",
        executable="chrono_flap_node",
        name="chrono_flap_node",
        parameters=[{"enable_visualization": enable_visualization}],
        output="both",
    )

    # Identity transform: sim/base_link overlaid exactly on base_link (x y z yaw pitch roll)
    static_tf_sim = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "sim/base_link"],
        output="both",
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_pub_node,
            sim_robot_state_pub_node,
            joint_state_broadcaster_spawner,
            effort_controller_spawner,
            velocity_pid_node,
            chrono_flap_node,
            static_tf_sim,
        ]
    )
