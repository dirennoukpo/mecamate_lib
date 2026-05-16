##
## mecamate_launch.py for mecamate_lib [SSH: ROSMASTER-YAHBOOM] in /home/rosmaster/mecamate_lib
##
## Made by dirennoukpo
## Login   <diren.noukpo@epitech.eu>
##
## Started on  Sat May 16 07:40:54 2026 dirennoukpo
## Last update Sun May 16 07:42:03 2026 dirennoukpo
##

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mecamate_bringup")

    # ── Arguments ──────────────────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        "serial_port", default_value="/dev/myserial",
        description="Serial port for Mecamate")

    use_ros2_control_arg = DeclareLaunchArgument(
        "use_ros2_control", default_value="true",
        description="Use ros2_control hardware interface (vs standalone node)")

    # ── URDF / robot_description ────────────────────────────────────────────
    robot_description = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([pkg, "urdf", "mecamate.urdf.xacro"]),
        " serial_port:=", LaunchConfiguration("serial_port"),
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # ── ros2_control path ───────────────────────────────────────────────────
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            PathJoinSubstitution([pkg, "config", "mecamate_controllers.yaml"]),
        ],
        output="screen",
    )

    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    spawn_mecanum = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # Spawn mecanum only after joint_state_broadcaster is running
    delay_mecanum = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_mecanum],
        )
    )

    # ── Standalone mecamate_node path (sans ros2_control) ──────────────────
    mecamate_node = Node(
        package="mecamate_lib",
        executable="mecamate_node",
        output="screen",
        parameters=[{
            "serial_port":    LaunchConfiguration("serial_port"),
            "car_type":       1,
            "publish_rate":   50.0,
            "publish_tf":     True,
            "cmd_vel_timeout": 0.5,
        }],
        condition=UnlessCondition(LaunchConfiguration("use_ros2_control")),
    )

    return LaunchDescription([
        serial_port_arg,
        use_ros2_control_arg,
        robot_state_publisher,
        controller_manager,
        spawn_jsb,
        delay_mecanum,
        mecamate_node,
    ])
