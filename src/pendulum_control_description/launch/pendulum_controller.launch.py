from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to xacro file
    xacro_path = os.path.join(
        get_package_share_directory("pendulum_control_description"),
        "urdf",
        "pendulum_robot_controllers.xacro",
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": Command(["xacro ", xacro_path])}],
    )

    # Spawn entity in Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "pendulum_controlled",
            "-allow_renaming",
            "true",
        ],
    )

    # Controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_position_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "position_control",
        ],
        output="screen",
    )

    load_velocity_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "velocity_control",
        ],
        output="screen",
    )

    # Start Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments=[("gz_args", [" -r -v 4 empty.sdf"])],
    )

    # Event: Spawn robot after robot_state_publisher starts
    spawn_after_rsp = RegisterEventHandler(
        OnProcessStart(
            target_action=node_robot_state_publisher,
            on_start=[gz_spawn_entity],
        )
    )

    # Event: Load joint_state_broadcaster after spawn
    load_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    # Event: Load position_controller after JSB
    load_pos_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_position_controller],
        )
    )

    # Event: Load velocity_controller after JSB (inactive)
    load_vel_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_velocity_controller],
        )
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    return LaunchDescription(
        [
            gz_sim,
            node_robot_state_publisher,
            spawn_after_rsp,
            load_jsb_after_spawn,
            load_pos_after_jsb,
            load_vel_after_jsb,
            gz_ros2_bridge,
        ]
    )
