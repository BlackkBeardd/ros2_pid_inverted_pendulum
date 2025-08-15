from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
import time


def generate_launch_description():

    conf_file_path = "pid.yaml"

    pid_conf = PathJoinSubstitution(
        [get_package_share_directory("pid_controller"), conf_file_path]
    )

    directory = get_package_share_directory("pid_controller")
    arg = os.path.join(directory, "config", conf_file_path)

    # Include the PID controller node
    pid_controller_node = Node(
        package="pid_controller",
        executable="pid_controller",
        name="pid_controller",
        output="screen",
        parameters=[pid_conf],
    )

    return LaunchDescription([pid_controller_node])
