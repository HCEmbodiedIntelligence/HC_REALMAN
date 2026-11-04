from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware_config_path"),
        DeclareLaunchArgument("arms_control", default_value="false"),
        DeclareLaunchArgument("hands_control", default_value="false"),
        Node(
            package="hc_realman_driver",
            executable="driver_node",
            output="screen",
            parameters=[{
                "hardware_config_path": LaunchConfiguration("hardware_config_path"),
                "arms_control": LaunchConfiguration("arms_control"),
                "hands_control": LaunchConfiguration("hands_control"),
            }],
        ),
    ])
