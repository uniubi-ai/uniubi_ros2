from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("uniubi_media_driver"), "config", "media_driver.yaml"]
                ),
                description="Path to a uniubi_media_driver YAML file",
            ),
            Node(
                package="uniubi_media_driver",
                executable="uniubi_media_driver_node",
                name="uniubi_media_driver",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
