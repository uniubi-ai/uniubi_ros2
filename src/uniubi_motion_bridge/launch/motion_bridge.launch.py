from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    device_id = LaunchConfiguration("device_id")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("uniubi_motion_bridge"), "config", "motion_bridge.yaml"]
                ),
                description="Optional path to a uniubi_motion_bridge YAML file",
            ),
            DeclareLaunchArgument(
                "device_id",
                default_value="",
                description="Target robot device ID; required on a shared DDS domain",
            ),
            Node(
                package="uniubi_motion_bridge",
                executable="uniubi_motion_bridge_node",
                name="uniubi_motion_bridge",
                output="screen",
                parameters=[config_file, {"device_id": device_id}],
            ),
        ]
    )
