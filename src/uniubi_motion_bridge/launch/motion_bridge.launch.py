from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def make_bridge_node(context, config_file, device_id, namespace, frame_prefix):
    overrides = {"device_id": device_id}
    # An omitted launch argument must not erase frame_prefix from a custom YAML file.
    if frame_prefix.perform(context):
        overrides["frame_prefix"] = frame_prefix
    return [Node(
        package="uniubi_motion_bridge",
        executable="uniubi_motion_bridge_node",
        name="uniubi_motion_bridge",
        namespace=namespace,
        output="screen",
        parameters=[config_file, overrides],
    )]


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    device_id = LaunchConfiguration("device_id")
    namespace = LaunchConfiguration("namespace")
    frame_prefix = LaunchConfiguration("frame_prefix")
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
            DeclareLaunchArgument(
                "namespace", default_value="", description="Optional ROS namespace for this bridge"
            ),
            DeclareLaunchArgument(
                "frame_prefix", default_value="", description="Optional prefix for odom, base and IMU frame IDs"
            ),
            OpaqueFunction(
                function=make_bridge_node,
                args=[config_file, device_id, namespace, frame_prefix],
            ),
        ]
    )
