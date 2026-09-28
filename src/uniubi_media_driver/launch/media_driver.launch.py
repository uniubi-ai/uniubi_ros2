from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _start_backend(context):
    backend = LaunchConfiguration("video_backend").perform(context)
    if backend == "mediabus":
        return [
            Node(
                package="uniubi_media_driver",
                executable="uniubi_media_driver_node",
                name="uniubi_media_driver",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            )
        ]
    if backend != "rtsp":
        raise ValueError("video_backend must be mediabus or rtsp")

    host = LaunchConfiguration("host").perform(context).strip()
    parameters = [LaunchConfiguration("rtsp_config_file")]
    if host:
        if any(c in host for c in "/@?# "):
            raise ValueError("host must be a hostname or IP address without a URL scheme")
        parameters.append(
            {
                "rtsp_urls": [
                    f"rtsp://{host}:554/live?channel=1&stream=0",
                    f"rtsp://{host}:554/live?channel=2&stream=0",
                ]
            }
        )
    return [
        Node(
            package="uniubi_media_driver",
            executable="uniubi_rtsp_driver_node",
            name="uniubi_rtsp_driver",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "video_backend", default_value="mediabus",
                description="Video backend: mediabus or rtsp",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("uniubi_media_driver"), "config", "media_driver.yaml"]
                ),
                description="MediaBus driver YAML configuration",
            ),
            DeclareLaunchArgument(
                "rtsp_config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("uniubi_media_driver"), "config", "rtsp_driver.yaml"]
                ),
                description="RTSP driver YAML configuration (use rtsp_urls when host is omitted)",
            ),
            DeclareLaunchArgument(
                "host", default_value="", description="RTSP host for two default camera URLs"
            ),
            OpaqueFunction(function=_start_backend),
        ]
    )
