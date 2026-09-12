from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [DeclareLaunchArgument(
        'config_file', default_value=PathJoinSubstitution([
            FindPackageShare('uniubi_media_driver'), 'config', 'audio_driver.yaml']))]
    for name in ('host', 'device_id', 'network_interface'):
        args.append(DeclareLaunchArgument(name, default_value=''))
    return LaunchDescription(args + [OpaqueFunction(function=launch_node)])


def launch_node(context):
    # Empty launch arguments preserve values supplied by the YAML file.
    overrides = {}
    for name in ('host', 'device_id', 'network_interface'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = ParameterValue(value, value_type=str)
    return [Node(
        package='uniubi_media_driver', executable='uniubi_media_driver_node',
        name='uniubi_media_driver', output='screen',
        parameters=[LaunchConfiguration('config_file').perform(context), overrides])]
