"""
MAVROS Launch File for MARY Drone
Connects to Cube Orange+ via USB serial
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('mary_hardware')

    return LaunchDescription([
        # FCU connection URL (Cube Orange+ via USB)
        DeclareLaunchArgument(
            name='fcu_url',
            default_value='/dev/ttyACM0:921600',
            description='FCU connection URL'
        ),

        # Ground control station URL (optional)
        DeclareLaunchArgument(
            name='gcs_url',
            default_value='',
            description='GCS connection URL'
        ),

        # Target system ID
        DeclareLaunchArgument(
            name='tgt_system',
            default_value='1',
            description='Target system ID'
        ),

        # Target component ID
        DeclareLaunchArgument(
            name='tgt_component',
            default_value='1',
            description='Target component ID'
        ),

        # MAVLink protocol version
        DeclareLaunchArgument(
            name='fcu_protocol',
            default_value='v2.0',
            description='MAVLink protocol version'
        ),

        # Plugin lists
        DeclareLaunchArgument(
            name='pluginlists_yaml',
            default_value=os.path.join(pkg_dir, 'config', 'px4_pluginlists.yaml'),
            description='MAVROS plugin list'
        ),

        # Config file
        DeclareLaunchArgument(
            name='config_yaml',
            default_value=os.path.join(pkg_dir, 'config', 'px4_config.yaml'),
            description='MAVROS configuration'
        ),

        # Namespace
        DeclareLaunchArgument(
            name='namespace',
            default_value='mavros',
            description='MAVROS namespace'
        ),

        # MAVROS node
        Node(
            package='mavros',
            executable='mavros_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                {'fcu_url': LaunchConfiguration('fcu_url')},
                {'gcs_url': LaunchConfiguration('gcs_url')},
                {'tgt_system': LaunchConfiguration('tgt_system')},
                {'tgt_component': LaunchConfiguration('tgt_component')},
                {'fcu_protocol': LaunchConfiguration('fcu_protocol')},
                LaunchConfiguration('pluginlists_yaml'),
                LaunchConfiguration('config_yaml')
            ],
        ),
    ])
