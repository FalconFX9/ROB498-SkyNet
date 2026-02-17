"""
Full System Launch for MARY Drone
Launches all nodes for complete autonomous operation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directories
    hardware_pkg = get_package_share_directory('mary_hardware')

    return LaunchDescription([
        # ========== Arguments ==========
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use simulation instead of real hardware'
        ),

        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyACM0:921600',
            description='FCU connection URL'
        ),

        # ========== Hardware Layer ==========

        # MAVROS
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hardware_pkg, 'launch', 'mavros.launch.py')
            ),
            launch_arguments={
                'fcu_url': LaunchConfiguration('fcu_url'),
            }.items(),
        ),

        # Sensors (T265, IMX219)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hardware_pkg, 'launch', 'sensors.launch.py')
            ),
        ),

        # ========== Perception Layer ==========

        # T265 Pose processing
        Node(
            package='mary_perception',
            executable='t265_pose_node',
            name='t265_pose_node',
            output='screen',
            parameters=[{
                'publish_rate': 30.0,
                'publish_tf': True,
            }],
        ),

        # Person tracking
        Node(
            package='mary_perception',
            executable='person_tracker_node',
            name='person_tracker_node',
            output='screen',
            parameters=[{
                'model_path': 'yolov8n.pt',
                'confidence_threshold': 0.5,
                'publish_debug_image': True,
            }],
        ),

        # ========== Control Layer ==========

        # Communication node
        Node(
            package='mary_control',
            executable='comm_node',
            name='comm_node',
            output='screen',
            parameters=[{
                'setpoint_rate': 20.0,
                'vision_pose_rate': 30.0,
            }],
        ),

        # Altitude controller
        Node(
            package='mary_control',
            executable='altitude_controller_node',
            name='altitude_controller_node',
            output='screen',
            parameters=[{
                'kp': 1.5,
                'ki': 0.1,
                'kd': 0.5,
                'max_output': 1.5,
            }],
        ),

        # Person follower
        Node(
            package='mary_control',
            executable='follower_node',
            name='follower_node',
            output='screen',
            parameters=[{
                'target_altitude': 2.5,
                'max_horizontal_speed': 2.0,
                'max_vertical_speed': 1.0,
                'position_p_gain': 1.0,
            }],
        ),

        # Mission manager
        Node(
            package='mary_control',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
            parameters=[{
                'takeoff_altitude': 2.5,
                'arming_timeout': 10.0,
            }],
        ),
    ])
