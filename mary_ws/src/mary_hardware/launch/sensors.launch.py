"""
Sensor Launch File for MARY Drone
Launches T265 tracking camera and IMX219 camera
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        # ========== Intel RealSense T265 ==========
        # Note: Requires realsense2_camera package
        # Install: sudo apt install ros-humble-realsense2-camera

        DeclareLaunchArgument(
            name='enable_t265',
            default_value='true',
            description='Enable T265 tracking camera'
        ),

        # T265 camera node
        # Uses the official RealSense ROS2 wrapper
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='t265_camera',
            namespace='camera',
            parameters=[{
                'serial_no': '',  # Empty = use first available T265
                'device_type': 't265',
                'enable_pose': True,
                'enable_fisheye1': True,
                'enable_fisheye2': True,
                'fisheye_fps': 30,
                'pose_fps': 200,
                'publish_odom_tf': False,  # We handle TF ourselves
            }],
            output='screen',
            remappings=[
                ('pose/sample', '/camera/pose/sample'),
                ('fisheye1/image_raw', '/camera/fisheye1/image_raw'),
                ('fisheye2/image_raw', '/camera/fisheye2/image_raw'),
            ],
        ),

        # ========== Sony IMX219 Camera (via GStreamer) ==========
        DeclareLaunchArgument(
            name='enable_imx219',
            default_value='true',
            description='Enable IMX219 camera'
        ),

        DeclareLaunchArgument(
            name='camera_width',
            default_value='640',
            description='Camera capture width'
        ),

        DeclareLaunchArgument(
            name='camera_height',
            default_value='480',
            description='Camera capture height'
        ),

        DeclareLaunchArgument(
            name='camera_fps',
            default_value='30',
            description='Camera frame rate'
        ),

        # IMX219 camera node using v4l2_camera or gscam
        # Note: On Jetson, use gscam with nvarguscamerasrc for better performance
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='imx219_camera',
            namespace='mary',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'pixel_format': 'YUYV',
                'camera_frame_id': 'camera_link',
            }],
            output='screen',
            remappings=[
                ('image_raw', '/mary/camera/image_raw'),
                ('camera_info', '/mary/camera/camera_info'),
            ],
        ),

        # ========== TeraRanger Evo 60m ToF ==========
        # Note: TeraRanger publishes via serial/I2C
        # This is a placeholder - actual driver depends on interface

        DeclareLaunchArgument(
            name='enable_tof',
            default_value='true',
            description='Enable ToF sensor'
        ),

        DeclareLaunchArgument(
            name='tof_port',
            default_value='/dev/ttyUSB1',
            description='ToF sensor serial port'
        ),

        # TeraRanger Evo driver node
        # Note: Install teraranger_array ROS2 package or use custom driver
        # Node(
        #     package='teraranger_array',
        #     executable='teraranger_evo',
        #     name='teraranger_evo',
        #     namespace='mary',
        #     parameters=[{
        #         'serial_port': LaunchConfiguration('tof_port'),
        #         'frame_id': 'tof_frame',
        #     }],
        #     output='screen',
        #     remappings=[
        #         ('range', '/mary/tof/range'),
        #     ],
        # ),
    ])
