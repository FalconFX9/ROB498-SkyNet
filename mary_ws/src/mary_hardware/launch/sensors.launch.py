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
        # Install: sudo apt install ros-foxy-realsense2-camera

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

        # ========== Sony IMX219-160 Camera ==========
        # Jetson CSI-2 camera using nvarguscamerasrc (hardware ISP, recommended)
        # Requires: sudo apt install ros-foxy-gscam2
        # Verify hardware: gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! xvimagesink

        DeclareLaunchArgument(
            name='enable_imx219',
            default_value='true',
            description='Enable IMX219-160 CSI camera'
        ),

        DeclareLaunchArgument(
            name='camera_width',
            default_value='1280',
            description='Camera capture width'
        ),

        DeclareLaunchArgument(
            name='camera_height',
            default_value='720',
            description='Camera capture height'
        ),

        DeclareLaunchArgument(
            name='camera_fps',
            default_value='30',
            description='Camera frame rate'
        ),

        # IMX219 via gscam2 with Jetson hardware ISP (nvarguscamerasrc)
        # sensor-id=0 for CAM0 port, sensor-id=1 for CAM1 port
        Node(
            package='gscam2',
            executable='gscam_main',
            name='imx219_camera',
            namespace='mary',
            parameters=[{
                'gscam_config': (
                    'nvarguscamerasrc sensor-id=0 ! '
                    'video/x-raw(memory:NVMM), width=1280, height=720, '
                    'format=NV12, framerate=30/1 ! '
                    'nvvidconv ! video/x-raw, format=BGRx ! '
                    'videoconvert ! video/x-raw, format=BGR'
                ),
                'camera_name': 'imx219',
                'camera_info_url': '',  # Optional: path to calibration yaml
                'frame_id': 'camera_link',
                'sync_sink': True,
            }],
            output='screen',
            remappings=[
                ('image_raw', '/mary/camera/image_raw'),
                ('camera_info', '/mary/camera/camera_info'),
            ],
        ),

    ])
