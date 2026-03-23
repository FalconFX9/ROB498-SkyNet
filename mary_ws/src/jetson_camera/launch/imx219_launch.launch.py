from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # This is the "Magic String" optimized for Jetson hardware
    # It converts to BGR in C++ which gscam2 handles efficiently
    gscam_config = (
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=RGB"
    )

    return LaunchDescription([
        Node(
            package='gscam2',
            executable='gscam_main',
            name='gscam_node',
            parameters=[{
                'gscam_config': gscam_config,
                'camera_name': 'narrow_stereo',
                'frame_id': 'camera_link',
                'sync_sink': True,
                'image_encoding': 'rgb8',
            }],
            remappings=[
                ('/image_raw', '/camera/image_raw'),
            ]
        ),
    ])
