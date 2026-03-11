"""
Flight Test #3: Waypoint Navigation — Launch File

Launches the nodes required for the waypoint navigation exercise:
  - MAVROS        (FCU bridge)
  - T265 camera   (tracking camera driver)
  - T265 Pose     (VIO processing — MAVROS relay disabled)
  - Waypoint Node (waypoint controller + course interface)

Usage:
  # Part I  — VICON-aided:
  ros2 launch mary_bringup flight_test_3.launch.py part:=1

  # Part II — on-board sensing only (T265, no VICON):
  ros2 launch mary_bringup flight_test_3.launch.py part:=2

  # With calibrated T265 offset:
  ros2 launch mary_bringup flight_test_3.launch.py part:=2 t265_z_offset:=0.1234
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


VICON_TOPIC = '/vicon/ROB498_Drone/ROB498_Drone'


def _resolve_params(context):
    """Pick vicon_topic and configure waypoint node based on 'part' argument."""
    part = int(LaunchConfiguration('part').perform(context))
    vicon_topic = VICON_TOPIC if part == 1 else ''
    record_log = LaunchConfiguration('record_log').perform(context).lower() == 'true'
    log_dir = LaunchConfiguration('log_dir').perform(context)

    nodes = [
        Node(
            package='mary_control',
            executable='waypoint_node',
            name='rob498_drone_10',
            output='screen',
            parameters=[{
                'drone_id':             LaunchConfiguration('drone_id'),
                'takeoff_altitude':     0.5,
                'setpoint_rate':        20.0,
                'vision_pose_rate':     30.0,
                'descent_speed':        0.15,
                'offboard_wait':        1.5,
                'mode_request_interval': 2.0,
                'vicon_topic':          vicon_topic,
                'vicon_timeout':        0.5,
                'vicon_yaw_offset':     float(LaunchConfiguration('vicon_yaw_offset').perform(context)),
                't265_z_offset':        float(LaunchConfiguration('t265_z_offset').perform(context)),
                'reach_radius':         0.4,
                'log_euler':            LaunchConfiguration('log_euler'),
            }],
        ),
    ]

    if record_log:
        nodes.append(Node(
            package='mary_perception',
            executable='pose_logger_node',
            name='pose_logger_node',
            output='screen',
            parameters=[{'log_dir': log_dir}],
        ))

    return nodes


def generate_launch_description():
    hardware_pkg = get_package_share_directory('mary_hardware')

    return LaunchDescription([

        # ── Arguments ─────────────────────────────────────────────────
        DeclareLaunchArgument(
            'drone_id',
            default_value='rob498_drone_10',
            description='Team drone identifier for course services',
        ),
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyACM0:921600',
            description='FCU serial connection URL',
        ),
        DeclareLaunchArgument(
            'gcs_url',
            default_value='udp://@10.42.0.129:14550',
            description='GCS URL',
        ),
        DeclareLaunchArgument(
            'part',
            default_value='1',
            description='Flight test part: 1 = VICON, 2 = T265 only',
        ),
        DeclareLaunchArgument(
            't265_z_offset',
            default_value='0.0',
            description='T265 height offset in metres (from calibration)',
        ),
        DeclareLaunchArgument(
            'vicon_yaw_offset',
            default_value='0.0',
            description='Vicon rigid body yaw correction in radians (positive=CCW)',
        ),
        DeclareLaunchArgument(
            'log_euler',
            default_value='false',
            description='Log yaw/pitch/roll of pose topics at ~2 Hz',
        ),
        DeclareLaunchArgument(
            'record_log',
            default_value='false',
            description='Enable flight logging (console + vision_pose CSV). Use record_flight.sh.',
        ),
        DeclareLaunchArgument(
            'log_dir',
            default_value='',
            description='Directory to write flight logs into (set by record_flight.sh)',
        ),

        # ── Hardware layer ────────────────────────────────────────────

        # MAVROS — bridges Jetson ↔ PX4 via MAVLink
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hardware_pkg, 'launch', 'mavros.launch.py'),
            ),
            launch_arguments={
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
            }.items(),
        ),

        # T265 tracking camera
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='t265_camera',
            namespace='camera',
            parameters=[{
                'serial_no':         '',
                'device_type':       't265',
                'enable_pose':       True,
                'enable_fisheye1':   True,
                'enable_fisheye2':   True,
                'fisheye_fps':       30,
                'pose_fps':          200,
                'publish_odom_tf':   False,
            }],
            output='screen',
            remappings=[
                ('pose/sample',          '/camera/pose/sample'),
                ('fisheye1/image_raw',   '/camera/fisheye1/image_raw'),
                ('fisheye2/image_raw',   '/camera/fisheye2/image_raw'),
            ],
        ),

        # ── Perception layer ──────────────────────────────────────────

        # T265 pose processing — MAVROS relay disabled (waypoint node handles it)
        Node(
            package='mary_perception',
            executable='t265_pose_node',
            name='t265_pose_node',
            output='screen',
            parameters=[{
                'publish_rate':      30.0,
                'publish_tf':        True,
                'publish_to_mavros': False,
                'log_euler':         LaunchConfiguration('log_euler'),
            }],
        ),

        # ── Control layer ─────────────────────────────────────────────
        OpaqueFunction(function=_resolve_params),
    ])
