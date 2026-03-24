"""
MARY Demo: Person Following -- Launch File

Launches the full person-following pipeline:
  - MAVROS         (FCU bridge)
  - T265 camera    (tracking camera driver -- pose + fisheye stereo)
  - T265 Pose      (VIO processing -- MAVROS relay disabled)
  - Stereo Tracker (VPI disparity + blob detection -> target position)
  - Follower       (tracking -> position setpoints -> MAVROS)

Usage:
  # With VICON ground truth:
  ros2 launch mary_bringup mary_demo.launch.py part:=1

  # T265 only (no VICON):
  ros2 launch mary_bringup mary_demo.launch.py part:=2

  # With calibrated T265 offset:
  ros2 launch mary_bringup mary_demo.launch.py part:=2 t265_z_offset:=0.1234
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
    """Configure nodes based on 'part' and 'launch_t265' arguments."""
    part = int(LaunchConfiguration('part').perform(context))
    vicon_topic = VICON_TOPIC if part == 1 else ''
    launch_t265 = LaunchConfiguration('launch_t265').perform(context).lower() == 'true'

    nodes = []

    # -- T265 camera + pose node (optional — can be started manually later) --
    if launch_t265:
        nodes.append(Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='t265_camera',
            namespace='camera',
            parameters=[{
                'serial_no':             '',
                'device_type':           't265',
                'enable_pose':           True,
                'enable_fisheye1':       True,
                'enable_fisheye2':       True,
                'fisheye_fps':           30,
                'pose_fps':              200,
                'publish_odom_tf':       False,
                'enable_relocalization': False,
            }],
            output='screen',
            remappings=[
                ('pose/sample',        '/camera/pose/sample'),
                ('fisheye1/image_raw', '/camera/fisheye1/image_raw'),
                ('fisheye2/image_raw', '/camera/fisheye2/image_raw'),
            ],
        ))
        nodes.append(Node(
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
        ))

    nodes += [
        # -- Stereo Tracker (perception) -----------------------------------
        Node(
            package='mary_perception',
            executable='stereo_tracker_node',
            name='stereo_tracker_node',
            output='screen',
            parameters=[{
                'libstereo_path':     LaunchConfiguration('libstereo_path'),
                'max_disparity':      256,
                'zoom_factor':        1.5,
                'median_filter_size': 3,
                'publish_debug':      True,
                'depth_min':          float(LaunchConfiguration('depth_min').perform(context)),
                'depth_max':          float(LaunchConfiguration('depth_max').perform(context)),
                'min_blob_area':      500,
                'ema_alpha':          0.3,
                'lost_threshold':     15,
                'axis_sign_x':        float(LaunchConfiguration('axis_sign_x').perform(context)),
                'axis_sign_y':        float(LaunchConfiguration('axis_sign_y').perform(context)),
            }],
        ),

        # -- Follower (control) --------------------------------------------
        Node(
            package='mary_control',
            executable='follower_node',
            name='rob498_drone_10',
            output='screen',
            parameters=[{
                'drone_id':              LaunchConfiguration('drone_id'),
                'takeoff_altitude':      float(LaunchConfiguration('takeoff_altitude').perform(context)),
                'target_altitude_above': float(LaunchConfiguration('target_alt_above').perform(context)),
                'setpoint_rate':         20.0,
                'vision_pose_rate':      30.0,
                'descent_speed':         0.15,
                'offboard_wait':         1.5,
                'mode_request_interval': 2.0,
                'vicon_topic':           vicon_topic,
                'vicon_timeout':         0.5,
                'vicon_yaw_offset':      float(LaunchConfiguration('vicon_yaw_offset').perform(context)),
                't265_z_offset':         float(LaunchConfiguration('t265_z_offset').perform(context)),
                'hover_timeout':         float(LaunchConfiguration('hover_timeout').perform(context)),
                'max_follow_speed':      1.5,
                'log_euler':             LaunchConfiguration('log_euler'),
            }],
        ),
    ]

    # Optional pose logger
    record_log = LaunchConfiguration('record_log').perform(context).lower() == 'true'
    if record_log:
        nodes.append(Node(
            package='mary_perception',
            executable='pose_logger_node',
            name='pose_logger_node',
            output='screen',
            parameters=[{
                'log_dir': LaunchConfiguration('log_dir'),
            }],
        ))

    return nodes


def generate_launch_description():
    hardware_pkg = get_package_share_directory('mary_hardware')

    return LaunchDescription([

        # -- Arguments ---------------------------------------------------------
        DeclareLaunchArgument('drone_id',         default_value='rob498_drone_10'),
        DeclareLaunchArgument('fcu_url',          default_value='/dev/ttyACM0:921600'),
        DeclareLaunchArgument('gcs_url',          default_value=''),
        DeclareLaunchArgument('part',             default_value='1',
                              description='1 = VICON, 2 = T265 only'),
        DeclareLaunchArgument('t265_z_offset',    default_value='0.0'),
        DeclareLaunchArgument('vicon_yaw_offset', default_value='0.0'),
        DeclareLaunchArgument('log_euler',        default_value='false'),
        DeclareLaunchArgument('record_log',       default_value='false'),
        DeclareLaunchArgument('log_dir',          default_value=''),

        # Follower parameters
        DeclareLaunchArgument('takeoff_altitude',  default_value='1.5',
                              description='Takeoff/hover altitude (m)'),
        DeclareLaunchArgument('target_alt_above',  default_value='1.0',
                              description='Target altitude above person (m)'),
        DeclareLaunchArgument('hover_timeout',     default_value='60.0',
                              description='Seconds without tracking before auto-land'),

        # Tracker parameters
        DeclareLaunchArgument('depth_min',         default_value='0.5',
                              description='Min depth for blob detection (m)'),
        DeclareLaunchArgument('depth_max',         default_value='2.5',
                              description='Max depth for blob detection (m)'),
        DeclareLaunchArgument('axis_sign_x',       default_value='1.0',
                              description='Body X axis sign (flip if tracking inverted)'),
        DeclareLaunchArgument('axis_sign_y',       default_value='1.0',
                              description='Body Y axis sign (flip if tracking inverted)'),
        DeclareLaunchArgument('launch_t265',       default_value='true',
                              description='Launch T265 camera + pose node (set false to start manually later)'),
        DeclareLaunchArgument('libstereo_path',
                              default_value=os.path.expanduser(
                                  '~/Documents/SkyNet/ROB498-SkyNet/scripts/libstereo.so')),

        # -- Hardware layer ----------------------------------------------------

        # MAVROS
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hardware_pkg, 'launch', 'mavros.launch.py')),
            launch_arguments={
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
            }.items(),
        ),

        # -- T265 + Perception + Control (resolved via OpaqueFunction) ---------
        OpaqueFunction(function=_resolve_params),
    ])
