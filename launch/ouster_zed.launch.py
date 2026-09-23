"""Launch a live Ouster lidar and a ZED camera together with one shared RViz view.

The two sensors publish independent TF trees (ZED: map -> odom -> zed_camera_link -> ...,
Ouster: os_sensor -> os_lidar/os_imu). A static transform zed_camera_link -> os_sensor
joins them so both point clouds can be shown in a single fixed frame. Set the
ouster_x/y/z/roll/pitch/yaw arguments to the Ouster's actual mounting offset
relative to the ZED camera.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    args = [
        # Ouster
        DeclareLaunchArgument('sensor_hostname', default_value='os-122247000785.local',
                              description='hostname or IP of the Ouster sensor'),
        DeclareLaunchArgument('proc_mask', default_value='IMU|PCL|ZONE',
                              description='Ouster processors to enable'),
        DeclareLaunchArgument('lidar_port', default_value='7502'),
        DeclareLaunchArgument('imu_port', default_value='7503'),
        DeclareLaunchArgument('zone_port', default_value='7504'),
        DeclareLaunchArgument('ouster_ns', default_value='ouster'),
        # ZED
        DeclareLaunchArgument('camera_model', default_value='zed2i',
                              description='ZED camera model'),
        DeclareLaunchArgument('camera_name', default_value='zed',
                              description='ZED camera name (prefix of its TF frames)'),
        # Ouster mounting pose relative to <camera_name>_camera_link (meters / radians)
        DeclareLaunchArgument('ouster_x', default_value='0.0'),
        DeclareLaunchArgument('ouster_y', default_value='0.0'),
        DeclareLaunchArgument('ouster_z', default_value='0.0'),
        DeclareLaunchArgument('ouster_roll', default_value='0.0'),
        DeclareLaunchArgument('ouster_pitch', default_value='0.0'),
        DeclareLaunchArgument('ouster_yaw', default_value='0.0'),
        # Viz
        DeclareLaunchArgument('viz', default_value='true', description='whether to run rviz'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('ouster_zed'), 'config', 'ouster_zed.rviz']),
            description='rviz config file'),
    ]

    ouster = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ouster_ros'), 'launch', 'sensor.launch.xml'])),
        launch_arguments={
            'ouster_ns': LaunchConfiguration('ouster_ns'),
            'sensor_hostname': LaunchConfiguration('sensor_hostname'),
            'proc_mask': LaunchConfiguration('proc_mask'),
            'lidar_port': LaunchConfiguration('lidar_port'),
            'imu_port': LaunchConfiguration('imu_port'),
            'zone_port': LaunchConfiguration('zone_port'),
            # the shared rviz below replaces the driver's own
            'viz': 'false',
        }.items(),
    )

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
            'camera_name': LaunchConfiguration('camera_name'),
        }.items(),
    )

    zed_to_ouster_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_to_ouster_tf',
        arguments=[
            '--x', LaunchConfiguration('ouster_x'),
            '--y', LaunchConfiguration('ouster_y'),
            '--z', LaunchConfiguration('ouster_z'),
            '--roll', LaunchConfiguration('ouster_roll'),
            '--pitch', LaunchConfiguration('ouster_pitch'),
            '--yaw', LaunchConfiguration('ouster_yaw'),
            '--frame-id', [LaunchConfiguration('camera_name'), '_camera_link'],
            '--child-frame-id', 'os_sensor',
        ],
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration('viz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription(args + [ouster, zed, zed_to_ouster_tf, rviz])
