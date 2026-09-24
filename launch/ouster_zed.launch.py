"""Launch a live Ouster lidar and a ZED camera together with one shared RViz view.

Both sensors are attached to a common base frame (default: base_link) through two
static transforms:

    base_link -> zed_camera_link   (zed_x/y/z/roll/pitch/yaw)
    base_link -> os_sensor         (ouster_x/y/z/roll/pitch/yaw)

Translations are in meters, rotations in radians. Each pose value is taken from,
in order of precedence:

  1. the matching launch argument, e.g. zed_yaw:=0.84
  2. calibration_file (default ~/.ros/ouster_zed/calibration.yaml), if it exists
  3. 0.0

With register:=true the global_registration node estimates the sensor poses and
writes them to calibration_file, so later launches start pre-calibrated.

Because a TF frame can only have one parent, the ZED node's own
odom -> zed_camera_link publishing is disabled here.
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


POSE_KEYS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
DEFAULT_CALIBRATION_FILE = os.path.join('~', '.ros', 'ouster_zed', 'calibration.yaml')


def pose_args(prefix, frame):
    """Declare <prefix>_x ... <prefix>_yaw; empty means "not given on the command line"."""
    return [
        DeclareLaunchArgument(
            f'{prefix}_{k}', default_value='',
            description=f'{k} of {frame} relative to base_frame '
                        f'({"m" if k in ("x", "y", "z") else "rad"}); '
                        'overrides calibration_file, 0 if neither is given')
        for k in POSE_KEYS
    ]


def load_calibration(path):
    """Return the parsed calibration file, or None if there is none."""
    if not os.path.isfile(path):
        return None
    with open(path, encoding='utf-8') as f:
        calibration = yaml.safe_load(f) or {}
    for prefix in ('ouster', 'zed'):
        missing = [k for k in POSE_KEYS if k not in calibration.get(prefix, {})]
        if missing:
            raise RuntimeError(f'{path}: "{prefix}" is missing {", ".join(missing)}')
    return calibration


def sensor_transforms(context):
    """Resolve both sensor poses and publish them as static transforms."""
    path = os.path.expanduser(LaunchConfiguration('calibration_file').perform(context))
    base_frame = LaunchConfiguration('base_frame').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)

    actions = []
    calibration = load_calibration(path) if path else None
    if calibration is None:
        actions.append(LogInfo(msg=f'No calibration file at "{path}"; sensor poses come '
                                   'from the launch arguments (0 when not given)'))
        calibration = {}
    else:
        actions.append(LogInfo(msg=f'Loaded sensor poses from "{path}" '
                                   f'(written {calibration.get("stamp", "at an unknown time")})'))
        if calibration.get('base_frame', base_frame) != base_frame:
            actions.append(LogInfo(msg=f'WARNING: {path} was calibrated for base_frame '
                                       f'"{calibration["base_frame"]}", not "{base_frame}"'))

    for prefix, child_frame in (('ouster', 'os_sensor'), ('zed', f'{camera_name}_camera_link')):
        pose = []
        for k in POSE_KEYS:
            value = LaunchConfiguration(f'{prefix}_{k}').perform(context)
            if value == '':
                value = str(calibration.get(prefix, {}).get(k, 0.0))
            pose += [f'--{k}', value]
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'base_to_{prefix}_tf',
            arguments=pose + ['--frame-id', base_frame, '--child-frame-id', child_frame],
        ))
    return actions


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
        DeclareLaunchArgument(
            'timestamp_mode', default_value='TIME_FROM_ROS_TIME',
            description='Ouster timestamp mode; ROS time puts both sensors on the same clock '
                        '(the sensor default TIME_FROM_INTERNAL_OSC counts from power-up)'),
        # ZED
        DeclareLaunchArgument('camera_model', default_value='zed2i',
                              description='ZED camera model'),
        DeclareLaunchArgument('camera_name', default_value='zed',
                              description='ZED camera name (prefix of its TF frames)'),
        # Mounting poses relative to base_frame
        DeclareLaunchArgument('base_frame', default_value='base_link',
                              description='common parent frame of both sensors'),
        *pose_args('ouster', 'os_sensor'),
        *pose_args('zed', '<camera_name>_camera_link'),
        # Calibration
        DeclareLaunchArgument(
            'calibration_file', default_value=DEFAULT_CALIBRATION_FILE,
            description='sensor poses to load if the file exists; with register:=true the '
                        'registration result is also written here. Empty disables both'),
        DeclareLaunchArgument(
            'register', default_value='false',
            description='run the global_registration node and save its result to '
                        'calibration_file'),
        DeclareLaunchArgument('reference_sensor', default_value='ouster',
                              description='sensor kept fixed by the registration: ouster or zed'),
        # Viz
        DeclareLaunchArgument('viz', default_value='true',
                              description='run rviz with rviz_config, showing both point clouds'),
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
            'timestamp_mode': LaunchConfiguration('timestamp_mode'),
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
            # base_frame -> zed_camera_link is published by sensor_transforms instead
            'publish_tf': 'false',
        }.items(),
    )

    registration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ouster_zed'), 'launch', 'global_registration.launch.py'])),
        condition=IfCondition(LaunchConfiguration('register')),
        launch_arguments={
            'base_frame': LaunchConfiguration('base_frame'),
            'zed_frame': [LaunchConfiguration('camera_name'), '_camera_link'],
            'reference_sensor': LaunchConfiguration('reference_sensor'),
            'calibration_file': LaunchConfiguration('calibration_file'),
        }.items(),
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration('viz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    # Arguments passed to an include leak into this launch file's own
    # configuration unless the include is scoped; without this, the ouster
    # include's viz:=false would switch off the shared rviz below.
    def scoped(include):
        return GroupAction([include], scoped=True)

    return LaunchDescription(args + [
        scoped(ouster), scoped(zed), OpaqueFunction(function=sensor_transforms),
        scoped(registration), rviz])
