"""Run the Ouster <-> ZED global registration node against already running sensors."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def registration_node(context):
    return [Node(
        package='ouster_zed',
        executable='global_registration_node',
        name='global_registration',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'base_frame': LaunchConfiguration('base_frame'),
                'zed_frame': LaunchConfiguration('zed_frame'),
                'reference_sensor': LaunchConfiguration('reference_sensor'),
                # expand ~ here, the node takes the path literally
                'calibration_file': os.path.expanduser(
                    LaunchConfiguration('calibration_file').perform(context)),
            },
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('ouster_zed'), 'config', 'global_registration.yaml']),
            description='global_registration parameters'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('zed_frame', default_value='zed_camera_link'),
        DeclareLaunchArgument('reference_sensor', default_value='ouster',
                              description='sensor that keeps its pose: ouster or zed'),
        DeclareLaunchArgument(
            'calibration_file', default_value='',
            description='where to save the registration result; empty to not save'),
        OpaqueFunction(function=registration_node),
    ])
