"""A2: start the eight-state EKF estimator and (optionally) the mission flown on its output."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('lrs_psenak_valko'), 'config', 'localization.yaml')
    params = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Estimator + mission parameters'),
        DeclareLaunchArgument('mission', default_value='true',
                              description='false: estimator only (characterisation, bag replay)'),
        Node(package='lrs_psenak_valko', executable='localization_estimator_node',
             name='localization_estimator', output='screen', parameters=[params]),
        Node(package='lrs_psenak_valko', executable='localization_mission_node',
             name='localization_mission', output='screen', parameters=[params],
             condition=IfCondition(LaunchConfiguration('mission'))),
    ])
