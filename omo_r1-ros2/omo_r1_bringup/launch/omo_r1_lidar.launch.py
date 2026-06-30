#!/usr/bin/env python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    bringup_share = get_package_share_directory('omo_r1_bringup')
    omo_r1_lidar_parameter = LaunchConfiguration(
        'omo_r1_lidar_parameter',
        default=os.path.join(
            bringup_share,
            'param',
            'omo_r1_lidar.yaml'
        )
    )
    laser_filter_config = LaunchConfiguration(
        'laser_filter_config',
        default=os.path.join(
            bringup_share,
            'config',
            'laser_filter.yaml'
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'omo_r1_lidar_parameter',
            default_value=omo_r1_lidar_parameter,
            description='Parameter file for the YDLIDAR driver node',
        ),
        DeclareLaunchArgument(
            'laser_filter_config',
            default_value=laser_filter_config,
            description='Laser filter polygon configuration',
        ),

        LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[omo_r1_lidar_parameter],
            namespace='/',
        ),
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter',
            output='screen',
            parameters=[laser_filter_config],
        )
    ])
