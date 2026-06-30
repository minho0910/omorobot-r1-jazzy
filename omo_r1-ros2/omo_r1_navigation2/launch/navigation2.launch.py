#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # --- 1. 기본 설정 변수 ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    log_level = LaunchConfiguration('log_level', default='info')
    
    # 패키지 경로 설정
    omo_r1_nav2_dir = get_package_share_directory('omo_r1_navigation2')
    launch_dir = os.path.join(omo_r1_nav2_dir, 'launch') 

    # 지도 파일 경로
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            omo_r1_nav2_dir,
            'map',
            'lk_map.yaml'))

    # 파라미터 파일 경로
    param_file_name = 'nav2_params.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            omo_r1_nav2_dir,
            'param',
            param_file_name))

    # --- 2. 컨테이너 설정 ---
    namespace = LaunchConfiguration('namespace', default='')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=param_dir,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    return LaunchDescription([
        # --- 3. 기본 인자 선언 ---
        DeclareLaunchArgument('map', default_value=map_dir),
        DeclareLaunchArgument('params_file', default_value=param_dir),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # --- 4. 'nav2_container' 노드 실행 ---
        Node(
            name='nav2_container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen',
        ),
        # --- 5. 로컬라이제이션 및 내비게이션 런치 파일 포함 ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/localization_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'autostart': autostart,
                'use_composition': 'True',
                'container_name': 'nav2_container'
            }.items(),
        ),

        # --- 6. 내비게이션 런치 파일 포함 ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'autostart': autostart,
                'use_composition': 'True',
                'container_name': 'nav2_container',
                # 'use_collision_monitor' 인자는 이제 필요 없습니다. (파일 자체를 수정했으므로)
            }.items(),
        )
    ])