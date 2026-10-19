#!/usr/bin/env python3
# DARON — EKF 단독 기동 런치 (배관 검증용)
# 배치 경로: ~/daron_ros/launch/ekf.launch.py
#
# 사용:
#   실시간  : ros2 launch ~/daron_ros/launch/ekf.launch.py
#   bag검증 : ros2 launch ~/daron_ros/launch/ekf.launch.py use_sim_time:=true
#             (동시에 ros2 bag play <bag> --clock 필요)

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # [daron_run] 이 파일 기준 ../config/ekf.yaml
    default_params = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                  'config', 'ekf.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_tf = LaunchConfiguration('publish_tf')
    ros_domain_id = LaunchConfiguration('ros_domain_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='ekf.yaml 경로',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='bag 재생 검증 시 true (bag play --clock 과 세트)',
        ),

        # [260929] daron_viz tf_source:=C 면 false 로 넘어온다 — odom→base_link TF 는 한 EKF 만 내야 한다
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='odom→base_link TF 발행 (ekf.yaml 값을 덮어씀). TF 를 다른 EKF 가 내면 false',
        ),
        # [261017] daron_viz 가 include 로 띄울 때 ros_domain_id 가 이 노드에만 안 넘어가던 문제
        #   (다른 프로세스는 additional_env 로 받는데 include 된 Node 는 셸 값 그대로였다).
        #   기본값은 셸의 ROS_DOMAIN_ID 라 인자를 안 주면 예전과 같다.
        DeclareLaunchArgument(
            'ros_domain_id',
            default_value=EnvironmentVariable('ROS_DOMAIN_ID', default_value='0'),
            description='이 EKF 프로세스의 ROS_DOMAIN_ID (daron_viz 가 자기 ros_domain_id 를 넘긴다)',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            additional_env={'ROS_DOMAIN_ID': ros_domain_id},
            # ⚠ 이름이 ekf.yaml 최상위 키(ekf_filter_node)와 반드시 일치해야
            #   파라미터가 로드된다. 다르면 조용히 전부 기본값으로 뜬다.
            name='ekf_filter_node',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
                {'publish_tf': publish_tf},
            ],
            # 필요 시 리매핑:
            # remappings=[('odometry/filtered', '/odometry/filtered')],
        ),
    ])
