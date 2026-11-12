#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Nav2 导航启动（实车 / 仿真通用）。

    # 实车（先启动 real_bringup.launch.py，并用 map_saver 存好地图）
    ros2 launch tb6612_car nav2.launch.py map:=$HOME/my_map.yaml

    # 仿真
    ros2 launch tb6612_car nav2.launch.py use_sim_time:=true map:=$HOME/sim_map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tb = get_package_share_directory('tb6612_car')
    nav2 = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.expanduser('~/my_map.yaml'),
            description='地图 yaml（用 map_saver_cli 生成）'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='仿真里跑要设成 true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(tb, 'config', 'nav2_params.yaml')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'slam': 'False',
                'use_composition': 'False',
            }.items(),
        ),
    ])
