#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键启动：小车桥接 + 机器人模型(TF) + 可选 RViz。

用法:
  ros2 launch tb6612_car bringup.launch.py                  # 桥接 + 模型
  ros2 launch tb6612_car bringup.launch.py rviz:=true       # 再加 RViz
  ros2 launch tb6612_car bringup.launch.py dry_run:=true    # 不动电机
  ros2 launch tb6612_car bringup.launch.py use_rsp:=false    # 只要桥接
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('tb6612_car')
    params = os.path.join(share, 'config', 'params.yaml')
    rviz_cfg = os.path.join(share, 'config', 'car.rviz')

    use_rsp = LaunchConfiguration('use_rsp')
    rviz = LaunchConfiguration('rviz')

    # my_bot 是可选的：没装也能只跑桥接
    rsp_actions = []
    try:
        mybot = get_package_share_directory('my_bot')
        rsp_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(mybot, 'launch', 'rsp.launch.py')),
                condition=IfCondition(use_rsp),
            )
        )
    except Exception:
        rsp_actions.append(
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 output='screen', condition=IfCondition(use_rsp),
                 parameters=[{'robot_description': ''}])
        )

    return LaunchDescription([
        DeclareLaunchArgument('port',
                              default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
        DeclareLaunchArgument('control_mode', default_value='closed_loop',
                              description='closed_loop 或 open_loop'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('invert_left', default_value='false'),
        DeclareLaunchArgument('invert_right', default_value='false'),
        DeclareLaunchArgument('use_rsp', default_value='true',
                              description='启动 robot_state_publisher（需要 my_bot 包）'),
        DeclareLaunchArgument('rviz', default_value='false', description='启动 RViz'),

        Node(
            package='tb6612_car',
            executable='car_bridge',
            name='car_bridge',
            output='screen',
            parameters=[
                params,
                {'port': LaunchConfiguration('port'),
                 'control_mode': LaunchConfiguration('control_mode'),
                 'dry_run': LaunchConfiguration('dry_run'),
                 'invert_left': LaunchConfiguration('invert_left'),
                 'invert_right': LaunchConfiguration('invert_right')},
            ],
        ),

        *rsp_actions,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(rviz),
        ),
    ])
